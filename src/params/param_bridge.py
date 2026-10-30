"""Long-lived ROS2 helper of the `params` binary: one process for all its
parameter access, instead of a `ros2 param` CLI process per node and request.

Run as `python3 -c <this file>`. Commands arrive on stdin as JSON lines:

    {"cmd": "list"}                                   parameters of all nodes
    {"cmd": "node", "node": N}                        parameters of node N
    {"cmd": "set", "node": N, "name": P, "value": V}  set P from the text V
    {"cmd": "dump", "node": N, "path": F}             N's parameters as YAML
    {"cmd": "load", "node": N, "path": F, "yaml": Y}  set N's parameters from Y

Output on stdout is frames of a little-endian u32 length, a u8 kind (0) and a
JSON event. stdin closing means the parent is gone.
"""

import json
import math
import os
import struct
import sys
import threading
import time

import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor

EVENT = 0
# How long a new client may take to discover its node's service.
DISCOVERY_TIMEOUT = 2.0
# How long a node may take to answer a parameter request.
CALL_TIMEOUT = 5.0
# A new node needs a moment to discover the graph, and the nodes' services
# arrive one by one: the first list waits until the nodes found stayed the
# same for SETTLE_TIME, at most until STARTUP_TIME after the start.
SETTLE_TIME = 0.5
STARTUP_TIME = 4.0

_out = sys.stdout.buffer
_out_lock = threading.Lock()


def event(**fields):
    payload = json.dumps(fields).encode()
    try:
        with _out_lock:
            _out.write(struct.pack("<IB", len(payload), EVENT))
            _out.write(payload)
            _out.flush()
    except (BrokenPipeError, ValueError):
        os._exit(0)


def python_value(value):
    """The Python value of a ParameterValue, None if not set."""
    t = value.type
    if t == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if t == ParameterType.PARAMETER_STRING:
        return value.string_value
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return [b[0] if isinstance(b, bytes) else int(b) for b in value.byte_array_value]
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(v) for v in value.bool_array_value]
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(v) for v in value.integer_array_value]
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(v) for v in value.double_array_value]
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None


def json_value(value):
    """Like python_value, with NaN and infinities as text, which JSON lacks."""
    def finite(v):
        return v if not isinstance(v, float) or math.isfinite(v) else str(v)
    v = python_value(value)
    return [finite(x) for x in v] if isinstance(v, list) else finite(v)


def parameter_value(v, current=ParameterType.PARAMETER_NOT_SET):
    """A ParameterValue from a Python value, None if it has no parameter type.

    `current` is the parameter's type: an integer is taken as a double for a
    double parameter, and an empty list takes the array type, as a parameter
    keeps its type unless declared dynamic.
    """
    T = ParameterType
    pv = ParameterValue()
    if isinstance(v, bool):
        pv.type, pv.bool_value = T.PARAMETER_BOOL, v
    elif isinstance(v, int) and current == T.PARAMETER_DOUBLE:
        pv.type, pv.double_value = T.PARAMETER_DOUBLE, float(v)
    elif isinstance(v, int):
        pv.type, pv.integer_value = T.PARAMETER_INTEGER, v
    elif isinstance(v, float):
        pv.type, pv.double_value = T.PARAMETER_DOUBLE, v
    elif isinstance(v, str):
        pv.type, pv.string_value = T.PARAMETER_STRING, v
    elif isinstance(v, (list, tuple)):
        if not v and current in (T.PARAMETER_BYTE_ARRAY, T.PARAMETER_BOOL_ARRAY,
                                 T.PARAMETER_INTEGER_ARRAY, T.PARAMETER_DOUBLE_ARRAY,
                                 T.PARAMETER_STRING_ARRAY):
            pv.type = current
        elif all(isinstance(x, bool) for x in v):
            pv.type, pv.bool_array_value = T.PARAMETER_BOOL_ARRAY, list(v)
        elif all(isinstance(x, int) and not isinstance(x, bool) for x in v):
            if current == T.PARAMETER_DOUBLE_ARRAY:
                pv.type, pv.double_array_value = T.PARAMETER_DOUBLE_ARRAY, [float(x) for x in v]
            elif current == T.PARAMETER_BYTE_ARRAY:
                pv.type, pv.byte_array_value = T.PARAMETER_BYTE_ARRAY, [bytes([x]) for x in v]
            else:
                pv.type, pv.integer_array_value = T.PARAMETER_INTEGER_ARRAY, list(v)
        elif all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
            pv.type, pv.double_array_value = T.PARAMETER_DOUBLE_ARRAY, [float(x) for x in v]
        elif all(isinstance(x, str) for x in v):
            pv.type, pv.string_array_value = T.PARAMETER_STRING_ARRAY, list(v)
        else:
            return None
    else:
        return None
    return pv


def parse_text(text, current):
    """Parse a typed-in value as YAML, like `ros2 param set` does."""
    if current == ParameterType.PARAMETER_STRING:
        return text
    try:
        v = yaml.safe_load(text)
    except yaml.YAMLError:
        return text
    return text if v is None or isinstance(v, dict) else v


def flatten(params, prefix=""):
    """Nested YAML mappings as dotted parameter names."""
    for key, value in params.items():
        name = prefix + str(key)
        if isinstance(value, dict):
            yield from flatten(value, name + ".")
        else:
            yield name, value


def nest(names_and_values):
    """Dotted parameter names as nested mappings, like `ros2 param dump`."""
    tree = {}
    for name, value in names_and_values:
        *path, leaf = name.split(".")
        level = tree
        for part in path:
            level = level.setdefault(part, {})
        level[leaf] = value
    return tree


class Bridge:
    def __init__(self, node):
        self.node = node
        self.clients = {}
        self.clients_lock = threading.Lock()
        self.listing = threading.Lock()
        self.started = time.monotonic()

    def client(self, node_name, srv_type, srv_name):
        key = (node_name, srv_name)
        with self.clients_lock:
            if key not in self.clients:
                self.clients[key] = self.node.create_client(
                    srv_type, node_name + "/" + srv_name)
            return self.clients[key]

    def wait_ready(self, clients):
        """Wait until the clients' services are discovered; the ready ones."""
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while True:
            ready = [c for c in clients if c.service_is_ready()]
            if len(ready) == len(clients) or time.monotonic() > deadline:
                return ready
            time.sleep(0.02)

    def call_all(self, calls):
        """Run (client, request) pairs at once; responses, None if failed."""
        futures = []
        for client, request in calls:
            done = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _, done=done: done.set())
            futures.append((future, done))
        deadline = time.monotonic() + CALL_TIMEOUT
        responses = []
        for future, done in futures:
            if done.wait(max(0.0, deadline - time.monotonic())) and future.exception() is None:
                responses.append(future.result())
            else:
                future.cancel()
                responses.append(None)
        return responses

    def parameter_nodes(self):
        """Visible nodes offering parameter services, like `ros2 node list`."""
        nodes = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            if name.startswith("_"):
                continue
            full = namespace.rstrip("/") + "/" + name
            if full in nodes:
                continue
            try:
                services = self.node.get_service_names_and_types_by_node(name, namespace)
            except Exception:  # the node left meanwhile
                continue
            if any(s == full + "/list_parameters" for s, _ in services):
                nodes.append(full)
        return sorted(nodes)

    def list_parameters(self):
        """Send the parameters of all nodes.

        Right after the start, nodes and their services are still being
        discovered: the list is sent as nodes turn up, until the nodes found
        stayed the same for SETTLE_TIME or STARTUP_TIME has passed.
        """
        params, sent = {}, False
        nodes, since = self.parameter_nodes(), time.monotonic()
        while True:
            new = [n for n in nodes if n not in params]
            if new:
                params.update(self.fetch(new))
            if new or not sent:
                if nodes or time.monotonic() - self.started >= STARTUP_TIME:
                    self.send_list({n: params[n] for n in nodes if n in params})
                    sent = True
            settled = nodes and time.monotonic() - since >= SETTLE_TIME
            if settled or time.monotonic() - self.started >= STARTUP_TIME:
                if not sent:
                    self.send_list({})
                return
            time.sleep(0.1)
            found = self.parameter_nodes()
            if found != nodes:
                nodes, since = found, time.monotonic()

    def send_list(self, params):
        event(type="param_list", nodes=[
            {"node": n, "params": self.params_json(p)} for n, p in params.items()])

    def fetch(self, nodes):
        """Parameters of the nodes: {node: [(name, ParameterValue)]}."""
        listers = {n: self.client(n, ListParameters, "list_parameters") for n in nodes}
        getters = {n: self.client(n, GetParameters, "get_parameters") for n in nodes}
        ready = set(self.wait_ready(list(listers.values()) + list(getters.values())))
        nodes = [n for n in nodes if listers[n] in ready and getters[n] in ready]

        listed = self.call_all([(listers[n], ListParameters.Request()) for n in nodes])
        names = {n: sorted(r.result.names) for n, r in zip(nodes, listed) if r is not None}
        nodes = list(names)
        values = self.call_all(
            [(getters[n], GetParameters.Request(names=names[n])) for n in nodes])
        return {
            n: [(name, value) for name, value in zip(names[n], r.values)
                if value.type != ParameterType.PARAMETER_NOT_SET]
            for n, r in zip(nodes, values) if r is not None
        }

    @staticmethod
    def params_json(params):
        return [{"name": name, "type": value.type, "value": json_value(value)}
                for name, value in params]

    def set(self, node_name, parameters, from_text=False, skip_unchanged=False):
        """Set (name, value) pairs; error texts of those that failed.

        Values are Python values, or typed-in text if `from_text`. With
        `skip_unchanged`, parameters that already have their value are left
        alone, so loading a dump does not trip over read-only parameters.
        """
        current = self.fetch([node_name]).get(node_name)
        if current is None:
            return ["%s does not answer" % node_name]
        current = dict(current)
        errors, msgs = [], []
        for name, v in parameters:
            current_value = current.get(name)
            current_type = current_value.type if current_value else ParameterType.PARAMETER_NOT_SET
            if from_text:
                v = parse_text(v, current_type)
            pv = parameter_value(v, current_type)
            if pv is None:
                errors.append("%s: unsupported value %r" % (name, v))
            elif skip_unchanged and current_value and (
                    pv.type, python_value(pv)) == (current_value.type, python_value(current_value)):
                continue
            else:
                msgs.append(Parameter(name=name, value=pv))
        if msgs:
            setter = self.client(node_name, SetParameters, "set_parameters")
            if not self.wait_ready([setter]):
                return errors + ["%s has no set_parameters service" % node_name]
            [response] = self.call_all([(setter, SetParameters.Request(parameters=msgs))])
            if response is None:
                return errors + ["%s did not answer" % node_name]
            errors += ["%s: %s" % (p.name, r.reason or "rejected")
                       for p, r in zip(msgs, response.results) if not r.successful]
        return errors

    def handle(self, c):
        cmd = c.get("cmd")
        if cmd == "list":
            # A refresh still running answers this one too.
            if not self.listing.acquire(blocking=False):
                return
            try:
                self.list_parameters()
            finally:
                self.listing.release()
        elif cmd == "node":
            params = self.fetch([c["node"]]).get(c["node"])
            if params is None:
                event(type="error", message="%s does not answer" % c["node"])
            else:
                event(type="node_params", node=c["node"], params=self.params_json(params))
        elif cmd == "set":
            errors = self.set(c["node"], [(c["name"], c["value"])], from_text=True)
            event(type="set_result", node=c["node"], name=c["name"], value=c["value"],
                  error=errors[0] if errors else None)
            self.handle({"cmd": "node", "node": c["node"]})
        elif cmd == "dump":
            params = self.fetch([c["node"]]).get(c["node"])
            if params is None:
                event(type="error", message="%s does not answer" % c["node"])
                return
            tree = nest((name, python_value(value)) for name, value in params)
            text = yaml.dump({c["node"]: {"ros__parameters": tree}}, default_flow_style=False)
            event(type="dumped", node=c["node"], path=c["path"], yaml=text)
        elif cmd == "load":
            data = yaml.safe_load(c["yaml"]) or {}
            if not isinstance(data, dict):
                raise ValueError("not a parameter file")
            parameters = {}
            # Wildcard entries first, so the node's own entry overrides them.
            for key in ("/**", c["node"].lstrip("/"), c["node"]):
                entry = data.get(key)
                if isinstance(entry, dict) and isinstance(entry.get("ros__parameters"), dict):
                    parameters.update(flatten(entry["ros__parameters"]))
            if not parameters:
                raise ValueError("no parameters for %s in the file" % c["node"])
            errors = self.set(c["node"], list(parameters.items()), skip_unchanged=True)
            event(type="loaded", node=c["node"], path=c["path"], count=len(parameters),
                  errors=errors)
            self.handle({"cmd": "node", "node": c["node"]})

    def run(self, command):
        try:
            self.handle(command)
        except Exception as e:  # report, and keep serving the rest
            event(type="error", message="%s %s: %s: %s" % (
                command.get("cmd"), command.get("node", ""), type(e).__name__, e))


def main():
    rclpy.init(args=[])
    # A leading underscore hides the node from `ros2 node list` and our list.
    node = rclpy.create_node("_ros2_tui_params_%d" % os.getpid())
    bridge = Bridge(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    event(type="ready")
    # Each request in its own thread, so a node that does not answer delays
    # only the requests to it.
    for line in sys.stdin:
        line = line.strip()
        if line:
            threading.Thread(target=bridge.run, args=(json.loads(line),), daemon=True).start()
    os._exit(0)


try:
    main()
except KeyboardInterrupt:
    pass
