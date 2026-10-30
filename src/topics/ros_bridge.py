"""Long-lived ROS2 helper of the `topics` binary: one process for all its ROS
access, instead of a `ros2` CLI process per topic list, rate, delay, echo
and viewer.

Run as `python3 -c <this file>`. Commands arrive on stdin as JSON lines:

    {"cmd": "list"}                                   topic list
    {"cmd": "watch", "topic": T} / {"cmd": "unwatch", "topic": T}
                                                      rate and delay of T
    {"cmd": "echo", "topic": T} / {"cmd": "unecho"}   messages of T as YAML
    {"cmd": "view", "id": N, "topic": T, "type": M, "fps": F}
    {"cmd": "fps", "id": N, "fps": F} / {"cmd": "unview", "id": N}
                                                      serialized messages
    {"cmd": "sim_time", "enabled": B}                 clock for delays

Output on stdout is frames of a little-endian u32 length, a u8 kind and the
payload. Kind 0 is a JSON event; kind 1 a viewer message: u32 view id, u32
count of messages received so far (to measure the rate despite throttling)
and the CDR payload. stdin closing means the parent is gone.
"""

import json
import os
import queue
import struct
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message

EVENT, FRAME = 0, 1
# Rates and delays are measured over this many seconds of messages.
WINDOW = 10.0
# Echo at most this many messages per second; more only floods the pane.
ECHO_MAX_RATE = 20.0
# A new node needs a moment to discover the graph; earlier topic lists
# would be incomplete.
DISCOVERY_TIME = 1.5

_out = sys.stdout.buffer
_out_lock = threading.Lock()


def send(kind, payload):
    try:
        with _out_lock:
            _out.write(struct.pack("<IB", len(payload), kind))
            _out.write(payload)
            _out.flush()
    except (BrokenPipeError, ValueError):
        os._exit(0)


def event(**fields):
    send(EVENT, json.dumps(fields).encode())


def choose_qos(publishers):
    """Match the publishers, like `ros2 topic hz` does.

    Reliable if every publisher is: best effort would match any publisher,
    but a lost fragment drops a whole message, which loses most large raw
    images. Durable (transient local) if every publisher is, so latched
    topics like maps deliver what they published before we subscribed.
    """
    reliable = publishers and all(
        p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in publishers
    )
    if not reliable:
        return qos_profile_sensor_data
    durable = all(
        p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers
    )
    return QoSProfile(
        depth=2,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE,
    )


class Subscription:
    """A subscription whose QoS follows the publishers once they appear."""

    def __init__(self, bridge, topic, type_name, callback, raw):
        self.bridge, self.topic, self.type_name = bridge, topic, type_name
        self.callback, self.raw = callback, raw
        self.handle = None
        self.matched = False
        self.subscribe()

    def subscribe(self):
        node = self.bridge.node
        publishers = node.get_publishers_info_by_topic(self.topic)
        if self.handle is not None:
            node.destroy_subscription(self.handle)
        self.matched = bool(publishers)
        self.handle = node.create_subscription(
            get_message(self.type_name), self.topic, self.callback,
            choose_qos(publishers), raw=self.raw)

    def rematch(self):
        """Resubscribe with matching QoS once publishers show up."""
        if not self.matched and self.bridge.node.get_publishers_info_by_topic(self.topic):
            self.subscribe()

    def destroy(self):
        if self.handle is not None:
            self.bridge.node.destroy_subscription(self.handle)
            self.handle = None


class Watch:
    """Rate and header delay of one topic, from raw messages."""

    def __init__(self, bridge, topic, type_name):
        self.bridge, self.topic = bridge, topic
        fields = get_message(type_name).get_fields_and_field_types()
        first = next(iter(fields.items()), (None, None))
        # Delay needs a header, and reading it from the raw bytes needs it first.
        self.has_header = first == ("header", "std_msgs/Header")
        self.arrivals = deque()
        self.delays = deque()
        self.ever_received = False
        self.subscription = Subscription(bridge, topic, type_name, self.on_message, raw=True)
        if not self.has_header:
            event(type="no_stamp", topic=topic)

    def on_message(self, payload):
        now = time.monotonic()
        self.arrivals.append(now)
        self.ever_received = True
        if self.has_header and len(payload) >= 12:
            # The header stamp leads the message: int32 sec, uint32 nanosec.
            endian = "<" if payload[1] & 1 else ">"
            sec, nanosec = struct.unpack(endian + "iI", payload[4:12])
            ros_now = self.bridge.node.get_clock().now().nanoseconds * 1e-9
            self.delays.append((now, ros_now - (sec + nanosec * 1e-9)))

    def report(self):
        now = time.monotonic()
        while self.arrivals and now - self.arrivals[0] > WINDOW:
            self.arrivals.popleft()
        while self.delays and now - self.delays[0][0] > WINDOW:
            self.delays.popleft()

        if len(self.arrivals) >= 2:
            periods = [b - a for a, b in zip(self.arrivals, list(self.arrivals)[1:])]
            mean = sum(periods) / len(periods)
            event(type="hz", topic=self.topic, hz=1.0 / mean if mean > 0 else 0.0,
                  std_dev=std_dev(periods, mean))
        elif self.ever_received:
            # The topic stopped: say so instead of repeating an old rate.
            event(type="hz", topic=self.topic, hz=0.0, std_dev=0.0)
        elif self.bridge.node.count_publishers(self.topic) == 0:
            event(type="not_published", topic=self.topic)

        if self.delays:
            values = [d for _, d in self.delays]
            mean = sum(values) / len(values)
            event(type="delay", topic=self.topic, delay=mean, std_dev=std_dev(values, mean))


def std_dev(values, mean):
    return (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5


class View:
    """Serialized messages for a viewer, throttled to its frame rate limit."""

    def __init__(self, bridge, view_id, topic, type_name, fps):
        self.view_id = view_id
        self.count = 0
        self.next = 0.0
        self.set_fps(fps)
        self.subscription = Subscription(bridge, topic, type_name, self.on_message, raw=True)

    def set_fps(self, fps):
        self.interval = 1.0 / max(float(fps), 0.1)

    def on_message(self, payload):
        self.count += 1
        now = time.monotonic()
        if now < self.next:
            return
        # Advance on a fixed schedule rather than from `now`, so publisher
        # jitter does not drop every other message of a topic at the limit.
        self.next = max(self.next, now - self.interval) + self.interval
        send(FRAME, struct.pack("<II", self.view_id, self.count & 0xFFFFFFFF) + bytes(payload))


class Echo:
    """Messages of one topic as YAML, without array contents."""

    def __init__(self, bridge, topic, type_name):
        self.last = 0.0
        self.subscription = Subscription(bridge, topic, type_name, self.on_message, raw=False)

    def on_message(self, msg):
        now = time.monotonic()
        if now - self.last < 1.0 / ECHO_MAX_RATE:
            return
        self.last = now
        lines = message_to_yaml(msg, no_arr=True).splitlines() + ["---"]
        event(type="echo", lines=lines)


class Bridge:
    def __init__(self, node):
        self.node = node
        self.watches = {}
        self.views = {}
        self.echo = None
        self.started = time.monotonic()
        self.list_timer = None
        self.commands = queue.Queue()
        # Commands are applied on the executor thread: rclpy entities must
        # not be created or destroyed while it is spinning elsewhere.
        self.wakeup = node.create_guard_condition(self.drain_commands)
        node.create_timer(1.0, self.on_timer)

    def topic_type(self, topic):
        for name, types in self.node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        return None

    def subscriptions(self):
        subs = [w.subscription for w in self.watches.values()]
        subs += [v.subscription for v in self.views.values()]
        if self.echo is not None:
            subs.append(self.echo.subscription)
        return subs

    def drain_commands(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                self.handle(command)
            except Exception as e:  # report, and keep serving the rest
                event(type="error", topic=command.get("topic"), view=command.get("id"),
                      message="%s: %s" % (type(e).__name__, e))

    def handle(self, c):
        cmd = c.get("cmd")
        if cmd == "list":
            self.request_list()
        elif cmd == "watch":
            if c["topic"] not in self.watches:
                type_name = self.topic_type(c["topic"])
                if type_name is None:
                    event(type="not_published", topic=c["topic"])
                else:
                    self.watches[c["topic"]] = Watch(self, c["topic"], type_name)
        elif cmd == "unwatch":
            watch = self.watches.pop(c["topic"], None)
            if watch is not None:
                watch.subscription.destroy()
        elif cmd == "echo":
            self.stop_echo()
            type_name = self.topic_type(c["topic"])
            if type_name is not None:
                self.echo = Echo(self, c["topic"], type_name)
        elif cmd == "unecho":
            self.stop_echo()
        elif cmd == "view":
            self.stop_view(c["id"])
            self.views[c["id"]] = View(self, c["id"], c["topic"], c["type"], c["fps"])
        elif cmd == "fps":
            if c["id"] in self.views:
                self.views[c["id"]].set_fps(c["fps"])
        elif cmd == "unview":
            self.stop_view(c["id"])
        elif cmd == "sim_time":
            self.node.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, bool(c["enabled"]))])
            for watch in self.watches.values():
                watch.delays.clear()

    def stop_echo(self):
        if self.echo is not None:
            self.echo.subscription.destroy()
            self.echo = None

    def stop_view(self, view_id):
        view = self.views.pop(view_id, None)
        if view is not None:
            view.subscription.destroy()

    def request_list(self):
        """List the topics, once discovery had time to settle."""
        remaining = DISCOVERY_TIME - (time.monotonic() - self.started)
        if remaining <= 0:
            self.list_topics()
        elif self.list_timer is None:
            self.list_timer = self.node.create_timer(remaining, self.delayed_list)

    def delayed_list(self):
        self.node.destroy_timer(self.list_timer)
        self.list_timer = None
        self.list_topics()

    def list_topics(self):
        own = {}
        for sub in self.subscriptions():
            own[sub.topic] = own.get(sub.topic, 0) + 1
        topics = []
        for name, types in self.node.get_topic_names_and_types():
            topics.append({
                "name": name,
                "type": types[0] if types else "",
                "publishers": self.node.count_publishers(name),
                # Leave out our own subscriptions, which are not the robot's.
                "subscribers": max(0, self.node.count_subscribers(name) - own.get(name, 0)),
            })
        event(type="topics", topics=topics)

    def on_timer(self):
        for sub in self.subscriptions():
            sub.rematch()
        for watch in list(self.watches.values()):
            watch.report()


def read_commands(bridge):
    for line in sys.stdin:
        line = line.strip()
        if line:
            bridge.commands.put(json.loads(line))
            bridge.wakeup.trigger()
    os._exit(0)


def main():
    rclpy.init(args=[])
    # A leading underscore hides the node from `ros2 node list`.
    node = rclpy.create_node("_ros2_tui_%d" % os.getpid())
    bridge = Bridge(node)
    threading.Thread(target=read_commands, args=(bridge,), daemon=True).start()
    event(type="ready")
    rclpy.spin(node)


try:
    main()
except KeyboardInterrupt:
    pass
