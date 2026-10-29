"""Stream one topic to ros2_tui as serialized messages.

Run by the `topics` binary as `python3 -c <this file> TOPIC TYPE MAX_FPS`.
Each message is written to stdout as a little-endian u32 payload length, a
u32 count of messages received so far (to measure the topic rate despite
throttling) and the CDR payload. A line with a new MAX_FPS on stdin changes
the throttle; stdin closing means the parent is gone.
"""

import os
import struct
import sys
import threading
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message


def choose_qos(node, topic):
    """Reliable if every publisher is, like `ros2 topic hz` does.

    Best effort would match any publisher, but a lost fragment drops a whole
    message, which loses most large raw images even on a good link.
    """
    deadline = time.monotonic() + 2.0
    publishers = []
    while not publishers and time.monotonic() < deadline:
        publishers = node.get_publishers_info_by_topic(topic)
        if not publishers:
            time.sleep(0.1)
    reliable = publishers and all(
        p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in publishers
    )
    if not reliable:
        return qos_profile_sensor_data
    return QoSProfile(
        depth=2,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def main():
    topic, type_name, max_fps = sys.argv[1], sys.argv[2], float(sys.argv[3])
    out = sys.stdout.buffer
    state = {"min_interval": 1.0 / max_fps, "next": 0.0, "count": 0}

    def read_commands():
        for line in sys.stdin:
            try:
                state["min_interval"] = 1.0 / max(float(line), 0.1)
            except ValueError:
                pass
        os._exit(0)

    def on_message(payload):
        state["count"] += 1
        now = time.monotonic()
        if now < state["next"]:
            return
        # Advance on a fixed schedule rather than from `now`, so publisher
        # jitter does not drop every other message of a topic at the limit.
        interval = state["min_interval"]
        state["next"] = max(state["next"], now - interval) + interval
        try:
            out.write(struct.pack("<II", len(payload), state["count"] & 0xFFFFFFFF))
            out.write(payload)
            out.flush()
        except (BrokenPipeError, ValueError):
            os._exit(0)

    rclpy.init(args=[])
    # A leading underscore hides the node from `ros2 node list`.
    node = rclpy.create_node("_ros2_tui_view_%d" % os.getpid())
    node.create_subscription(
        get_message(type_name), topic, on_message, choose_qos(node, topic), raw=True
    )
    threading.Thread(target=read_commands, daemon=True).start()
    rclpy.spin(node)


try:
    main()
except KeyboardInterrupt:
    pass
