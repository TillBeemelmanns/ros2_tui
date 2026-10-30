#!/usr/bin/env python3
"""
Diagnostics publisher for testing the diagnostics viewer.
Publishes /diagnostics as several drivers would: separate partial arrays at
different rates, with levels that change over time and a flaky driver that
keeps dropping out, so its status goes stale and recovers.
"""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR
# The GPS driver reports for GPS_UP seconds, then is silent for GPS_DOWN.
GPS_UP, GPS_DOWN = 20.0, 15.0


def status(level, name, message, hardware_id, **values):
    return DiagnosticStatus(
        level=level, name=name, message=message, hardware_id=hardware_id,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])


class DiagnosticsPublisher(Node):
    """Simulates the diagnostics of a lidar, camera, battery and GPS driver"""

    def __init__(self):
        super().__init__('diagnostics_publisher')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.start = time.time()
        self.create_timer(1.0, self.publish_lidar)
        self.create_timer(2.0, self.publish_camera_and_battery)
        self.create_timer(1.0, self.publish_gps)
        self.get_logger().info('Diagnostics publisher started')

    def publish(self, statuses):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = statuses
        self.pub.publish(msg)

    def publish_lidar(self):
        rate = 10.0 + 0.2 * math.sin(time.time())
        self.publish([
            status(OK, 'lidar_driver: connection', 'Connected', 'lidar-32',
                   address='192.168.1.201', packets_per_s=1810),
            status(OK, 'lidar_driver: scan rate', f'{rate:.1f} Hz', 'lidar-32',
                   expected_hz=10.0, measured_hz=round(rate, 2)),
            status(WARN, 'lidar_driver: channels', 'Ring 7 returns no points', 'lidar-32',
                   dead_rings='7', active_rings=31),
        ])

    def publish_camera_and_battery(self):
        elapsed = time.time() - self.start
        dropping = int(elapsed / 10) % 3 == 2
        battery = 100.0 - (elapsed * 0.5) % 90.0
        self.publish([
            status(ERROR if dropping else OK, 'camera_driver: frames',
                   'Dropping frames' if dropping else 'Streaming', 'cam-front',
                   fps=12.0 if dropping else 30.0, dropped=37 if dropping else 0),
            status(OK, 'camera_driver: exposure', 'Auto exposure', 'cam-front',
                   exposure_us=8000, gain_db=3.5),
            status(WARN if battery < 30 else OK, 'power: battery',
                   'Low battery' if battery < 30 else 'Discharging', 'bms-1',
                   charge_percent=round(battery, 1), voltage=round(22.0 + battery * 0.05, 2),
                   current_a=-4.2),
        ])

    def publish_gps(self):
        if (time.time() - self.start) % (GPS_UP + GPS_DOWN) > GPS_UP:
            return
        self.publish([
            status(OK, 'gps_driver: fix', 'RTK fixed', 'gnss-1', satellites=17, hdop=0.7),
        ])


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
