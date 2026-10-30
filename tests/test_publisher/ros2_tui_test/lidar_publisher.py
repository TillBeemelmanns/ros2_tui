#!/usr/bin/env python3
"""
Point cloud publisher for testing the point cloud viewer.
Simulates a 32-beam spinning lidar in a room with parked boxes, a pole and a
moving box, plus a small colored cloud for the rgb coloring.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

SENSOR_HEIGHT = 1.8
ROOM = ((-25.0, 35.0), (-15.0, 15.0))   # x and y extent of the walls
RINGS, COLUMNS = 32, 1024
DEAD_RING = 7                            # a failed channel, for the viewer to show
SCAN_BEAMS = 1081                        # a 270 degree 2D scanner at 0.25 degrees

LIDAR_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
    'offsets': [0, 4, 8, 12, 16],
    'itemsize': 20,
})
RGB_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'rgb'],
    'formats': ['<f4', '<f4', '<f4', '<u4'],
    'offsets': [0, 4, 8, 12],
    'itemsize': 16,
})


def box_hits(d, lo, hi):
    """Ray distances to an axis-aligned box from the origin (inf on a miss)"""
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (np.asarray(lo)[None, :] - 0.0) / d
        t2 = (np.asarray(hi)[None, :] - 0.0) / d
    near = np.nanmax(np.minimum(t1, t2), axis=1)
    far = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (far >= near) & (near > 0)
    return np.where(hit, near, np.inf)


class LidarPublisher(Node):
    """Publishes a raycast lidar scan and a colored cloud"""

    def __init__(self):
        super().__init__('lidar_publisher')
        self.lidar_pub = self.create_publisher(PointCloud2, '/test/lidar/points', 10)
        self.rgb_pub = self.create_publisher(PointCloud2, '/test/points_rgb', 10)
        self.scan = 0

        elevation = np.radians(np.linspace(-25.0, 15.0, RINGS))
        azimuth = np.linspace(-np.pi, np.pi, COLUMNS, endpoint=False)
        el, az = np.meshgrid(elevation, azimuth, indexing='ij')
        self.dirs = np.stack([
            np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)
        ], axis=-1).reshape(-1, 3)
        self.rings = np.repeat(np.arange(RINGS, dtype=np.uint16), COLUMNS)
        self.scan_angles = np.radians(np.linspace(-135.0, 135.0, SCAN_BEAMS))
        # A 2D scanner 30 cm above the ground sees the boxes and the pole.
        self.scan_dirs = np.stack([
            np.cos(self.scan_angles), np.sin(self.scan_angles), np.zeros(SCAN_BEAMS)
        ], axis=-1)
        self.scan_pub = self.create_publisher(LaserScan, '/test/scan', 10)

        self.create_timer(0.1, self.publish_lidar)
        self.create_timer(0.2, self.publish_rgb)
        self.create_timer(1.0 / 15, self.publish_scan)
        self.get_logger().info('Lidar publisher started')

    def raycast(self, t_now, d=None, height=SENSOR_HEIGHT):
        """Distance and intensity along directions `d` from a sensor `height` above ground"""
        if d is None:
            d = self.dirs
        dist = np.full(len(d), np.inf)
        intensity = np.zeros(len(d))

        def take(t, value):
            closer = t < dist
            dist[closer] = t[closer]
            intensity[closer] = value

        def exit_distance(component, lo, hi):
            """Distance to leave [lo, hi] along one axis; inf when parallel"""
            with np.errstate(divide='ignore', invalid='ignore'):
                return np.where(component > 0, hi / component,
                                np.where(component < 0, lo / component, np.inf))

        ground = exit_distance(d[:, 2], -height, np.inf)
        # The room: leave through the nearest wall, if below its top.
        wall = np.minimum(exit_distance(d[:, 0], *ROOM[0]), exit_distance(d[:, 1], *ROOM[1]))
        wall = np.where(wall * d[:, 2] < 3.0 - height, wall, np.inf)
        take(ground, 20.0)
        take(wall, 90.0)

        floor = -height
        boxes = [
            ((6.0, 3.0, floor), (10.5, 5.0, floor + 1.5)),
            ((-12.0, -6.0, floor), (-7.5, -4.0, floor + 1.6)),
            ((15.0, -9.0, floor), (17.0, -7.0, floor + 2.5)),
        ]
        moving_x = 10.0 * math.sin(t_now * 0.4)
        boxes.append(((moving_x - 2.0, -4.0, floor), (moving_x + 2.0, -2.2, floor + 1.4)))
        for lo, hi in boxes:
            take(box_hits(d, lo, hi), 170.0)

        # A thin pole: circle in the ground plane, limited in height.
        px, py, radius = 8.0, -6.0, 0.15
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = -2 * (d[:, 0] * px + d[:, 1] * py)
        c = px ** 2 + py ** 2 - radius ** 2
        disc = b ** 2 - 4 * a * c
        with np.errstate(invalid='ignore', divide='ignore'):
            t_pole = (-b - np.sqrt(disc)) / (2 * a)
        pole = (disc >= 0) & (t_pole > 0) & (t_pole * d[:, 2] < 4.0 - height)
        take(np.where(pole, t_pole, np.inf), 250.0)

        dist = dist + np.random.normal(0, 0.02, len(d))
        dist[dist > 100.0] = np.nan
        if len(d) == len(self.rings):
            dist[self.rings == DEAD_RING] = np.nan
        # Far returns are weaker.
        intensity = intensity * np.clip(8.0 / np.maximum(dist, 1.0), 0.3, 1.0)
        intensity += np.random.normal(0, 3.0, len(d))
        return dist, intensity

    def publish_lidar(self):
        self.scan += 1
        dist, intensity = self.raycast(self.scan * 0.1)
        points = np.zeros(len(dist), dtype=LIDAR_DTYPE)
        for i, axis in enumerate('xyz'):
            points[axis] = self.dirs[:, i] * dist
        points['intensity'] = np.clip(intensity, 0, 255)
        points['ring'] = self.rings

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.height, msg.width = RINGS, COLUMNS
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = LIDAR_DTYPE.itemsize
        msg.row_step = msg.point_step * COLUMNS
        msg.is_dense = False
        msg.data = points.tobytes()
        self.lidar_pub.publish(msg)

    def publish_scan(self):
        dist, intensity = self.raycast(self.scan * 0.1, self.scan_dirs, height=0.3)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = float(self.scan_angles[0])
        msg.angle_max = float(self.scan_angles[-1])
        msg.angle_increment = float(self.scan_angles[1] - self.scan_angles[0])
        msg.scan_time = 1.0 / 15
        msg.time_increment = msg.scan_time / SCAN_BEAMS
        msg.range_min = 0.05
        msg.range_max = 30.0
        # No return reads as +inf (REP 117).
        msg.ranges = np.where(np.isnan(dist), np.inf, dist).astype(np.float32).tolist()
        msg.intensities = np.clip(intensity, 0, 255).astype(np.float32).tolist()
        self.scan_pub.publish(msg)

    def publish_rgb(self):
        """A rainbow torus knot floating in front of the sensor"""
        n = 20000
        t = np.linspace(0, 2 * np.pi, n) + self.scan * 0.02
        tube = np.random.uniform(0, 2 * np.pi, n)
        r = 2.0 + 0.6 * np.cos(3 * t)
        points = np.zeros(n, dtype=RGB_DTYPE)
        points['x'] = 6.0 + r * np.cos(2 * t) + 0.15 * np.cos(tube)
        points['y'] = r * np.sin(2 * t) + 0.15 * np.sin(tube)
        points['z'] = 0.6 * np.sin(3 * t)
        hue = (t / (2 * np.pi)) % 1.0
        rgb = [((0.5 + 0.5 * np.cos(2 * np.pi * (hue + k / 3))) * 255).astype(np.uint32)
               for k in range(3)]
        points['rgb'] = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.height, msg.width = 1, n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # Packed colors are declared FLOAT32 by convention (PCL).
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = RGB_DTYPE.itemsize
        msg.row_step = msg.point_step * n
        msg.is_dense = True
        msg.data = points.tobytes()
        self.rgb_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
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
