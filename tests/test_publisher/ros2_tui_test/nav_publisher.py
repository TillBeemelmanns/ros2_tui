#!/usr/bin/env python3
"""
Navigation publisher for testing the map, GNSS and IMU viewers.
Publishes a latched map of the lidar test room, a local costmap, a GNSS fix
driving a figure eight with changing fix quality, and a 100 Hz IMU whose
orientation agrees with the gravity it measures.
"""

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

# The room and boxes of lidar_publisher, in meters
ROOM = ((-25.0, 35.0), (-15.0, 15.0))
BOXES = [((6.0, 3.0), (10.5, 5.0)), ((-12.0, -6.0), (-7.5, -4.0)), ((15.0, -9.0), (17.0, -7.0))]
POLE = (8.0, -6.0, 0.15)
RESOLUTION = 0.05
GRAVITY = 9.81
BASE_LAT, BASE_LON = 50.7779, 6.0778


def moving_box(t):
    x = 10.0 * math.sin(t * 0.4)
    return (x - 2.0, -4.0), (x + 2.0, -2.2)


def box_distance(xs, ys, lo, hi):
    """Distance from each point to an axis-aligned box (0 inside)"""
    dx = np.maximum(np.maximum(lo[0] - xs, 0), xs - hi[0])
    dy = np.maximum(np.maximum(lo[1] - ys, 0), ys - hi[1])
    return np.hypot(dx, dy)


class NavPublisher(Node):
    """Publishes a map, a costmap, a GNSS fix and an IMU"""

    def __init__(self):
        super().__init__('nav_publisher')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/test/map', latched)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/test/costmap', 10)
        self.fix_pub = self.create_publisher(NavSatFix, '/test/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/test/imu', 10)
        self.start = time.time()

        # Published once: the viewer must get it through transient local.
        self.map_pub.publish(self.make_map())
        self.create_timer(0.5, self.publish_costmap)
        self.create_timer(0.2, self.publish_fix)
        self.create_timer(0.01, self.publish_imu)
        self.get_logger().info('Nav publisher started')

    def grid_msg(self, values, origin):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = RESOLUTION
        msg.info.height, msg.info.width = values.shape
        msg.info.origin.position.x, msg.info.origin.position.y = origin
        msg.info.origin.orientation.w = 1.0
        msg.data = values.astype(np.int8).ravel().tolist()
        return msg

    def make_map(self):
        origin = (ROOM[0][0] - 2.0, ROOM[1][0] - 2.0)
        width = int((ROOM[0][1] - ROOM[0][0] + 4.0) / RESOLUTION)
        height = int((ROOM[1][1] - ROOM[1][0] + 4.0) / RESOLUTION)
        ys, xs = np.mgrid[0:height, 0:width]
        xs = origin[0] + (xs + 0.5) * RESOLUTION
        ys = origin[1] + (ys + 0.5) * RESOLUTION

        inside = ((xs > ROOM[0][0]) & (xs < ROOM[0][1]) & (ys > ROOM[1][0]) & (ys < ROOM[1][1]))
        grid = np.where(inside, 0, -1)
        walls = box_distance(xs, ys, (ROOM[0][0], ROOM[1][0]), (ROOM[0][1], ROOM[1][1]))
        edge = np.minimum.reduce([np.abs(xs - ROOM[0][0]), np.abs(xs - ROOM[0][1]),
                                  np.abs(ys - ROOM[1][0]), np.abs(ys - ROOM[1][1])])
        grid[(edge < 0.15) & (walls < 0.15)] = 100
        for lo, hi in BOXES:
            grid[box_distance(xs, ys, lo, hi) == 0] = 100
        px, py, r = POLE
        grid[np.hypot(xs - px, ys - py) < r + RESOLUTION] = 100
        # An unexplored corner and a fuzzy patch, as SLAM leaves them.
        grid[(xs > 25) & (ys > 8) & inside] = -1
        grid[(np.hypot(xs + 18, ys - 9) < 2.0) & inside] = 50
        return self.grid_msg(grid, origin)

    def publish_costmap(self):
        t = time.time() - self.start
        size = int(10.0 / RESOLUTION)
        origin = (-5.0, -5.0)
        ys, xs = np.mgrid[0:size, 0:size]
        xs = origin[0] + (xs + 0.5) * RESOLUTION
        ys = origin[1] + (ys + 0.5) * RESOLUTION
        distance = np.full(xs.shape, np.inf)
        for lo, hi in BOXES + [moving_box(t)]:
            distance = np.minimum(distance, box_distance(xs, ys, lo, hi))
        # Lethal on obstacles, inflated cost decaying over 1 m.
        cost = np.where(distance == 0, 100, np.clip(98 * np.exp(-3.0 * distance), 0, 98))
        cost[distance > 1.0] = 0
        self.costmap_pub.publish(self.grid_msg(cost.astype(np.int16), origin))

    def publish_fix(self):
        t = time.time() - self.start
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_GALILEO)
        phase = t % 40.0
        if phase > 37.0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = math.nan
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.fix_pub.publish(msg)
            return
        rtk = phase < 28.0
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX if rtk else NavSatStatus.STATUS_FIX
        sigma = 0.02 if rtk else 1.5
        # A figure eight, 40 m across, about 1.3 m/s.
        a = t * 0.06
        east = 20.0 * math.sin(a) + np.random.normal(0, sigma)
        north = 10.0 * math.sin(2 * a) + np.random.normal(0, sigma)
        msg.latitude = BASE_LAT + math.degrees(north / 6371000.0)
        msg.longitude = BASE_LON + math.degrees(
            east / (6371000.0 * math.cos(math.radians(BASE_LAT))))
        msg.altitude = 182.0 + np.random.normal(0, sigma * 2)
        msg.position_covariance = [sigma ** 2, 0.0, 0.0, 0.0, sigma ** 2, 0.0,
                                   0.0, 0.0, (2 * sigma) ** 2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(msg)

    def publish_imu(self):
        t = time.time() - self.start
        roll = math.radians(4.0) * math.sin(t * 0.7)
        pitch = math.radians(3.0) * math.sin(t * 0.45)
        yaw = (t * 0.2) % (2 * math.pi) - math.pi
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-3]

        noise = np.random.normal(0, 0.002, 3)
        msg.angular_velocity.x = math.radians(4.0) * 0.7 * math.cos(t * 0.7) + noise[0]
        msg.angular_velocity.y = math.radians(3.0) * 0.45 * math.cos(t * 0.45) + noise[1]
        msg.angular_velocity.z = 0.2 + noise[2]
        msg.angular_velocity_covariance = [4e-6, 0.0, 0.0, 0.0, 4e-6, 0.0, 0.0, 0.0, 4e-6]

        # Gravity seen in the tilted body frame, plus engine vibration.
        vibration = 0.15 * math.sin(t * 2 * math.pi * 23)
        noise = np.random.normal(0, 0.03, 3)
        msg.linear_acceleration.x = -math.sin(pitch) * GRAVITY + noise[0]
        msg.linear_acceleration.y = math.sin(roll) * math.cos(pitch) * GRAVITY + noise[1]
        msg.linear_acceleration.z = (math.cos(roll) * math.cos(pitch) * GRAVITY
                                     + vibration + noise[2])
        msg.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavPublisher()
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
