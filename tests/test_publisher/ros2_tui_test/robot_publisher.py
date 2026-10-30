#!/usr/bin/env python3
"""
Robot publisher for testing the TF, odometry, path, joint state and battery
viewers. Simulates a robot driving a circle: its TF tree (static sensor
mounts, odometry, wheels), odometry, the planned path ahead, the joint
states of its wheels and arm, and a slowly draining battery.

Two TF problems come and go, for the viewer to catch: the right wheel stops
updating for 10 s every 40 s, and for 5 s of every 40 a second node also
publishes base_link, with map as its parent.
"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, JointState
from tf2_msgs.msg import TFMessage

RADIUS = 8.0
SPEED = 1.0
WHEEL_RADIUS = 0.15
TRACK = 0.6
CYCLE = 40.0
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist']
CELLS = 6


def quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def set_rotation(target, q):
    target.x, target.y, target.z, target.w = q


class RobotPublisher(Node):
    """Publishes TF, odometry, a path, joint states and a battery"""

    def __init__(self):
        super().__init__('robot_publisher')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.static_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.odom_pub = self.create_publisher(Odometry, '/test/odom', 10)
        self.path_pub = self.create_publisher(Path, '/test/plan', 10)
        self.joint_pub = self.create_publisher(JointState, '/test/joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/test/battery', 10)
        self.start = time.time()

        # Published once, all in one message: the latched topic keeps only
        # the last message.
        self.static_pub.publish(TFMessage(transforms=self.static_transforms()))
        self.create_timer(0.02, self.publish_motion)
        self.create_timer(0.1, self.publish_map_to_odom)
        self.create_timer(0.1, self.publish_gripper)
        self.create_timer(0.5, self.publish_path)
        self.create_timer(1.0, self.publish_battery)
        self.get_logger().info('Robot publisher started')

    def transform(self, parent, child, xyz, q, stamp=None):
        t = TransformStamped()
        t.header.stamp = stamp or self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        set_rotation(t.transform.rotation, q)
        return t

    def static_transforms(self):
        return [
            self.transform('base_link', 'lidar_link', (0.3, 0.0, 0.45), quaternion()),
            self.transform('base_link', 'imu_link', (0.0, 0.0, 0.12), quaternion()),
            self.transform('base_link', 'gps_link', (-0.2, 0.0, 0.6), quaternion()),
            self.transform('base_link', 'camera_link', (0.4, 0.0, 0.35),
                           quaternion(pitch=math.radians(10))),
            self.transform('camera_link', 'camera_optical_frame', (0.0, 0.0, 0.0),
                           quaternion(-math.pi / 2, 0.0, -math.pi / 2)),
            self.transform('base_link', 'arm_base_link', (0.1, 0.0, 0.3), quaternion()),
        ]

    def elapsed(self):
        return time.time() - self.start

    def pose(self, t):
        """Position and heading on the circle, counter-clockwise."""
        angle = SPEED / RADIUS * t
        return (RADIUS * math.sin(angle), RADIUS * (1 - math.cos(angle)), angle)

    def publish_motion(self):
        t = self.elapsed()
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.pose(t)
        transforms = [self.transform('odom', 'base_link', (x, y, 0.0),
                                     quaternion(yaw=yaw), stamp)]
        # Wheels turn at the speed of their side of the robot.
        omega = SPEED / RADIUS
        left = (SPEED - omega * TRACK / 2) * t / WHEEL_RADIUS
        right = (SPEED + omega * TRACK / 2) * t / WHEEL_RADIUS
        transforms.append(self.transform('base_link', 'wheel_left_link', (0.0, TRACK / 2, 0.0),
                                         quaternion(pitch=left % (2 * math.pi)), stamp))
        if not 15.0 <= t % CYCLE < 25.0:
            transforms.append(self.transform(
                'base_link', 'wheel_right_link', (0.0, -TRACK / 2, 0.0),
                quaternion(pitch=right % (2 * math.pi)), stamp))
        self.tf_pub.publish(TFMessage(transforms=transforms))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        set_rotation(odom.pose.pose.orientation, quaternion(yaw=yaw))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01 + 0.0005 * (t % CYCLE)
        cov[35] = 0.002
        odom.pose.covariance = cov
        odom.twist.twist.linear.x = SPEED + random.gauss(0, 0.02)
        odom.twist.twist.angular.z = omega + random.gauss(0, 0.005)
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['wheel_left_joint', 'wheel_right_joint'] + ARM_JOINTS
        arm = [0.8 * math.sin(t * 0.3), -0.6 + 0.4 * math.sin(t * 0.5),
               1.2 + 0.5 * math.sin(t * 0.7), 0.3 * math.sin(t * 1.1)]
        arm_velocity = [0.24 * math.cos(t * 0.3), 0.2 * math.cos(t * 0.5),
                        0.35 * math.cos(t * 0.7), 0.33 * math.cos(t * 1.1)]
        joints.position = [left, right] + arm
        joints.velocity = [(SPEED - omega * TRACK / 2) / WHEEL_RADIUS,
                           (SPEED + omega * TRACK / 2) / WHEEL_RADIUS] + arm_velocity
        joints.effort = [2.1 + random.gauss(0, 0.1), 2.3 + random.gauss(0, 0.1),
                         4.0, 9.5 + 2 * math.sin(t * 0.5), 6.0, 0.8]
        self.joint_pub.publish(joints)

    def publish_map_to_odom(self):
        t = self.elapsed()
        stamp = self.get_clock().now().to_msg()
        # Localization corrects a slow odometry drift.
        drift = (0.15 * math.sin(t / 30.0), 0.1 * math.sin(t / 45.0), 0.0)
        transforms = [self.transform('map', 'odom', drift,
                                     quaternion(yaw=0.01 * math.sin(t / 60.0)), stamp)]
        if 30.0 <= t % CYCLE < 35.0:
            # A misconfigured second localization also publishes base_link.
            x, y, yaw = self.pose(t)
            transforms.append(self.transform('map', 'base_link', (x, y, 0.0),
                                             quaternion(yaw=yaw), stamp))
        self.tf_pub.publish(TFMessage(transforms=transforms))

    def publish_gripper(self):
        # A second source, publishing only its own joint.
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = ['gripper_finger']
        joints.position = [0.02 + 0.02 * math.sin(self.elapsed() * 0.4)]
        self.joint_pub.publish(joints)

    def publish_path(self):
        t = self.elapsed()
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        # The next 15 m of the circle, a pose every 25 cm.
        for i in range(61):
            x, y, yaw = self.pose(t + i * 0.25 / SPEED)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            set_rotation(pose.pose.orientation, quaternion(yaw=yaw))
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_battery(self):
        t = self.elapsed()
        # Drains 1 % a minute from 80 %, then starts over.
        fraction = 0.8 - (t / 6000.0) % 0.7
        cell = 3.5 + 0.7 * fraction
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cell_voltage = [cell + random.gauss(0, 0.003) for _ in range(CELLS)]
        msg.cell_voltage[4] -= 0.08  # a weak cell
        msg.voltage = sum(msg.cell_voltage)
        msg.current = -6.5 + random.gauss(0, 0.4) - 1.5 * math.sin(t / 10.0) ** 2
        msg.capacity = 20.0
        msg.design_capacity = 22.0
        msg.charge = fraction * msg.capacity
        msg.percentage = fraction
        msg.temperature = 31.0 + 2.0 * math.sin(t / 120.0)
        msg.cell_temperature = [msg.temperature + random.gauss(0, 0.3) for _ in range(CELLS)]
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.location = 'base'
        msg.serial_number = 'TEST-6S-0042'
        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPublisher()
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
