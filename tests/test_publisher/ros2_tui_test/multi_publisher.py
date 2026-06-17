#!/usr/bin/env python3
"""
ROS2 Multi-Publisher for testing ros2_tui tools
Creates realistic automotive/robotics topics with dummy data
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
import math
import random
from typing import Dict, Any

# Message imports
from std_msgs.msg import String, Bool, Float64, Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, Imu, NavSatFix, TimeReference
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray, Marker
from rcl_interfaces.msg import Log


class MultiPublisher(Node):
    """Multi-topic publisher for comprehensive ROS2 testing"""
    
    def __init__(self):
        super().__init__('multi_publisher')
        
        # Declare parameters
        self.declare_parameters(namespace='', parameters=[
            ('publish_rate', 10.0),
            ('enable_cameras', True),
            ('enable_lidars', True),
            ('enable_imu', True),
            ('enable_debug', False),
            ('camera_width', 1920),
            ('camera_height', 1080),
            ('num_point_clouds', 4),
        ])
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_cameras = self.get_parameter('enable_cameras').value
        self.enable_lidars = self.get_parameter('enable_lidars').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.enable_debug = self.get_parameter('enable_debug').value
        
        # Initialize publishers
        self.all_publishers = {}
        self.setup_all_publishers()
        
        # Counter for varying data
        self.counter = 0
        self.start_time = time.time()

        # Dedicated callback group for the (blocking) publish timers, kept
        # separate from the parameter services so param queries stay responsive.
        # MutuallyExclusive means all timers together occupy at most one executor
        # thread, leaving the remaining threads free to service param requests.
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Add interesting timing variations for different message types
        self.timing_patterns = {
            'camera': {'base_hz': 30, 'jitter_ms': 2, 'sine_period': 10.0, 'sine_amplitude': 5.0},
            'lidar': {'base_hz': 10, 'jitter_ms': 1, 'sine_period': 15.0, 'sine_amplitude': 3.0}, 
            'imu': {'base_hz': 100, 'jitter_ms': 0.5, 'sine_period': 8.0, 'sine_amplitude': 10.0},
            'nav': {'base_hz': 5, 'jitter_ms': 5, 'sine_period': 20.0, 'sine_amplitude': 8.0},
            'planning': {'base_hz': 2, 'jitter_ms': 10, 'sine_period': 25.0, 'sine_amplitude': 15.0},
            'ai_module': {'base_hz': 15, 'jitter_ms': 25, 'sine_period': 12.0, 'sine_amplitude': 30.0},
        }
        
        # Create separate timers for different message types with varying frequencies
        self.setup_variable_timers()
        
        self.get_logger().info(f'Multi-publisher started with {len(self.all_publishers)} topics with variable timing patterns')
    
    def setup_variable_timers(self):
        """Setup separate timers for different message types with jitter and sine wave variations"""
        self.custom_timers = {}

        # Run all timers in a dedicated callback group so their blocking
        # time.sleep() jitter does not starve the node's parameter services.
        # The parameter services stay in the node's default callback group and
        # are serviced on a separate thread by the MultiThreadedExecutor, which
        # keeps `ros2 param dump/list` responsive while topics keep publishing.
        cb_group = self.timer_callback_group

        # Camera topics - high frequency with small jitter
        if self.enable_cameras:
            self.custom_timers['camera'] = self.create_timer(1.0/30, lambda: self.publish_camera_topics(), callback_group=cb_group)

        # LiDAR topics - medium frequency with moderate jitter
        if self.enable_lidars:
            self.custom_timers['lidar'] = self.create_timer(1.0/10, lambda: self.publish_lidar_topics(), callback_group=cb_group)

        # IMU topics - very high frequency with minimal jitter
        if self.enable_imu:
            self.custom_timers['imu'] = self.create_timer(1.0/100, lambda: self.publish_imu_topics(), callback_group=cb_group)

        # Navigation topics - low frequency with high jitter
        self.custom_timers['nav'] = self.create_timer(1.0/5, lambda: self.publish_nav_topics(), callback_group=cb_group)

        # AI module topics - variable frequency with high latency variance
        self.custom_timers['ai_module'] = self.create_timer(1.0/15, lambda: self.publish_ai_module_topics(), callback_group=cb_group)

        # Planning topics - very low frequency with high variance
        self.custom_timers['planning'] = self.create_timer(1.0/2, lambda: self.publish_planning_topics(), callback_group=cb_group)
    
    def get_varied_delay(self, category: str) -> float:
        """Calculate delay with jitter and sine wave patterns for more interesting timing"""
        pattern = self.timing_patterns[category]
        elapsed_time = time.time() - self.start_time
        
        # Base jitter (random component)
        jitter = np.random.normal(0, pattern['jitter_ms'] / 1000.0)
        
        # Sine wave component for periodic variations
        sine_component = (pattern['sine_amplitude'] / 1000.0) * math.sin(
            2 * math.pi * elapsed_time / pattern['sine_period']
        )
        
        # Occasional random spikes (5% chance)
        spike = 0
        if random.random() < 0.05:
            spike = random.uniform(10, 50) / 1000.0  # 10-50ms spike
        
        total_delay = max(0.001, jitter + sine_component + spike)  # Minimum 1ms
        return total_delay
    
    def add_variable_delay(self, category: str):
        """Add variable delay to simulate real-world timing variations"""
        delay = self.get_varied_delay(category)
        time.sleep(delay)
    
    def setup_all_publishers(self):
        """Setup all topic publishers based on realistic automotive/robotics scenarios"""
        
        # Camera topics (removing brand names like 'zed')
        if self.enable_cameras:
            camera_positions = [
                'front_center', 'front_left', 'front_right',
                'rear_center', 'rear_left', 'rear_right'
            ]
            
            for pos in camera_positions:
                # Camera info
                topic_name = f'/drivers/camera/{pos}/left/camera_info'
                self.all_publishers[topic_name] = self.create_publisher(CameraInfo, topic_name, 10)
                
                # Compressed images
                topic_name = f'/drivers/camera/{pos}/left/image_rect_color/compressed'
                self.all_publishers[topic_name] = self.create_publisher(CompressedImage, topic_name, 10)
        
        # LiDAR topics
        if self.enable_lidars:
            lidar_positions = ['fl', 'fr', 'rl', 'rr']  # front-left, front-right, rear-left, rear-right
            
            for pos in lidar_positions:
                topic_name = f'/drivers/lidar_{pos}/points'
                self.all_publishers[topic_name] = self.create_publisher(PointCloud2, topic_name, 10)
            
            # Additional main LiDAR
            self.all_publishers['/drivers/main_lidar/point_cloud'] = self.create_publisher(PointCloud2, '/drivers/main_lidar/point_cloud', 10)
        
        # IMU and navigation
        if self.enable_imu:
            self.all_publishers['/imu/data'] = self.create_publisher(Imu, '/imu/data', 10)
            self.all_publishers['/imu/nav_sat_fix'] = self.create_publisher(NavSatFix, '/imu/nav_sat_fix', 10)
            self.all_publishers['/imu/odometry'] = self.create_publisher(Odometry, '/imu/odometry', 10)
            self.all_publishers['/imu/utc_ref'] = self.create_publisher(TimeReference, '/imu/utc_ref', 10)
            self.all_publishers['/imu/velocity'] = self.create_publisher(TwistStamped, '/imu/velocity', 10)
        
        # Control and actuation
        self.all_publishers['/drivers/dbw/lat_active'] = self.create_publisher(Bool, '/drivers/dbw/lat_active', 10)
        self.all_publishers['/drivers/dbw/lon_active'] = self.create_publisher(Bool, '/drivers/dbw/lon_active', 10)
        
        # Navigation and planning
        self.all_publishers['/clicked_point'] = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.all_publishers['/goal_pose'] = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.all_publishers['/initialpose'] = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Localization
        self.all_publishers['/localization/nav_sat_fix'] = self.create_publisher(NavSatFix, '/localization/nav_sat_fix', 10)
        
        # Computer vision / AI outputs
        self.all_publishers['/ai_module/output/depth_image'] = self.create_publisher(Image, '/ai_module/output/depth_image', 10)
        self.all_publishers['/ai_module/output/depth_image_debug'] = self.create_publisher(Image, '/ai_module/output/depth_image_debug', 10)
        self.all_publishers['/ai_module/output/point_cloud'] = self.create_publisher(PointCloud2, '/ai_module/output/point_cloud', 10)
        
        # Planning and control
        self.all_publishers['/planning/trajectory_optimization/visualization/object_circles'] = self.create_publisher(MarkerArray, '/planning/trajectory_optimization/visualization/object_circles', 10)
        
        # System info
        self.all_publishers['/robot_description'] = self.create_publisher(String, '/robot_description', 10)
        
        # TF transforms
        self.all_publishers['/tf'] = self.create_publisher(TFMessage, '/tf', 10)
        self.all_publishers['/tf_static'] = self.create_publisher(TFMessage, '/tf_static', 10)
        
        # Log messages
        self.all_publishers['/rosout'] = self.create_publisher(Log, '/rosout', 10)
    
    def create_dummy_camera_info(self) -> CameraInfo:
        """Create dummy camera info message"""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.width = self.get_parameter('camera_width').value
        msg.height = self.get_parameter('camera_height').value
        msg.distortion_model = "plumb_bob"
        
        # Dummy camera matrix
        msg.k = [1000.0, 0.0, float(msg.width/2), 0.0, 1000.0, float(msg.height/2), 0.0, 0.0, 1.0]
        msg.d = [0.1, -0.2, 0.0, 0.0, 0.0]  # Distortion coefficients
        
        return msg
    
    def create_dummy_compressed_image(self) -> CompressedImage:
        """Create dummy compressed image message"""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.format = "jpeg"
        
        # Create a small dummy JPEG-like data
        # In reality, this would be actual compressed image data
        dummy_size = 1024 + (self.counter % 512)  # Varying size
        msg.data = bytes([0xFF, 0xD8] + [i % 256 for i in range(dummy_size-2)])  # JPEG header + dummy data
        
        return msg
    
    def create_dummy_pointcloud2(self) -> PointCloud2:
        """Create dummy PointCloud2 message"""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_frame"
        
        # Dummy point cloud data
        msg.height = 1
        msg.width = 1000 + (self.counter % 500)  # Varying number of points
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Simple XYZ point format
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        # Create dummy point data
        points_data = []
        for i in range(msg.width):
            # Create circular pattern with some noise
            angle = (i / msg.width) * 2 * math.pi + (self.counter * 0.01)
            x = math.cos(angle) * (5.0 + np.random.normal(0, 0.1))
            y = math.sin(angle) * (5.0 + np.random.normal(0, 0.1))
            z = np.random.normal(0, 0.5)
            
            # Pack as bytes (simplified - normally would use struct.pack)
            points_data.extend([int(x * 100) % 256, int(y * 100) % 256, int(z * 100) % 256] * 4)
        
        msg.data = bytes(points_data)
        return msg
    
    def create_dummy_imu(self) -> Imu:
        """Create dummy IMU message with realistic motion patterns"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"
        
        elapsed = time.time() - self.start_time
        
        # Simulate vehicle motion with sine waves and noise
        # Linear acceleration with turning and braking patterns
        msg.linear_acceleration.x = (
            2.0 * math.sin(elapsed * 0.1) +           # Acceleration/deceleration cycles
            0.5 * math.sin(elapsed * 0.8) +           # Higher frequency variations
            np.random.normal(0.0, 0.3)                # Sensor noise
        )
        
        msg.linear_acceleration.y = (
            1.5 * math.sin(elapsed * 0.15 + math.pi/4) +  # Lateral acceleration (turning)
            np.random.normal(0.0, 0.2)
        )
        
        # Gravity with vehicle pitch variations
        msg.linear_acceleration.z = (
            9.81 +                                     # Gravity
            0.5 * math.sin(elapsed * 0.05) +          # Slow pitch changes
            np.random.normal(0.0, 0.1)                # Noise
        )
        
        # Angular velocity with more interesting patterns
        msg.angular_velocity.x = (                    # Roll rate
            0.2 * math.sin(elapsed * 0.3) + 
            np.random.normal(0.0, 0.05)
        )
        
        msg.angular_velocity.y = (                    # Pitch rate  
            0.1 * math.sin(elapsed * 0.08) +
            np.random.normal(0.0, 0.03)
        )
        
        msg.angular_velocity.z = (                    # Yaw rate (turning)
            0.8 * math.sin(elapsed * 0.12) +          # Turning maneuvers
            0.2 * math.sin(elapsed * 0.7) +           # Quick corrections
            np.random.normal(0.0, 0.08)
        )
        
        # Dynamic orientation based on motion
        yaw = 0.5 * elapsed + 2.0 * math.sin(elapsed * 0.1)  # Gradual turning
        pitch = 0.1 * math.sin(elapsed * 0.05)               # Small pitch variations
        roll = 0.2 * math.sin(elapsed * 0.3)                 # Banking in turns
        
        # Convert to quaternion (simplified)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)  
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        return msg
    
    def create_dummy_navsatfix(self) -> NavSatFix:
        """Create dummy GPS message with realistic movement patterns"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_frame"
        
        elapsed = time.time() - self.start_time
        
        # Simulate vehicle moving in a figure-8 pattern with GPS noise
        # Base coordinates (Berlin area)
        base_lat = 52.5200
        base_lon = 13.4050
        base_alt = 50.0
        
        # Figure-8 movement pattern
        scale_lat = 0.001   # ~100m movement range in latitude
        scale_lon = 0.0015  # ~100m movement range in longitude  
        
        # Create figure-8 pattern with different frequencies
        lat_movement = (
            scale_lat * math.sin(elapsed * 0.05) * math.cos(elapsed * 0.025) +    # Figure-8
            scale_lat * 0.3 * math.sin(elapsed * 0.2) +                          # Small wobbles
            np.random.normal(0.0, 0.00005)                                       # GPS noise (~5m)
        )
        
        lon_movement = (
            scale_lon * math.sin(elapsed * 0.05) +                               # Figure-8
            scale_lon * 0.2 * math.cos(elapsed * 0.3) +                          # Small variations
            np.random.normal(0.0, 0.00005)                                       # GPS noise
        )
        
        # Altitude with terrain following and noise
        alt_movement = (
            10.0 * math.sin(elapsed * 0.03) +                                    # Hills/valleys
            3.0 * math.sin(elapsed * 0.1) +                                      # Smaller variations  
            np.random.normal(0.0, 2.0)                                           # Altitude noise
        )
        
        msg.latitude = base_lat + lat_movement
        msg.longitude = base_lon + lon_movement
        msg.altitude = base_alt + alt_movement
        
        # Vary GPS fix quality over time (simulate tunnels/buildings)
        fix_quality = math.sin(elapsed * 0.08) + math.cos(elapsed * 0.15)
        if fix_quality > 0.5:
            msg.status.status = 0  # Fix available
        elif fix_quality > -0.5:
            msg.status.status = 1  # DGPS fix
        else:
            msg.status.status = -1  # No fix (simulate GPS loss)
            
        msg.status.service = 1  # GPS
        
        return msg
    
    def publish_camera_topics(self):
        """Publish camera topics with timing jitter and sine wave variations"""
        self.add_variable_delay('camera')
        self.counter += 1
        
        for topic_name, publisher in self.all_publishers.items():
            if 'camera' in topic_name:
                try:
                    if 'camera_info' in topic_name:
                        publisher.publish(self.create_dummy_camera_info())
                    elif 'image_rect_color/compressed' in topic_name:
                        publisher.publish(self.create_dummy_compressed_image())
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish camera topic {topic_name}: {e}")
    
    def publish_lidar_topics(self):
        """Publish LiDAR topics with moderate timing variations"""
        self.add_variable_delay('lidar')
        
        for topic_name, publisher in self.all_publishers.items():
            if 'lidar' in topic_name or 'points' in topic_name or 'point_cloud' in topic_name:
                try:
                    publisher.publish(self.create_dummy_pointcloud2())
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish LiDAR topic {topic_name}: {e}")
    
    def publish_imu_topics(self):
        """Publish IMU topics with minimal jitter (high frequency sensor)"""
        self.add_variable_delay('imu')
        
        for topic_name, publisher in self.all_publishers.items():
            if 'imu' in topic_name:
                try:
                    if topic_name == '/imu/data':
                        publisher.publish(self.create_dummy_imu())
                    elif 'nav_sat_fix' in topic_name:
                        publisher.publish(self.create_dummy_navsatfix())
                    elif 'velocity' in topic_name:
                        msg = TwistStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "base_link"
                        # Add sine wave patterns to velocity for interesting plots
                        elapsed = time.time() - self.start_time
                        msg.twist.linear.x = 5.0 + 2.0 * math.sin(elapsed * 0.5) + np.random.normal(0, 0.1)
                        msg.twist.linear.y = 1.0 * math.cos(elapsed * 0.3) + np.random.normal(0, 0.05)
                        msg.twist.angular.z = 0.5 * math.sin(elapsed * 0.8) + np.random.normal(0, 0.02)
                        publisher.publish(msg)
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish IMU topic {topic_name}: {e}")
    
    def publish_nav_topics(self):
        """Publish navigation topics with high jitter and variance"""
        self.add_variable_delay('nav')
        
        for topic_name, publisher in self.all_publishers.items():
            if any(x in topic_name for x in ['nav_sat_fix', 'localization', 'clicked_point', 'goal_pose', 'initialpose']):
                try:
                    if 'nav_sat_fix' in topic_name:
                        publisher.publish(self.create_dummy_navsatfix())
                    # Add other navigation message types as needed
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish nav topic {topic_name}: {e}")
    
    def publish_ai_module_topics(self):
        """Publish AI module topics with realistic inference latency and variable processing times"""
        # AI inference has highly variable timing - simulate GPU processing delays
        elapsed = time.time() - self.start_time
        
        # Create more complex delay pattern for AI inference
        base_delay = 10.0  # 10ms base processing time
        
        # Simulate GPU load variations with sine waves
        gpu_load_factor = 1.0 + 0.8 * math.sin(elapsed * 0.1) + 0.3 * math.sin(elapsed * 0.4)
        
        # Occasional processing spikes (thermal throttling, memory allocation, etc.)
        processing_spike = 0
        if random.random() < 0.08:  # 8% chance of processing spike
            processing_spike = random.uniform(50, 150)  # 50-150ms spike
            
        # Model complexity variation (different inference paths)
        complexity_factor = 1.0 + 0.5 * math.sin(elapsed * 0.05)  # Slow variation
        
        # Batch processing effects (occasional faster processing when batching)
        batch_speedup = 0
        if random.random() < 0.15:  # 15% chance of batch processing
            batch_speedup = -random.uniform(10, 25)  # 10-25ms speedup
        
        total_ai_delay = base_delay * gpu_load_factor * complexity_factor + processing_spike + batch_speedup
        
        # Add the calculated delay
        self.add_variable_delay('ai_module')
        additional_delay = max(0.001, total_ai_delay / 1000.0)
        time.sleep(additional_delay)
        
        for topic_name, publisher in self.all_publishers.items():
            if 'ai_module' in topic_name:
                try:
                    if 'depth_image' in topic_name:
                        msg = Image()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "ai_camera_frame"
                        msg.width = 640
                        msg.height = 480
                        msg.encoding = "32FC1"
                        msg.step = msg.width * 4
                        
                        # Create more interesting depth data with patterns that change over time
                        depth_data = []
                        for y in range(msg.height):
                            for x in range(msg.width):
                                # Create depth gradient with time-varying patterns
                                base_depth = 5.0 + 3.0 * math.sin((x + y + elapsed * 10) * 0.01)
                                noise = np.random.normal(0, 0.2)
                                depth_val = max(0.1, base_depth + noise)
                                # Convert to bytes (simplified depth representation)
                                depth_bytes = int(depth_val * 100) % 65536
                                depth_data.extend([
                                    depth_bytes & 0xFF, 
                                    (depth_bytes >> 8) & 0xFF,
                                    0, 0  # Pad to 4 bytes for 32FC1
                                ])
                        
                        msg.data = bytes(depth_data)
                        publisher.publish(msg)
                        
                    elif 'point_cloud' in topic_name:
                        # Publish AI-processed point cloud with variable density
                        msg = self.create_dummy_pointcloud2()
                        # Modify the point cloud to simulate AI processing effects
                        msg.header.frame_id = "ai_lidar_frame"
                        # Variable point density based on processing complexity
                        density_factor = 0.5 + 0.5 * math.sin(elapsed * 0.2)
                        msg.width = int(msg.width * density_factor)
                        msg.row_step = msg.point_step * msg.width
                        publisher.publish(msg)
                        
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish AI module topic {topic_name}: {e}")
    
    def publish_planning_topics(self):
        """Publish planning topics with very high variance and occasional bursts"""
        self.add_variable_delay('planning')
        
        for topic_name, publisher in self.all_publishers.items():
            if any(x in topic_name for x in ['planning', 'dbw', 'robot_description', 'rosout', 'tf']):
                try:
                    if topic_name in ['/drivers/dbw/lat_active', '/drivers/dbw/lon_active']:
                        msg = Bool()
                        # Create interesting on/off patterns with sine waves
                        elapsed = time.time() - self.start_time
                        sine_val = math.sin(elapsed * 0.2) + 0.3 * math.sin(elapsed * 0.7)
                        msg.data = sine_val > 0 or (self.counter % 100) > 80
                        publisher.publish(msg)
                    elif topic_name == '/robot_description':
                        msg = String()
                        msg.data = "<?xml version='1.0'?><robot name='test_robot'><!-- Dummy URDF --></robot>"
                        publisher.publish(msg)
                    elif topic_name == '/rosout':
                        msg = Log()
                        msg.stamp = self.get_clock().now().to_msg()
                        msg.level = Log.INFO
                        msg.name = "multi_publisher"
                        msg.msg = f"Variable timing log message #{self.counter}"
                        msg.file = "multi_publisher.py"
                        msg.function = "publish_planning_topics"
                        msg.line = 300
                        publisher.publish(msg)
                except Exception as e:
                    if self.enable_debug:
                        self.get_logger().warn(f"Failed to publish planning topic {topic_name}: {e}")
        
        if self.counter % 200 == 0:
            self.get_logger().info(f"Published {self.counter} variable-timing cycles")


def main(args=None):
    rclpy.init(args=args)

    node = MultiPublisher()

    # MultiThreadedExecutor lets the parameter services (default callback group)
    # run on a separate thread from the blocking publish timers, so tools like
    # ros2_tui can dump/list this node's parameters without hanging.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Multi-publisher stopped by user")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()