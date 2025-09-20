#!/usr/bin/env python3
"""
ROS2 Parameter Setter for testing ros2_tui parameter management
Creates realistic parameters with various types for debugging
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time
import random


class ParamSetter(Node):
    """Node that manages various types of parameters for testing"""
    
    def __init__(self):
        super().__init__('param_setter')
        
        # Declare comprehensive set of parameters for testing
        self.declare_test_parameters()
        
        # Timer to periodically update some parameters
        self.timer = self.create_timer(5.0, self.update_dynamic_parameters)
        self.counter = 0
        
        self.get_logger().info('Parameter setter node started with comprehensive test parameters')
    
    def declare_test_parameters(self):
        """Declare various types of parameters for comprehensive testing"""
        
        # === CAMERA PARAMETERS ===
        self.declare_parameters(namespace='camera', parameters=[
            ('width', 1920),
            ('height', 1080),
            ('fps', 30.0),
            ('exposure_time', 33.33),
            ('gain', 1.0),
            ('brightness', 0.5),
            ('contrast', 1.0),
            ('saturation', 1.0),
            ('gamma', 2.2),
            ('white_balance_auto', True),
            ('focus_mode', 'auto'),
            ('encoding', 'bgr8'),
            ('compressed', True),
            ('quality', 95),
        ])
        
        # === AI/ML MODULE PARAMETERS ===
        self.declare_parameters(namespace='ai_module', parameters=[
            ('model_path', '/opt/models/depth_estimation.onnx'),
            ('input_width', 640),
            ('input_height', 480),
            ('batch_size', 1),
            ('inference_device', 'cuda'),
            ('confidence_threshold', 0.85),
            ('nms_threshold', 0.45),
            ('max_detections', 100),
            ('enable_tracking', True),
            ('tracking_algorithm', 'sort'),
            ('debug_output', False),
            ('colormap_min_depth', 0.1),
            ('colormap_max_depth', 50.0),
            ('depth_scale_factor', 1000.0),
            ('enable_filtering', True),
            ('bilateral_d', 9),
            ('bilateral_sigma_color', 75.0),
            ('bilateral_sigma_space', 75.0),
        ])
        
        # === LIDAR PARAMETERS ===
        self.declare_parameters(namespace='lidar', parameters=[
            ('range_min', 0.1),
            ('range_max', 100.0),
            ('angle_min', -3.14159),
            ('angle_max', 3.14159),
            ('angle_increment', 0.005),
            ('scan_time', 0.1),
            ('time_increment', 0.0001),
            ('intensity_threshold', 100),
            ('filter_outliers', True),
            ('voxel_size', 0.05),
            ('enable_ground_removal', True),
            ('ground_threshold', 0.2),
            ('cluster_tolerance', 0.5),
            ('min_cluster_size', 10),
            ('max_cluster_size', 10000),
        ])
        
        # === IMU PARAMETERS ===
        self.declare_parameters(namespace='imu', parameters=[
            ('gyro_scale', 1.0),
            ('accel_scale', 1.0),
            ('mag_scale', 1.0),
            ('bias_x', 0.001),
            ('bias_y', -0.002),
            ('bias_z', 0.0005),
            ('noise_variance', 0.01),
            ('sample_rate', 100.0),
            ('enable_calibration', False),
            ('filter_type', 'kalman'),
            ('gravity_compensation', True),
        ])
        
        # === NAVIGATION PARAMETERS ===
        self.declare_parameters(namespace='navigation', parameters=[
            ('max_vel_x', 2.0),
            ('max_vel_y', 0.0),
            ('max_vel_theta', 1.57),
            ('min_vel_x', 0.1),
            ('min_vel_theta', 0.1),
            ('acc_lim_x', 1.0),
            ('acc_lim_theta', 2.0),
            ('goal_tolerance', 0.2),
            ('yaw_goal_tolerance', 0.1),
            ('planner_frequency', 10.0),
            ('controller_frequency', 20.0),
            ('recovery_behavior_enabled', True),
            ('clearing_rotation_allowed', True),
            ('max_planning_retries', 3),
            ('planner_patience', 5.0),
        ])
        
        # === CONTROL PARAMETERS ===
        self.declare_parameters(namespace='control', parameters=[
            ('lateral_active', False),
            ('longitudinal_active', False),
            ('max_steering_angle', 0.52),  # ~30 degrees
            ('max_acceleration', 2.0),
            ('max_deceleration', -4.0),
            ('steering_ratio', 16.0),
            ('wheelbase', 2.8),
            ('pid_kp', 0.8),
            ('pid_ki', 0.1),
            ('pid_kd', 0.05),
            ('control_frequency', 50.0),
            ('safety_timeout', 0.5),
            ('emergency_stop_enabled', True),
        ])
        
        # === SYSTEM PARAMETERS ===
        self.declare_parameters(namespace='system', parameters=[
            ('node_name', 'test_system'),
            ('log_level', 'INFO'),
            ('debug_mode', False),
            ('config_file', '/opt/config/default.yaml'),
            ('data_directory', '/tmp/ros2_data'),
            ('max_memory_usage', 1024),  # MB
            ('cpu_limit_percent', 80.0),
            ('enable_diagnostics', True),
            ('heartbeat_rate', 1.0),
            ('version', '1.2.3'),
            ('build_timestamp', '2024-01-15T10:30:45Z'),
        ])
        
        # === SENSOR FUSION PARAMETERS ===
        self.declare_parameters(namespace='fusion', parameters=[
            ('enable_sensor_fusion', True),
            ('imu_weight', 0.7),
            ('lidar_weight', 0.9),
            ('camera_weight', 0.5),
            ('gps_weight', 0.8),
            ('fusion_frequency', 30.0),
            ('timeout_threshold', 0.2),
            ('outlier_rejection', True),
            ('kalman_q', 0.01),  # Process noise
            ('kalman_r', 0.1),   # Measurement noise
            ('prediction_horizon', 0.5),
        ])
        
        # === ARRAY PARAMETERS ===
        self.declare_parameters(namespace='arrays', parameters=[
            ('int_array', [1, 2, 3, 4, 5]),
            ('double_array', [1.1, 2.2, 3.3, 4.4, 5.5]),
            ('string_array', ['camera', 'lidar', 'imu', 'gps']),
            ('bool_array', [True, False, True, False]),
            ('transform_matrix', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            ('calibration_points', [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0]),
        ])
        
        # === EDGE CASE PARAMETERS ===
        self.declare_parameters(namespace='edge_cases', parameters=[
            ('zero_int', 0),
            ('zero_double', 0.0),
            ('negative_int', -42),
            ('negative_double', -3.14159),
            ('large_int', 2147483647),
            ('small_double', 0.000001),
            ('large_double', 1e6),
            ('empty_string', ''),
            ('special_chars', '!@#$%^&*()'),
            ('unicode_string', 'こんにちは世界'),  # "Hello World" in Japanese
            ('path_with_spaces', '/path/with spaces/file.txt'),
            ('scientific_notation', 1.23e-4),
        ])
    
    def update_dynamic_parameters(self):
        """Update some parameters dynamically to test real-time parameter changes"""
        self.counter += 1
        
        try:
            # Update some parameters with dynamic values
            dynamic_updates = [
                ('ai_module.confidence_threshold', 0.75 + 0.1 * (self.counter % 5)),
                ('camera.brightness', 0.3 + 0.4 * (self.counter % 10) / 10.0),
                ('navigation.max_vel_x', 1.0 + (self.counter % 3)),
                ('system.cpu_limit_percent', 60.0 + 20 * (self.counter % 4)),
                ('lidar.range_max', 80.0 + 20 * (self.counter % 3)),
                ('control.pid_kp', 0.5 + 0.3 * (self.counter % 6) / 6.0),
            ]
            
            for param_name, value in dynamic_updates:
                self.set_parameters([Parameter(param_name, value=value)])
            
            # Occasionally toggle boolean parameters
            if self.counter % 10 == 0:
                bool_toggles = [
                    ('camera.compressed', random.choice([True, False])),
                    ('ai_module.debug_output', random.choice([True, False])),
                    ('system.debug_mode', random.choice([True, False])),
                    ('control.emergency_stop_enabled', random.choice([True, False])),
                ]
                
                for param_name, value in bool_toggles:
                    self.set_parameters([Parameter(param_name, value=value)])
            
            # Update array parameters occasionally
            if self.counter % 15 == 0:
                new_int_array = [random.randint(1, 100) for _ in range(5)]
                new_double_array = [round(random.uniform(0.1, 10.0), 2) for _ in range(5)]
                
                self.set_parameters([
                    Parameter('arrays.int_array', value=new_int_array),
                    Parameter('arrays.double_array', value=new_double_array),
                ])
            
            if self.counter % 20 == 0:
                self.get_logger().info(f"Updated dynamic parameters (cycle #{self.counter})")
                
        except Exception as e:
            self.get_logger().error(f"Failed to update dynamic parameters: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = ParamSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Parameter setter stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()