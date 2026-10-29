#!/usr/bin/env python3
"""
Launch file for ROS2 TUI test infrastructure
Starts both the multi-publisher and parameter setter nodes for comprehensive testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz for the multi-publisher'
    )
    
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='Enable camera topic publishing'
    )
    
    enable_lidars_arg = DeclareLaunchArgument(
        'enable_lidars',
        default_value='true',
        description='Enable LiDAR topic publishing'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU topic publishing'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug output'
    )

    # Multi-publisher node
    multi_publisher_node = Node(
        package='ros2_tui_test',
        executable='multi_publisher',
        name='multi_publisher',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_cameras': LaunchConfiguration('enable_cameras'),
            'enable_lidars': LaunchConfiguration('enable_lidars'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'enable_debug': LaunchConfiguration('enable_debug'),
            'camera_width': 1920,
            'camera_height': 1080,
            'num_point_clouds': 4,
        }],
        output='screen'
    )

    # Parameter setter node
    param_setter_node = Node(
        package='ros2_tui_test',
        executable='param_setter',
        name='param_setter',
        output='screen'
    )

    # Animated camera and depth images for the image viewer
    image_publisher_node = Node(
        package='ros2_tui_test',
        executable='image_publisher',
        name='image_publisher',
        output='screen'
    )

    # Simulated lidar and colored point clouds for the point cloud viewer
    lidar_publisher_node = Node(
        package='ros2_tui_test',
        executable='lidar_publisher',
        name='lidar_publisher',
        output='screen'
    )

    # /diagnostics from several simulated drivers
    diagnostics_publisher_node = Node(
        package='ros2_tui_test',
        executable='diagnostics_publisher',
        name='diagnostics_publisher',
        output='screen'
    )

    return LaunchDescription([
        publish_rate_arg,
        enable_cameras_arg,
        enable_lidars_arg,
        enable_imu_arg,
        enable_debug_arg,
        multi_publisher_node,
        param_setter_node,
        image_publisher_node,
        lidar_publisher_node,
        diagnostics_publisher_node
    ])