# ROS2 TUI Test Package

This package provides comprehensive test infrastructure for developing and debugging the `ros2_tui` tools. It includes dummy publishers and parameter setters that create realistic automotive/robotics ROS2 environments.

## Components

### 1. Multi Publisher (`multi_publisher.py`)
A comprehensive ROS2 publisher that creates realistic automotive/robotics topics including:

- **Camera Topics**: Multiple camera positions (front, mid, rear) with camera info and compressed images
- **LiDAR Topics**: Multiple LiDAR sensors with point cloud data
- **IMU/Navigation**: IMU data, GPS, odometry, and velocity information
- **Control**: Drive-by-wire (DBW) lateral/longitudinal control states
- **AI/ML**: Computer vision outputs including depth images and point clouds
- **System**: Robot description, TF transforms, and logging

**Features:**
- Configurable publishing rate and sensor enablement
- Realistic data with noise and variation
- Dynamic message sizes and content
- 35+ different topic types

### 2. Parameter Setter (`param_setter.py`)
A comprehensive parameter management node with extensive parameter sets:

- **Camera Parameters**: Resolution, exposure, gain, brightness, etc.
- **AI/ML Parameters**: Model paths, confidence thresholds, inference settings
- **LiDAR Parameters**: Range limits, filtering, clustering settings
- **IMU Parameters**: Calibration, bias correction, filtering
- **Navigation Parameters**: Velocity limits, tolerances, planning settings
- **Control Parameters**: PID values, safety timeouts, steering limits
- **System Parameters**: Logging, diagnostics, resource limits
- **Array Parameters**: Various array types for testing
- **Edge Cases**: Zero values, negatives, unicode strings, special characters

**Features:**
- 100+ parameters across 8 namespaces
- Dynamic parameter updates every 5 seconds
- Various data types (int, double, bool, string, arrays)
- Realistic automotive/robotics parameter values

### 3. Simple Dummy Publisher (`dummy_publisher.py`)
A basic publisher for simple testing scenarios with just 4 basic topics:
- String topic
- Integer topic 
- Float topic
- Boolean topic

## Installation

1. **Prerequisites**: Ensure you have ROS2 installed and sourced
2. **Build the package**:
   ```bash
   cd /path/to/your/ros2_workspace/src
   git clone <this-repo> ros2_tui_test  # or copy the package
   cd ..
   colcon build --packages-select ros2_tui_test
   source install/setup.bash
   ```

## Usage

### Using the Launch File (Recommended)
Start both the multi-publisher and parameter setter:

```bash
ros2 launch ros2_tui_test dummy_publishers.launch.py
```

**Launch Arguments:**
```bash
# Custom publishing rate (default: 10.0 Hz)
ros2 launch ros2_tui_test dummy_publishers.launch.py publish_rate:=5.0

# Disable specific sensor types
ros2 launch ros2_tui_test dummy_publishers.launch.py enable_cameras:=false enable_lidars:=false

# Enable debug output
ros2 launch ros2_tui_test dummy_publishers.launch.py enable_debug:=true
```

### Running Nodes Individually

**Multi Publisher:**
```bash
ros2 run ros2_tui_test multi_publisher
```

**Parameter Setter:**
```bash
ros2 run ros2_tui_test param_setter
```

**Simple Dummy Publisher:**
```bash
ros2 run ros2_tui_test dummy_publisher
```

### Testing with ros2_tui Tools

Once the test nodes are running, you can test the ros2_tui tools:

```bash
# Test the topics TUI
ros2-topics

# Test the parameters TUI  
ros2-params
```

## Testing Scenarios

### Topics Testing
The multi-publisher creates these topic categories:
- `/drivers/camera/*/left/*` - Camera topics with varying data sizes
- `/drivers/lidar_*/points` - LiDAR point clouds
- `/imu/*` - IMU and navigation data
- `/drivers/dbw/*` - Control system states
- `/ai_module/output/*` - Computer vision outputs
- `/planning/*` - Planning and trajectory topics
- `/tf` and `/tf_static` - Transform data

### Parameters Testing
The parameter setter provides:
- **Type Variety**: int, double, bool, string, and array parameters
- **Namespace Organization**: 8 logical namespaces (camera, ai_module, lidar, etc.)
- **Dynamic Updates**: Parameters change values every 5 seconds for testing real-time updates
- **Edge Cases**: Special values like zeros, negatives, unicode text
- **Realistic Values**: Automotive/robotics parameter ranges and types

### Performance Testing
- **High Frequency**: Up to 10 Hz publishing by default (configurable)
- **Variable Load**: Message sizes and counts vary over time
- **Concurrent Topics**: 35+ simultaneous topics
- **Parameter Volume**: 100+ parameters across multiple nodes

## Configuration

### Multi Publisher Parameters
Configure via ROS2 parameters or launch arguments:
- `publish_rate` (double): Publishing frequency in Hz
- `enable_cameras` (bool): Enable camera topic publishing
- `enable_lidars` (bool): Enable LiDAR topic publishing  
- `enable_imu` (bool): Enable IMU topic publishing
- `enable_debug` (bool): Enable debug output
- `camera_width` (int): Camera resolution width
- `camera_height` (int): Camera resolution height
- `num_point_clouds` (int): Number of point cloud topics

## Development Notes

### Topic Naming Convention
Topics follow realistic automotive naming patterns:
- `/drivers/<sensor_type>/<position>/` for sensor hardware
- `/ai_module/output/` for AI/ML processing results
- `/planning/` for planning and control outputs
- Standard ROS2 conventions for system topics (`/tf`, `/rosout`)

### Parameter Organization
Parameters are organized into logical namespaces matching typical robotics systems:
- `camera.*` - Camera and imaging parameters
- `ai_module.*` - AI/ML model and inference parameters
- `lidar.*` - LiDAR sensor and processing parameters
- `imu.*` - IMU calibration and filtering
- `navigation.*` - Path planning and navigation
- `control.*` - Vehicle control and safety
- `system.*` - System configuration and diagnostics
- `fusion.*` - Sensor fusion parameters

### Brand-Free Design
All topic and parameter names avoid specific vendor/brand names to maintain neutrality while providing realistic testing scenarios.

## License

MIT License - See package.xml for details.