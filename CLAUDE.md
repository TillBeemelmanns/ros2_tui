# CLAUDE.md - toptop Development Notes

## Project Overview
`toptop` is a Terminal User Interface (TUI) application for monitoring ROS2 topics in real-time. It provides a responsive, interactive interface for viewing topic information, message types, publisher/subscriber counts, and real-time Hz/Delay measurements.

## Architecture

### Core Components
- **main.rs**: Application entry point with command-line argument parsing and main event loop
- **app.rs**: Application state management and message handling 
- **ros.rs**: ROS2 command execution and output parsing
- **topic_watcher.rs**: Background worker threads for continuous monitoring
- **ui.rs**: Terminal UI rendering using ratatui framework

### Threading Model
- **Main Thread**: UI rendering and event handling
- **Topic List Watcher**: Periodic refresh of available topics (default: 5 seconds)
- **Topic Detail Watcher**: Manages continuous Hz/Delay measurement streams
- **Message Passing**: Uses crossbeam channels for thread communication

## Key Features

### Topic Monitoring
- **Continuous Streaming**: Uses `ros2 topic hz` and `ros2 topic delay` as long-running processes
- **Real-time Updates**: Measurements update immediately as data arrives
- **Selective Monitoring**: Only measure Hz/Delay for watched topics
- **Error Handling**: Graceful handling of topics without headers ("No Stamp")

### User Interface
- **Scrollable Topic List**: Navigate through unlimited topics with keyboard
- **Loading Animation**: Fast blinking yellow dots during measurement startup
- **Status Indicators**: Visual feedback for measurement states
- **Alphabetical Sorting**: Topics sorted by name for easy navigation
- **Responsive Design**: Optimized column widths based on content

### Performance Optimizations
- **Shared Tokio Runtime**: Avoid expensive runtime creation
- **No Blocking Operations**: All ROS commands run asynchronously
- **Efficient Polling**: 200ms UI update cycle with selective animation updates
- **Resource Management**: Proper cleanup of background processes

## Technical Implementation

### ROS2 Integration
The application wraps these ROS2 CLI commands:
- `ros2 topic list -v`: Get topics with publisher/subscriber counts
- `ros2 topic hz <topic>`: Continuous frequency measurement
- `ros2 topic delay <topic>`: Continuous delay measurement

### State Management
- **MeasurementStatus Enum**: Tracks measurement state (NotMeasuring, Loading, HasValue, NoStamp)
- **Topic Preservation**: Maintains watch state across topic list refreshes
- **Message Merging**: Combines Hz and Delay updates as they arrive

### Error Handling
- **No Header Detection**: Monitors stderr for "msg does not have header" messages
- **Process Management**: Automatic restart of failed measurement processes
- **Graceful Degradation**: Continues operation even if some measurements fail

## Command Line Options
- `--refresh <seconds>`: Topic list refresh rate (default: 5 seconds)
- `--detail-refresh <seconds>`: Detail refresh interval (legacy, now unused)
- `--no-initial-fetch`: Skip initial topic fetch (for debugging)

## Keyboard Controls
- **↑/↓ or k/j**: Navigate topic list
- **Enter/Space**: Toggle topic watching (start/stop measurements)
- **r/F5**: Refresh topic list
- **q/Ctrl+C/Esc**: Quit application

## Dependencies
- **ratatui**: Terminal UI framework
- **crossterm**: Cross-platform terminal manipulation
- **tokio**: Async runtime for ROS command execution
- **crossbeam**: Channel-based message passing
- **regex**: Pattern matching for ROS output parsing
- **serde**: Serialization for data structures
- **clap**: Command-line argument parsing

## Development Notes

### Performance Considerations
- Removed concurrent `ros2 topic info` calls that were causing system slowdown
- Implemented continuous streaming instead of timeout-based commands
- Optimized UI update frequency and animation timing

### Debug Logging
The application writes comprehensive debug logs to `toptop_debug.log`:
- ROS command execution timing
- Message parsing results
- Thread lifecycle events
- Error conditions and recovery

### Testing
Includes unit tests for:
- ROS output parsing functions
- Topic list processing
- Hz/Delay measurement parsing

### Known Limitations
- Requires ROS2 environment to be properly sourced
- Performance depends on ROS2 system load
- Limited to ROS2 topics (no ROS1 support)

## Build Instructions
```bash
# Development build
cargo build

# Release build
cargo build --release

# Run tests
cargo test
```

## Future Enhancements
- Topic message content preview
- Export functionality for measurements
- Configuration file support
- Multiple workspace support
- Performance metrics display