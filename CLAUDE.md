# CLAUDE.md - topics Development Notes

## Project Overview
`topics` is a Terminal User Interface (TUI) application for monitoring ROS2 topics in real-time. It provides a responsive, interactive interface for viewing topic information, message types, publisher/subscriber counts, and real-time Hz/Delay measurements.

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
- **Standard Deviation Tracking**: Captures std dev for both Hz and Delay measurements
- **Bollinger Bands Visualization**: Real-time charting with mean ± std dev bands

### User Interface
- **Scrollable Topic List**: Navigate through unlimited topics with keyboard
- **Loading Animation**: Ultra-fast time-based blinking dots (80ms per frame: `.  ` → `.. ` → `...` → repeating cycle in 240ms, independent of measurement timing)
- **Status Indicators**: Visual feedback for measurement states
- **Alphabetical Sorting**: Topics sorted by name for easy navigation
- **Responsive Design**: Optimized column widths based on content
- **Detail View**: Expandable topic details with statistical display (e.g., "Frequency (Hz): 1.09 ± 0.042")
- **Real-time Charts**: Live Hz and Delay visualization with Bollinger bands (Green main lines, Red std dev bands)
- **Proportional Scaling**: Charts automatically adapt to data range with 10% padding

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
- **FIFO Data Queues**: Synchronized history tracking for main values and std dev (max 100 data points)
- **Multi-line Parsing**: Handles ROS2 output where average and std dev appear on separate lines
- **Unit Conversion**: Automatic conversion of delay values to milliseconds for display

### Error Handling
- **No Header Detection**: Monitors stderr for "msg does not have header" messages
- **Process Management**: Automatic restart of failed measurement processes
- **Graceful Degradation**: Continues operation even if some measurements fail

## Command Line Options
- `--refresh <seconds>`: Topic list refresh rate (default: 5 seconds)
- `--detail-refresh <seconds>`: Detail refresh interval (legacy, now unused)
- `--no-initial-fetch`: Skip initial topic fetch (for debugging)

## Keyboard Controls
- **↑↓ ←→**: Navigation (up/down select, left/right collapse/expand groups)
- **Enter**: Toggle topic watching (start/stop measurements) - groups ignored
- **Tab**: Toggle group expansion
- **Space**: Refresh topic list (in topic list mode) or close detail view (in topic detail mode)
- **c**: Collapse all groups
- **e**: Expand all groups
- **d**: Enter detail view for current topic (shows charts with Bollinger bands)
- **F4**: Enter search mode (auto-expands groups with matches, highlights matching groups with underline, persists filter for editing, supports hjkl input, ESC to cancel, Enter to confirm)
- **r/F5**: Refresh topic list
- **q/Ctrl+C/Esc**: Quit application
- **Backspace**: Delete search text (in search mode)

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
- Simplified FIFO approach for std dev synchronization instead of complex fusion logic
- Efficient Bollinger bands calculation using synchronized data queues

### Debug Logging
The application writes comprehensive debug logs to `topics_debug.log`:
- ROS command execution timing
- Message parsing results
- Thread lifecycle events
- Error conditions and recovery

### Testing
Includes unit tests for:
- ROS output parsing functions
- Topic list processing
- Hz/Delay measurement parsing
- Multi-line ROS2 output handling
- Standard deviation extraction and parsing

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

## Recent Improvements

### Bollinger Bands Implementation (Latest)
- **Standard Deviation Tracking**: Added std dev capture for both Hz and Delay measurements  
- **Real-time Visualization**: Implemented Bollinger bands (mean ± std dev) for both charts
- **Synchronized Data**: FIFO queues ensure Hz/delay values and std dev stay synchronized
- **Consistent Display**: Both charts use identical proportional scaling (10% padding) and color scheme
- **Statistical Detail View**: Enhanced topic details to show values with std dev (e.g., "1.09 ± 0.042")

### Chart Enhancements
- **Green Main Lines**: Primary Hz and Delay data visualization
- **Red Bollinger Bands**: Upper and lower std dev bands for variance visualization  
- **Braille Markers**: Consistent high-resolution chart rendering
- **Adaptive Bounds**: Automatic y-axis scaling based on data range (no forced zero baseline)
- **Unit Consistency**: Delay values properly converted to milliseconds throughout the pipeline

## Future Enhancements
- Topic message content preview
- Export functionality for measurements
- Configuration file support
- Multiple workspace support
- Performance metrics display