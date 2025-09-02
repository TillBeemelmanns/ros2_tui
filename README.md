# toptop

A lightweight terminal user interface (TUI) for monitoring ROS2 topics. Inspired by [turm](https://github.com/kabouzeid/turm), toptop provides real-time monitoring of ROS2 topics with minimal dependencies by wrapping around standard `ros2 topic` commands.

## Features

- 📊 **Real-time topic monitoring** - Continuous streaming of topic metrics with immediate updates
- 🎯 **Selective monitoring** - Choose which topics to watch for Hz/Delay measurements
- ⚡ **Fast loading animation** - Yellow blinking dots provide instant visual feedback
- 📋 **Complete topic info** - Publisher/subscriber counts, message types, sorted alphabetically
- 📜 **Automatic scrolling** - Handles unlimited topics with smooth navigation
- ⌨️ **Keyboard navigation** - Vim-like controls for easy navigation
- 🎨 **Visual indicators** - Color-coded watched topics, loading states, and error conditions
- 🚫 **Smart error handling** - Shows "No Stamp" for topics without header information
- ⚡ **Lightweight** - No ROS2 Python dependencies, wraps CLI commands efficiently
- 🚀 **High performance** - Multi-threaded continuous streaming architecture

## Prerequisites

- ROS2 installed and sourced
- `ros2 topic` command available in PATH

## Installation

### From Source

```bash
git clone https://github.com/beemelmanns/toptop.git
cd toptop/toptop
cargo build --release
```

The binary will be available at `target/release/toptop`.

### From crates.io (coming soon)

```bash
cargo install toptop
```

## Usage

### Basic Usage

```bash
# Start monitoring with default settings
toptop

# Custom refresh rate (topics list updates every 5 seconds by default)
toptop --refresh 10
```

### Topic Watching

By default, toptop shows all available ROS2 topics but **only displays Hz/Delay metrics for topics you choose to "watch"**. This prevents performance issues and provides instant feedback:

1. **Navigate** to any topic using `↑`/`↓` or `j`/`k`
2. **Press `Enter` or `Space`** to toggle watching for that topic
3. **Loading animation**: Fast blinking yellow dots show measurement startup
4. **Watched topics** show a `●` indicator and display live Hz/Delay metrics
5. **Topics without headers** show "No Stamp" instead of delay values
6. **Unwatched topics** show basic info only (name, type, pub/sub counts)

This selective monitoring approach allows you to:
- ✅ Monitor hundreds of topics without performance impact
- ✅ Focus on specific topics of interest  
- ✅ Reduce system resource usage
- ✅ Get responsive UI even with many active topics

### Command Line Options

- `--refresh <SECONDS>` - Topic list refresh rate (default: 5)
- `--detail-refresh <SECONDS>` - Detail refresh interval (legacy, unused)
- `--no-initial-fetch` - Skip initial topic fetch (for debugging)
- `--help` - Show help information
- `--version` - Show version

### Keyboard Controls

- `↑`/`k` - Move selection up
- `↓`/`j` - Move selection down  
- `Enter`/`Space` - Toggle topic watching (automatically starts Hz/Delay monitoring)
- `r`/`F5` - Force refresh topic list
- `q`/`Ctrl+C`/`Esc` - Quit

## Interface Overview

The interface is divided into three sections:

1. **Topics Table** (70% of screen)
   - Watch indicator (`●` for monitored topics)
   - Topic Name  
   - Message Type
   - Publisher Count
   - Subscriber Count  
   - Frequency (Hz) - only for watched topics
   - Delay (ms) - only for watched topics

2. **Topic Details** (25% of screen)
   - Detailed information for selected topic
   - Real-time Hz and Delay measurements
   - Publisher/Subscriber information

3. **Status Bar** (5% of screen)
   - Current status and topic count
   - Keyboard shortcuts reminder
   - Error messages (if any)

## Architecture

toptop is built as a lightweight wrapper around ROS2's built-in CLI tools using a **multi-threaded worker architecture** inspired by [turm](https://github.com/kabouzeid/turm):

### ROS2 Command Integration
- `ros2 topic list -v` - Get topic list with types and publisher/subscriber counts
- `ros2 topic hz <topic>` - Continuous topic frequency monitoring (streaming)
- `ros2 topic delay <topic>` - Continuous topic delay monitoring (streaming)

### Performance Architecture
- **Continuous Streaming**: ROS2 commands run as long-running processes for real-time updates
- **Message Passing**: Uses `crossbeam` channels for thread-safe communication
- **Non-blocking UI**: Main thread handles only UI rendering and user input (~5 FPS)
- **Responsive Controls**: Keyboard input is processed immediately regardless of ROS2 command latency
- **Smart Resource Management**: Only monitors selected topics to prevent system overload

This approach ensures:
- ✅ **Real-time updates** - Continuous streaming provides immediate metric updates
- ✅ **Scalable performance** - Only monitors topics you select for detailed metrics
- ✅ **Auto-scrolling interface** - Handles unlimited topics smoothly with alphabetical sorting
- ✅ **Visual feedback** - Fast loading animations and status indicators
- ✅ **Error resilience** - Graceful handling of topics without header information
- ✅ **Minimal dependencies** - Only standard ROS2 CLI tools required
- ✅ **Always compatible** - Works with any ROS2 installation
- ✅ **Lightweight and fast** - No Python/ROS2 library overhead

## Development

### Building

```bash
cargo build
```

### Running in Development

```bash
cargo run -- --refresh 1
```

### Testing

```bash
cargo test
cargo check
```

## Comparison with Other Tools

| Tool | Language | Dependencies | Features | Performance |
|------|----------|-------------|----------|-------------|
| toptop | Rust | ros2 CLI only | TUI, Real-time metrics | ⚡ Fast |
| rqt_topic | Python | Full ROS2 + Qt | GUI, Rich features | 🐌 Heavy |
| ros2 topic | Python | Full ROS2 | CLI only | 🚀 Fast but limited |

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by [turm](https://github.com/kabouzeid/turm) - A TUI for the Slurm Workload Manager
- Built with [ratatui](https://github.com/ratatui-org/ratatui) - A Rust library for building rich terminal interfaces
- Thanks to the ROS2 community for the excellent CLI tools

## Roadmap

- [ ] Add topic message preview
- [ ] Export metrics to CSV/JSON
- [ ] Topic filtering and search
- [ ] Custom refresh intervals per topic
- [ ] Package for common Linux distributions
- [ ] Homebrew formula for macOS
- [ ] Windows support
