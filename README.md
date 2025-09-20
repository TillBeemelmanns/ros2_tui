# ros2_tui

[![Crates.io](https://img.shields.io/crates/v/ros2_tui.svg)](https://crates.io/crates/ros2_tui)
[![PyPI](https://img.shields.io/pypi/v/ros2-tui.svg)](https://pypi.org/project/ros2-tui/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/beemelmanns/ros2_tui/workflows/Test/badge.svg)](https://github.com/beemelmanns/ros2_tui/actions)

A powerful Terminal User Interface (TUI) for monitoring and managing ROS2 topics and parameters in real-time.

## Features

- 📊 **Real-time topic monitoring** - Continuous streaming of topic metrics with immediate updates
- 🎯 **Selective monitoring** - Choose which topics to watch for Hz/Delay measurements
- 🌳 **Hierarchical grouping** - Topics organized by namespace with collapsible groups
- 🔍 **Search functionality** - Filter topics with F4 search mode
- ⚡ **Fast loading animation** - Ultra-fast blinking dots provide instant visual feedback
- 📋 **Complete topic info** - Publisher/subscriber counts, message types, sorted alphabetically
- 📜 **Automatic scrolling** - Handles unlimited topics with smooth navigation
- ⌨️ **Keyboard navigation** - Vim-like controls and hjkl navigation
- 🎨 **Visual indicators** - Color-coded watched topics, loading states, and error conditions
- 🚫 **Smart error handling** - Shows "No Stamp" for topics without header information
- ⚡ **Lightweight** - No ROS2 Python dependencies, wraps CLI commands efficiently
- 🚀 **High performance** - Multi-threaded continuous streaming architecture

## Prerequisites

- ROS2 installed and sourced
- `ros2 topic` command available in PATH

## Installation

### From crates.io (coming soon)

```bash
cargo install ros2_tui
```

### From pypi (coming soon)

```bash
pip install ros2-tui
```

### From Source

```bash
git clone https://github.com/beemelmanns/topics.git
cd topics/topics
cargo build --release
```

The binary will be available at `target/release/topics`.


## Usage

### Basic Usage

```bash
# Start monitoring with default settings
topics

# Custom refresh rate (topics list updates every 5 seconds by default)
topics --refresh 10
```

### Topic Watching and Navigation

The interface organizes topics hierarchically by namespace and **only displays Hz/Delay metrics for topics you choose to "watch"**. This prevents performance issues and provides instant feedback:

**Navigation:**
1. **Browse groups**: Topics are organized by their first namespace segment (e.g., `/camera/image` appears under "camera" group)
2. **Expand/collapse groups**: Use `→`/`←` arrows, `Tab`, or `Enter` on group names
3. **Navigate topics**: Use `↑`/`↓` or `j`/`k` to move between topics and groups
4. **Search**: Press `F4` to filter topics and auto-expand matching groups
5. **Toggle all groups**: Press `c` to collapse or expand all groups at once

**Topic Watching:**
1. **Select any topic** and press `Enter` to toggle watching
2. **Loading animation**: Ultra-fast blinking dots show measurement startup
3. **Watched topics** show a `●` indicator and display live Hz/Delay metrics
4. **Topics without headers** show "No Stamp" instead of delay values
5. **Unwatched topics** show basic info only (name, type, pub/sub counts)

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
- `←`/`h` - Collapse current group
- `→`/`l` - Expand current group
- `Enter` - Toggle topic watching (starts Hz/Delay monitoring) / Toggle group expansion
- `Tab` - Toggle current group expansion
- `Space` - Refresh topic list
- `c` - Toggle collapse/uncollapse all groups
- `F4` - Enter search mode (ESC to cancel, Enter to confirm)
- `r`/`F5` - Force refresh topic list
- `q`/`Ctrl+C`/`Esc` - Quit


## Architecture

topics is built as a lightweight wrapper around ROS2's built-in CLI tools using a **multi-threaded worker architecture** inspired by [turm](https://github.com/kabouzeid/turm):

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
cargo build --release
```

### Testing

```bash
cargo test
cargo check
```


### Before Pushing
Check formatting
```bash
cargo fmt --all -- --check
```

Lint code
```bash
cargo clippy --all-targets --all-features -- -D warningss
```


## Comparison with Other Tools

| Tool | Language | Dependencies | Features | Performance |
|------|----------|-------------|----------|-------------|
| topics | Rust | ros2 CLI only | TUI, Real-time metrics | ⚡ Fast |
| rqt_topic | Python | Full ROS2 + Qt | GUI, Rich features | 🐌 Heavy |
| ros2 topic | Python | Full ROS2 | CLI only | 🚀 Fast but limited |

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by [turm](https://github.com/kabouzeid/turm) - A TUI for the Slurm Workload Manager
- Built with [ratatui](https://github.com/ratatui-org/ratatui) - A Rust library for building rich terminal interfaces

## Roadmap

- [x] Topic filtering and search (F4 search mode)
- [x] Hierarchical topic grouping with collapsible groups
- [x] Add topic message preview/echo functionality  
- [ ] services app
- [ ] actions app
