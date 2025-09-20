# ros2_tui

[![Crates.io](https://img.shields.io/crates/v/ros2_tui.svg)](https://crates.io/crates/ros2_tui)
[![PyPI](https://img.shields.io/pypi/v/ros2-tui.svg)](https://pypi.org/project/ros2-tui/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Test](https://github.com/TillBeemelmanns/ros2_tui/actions/workflows/test.yml/badge.svg)](https://github.com/TillBeemelmanns/ros2_tui/actions/workflows/test.yml)
[![Release](https://github.com/TillBeemelmanns/ros2_tui/actions/workflows/release.yml/badge.svg)](https://github.com/TillBeemelmanns/ros2_tui/actions/workflows/release.yml)

A powerful Terminal User Interface (TUI) for monitoring and managing ROS2 topics and parameters in real-time.

## Prerequisites

- ROS2 installed and sourced
- `ros2 topic` and `ros2 param` commands available in PATH

## Installation

### From crates.io

```bash
cargo install ros2_tui
```

### From pypi

```bash
pip install ros2-tui
```

### From Source

```bash
git clone https://github.com/beemelmanns/topics.git
cd topics/topics
cargo build --release
```

The binaries `topics` and `params` will be available at `target/release/`.

## Topics TUI (`topics`)

### Highlights
- 📊 Real-time Hz/Delay monitoring backed by streaming `ros2 topic hz` / `delay`
- 🌳 Namespace-aware topic tree with collapsible groups and instant search
- 🎯 Watch-only metrics so large graphs stay responsive and low overhead
- 🎨 Status indicators, Bollinger-band charts, and debug logging for on-call triage

### Usage
```bash
# Start monitoring with default settings
topics

# Custom refresh rate (topics list updates every 5 seconds by default)
topics --refresh 10
```

Toggling a row with `Enter` starts Hz/Delay measurement. Unwatched topics stay lightweight, ensuring responsive navigation even with hundreds of topics.

### Navigation & Controls
- `↑`/`↓` or `j`/`k` move between topics; `←`/`→` or `h`/`l` collapse and expand namespaces
- `Enter` toggles watching on the focused topic or expands/collapses a group
- `Tab` toggles the current group; `c` collapses/expands all groups at once
- `F4` opens live search with auto-expansion; `Space`, `r`, or `F5` refresh immediately
- `q`, `Esc`, or `Ctrl+C` quit; `--verbose` writes detailed logs to `topics_debug.log`

### Command-Line Options
- `--refresh <SECONDS>` – Topic list refresh cadence (default: 5)
- `--detail-refresh <SECONDS>` – Legacy detail polling interval (still accepted)
- `--no-initial-fetch` – Skip the initial `ros2 topic list` call
- `--help`, `--version` – Standard metadata flags

### Architecture Highlights
- Background workers keep `ros2 topic list -v`, `ros2 topic hz`, and `ros2 topic delay` streaming without blocking the UI
- Crossbeam channels drive a non-blocking event loop that renders at ~5 FPS and processes input instantly
- Watched topics maintain FIFO histories (including std dev) to power Bollinger-band charts and statistical readouts
- Selective monitoring conserves system resources by spawning measurement processes only when needed

## Params TUI (`params`)

### Highlights
- 🧭 Node/namespace browser that mirrors `ros2 param list` hierarchies
- 🔄 Live value polling keeps displayed values fresh without manual refreshes
- ✏️ In-terminal editing with type validation plus YAML dump/load workflows
- 🧰 Built-in search, success/error banners, and contextual help overlays

### Usage
```bash
# Parameter dashboard with 5 second refresh
params

# Faster poll rate for parameters
params --refresh 2
```

The app groups dotted parameter names into expandable namespaces so large graphs stay navigable. Value edits, dumps, and loads run through the ROS2 CLI and report their outcome inline.

### Navigation & Controls
- `↑`/`↓` or `j`/`k` move through nodes and parameters; `←`/`→` or `h`/`l` collapse/expand namespaces
- `?` opens the help overlay; `F4` enters search mode with persistent filtering
- `Space`, `r`, or `F5` refresh on demand; `Esc` exits dialogs, cancels search, or quits

### Parameter Actions
- `s` edits the selected parameter (array values are normalised for ROS2 compatibility)
- `d` dumps the current node to YAML via `ros2 param dump`
- `Ctrl+l` loads YAML into the active node using `ros2 param load`
- Inline success/error banners acknowledge operations and fade automatically

### Command-Line Options
- `--refresh <SECONDS>` – Parameter polling cadence (default: 5)
- `--no-initial-fetch` – Skip the initial `ros2 param list` scan
- `--verbose` / `-v` – Emit detailed logs to `params_debug.log`
- `--help`, `--version` – Standard metadata flags

### Architecture Highlights
- A primary watcher repeatedly shells out to `ros2 param list`, building a `ParamTree` that mirrors node hierarchies
- A secondary watcher streams value lookups to populate the table without blocking the UI thread
- Dump/load/edit workflows wrap the ROS2 CLI while scheduling delayed refreshes to reflect state changes
- All ROS2 interaction happens on background threads, keeping the Crossterm-driven interface responsive under load

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
cargo clippy --all-targets --all-features -- -D warnings
```


## Comparison with Other Tools

| Tool | Language | Dependencies | Features | Performance |
|------|----------|-------------|----------|-------------|
| topics | Rust | ros2 CLI only | TUI, Real-time metrics | ⚡ Fast |
| params | Rust | ros2 CLI only | TUI, Parameter management | ⚡ Fast |
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
