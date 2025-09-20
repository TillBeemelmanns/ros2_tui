# Project Overview

This project is a lightweight terminal user interface (TUI) for monitoring ROS2 topics. It is written in Rust and uses the `ratatui` library for rendering the UI. The application provides real-time monitoring of ROS2 topics, including frequency, delay, publisher and subscriber counts. It is designed to be a lightweight alternative to graphical tools like `rqt_topic`.

## Building and Running

### Prerequisites

- Rust and Cargo installed
- ROS2 installed and sourced

### Building the project

```bash
cargo build --release
```

The binary will be available at `target/release/topics`.

### Running the application

```bash
./target/release/topics
```

### Running tests

```bash
cargo test
```

## Development Conventions

The project follows standard Rust conventions. The code is organized into several modules:

- `main`: The entry point of the application.
- `app`: Contains the main application logic and state management.
- `ros`: Handles the interaction with the ROS2 command-line tools.
- `ui`: Renders the terminal user interface using `ratatui`.
- `topic_watcher`: Manages the background threads for monitoring topics.
- `tree`: Implements the topic tree data structure.

The application uses `crossbeam` channels for communication between the main UI thread and the background worker threads. This ensures that the UI remains responsive while fetching data from ROS2.
