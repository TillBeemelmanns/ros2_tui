# Repository Guidelines

## Project Structure & Module Organization
Core Rust code lives in `src/`. CLI entry points sit in `src/bin/` (`topics.rs`, `params.rs`). Shared UI, trees, and logging helpers are under `src/common/`, while domain modules are mirrored in `src/topics/` and `src/params/` (each holding `app.rs`, `ros.rs`, `ui.rs`, and `watcher.rs`). Integration suites reside in `tests/` alongside the ROS-enabled fixtures in `tests/test_publisher/`, which provides launch files and dummy publishers for manual trials. Python packaging metadata is defined in `pyproject.toml` to expose the Rust binaries through `maturin`-built wheels.

## Build, Test, and Development Commands
- `cargo build` — compile the binaries; add `--release` before making artifacts.
- `cargo run --bin topics` / `cargo run --bin params` — exercise each TUI against a live ROS2 graph.
- `cargo fmt --all` and `cargo fmt --all -- --check` — format locally and match CI.
- `cargo clippy --all-targets --all-features -- -D warnings` — lint with zero tolerated warnings.
- `cargo test` and `cargo check` — run unit/integration suites and catch type regressions early.
- `maturin develop` — build the Python wheel in editable mode when validating the PyPI package.
- `./docker/run.sh [topics|params|shell]` — build a ROS2 Humble image with the dummy publishers and run a TUI without a local ROS2 install.

## Coding Style & Naming Conventions
Follow `rustfmt` defaults (4-space indent, trailing commas). Modules and files stay `snake_case`; types use `CamelCase`; functions and constants stay `snake_case`/`SCREAMING_SNAKE_CASE`. Prefer explicit error propagation over `unwrap` in the binaries so CLI users receive actionable messages. Use `common::debug_log` for optional tracing instead of ad hoc prints.

## Testing Guidelines
Run `cargo test` before every push; new ROS-dependent scenarios belong in `tests/` using the `*_ros_tests.rs` suffix. Spin up fixtures with `ros2 launch ros2_tui_test dummy_publishers.launch.py` to validate topic and parameter flows manually. Add regression coverage for tree navigation and filtering when touching `src/common/` data structures.

## Commit & Pull Request Guidelines
Commit messages are short and imperative (`fix delay parsing`, `add params watcher retry`), matching the existing history. Squash noisy work-in-progress commits locally. Pull requests should outline behaviour changes, note impacted commands, and link related ROS2 tickets. Confirm `cargo fmt`, `cargo clippy`, `cargo test`, and (if relevant) `maturin develop` before requesting review, and attach screenshots or terminal recordings when UI output shifts.

## ROS2 & Environment Notes
Ensure the ROS2 CLI is available (`ros2 --help`) before running binaries; the apps exit early when the toolchain is missing. Keep test fixtures under `tests/test_publisher/` aligned with upstream ROS2 message changes so launch files remain compatible across distributions.
