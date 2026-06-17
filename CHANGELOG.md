# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.6] - 2026-06-17

### Security
- Params command layer no longer builds shell command strings. `set`, `dump`,
  `load`, and `list` now pass node names, parameter values, and file paths
  directly to the `ros2` process, eliminating shell word-splitting and
  command-injection vectors. `ros2 param dump` writes its output file in Rust
  instead of a shell `>` redirect, so paths containing spaces work correctly.

### Fixed
- Every one-shot `ros2` CLI call now runs with a timeout. A single unresponsive
  node (one whose parameter or topic services never answer) previously stalled
  an entire refresh and froze the UI; such a node is now skipped after the
  timeout while the rest of the graph continues to load.

### Changed
- Replaced the deprecated and unmaintained `serde_yaml` dependency with its
  maintained drop-in fork `serde_yaml_ng`.

### Added
- Async unit tests for the command helper (timeout and no-shell-interpretation
  guarantees) and `#[ignore]`-gated live ROS2 integration tests for the
  parameter fetch and dump-to-path-with-spaces flows.

## [0.1.5] - 2026-06-17

### Added
- Dockerized test environment (`docker/run.sh`) that builds a ROS2 Humble image
  with the dummy publishers, so the TUIs can be tried without a local ROS2
  install.

### Changed
- Upgraded `ratatui` to 0.30 (and refreshed `clap`, `clap_complete`,
  `itertools`, `once_cell`, and `regex`).
- `topics` and `params` now report their version from `CARGO_PKG_VERSION`
  instead of a hardcoded string.

### Fixed
- Resolved a Clippy `collapsible_match` failure that broke CI.
- Fixed the `multi_publisher` test node hanging parameter queries: it now runs
  on a `MultiThreadedExecutor` with its publish timers in a dedicated callback
  group, keeping parameter services responsive.

## Earlier releases

Releases [0.1.0] through [0.1.4] predate this changelog. See the
[GitHub releases](https://github.com/TillBeemelmanns/ros2_tui/releases) for
their history.

[Unreleased]: https://github.com/TillBeemelmanns/ros2_tui/compare/v0.1.6...HEAD
[0.1.6]: https://github.com/TillBeemelmanns/ros2_tui/compare/v0.1.5...v0.1.6
[0.1.5]: https://github.com/TillBeemelmanns/ros2_tui/compare/v0.1.4...v0.1.5
[0.1.4]: https://github.com/TillBeemelmanns/ros2_tui/releases/tag/v0.1.4
[0.1.0]: https://github.com/TillBeemelmanns/ros2_tui/releases/tag/v0.1.0
