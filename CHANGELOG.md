# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- ROS2 Lyrical Luth support. Both TUIs and the `ros2_tui_test` fixtures are
  verified against Lyrical; `docker/run.sh` now honours `ROS_DISTRO`
  (e.g. `ROS_DISTRO=lyrical ./docker/run.sh topics`, default `humble`).

- CI `ros-integration` job that runs the full test suite, including the live
  graph tests, against the dummy publishers on Humble and Lyrical.
- `--no-initial-fetch` is now honoured by both TUIs.

### Changed
- Updated dependencies to their latest releases (ratatui 0.30.2, clap 4.6.7,
  tokio 1.54.1, crossbeam 0.8.5, regex 1.13.1, serde 1.0.229, ...) and
  refreshed `Cargo.lock`.
- Removed the unused `clap_complete`, `itertools`, and `serde_json`
  dependencies.

### Fixed
- Topics: unwatching a topic or quitting no longer leaves `ros2 topic hz` /
  `ros2 topic delay` / `ros2 topic echo` processes running in the background.
- Topics: delay now shows "no stamp" for header-less messages instead of
  loading forever (the CLI reports this on stderr, which was never read).
- Topics: `Space`, `r`, and `F5` now refresh the topic list as documented.
- Topics: `Enter` in the detail view toggles the topic being viewed, not
  whichever row the list selection has drifted to.
- Topics: re-watching a topic whose measurement had ended no longer stays
  stuck loading; late metric updates no longer show values on unwatched topics.
- Topics: "collapse/expand all" (`c`) is no longer undone by the next refresh.
- Topics/params: the selected row stays put when a refresh adds rows above it.
- Params: periodic refreshes now update displayed values and drop parameters
  of nodes that have gone away.
- Params: editing parameters of namespaced nodes (e.g. `/robot/driver`) sends
  the correct node and parameter name to `ros2 param set`.
- Params: dump/load default to a relative file name (`robot_driver.yaml`
  instead of `/robot_driver.yaml` at the filesystem root) and work from a
  parameter namespace group, targeting its owning node.
- Params: any key now dismisses the warning popup (`q` used to quit).
- `--refresh 0` and non-numeric refresh values are rejected with an error
  instead of busy-looping or being silently replaced by the default.
- The terminal is restored if either TUI panics.
- Docker builds no longer copy the host `target/` directory into the image.
- Dropped the obsolete `tests_require` option from the test publisher's
  `setup.py`, which newer setuptools reports as an unknown option.

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
