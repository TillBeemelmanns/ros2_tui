# Roadmap

A living list of planned features, improvements, and fixes for `ros2_tui`.
Items are grouped by theme and roughly ordered by impact within each group.
See [CHANGELOG.md](CHANGELOG.md) for what has already shipped.

## Recently shipped

- ✅ **One long-lived `rclpy` helper for all ROS access** — replaces the
  `ros2 topic list`/`hz`/`delay`/`echo` processes and the per-viewer helper;
  restarts itself with its state if it exits (0.2.0).
- ✅ **TF, odometry, path, joint state and battery viewers** — the TF tree
  from `/tf` and `/tf_static` with rates, ages and stale frames, frames with
  two parents and cycles; odometry with trail, jumps and velocity plots;
  paths; merged joint tables with plots; battery charge, cells and plots
  (0.2.0).
- ✅ **Params app through an `rclpy` helper** — neither app runs the `ros2`
  CLI any more (0.2.0).
- ✅ **Sensor viewers** — `v` opens a viewer for images, compressed images,
  maps and costmaps, point clouds, laser scans, GNSS fixes, IMUs and
  diagnostics, with kitty graphics over SSH and text fallbacks (0.2.0).
- ✅ **Realistic test publishers** — real JPEG cameras, a simulated lidar and
  2D scanner, a latched map, GNSS, IMU and diagnostics replace placeholder
  bytes (0.2.0).
- ✅ **Parallel per-node parameter fetch** — a params refresh dumps up to 8
  nodes concurrently (0.1.9).
- ✅ **Multi-distro ROS2 CI** — the integration job runs on Humble and
  Lyrical (0.1.8).
- ✅ **Timeouts on every one-shot `ros2` call** — an unresponsive node can no
  longer freeze a refresh (0.1.6).
- ✅ **Shell-safe params command layer** — arguments are passed directly to
  `ros2` instead of building shell strings, fixing injection/quoting bugs and
  `param dump` to paths with spaces (0.1.6).
- ✅ **Replaced deprecated `serde_yaml`** with maintained `serde_yaml_ng` (0.1.6).
- ✅ **`CHANGELOG.md`** added and linked from `pyproject.toml` (0.1.6).
- ✅ **Dockerized test environment** (`docker/run.sh`) and **ratatui 0.30**
  upgrade; fixed the `multi_publisher` test node hanging parameter queries
  (0.1.5).

## Viewers

Next message types, ordered by how much they help check a robot over SSH:

- [ ] **Plot any numeric field** — pick fields of any message and plot them,
  like rqt_plot (e.g. `/cmd_vel.linear.x`). The most generic next viewer:
  needs a decoder driven by the message definition, which the helper can
  read from the type support, and would also enable full echo with arrays.
- [ ] **PoseStamped and Twist/TwistStamped (`cmd_vel`)** — pose readout and
  linear/angular velocity plots, reusing the odometry viewer.
- [ ] **TF: more checks** — the transform between any two selected frames,
  frames that sensor topics use but TF lacks, and the broadcasting node of
  each frame.
- [ ] **Scalar sensors** — `Range`, `Temperature`, `FluidPressure`,
  `Illuminance`, `MagneticField`, `RelativeHumidity` as a value with a plot.
- [ ] **CameraInfo next to its image** — intrinsics and distortion, and a
  warning when the size does not match the image.
- [ ] **MarkerArray** in the point cloud viewer.
- [ ] **Fixed frame for point clouds** — transform clouds and scans into a
  frame such as `base_link` via TF instead of showing the sensor frame.
- [ ] **Screenshots and a terminal recording** (asciinema/GIF) of the
  viewers in the README.

## Planned features

- [ ] **One `ros2_tui` binary with tabs** — topics, params and later
  services, actions and TF as tabs of one app, sharing the terminal setup
  and the event loop that `topics` and `params` duplicate today; the
  existing binaries stay as shortcuts to their tab.
- [ ] **Services app** — browse/list/call ROS2 services (mirrors the topics/params TUIs).
- [ ] **Actions app** — browse/send/monitor ROS2 actions.
- [ ] **Config file & theme support** — persist refresh rates, timeout, and color
  themes instead of relying solely on CLI flags.

## Robustness

- [ ] **Configurable service timeout** — the params helper gives a node 5 s
  to answer a parameter request; expose it as a CLI flag / env var for very
  large or slow graphs.
- [ ] **Detect a hung helper** — a helper that exits is restarted, but one
  that hangs (e.g. blocked in a callback) is not noticed; a heartbeat event
  every second, and a restart after a few missed ones, would cover it for
  both apps.

## Architecture

- ✅ **Params app through the helper too** — `params` calls the parameter
  services from its own `rclpy` helper; neither app runs the `ros2` CLI.
- [ ] **Optional Python-free transport** — receive the serialized messages
  directly with a pure-Rust DDS implementation (e.g. RustDDS) and a Zenoh
  client for `rmw_zenoh`. The Rust side already decodes CDR, so this would
  need neither Python nor ROS libraries at runtime and could even run on a
  machine without ROS. Costs: each RMW's wire format and discovery setup
  (domain ID, discovery server, security) must be handled, and shared-memory
  transport is not available. Linking ROS directly (`r2r`, `rclrs`) is not an
  option, as it ties every binary to one distribution.

## Performance

- [ ] **Reduce cloning in the app layer** — `app.rs` does a lot of `.clone()` on
  tree/visible-item state each frame; profile and trim where it matters.

## Testing

- [ ] **Tree navigation & filtering unit tests** — `src/common/tree.rs` /
  `abstract_tree.rs` have no coverage, despite the contributor guide explicitly
  asking for it. Add regression tests for selection, collapse/expand, and search.
- [ ] **Wire the live ROS2 tests into CI** — `tests/params_live_ros_tests.rs` is
  `#[ignore]`d; add a CI job that runs it (and topic equivalents) inside the
  Docker image so the real `ros2` integration path is exercised on every PR.
- [ ] **Topic-side live integration tests** — start the ROS helper against the
  dummy publishers and check its topic list, rates, delays and echo.
- [ ] **Viewer end-to-end tests in CI** (next up) — drive the `topics` and
  `params` binaries in a pseudo terminal against the test publishers (pyte as
  the terminal emulator), answering the terminal queries like a
  kitty-capable terminal, and check the decoded frames, info lines, helper
  recovery and parameter set/dump/load on each distribution. The drivers
  used during development only need moving into `tests/e2e/` and a step in
  the Docker job.
- [ ] **Code coverage reporting** — add `cargo llvm-cov` (or similar) to CI.

## Maintainability

- [ ] **Shared TUI harness to cut topics/params duplication** (a first step
  towards the tabbed binary above) — both have
  near-identical `main()`/`run_app()` event loops (terminal setup, the
  `where io::Error: From<B::Error>` draw loop, key dispatch scaffolding) and
  parallel `app/ros/ui/watcher` modules. Factor the common scaffolding into
  `common`.
- [ ] **Audit remaining `unwrap()`s** — 11 across `src/` (mostly parser regex in
  `*/ros.rs` and clap arg parsing in `src/bin/`); low risk, but worth replacing
  with explicit handling per the contributor guide.
- [ ] **Remove the legacy `--detail-refresh` flag** — documented as "Legacy
  detail polling interval (still accepted)" and no longer drives behavior; drop
  it (or restore a real use).
- [ ] **Simplify `convert_to_ros2_format`** — with arguments now passed directly
  to `ros2` (no shell word-splitting), the array space-stripping is largely
  redundant; revisit whether it's still needed.

## Developer experience & CI

- [ ] **Declare an MSRV** — add `rust-version` to `Cargo.toml` and a pinned-toolchain
  job to CI so the minimum supported Rust is explicit and enforced.
- [ ] **Verify the wheel build in CI** — run `maturin build` on PRs so packaging
  regressions are caught before a release tag.
