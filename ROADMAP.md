# Roadmap

A living list of planned features, improvements, and fixes for `ros2_tui`.
Items are grouped by theme and roughly ordered by impact within each group.
See [CHANGELOG.md](CHANGELOG.md) for what has already shipped.

## Recently shipped

- ✅ **One long-lived `rclpy` helper for all ROS access** — replaces the
  `ros2 topic list`/`hz`/`delay`/`echo` processes and the per-viewer helper;
  restarts itself with its state if it exits (unreleased).
- ✅ **Sensor viewers** — `v` opens a viewer for images, compressed images,
  maps and costmaps, point clouds, laser scans, GNSS fixes, IMUs and
  diagnostics, with kitty graphics over SSH and text fallbacks (unreleased).
- ✅ **Realistic test publishers** — real JPEG cameras, a simulated lidar and
  2D scanner, a latched map, GNSS, IMU and diagnostics replace placeholder
  bytes (unreleased).
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

- [ ] **TF tree** (`/tf`, `/tf_static`) — frames as a tree with parent,
  publish rate, age and stale/missing links; the most common cause of "the
  sensor works but nothing shows up in RViz".
- [ ] **Odometry, Path and PoseStamped** — trajectory and current pose in the
  point cloud viewer's top view, with speed and covariance readouts; plus
  `Twist`/`TwistStamped` (`cmd_vel`) as linear/angular velocity plots.
- [ ] **JointState** — a table of positions, velocities and efforts with bars,
  to spot a stuck or noisy joint.
- [ ] **BatteryState** — charge, voltage, current, temperature and health.
- [ ] **Scalar sensors** — `Range`, `Temperature`, `FluidPressure`,
  `Illuminance`, `MagneticField`, `RelativeHumidity` as a value with a plot.
- [ ] **Plot any numeric field** — pick fields of any message and plot them,
  like rqt_plot. Needs a generic decoder driven by the message definition
  (`ros2 interface show`), which would also enable full echo with arrays.
- [ ] **CameraInfo next to its image** — intrinsics and distortion, and a
  warning when the size does not match the image.
- [ ] **MarkerArray** in the point cloud viewer.
- [ ] **Fixed frame for point clouds** — transform clouds and scans into a
  frame such as `base_link` via TF instead of showing the sensor frame.
- [ ] **Screenshots of the viewers** in the README.

## Planned features

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
  would let the app restart it too.

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
- [ ] **Viewer end-to-end tests in CI** — drive the `topics` binary in a
  pseudo terminal against the test publishers, answering the terminal queries
  like a kitty-capable terminal, and check the decoded frames and info lines
  for every viewer on each distribution.
- [ ] **Code coverage reporting** — add `cargo llvm-cov` (or similar) to CI.

## Maintainability

- [ ] **Shared TUI harness to cut topics/params duplication** — both have
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
