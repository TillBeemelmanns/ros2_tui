# Roadmap

A living list of planned features, improvements, and fixes for `ros2_tui`.
Items are grouped by theme and roughly ordered by impact within each group.
See [CHANGELOG.md](CHANGELOG.md) for what has already shipped.

## Recently shipped

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

## Planned features

- [ ] **Services app** — browse/list/call ROS2 services (mirrors the topics/params TUIs).
- [ ] **Actions app** — browse/send/monitor ROS2 actions.
- [ ] **Config file & theme support** — persist refresh rates, timeout, and color
  themes instead of relying solely on CLI flags.

## Robustness

- [ ] **Configurable command timeout** — expose `ROS2_COMMAND_TIMEOUT` as a CLI
  flag / env var; a fixed 10s is not ideal for every graph size.
- [ ] **Dedicated `Timeout` error variant** — currently a timeout surfaces as a
  generic `ParamError::Io`/`TopicError::Io`; a distinct variant gives clearer
  in-UI messaging ("node X timed out") vs. real I/O failures.
- [ ] **Liveness detection for streaming children** — the long-lived
  `ros2 topic hz`/`delay`/`echo` processes have no staleness/restart logic if
  they silently die; detect and re-spawn or flag the row.
- [ ] **Graceful mid-session loss of `ros2`** — the binaries only check
  `ros2 --help` at startup; handle the CLI/daemon disappearing while running.

## Performance

- [ ] **Parallelize per-node parameter fetch** — `get_param_list_with_values`
  (`src/params/ros.rs:577`) dumps nodes sequentially; now that each call is
  bounded by a timeout, fan them out (e.g. `join_all`) for much faster refreshes
  on large graphs.
- [ ] **Reduce cloning in the app layer** — `app.rs` does a lot of `.clone()` on
  tree/visible-item state each frame; profile and trim where it matters.

## Testing

- [ ] **Tree navigation & filtering unit tests** — `src/common/tree.rs` /
  `abstract_tree.rs` have no coverage, despite the contributor guide explicitly
  asking for it. Add regression tests for selection, collapse/expand, and search.
- [ ] **Wire the live ROS2 tests into CI** — `tests/params_live_ros_tests.rs` is
  `#[ignore]`d; add a CI job that runs it (and topic equivalents) inside the
  Docker image so the real `ros2` integration path is exercised on every PR.
- [ ] **Topic-side live integration tests** — mirror the params live tests for
  `get_topic_list` and the hz/delay parsers against the dummy publishers.
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
- [ ] **Multi-distro ROS2 CI matrix** — currently only Humble is exercised
  (locally, via Docker); add Jazzy/Rolling to catch CLI output drift.
- [ ] **Verify the wheel build in CI** — run `maturin build` on PRs so packaging
  regressions are caught before a release tag.
