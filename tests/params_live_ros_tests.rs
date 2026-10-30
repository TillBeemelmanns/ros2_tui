//! Live ROS2 integration tests for the parameter helper.
//!
//! These require a running ROS2 graph with the `ros2_tui_test` dummy nodes and
//! are therefore `#[ignore]`d by default so the standard `cargo test` run stays
//! green without ROS2. Run them manually with a live graph:
//!
//! ```bash
//! ros2 launch ros2_tui_test dummy_publishers.launch.py &
//! cargo test --test params_live_ros_tests -- --ignored
//! ```

use crossbeam::channel::Receiver;
use ros2_tui::params::watcher::{ParamBridgeHandle, ParamMessage, ParamWatchMessage};
use std::time::{Duration, Instant};

fn bridge(initial_fetch: bool) -> (ParamBridgeHandle, Receiver<ParamMessage>) {
    let (sender, receiver) = crossbeam::channel::unbounded();
    let handle = ParamBridgeHandle::new(sender, Duration::from_secs(3600), initial_fetch);
    (handle, receiver)
}

/// The first message `pick` accepts, within 20 seconds.
fn wait_for<T>(receiver: &Receiver<ParamMessage>, pick: impl Fn(ParamMessage) -> Option<T>) -> T {
    let deadline = Instant::now() + Duration::from_secs(20);
    while let Ok(msg) = receiver.recv_deadline(deadline) {
        if let Some(found) = pick(msg) {
            return found;
        }
    }
    panic!("no matching message within 20 s");
}

#[test]
#[ignore = "requires a live ROS2 graph with the ros2_tui_test dummy nodes"]
fn fetches_params_for_all_nodes() {
    let (handle, receiver) = bridge(true);
    let params = wait_for(&receiver, |msg| match msg {
        ParamMessage::ParamList(params) => Some(params),
        _ => None,
    });
    handle.shutdown();

    // The dummy publishers expose both nodes' parameters.
    for node in ["/param_setter", "/multi_publisher"] {
        assert!(
            params
                .iter()
                .any(|p| p.node_name == node && p.value.is_some()),
            "expected parameters with values from {}",
            node
        );
    }
}

#[test]
#[ignore = "requires a live ROS2 graph with the ros2_tui_test dummy nodes"]
fn dumps_and_loads_node_params_with_a_path_with_spaces() {
    let dir = std::env::temp_dir().join("ros2 tui dump test");
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let target = dir.join("param dump.yaml");
    let path = target.to_str().unwrap().to_string();

    let (handle, receiver) = bridge(false);
    handle.send(ParamWatchMessage::DumpParam {
        node_name: "/param_setter".into(),
        file_path: path.clone(),
    });
    wait_for(&receiver, |msg| match msg {
        ParamMessage::DumpSuccess { .. } => Some(()),
        ParamMessage::Error(e) => panic!("dump failed: {}", e),
        _ => None,
    });
    let contents = std::fs::read_to_string(&target).expect("dump file should exist");
    assert!(
        contents.contains("/param_setter:") && contents.contains("ros__parameters"),
        "dump file should contain a YAML parameter section:\n{}",
        contents
    );

    // Loading the dump back sets every parameter to the value it has.
    handle.send(ParamWatchMessage::LoadParam {
        node_name: "/param_setter".into(),
        file_path: path,
    });
    wait_for(&receiver, |msg| match msg {
        ParamMessage::LoadSuccess { .. } => Some(()),
        ParamMessage::Error(e) => panic!("load failed: {}", e),
        _ => None,
    });
    handle.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}
