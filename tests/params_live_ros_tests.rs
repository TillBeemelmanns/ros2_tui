//! Live ROS2 integration tests for the parameter command layer.
//!
//! These require a running ROS2 graph with the `ros2_tui_test` dummy nodes and
//! are therefore `#[ignore]`d by default so the standard `cargo test` run (and
//! CI, which has no ROS2) stays green. Run them manually with a live graph:
//!
//! ```bash
//! ros2 launch ros2_tui_test dummy_publishers.launch.py &
//! cargo test --test params_live_ros_tests -- --ignored
//! ```

use ros2_tui::params::ros::*;

#[tokio::test]
#[ignore = "requires a live ROS2 graph with the ros2_tui_test dummy nodes"]
async fn fetches_params_for_all_nodes() {
    let params = get_param_list_with_values()
        .await
        .expect("parameter fetch should succeed against a live graph");

    // The dummy publishers expose both nodes' parameters.
    assert!(
        params.iter().any(|p| p.node_name == "/param_setter"),
        "expected parameters from /param_setter"
    );
    assert!(
        params.iter().any(|p| p.node_name == "/multi_publisher"),
        "expected parameters from /multi_publisher"
    );
}

#[tokio::test]
#[ignore = "requires a live ROS2 graph with the ros2_tui_test dummy nodes"]
async fn dumps_node_params_to_path_with_spaces() {
    let dir = std::env::temp_dir().join("ros2 tui dump test");
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let target = dir.join("param dump.yaml");
    let target_str = target.to_str().unwrap();

    // A shell `>` redirect with an unquoted path would split on the spaces;
    // writing the captured stdout ourselves handles the path correctly.
    dump_params("/param_setter", target_str)
        .await
        .expect("dump should succeed for a path containing spaces");

    let contents = std::fs::read_to_string(&target).expect("dump file should exist");
    assert!(
        contents.contains("ros__parameters"),
        "dump file should contain a YAML parameter section"
    );

    let _ = std::fs::remove_dir_all(&dir);
}
