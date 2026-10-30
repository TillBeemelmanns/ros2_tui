use ros2_tui::params::ros::ParamInfo;
use ros2_tui::params::watcher::ParamMessage;
use ros2_tui::params::{AppMode, ParamsApp};
use std::time::Duration;

/// An app whose watchers never call `ros2` during a test.
fn app() -> ParamsApp {
    ParamsApp::new(Duration::from_secs(3600), false)
}

fn param(node: &str, name: &str, value: &str) -> ParamInfo {
    ParamInfo {
        node_name: node.to_string(),
        param_name: name.to_string(),
        value: Some(value.to_string()),
        param_type: "Integer".to_string(),
    }
}

fn select(app: &mut ParamsApp, full_path: &str) {
    app.selected_index = app
        .visible_items
        .iter()
        .position(|item| item.node.full_path == full_path)
        .unwrap_or_else(|| panic!("{} not visible", full_path));
}

#[test]
fn test_set_parameter_on_namespaced_node() {
    let mut app = app();
    app.handle_message(ParamMessage::ParamList(vec![param(
        "/robot/driver",
        "camera.fps",
        "30",
    )]));
    app.expand_all();
    select(&mut app, "/robot/driver/camera.fps");

    app.on_set_parameter();

    let edit = app.edit_state.as_ref().expect("edit dialog should open");
    assert_eq!(app.mode, AppMode::SetParameter);
    assert_eq!(edit.node_name, "/robot/driver");
    assert_eq!(edit.param_name, "camera.fps");
    assert_eq!(edit.current_value, "30");
}

#[test]
fn test_refresh_updates_values_and_drops_vanished_nodes() {
    let mut app = app();
    app.handle_message(ParamMessage::ParamList(vec![
        param("/a", "rate", "1"),
        param("/b", "rate", "2"),
    ]));
    app.handle_message(ParamMessage::ParamList(vec![param("/a", "rate", "5")]));

    assert_eq!(app.param_map["/a/rate"].value.as_deref(), Some("5"));
    assert!(!app.param_map.contains_key("/b/rate"));
}

#[test]
fn test_dump_dialog_targets_owning_node_with_relative_file() {
    let mut app = app();
    app.handle_message(ParamMessage::ParamList(vec![
        param("/robot/driver", "rate", "1"),
        param("/robot/driver", "camera.fps", "30"),
    ]));

    // Both the node group and a parameter namespace group resolve to the node
    for group in ["/robot/driver", "/robot/driver/camera"] {
        select(&mut app, group);
        app.on_dump_parameters();
        assert_eq!(app.mode, AppMode::DumpParameters);
        assert_eq!(app.file_target_node.as_deref(), Some("/robot/driver"));
        assert_eq!(app.file_input, "robot_driver.yaml");
        app.on_escape();
        assert_eq!(app.file_target_node, None);
    }
}

#[test]
fn test_selection_survives_refresh_adding_rows_above() {
    let mut app = app();
    app.handle_message(ParamMessage::ParamList(vec![param("/m", "rate", "1")]));
    select(&mut app, "/m");

    app.handle_message(ParamMessage::ParamList(vec![
        param("/a", "rate", "1"),
        param("/m", "rate", "1"),
    ]));

    assert_eq!(app.visible_items[app.selected_index].node.full_path, "/m");
}

#[test]
fn test_refresh_keeps_a_set_error() {
    let mut app = app();
    app.handle_message(ParamMessage::ParamList(vec![param("/n", "rate", "1")]));
    app.handle_message(ParamMessage::ParamSetError {
        node_name: "/n".to_string(),
        param_name: "rate".to_string(),
        error: "Setting parameter failed: wrong type".to_string(),
    });
    app.handle_message(ParamMessage::ParamList(vec![param("/n", "rate", "1")]));
    assert!(app
        .error_message
        .as_deref()
        .is_some_and(|e| e.contains("wrong type")));
}
