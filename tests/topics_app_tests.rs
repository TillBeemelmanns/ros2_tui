use ros2_tui::topics::ros::{MeasurementStatus, TopicInfo};
use ros2_tui::topics::watcher::TopicMessage;
use ros2_tui::topics::{App, AppMode};
use std::time::Duration;

/// An app whose topic watcher never calls `ros2` during a test.
fn app() -> App {
    App::new(Duration::from_secs(3600), Duration::from_secs(3600), false)
}

fn topics(names: &[&str]) -> TopicMessage {
    TopicMessage::TopicList(
        names
            .iter()
            .map(|name| TopicInfo {
                name: name.to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                ..Default::default()
            })
            .collect(),
    )
}

fn select(app: &mut App, full_path: &str) {
    app.selected_index = app
        .visible_items
        .iter()
        .position(|item| item.node.full_path == full_path)
        .unwrap_or_else(|| panic!("{} not visible", full_path));
}

#[test]
fn test_selection_survives_refresh_adding_rows_above() {
    let mut app = app();
    app.handle_message(topics(&["/m_topic"]));
    select(&mut app, "/m_topic");

    app.handle_message(topics(&["/a_topic", "/m_topic"]));

    assert_eq!(
        app.visible_items[app.selected_index].node.full_path,
        "/m_topic"
    );
}

#[test]
fn test_enter_in_detail_view_toggles_the_detail_topic() {
    let mut app = app();
    app.handle_message(topics(&["/m_topic", "/z_topic"]));
    select(&mut app, "/m_topic");
    app.on_key('d');
    assert_eq!(app.mode, AppMode::TopicDetail);

    // Move the list selection elsewhere behind the detail view
    select(&mut app, "/z_topic");
    app.on_enter();

    assert!(app.topic_map["/m_topic"].watched);
    assert!(!app.topic_map["/z_topic"].watched);
}

#[test]
fn test_metric_updates_ignored_for_unwatched_topics() {
    let mut app = app();
    app.handle_message(topics(&["/t"]));

    app.handle_message(TopicMessage::TopicHzUpdate {
        topic_name: "/t".to_string(),
        hz: 10.0,
        std_dev: None,
    });

    let topic = &app.topic_map["/t"];
    assert_eq!(topic.hz, None);
    assert_eq!(topic.hz_status, MeasurementStatus::NotMeasuring);
    assert!(topic.hz_history.is_empty());
}

#[test]
fn test_expand_all_survives_refresh() {
    let mut app = app();
    let list = [
        "/drivers/dbw/lat_active",
        "/drivers/dbw/lon_active",
        "/imu/data",
    ];
    app.handle_message(topics(&list));
    let collapsed_rows = app.visible_items.len();

    app.toggle_collapse_all();
    let expanded_rows = app.visible_items.len();
    assert!(expanded_rows > collapsed_rows);

    app.handle_message(topics(&list));
    assert_eq!(app.visible_items.len(), expanded_rows);
}

#[test]
fn test_selection_stops_at_the_ends() {
    let mut app = app();
    app.handle_message(topics(&["/a_topic", "/b_topic"]));
    let last = app.visible_items.len() - 1;

    app.on_up();
    assert_eq!(app.selected_index, 0);

    app.selected_index = last;
    app.on_down();
    assert_eq!(app.selected_index, last);
}

#[test]
fn test_page_keys_jump_to_the_ends() {
    let mut app = app();
    app.handle_message(topics(&["/a_topic", "/b_topic"]));

    app.on_page_down();
    assert_eq!(app.selected_index, app.visible_items.len() - 1);
    app.on_page_up();
    assert_eq!(app.selected_index, 0);
}
