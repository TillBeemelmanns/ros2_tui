use ros2_tui::topics::ros::*;

#[test]
fn test_parse_topic_hz() {
    assert_eq!(
        parse_topic_hz("average rate: 10.5"),
        (Some(10.5), None, MeasurementStatus::HasValue)
    );

    assert_eq!(
        parse_topic_hz(
            "average rate: 1.125\n        min: 0.856s max: 0.920s std dev: 0.02596s window: 3"
        ),
        (Some(1.125), Some(0.02596), MeasurementStatus::HasValue)
    );

    // Test std dev only line (separate from Hz line in ROS2 output)
    assert_eq!(
        parse_topic_hz("        min: 0.856s max: 0.920s std dev: 0.05346s window: 2"),
        (None, Some(0.05346), MeasurementStatus::Loading(0))
    );

    assert_eq!(
        parse_topic_hz("no data"),
        (None, None, MeasurementStatus::Loading(0))
    );

    assert_eq!(
        parse_topic_hz("WARNING: topic [/test] does not appear to be published yet"),
        (None, None, MeasurementStatus::NotMeasuring)
    );
}

#[test]
fn test_parse_topic_delay() {
    assert_eq!(
        parse_topic_delay("average delay: 0.025"),
        (Some(0.025), None, MeasurementStatus::HasValue)
    );

    // Test delay with std dev on same line
    assert_eq!(
        parse_topic_delay(
            "average delay: 0.425\n        min: 0.419s max: 0.430s std dev: 0.00398s window: 7"
        ),
        (Some(0.425), Some(0.00398), MeasurementStatus::HasValue)
    );

    // Test std dev only line (separate from delay line in ROS2 output)
    assert_eq!(
        parse_topic_delay("        min: 0.419s max: 0.430s std dev: 0.00382s window: 8"),
        (None, Some(0.00382), MeasurementStatus::Loading(0))
    );

    assert_eq!(
        parse_topic_delay("msg does not have header"),
        (None, None, MeasurementStatus::NoStamp)
    );

    assert_eq!(
        parse_topic_delay("no delay data"),
        (None, None, MeasurementStatus::Loading(0))
    );

    assert_eq!(
        parse_topic_delay("WARNING: topic [/test] does not appear to be published yet"),
        (None, None, MeasurementStatus::NotMeasuring)
    );
}

#[test]
fn test_parse_topic_list_verbose() {
    let sample_output = "Published topics:\n * /my_topic [std_msgs/msg/String] 1 publisher\n * /another/topic [sensor_msgs/msg/Image] 2 publishers\n\nSubscribed topics:\n * /my_topic [std_msgs/msg/String] 3 subscribers\n * /third/topic [geometry_msgs/msg/Twist] 1 subscriber";
    let topics = parse_topic_list_verbose(sample_output);

    assert_eq!(topics.len(), 3);

    // Find topics by name since HashMap doesn't guarantee order
    let my_topic = topics.iter().find(|t| t.name == "/my_topic").unwrap();
    assert_eq!(my_topic.msg_type, "std_msgs/msg/String");
    assert_eq!(my_topic.publisher_count, 1);
    assert_eq!(my_topic.subscriber_count, 3);

    let another_topic = topics.iter().find(|t| t.name == "/another/topic").unwrap();
    assert_eq!(another_topic.msg_type, "sensor_msgs/msg/Image");
    assert_eq!(another_topic.publisher_count, 2);
    assert_eq!(another_topic.subscriber_count, 0);

    let third_topic = topics.iter().find(|t| t.name == "/third/topic").unwrap();
    assert_eq!(third_topic.msg_type, "geometry_msgs/msg/Twist");
    assert_eq!(third_topic.publisher_count, 0);
    assert_eq!(third_topic.subscriber_count, 1);
}
