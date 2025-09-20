use ros2_tui::params::ros::*;

#[test]
fn test_parse_param_list() {
    let output = r#"
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
"#;
    let params = parse_param_list(output).unwrap();
    assert_eq!(params.len(), 12);

    // Check some key parameters are present with correct node names
    let turtle_params: Vec<&ParamInfo> = params
        .iter()
        .filter(|p| p.node_name == "/teleop_turtle")
        .collect();
    let sim_params: Vec<&ParamInfo> = params
        .iter()
        .filter(|p| p.node_name == "/turtlesim")
        .collect();

    assert_eq!(turtle_params.len(), 8); // qos_overrides, qos_overrides./parameter_events, qos_overrides./parameter_events.publisher, depth, durability, scale_angular, scale_linear, use_sim_time
    assert_eq!(sim_params.len(), 4); // background_b, background_g, background_r, use_sim_time

    // Check that specific parameters exist
    assert!(params
        .iter()
        .any(|p| p.node_name == "/teleop_turtle" && p.param_name == "scale_angular"));
    assert!(params
        .iter()
        .any(|p| p.node_name == "/turtlesim" && p.param_name == "background_b"));
}

#[test]
fn test_convert_to_ros2_format() {
    // Test single values (should remain unchanged)
    assert_eq!(convert_to_ros2_format("true"), "true");
    assert_eq!(convert_to_ros2_format("false"), "false");
    assert_eq!(convert_to_ros2_format("True"), "True");
    assert_eq!(convert_to_ros2_format("False"), "False");
    assert_eq!(convert_to_ros2_format("42"), "42");
    assert_eq!(convert_to_ros2_format("hello"), "hello");
    assert_eq!(convert_to_ros2_format("3.14"), "3.14");

    // Test arrays - remove spaces after commas
    assert_eq!(convert_to_ros2_format("[true]"), "[true]");
    assert_eq!(convert_to_ros2_format("[false]"), "[false]");
    assert_eq!(convert_to_ros2_format("[true, false]"), "[true,false]");
    assert_eq!(
        convert_to_ros2_format("[true, true, false]"),
        "[true,true,false]"
    );
    assert_eq!(convert_to_ros2_format("[True, False]"), "[True,False]");

    // Test mixed arrays
    assert_eq!(convert_to_ros2_format("[true, 42]"), "[true,42]");
    assert_eq!(convert_to_ros2_format("[false, hello]"), "[false,hello]");

    // Test arrays with extra whitespace
    assert_eq!(
        convert_to_ros2_format("[  true  , false  ]"),
        "[true,false]"
    );
    assert_eq!(convert_to_ros2_format("[ 1 , 2 , 3 ]"), "[1,2,3]");

    // Test arrays already without spaces (should remain unchanged)
    assert_eq!(convert_to_ros2_format("[true,false]"), "[true,false]");
    assert_eq!(convert_to_ros2_format("[1,2,3]"), "[1,2,3]");
}

#[test]
fn test_detect_array_element_type() {
    use serde_yaml::Value;

    // Test boolean array
    let bool_array = vec![Value::Bool(true), Value::Bool(false)];
    assert_eq!(detect_array_element_type(&bool_array), "Boolean");

    // Test integer array
    let int_array = vec![
        Value::Number(serde_yaml::Number::from(1)),
        Value::Number(serde_yaml::Number::from(2)),
    ];
    assert_eq!(detect_array_element_type(&int_array), "Integer");

    // Test double array
    let double_array = vec![
        Value::Number(serde_yaml::Number::from(1.5)),
        Value::Number(serde_yaml::Number::from(2.5)),
    ];
    assert_eq!(detect_array_element_type(&double_array), "Double");

    // Test string array
    let string_array = vec![
        Value::String("hello".to_string()),
        Value::String("world".to_string()),
    ];
    assert_eq!(detect_array_element_type(&string_array), "String");

    // Test empty array
    let empty_array: Vec<Value> = vec![];
    assert_eq!(detect_array_element_type(&empty_array), "Array");
}
