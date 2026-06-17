use serde::{Deserialize, Serialize};
use serde_yaml_ng::Value;
use std::fmt;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ParamInfo {
    pub node_name: String,
    pub param_name: String,
    pub value: Option<String>,
    pub param_type: String,
}

#[derive(Debug)]
pub enum ParamError {
    Io(std::io::Error),
    ParseError(String),
    InvalidType(String),
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ParamError::Io(e) => write!(f, "IO error: {}", e),
            ParamError::ParseError(s) => write!(f, "Parse error: {}", s),
            ParamError::InvalidType(s) => write!(f, "Type validation error: {}", s),
        }
    }
}

impl From<std::io::Error> for ParamError {
    fn from(err: std::io::Error) -> Self {
        ParamError::Io(err)
    }
}

pub async fn get_param_list() -> Result<Vec<ParamInfo>, ParamError> {
    let start_time = std::time::Instant::now();

    crate::debug_log("Executing: ros2 param list");

    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "list"],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    let _duration = start_time.elapsed();
    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);

    crate::debug_log(&format!("Command exit status: {}", output.status));
    crate::debug_log(&format!("Command stderr: '{}'", stderr));
    crate::debug_log(&format!("Command stdout: '{}'", stdout));

    if !output.status.success() {
        crate::debug_log(&format!(
            "ros2 param list command failed with status: {}",
            output.status
        ));
        return Err(ParamError::ParseError(format!(
            "ros2 param list failed: {} - stderr: {}",
            output.status, stderr
        )));
    }

    crate::debug_log(&format!("ros2 param list raw output: '{}'", stdout));
    let params = parse_param_list(&stdout)?;
    crate::debug_log(&format!("Parsed {} parameters from output", params.len()));

    Ok(params)
}

pub fn parse_param_list(output: &str) -> Result<Vec<ParamInfo>, ParamError> {
    let mut params = Vec::new();
    let mut current_node = String::new();

    crate::debug_log(&format!(
        "Parsing parameter list from {} lines",
        output.lines().count()
    ));

    for (line_num, line) in output.lines().enumerate() {
        let original_line = line;
        let trimmed_line = line.trim();
        crate::debug_log(&format!(
            "Line {}: '{}' (original: '{}')",
            line_num, trimmed_line, original_line
        ));

        if trimmed_line.is_empty()
            || trimmed_line.contains("WARN")
            || trimmed_line.contains("ERROR")
            || trimmed_line.starts_with("20")
        {
            // Added checks for WARN, ERROR, and timestamp
            crate::debug_log(&format!(
                "  -> Skipping non-parameter line: '{}'",
                trimmed_line
            ));
            continue;
        }

        // Check if this is a node name (starts with / and ends with :)
        if trimmed_line.starts_with('/') && trimmed_line.ends_with(':') {
            // Remove the trailing ':'
            current_node = trimmed_line[0..trimmed_line.len() - 1].to_string();
            crate::debug_log(&format!("  -> Found node: '{}'", current_node));
        } else if !current_node.is_empty() && original_line.starts_with(' ') {
            // This is a parameter name (indented under the node) - check original line for indentation
            let param_name = trimmed_line.to_string();
            crate::debug_log(&format!(
                "  -> Found parameter '{}' for node '{}'",
                param_name, current_node
            ));

            // Handle namespaced parameters (e.g., "ai_module.batch_size")
            if param_name.contains('.') {
                create_namespace_hierarchy(&mut params, &current_node, &param_name);
            } else {
                // Regular parameter (no namespace)
                params.push(ParamInfo {
                    node_name: current_node.clone(),
                    param_name,
                    value: None,
                    param_type: String::new(),
                });
            }
        } else {
            crate::debug_log(&format!("  -> Unmatched line: '{}'", trimmed_line));
        }
    }

    crate::debug_log(&format!(
        "Finished parsing - found {} parameters",
        params.len()
    ));
    Ok(params)
}

fn create_namespace_hierarchy(params: &mut Vec<ParamInfo>, node_name: &str, param_name: &str) {
    let parts: Vec<&str> = param_name.split('.').collect();

    // Create namespace entries for each level of hierarchy
    for i in 0..parts.len() {
        let current_path = parts[0..=i].join(".");
        let is_leaf = i == parts.len() - 1;

        // Check if this path already exists
        let exists = params
            .iter()
            .any(|p| p.node_name == node_name && p.param_name == current_path);

        if !exists {
            if is_leaf {
                // This is the actual parameter
                crate::debug_log(&format!("    -> Created parameter: '{}'", current_path));
                params.push(ParamInfo {
                    node_name: node_name.to_string(),
                    param_name: current_path,
                    value: None,
                    param_type: String::new(),
                });
            } else {
                // This is a namespace container
                crate::debug_log(&format!("    -> Created namespace: '{}'", current_path));
                params.push(ParamInfo {
                    node_name: node_name.to_string(),
                    param_name: current_path,
                    value: Some("namespace".to_string()),
                    param_type: "Namespace".to_string(),
                });
            }
        }
    }
}

/// Convert array values to ROS2-compatible format by removing spaces after commas
/// ROS2 treats spaces in arrays as separate command-line arguments, causing parse errors
/// - [true, false] -> [true,false]
/// - [1, 2, 3] -> [1,2,3]
pub fn convert_to_ros2_format(value: &str) -> String {
    // Handle array values like [true, false] -> [true,false] (remove spaces after commas)
    if value.starts_with('[') && value.ends_with(']') {
        let inner = &value[1..value.len() - 1]; // Remove brackets
        let elements: Vec<String> = inner
            .split(',')
            .map(|s| s.trim().to_string()) // Trim whitespace from each element
            .collect();
        format!("[{}]", elements.join(",")) // Join without spaces
    } else {
        // Single values don't need modification
        value.to_string()
    }
}

pub async fn set_param_value(
    node_name: &str,
    param_name: &str,
    value: &str,
) -> Result<String, ParamError> {
    // Normalise array values to ROS2's compact form (e.g. [1,2,3]). Each
    // argument is passed directly to the process, so values are never split or
    // interpreted by a shell.
    let ros2_value = convert_to_ros2_format(value);
    crate::debug_log(&format!(
        "Executing: ros2 param set {} {} {}",
        node_name, param_name, ros2_value
    ));

    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "set", node_name, param_name, &ros2_value],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Check ROS2's actual response
    if stdout.contains("Set parameter successful") {
        return Ok("Set parameter successful".to_string());
    }

    // If we get a failure, return the original error message from stdout
    if stdout.contains("Setting parameter failed") {
        let error_msg = stdout
            .lines()
            .find(|line| line.contains("Setting parameter failed"))
            .unwrap_or("Setting parameter failed")
            .to_string();
        return Err(ParamError::ParseError(error_msg));
    }

    // If no clear success/failure indication, check stderr
    if !stderr.is_empty() {
        return Err(ParamError::ParseError(stderr.to_string()));
    }

    // Fallback error
    Err(ParamError::ParseError(
        "Unknown parameter setting result".to_string(),
    ))
}

pub async fn dump_params(node_name: &str, file_path: &str) -> Result<(), ParamError> {
    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "dump", node_name],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(ParamError::ParseError(format!(
            "ros2 param dump failed: {}",
            stderr
        )));
    }

    // Write the captured YAML to the target file ourselves rather than relying
    // on a shell `>` redirect, so paths with spaces or shell metacharacters are
    // handled safely.
    tokio::fs::write(file_path, &output.stdout).await?;

    Ok(())
}

pub async fn load_params(node_name: &str, file_path: &str) -> Result<(), ParamError> {
    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "load", node_name, file_path],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(ParamError::ParseError(format!(
            "ros2 param load failed: {}",
            stderr
        )));
    }

    Ok(())
}

pub async fn get_params_for_node(node_name: &str) -> Result<Vec<ParamInfo>, ParamError> {
    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "list", node_name],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(ParamError::ParseError(format!(
            "ros2 param list failed: {}",
            stderr
        )));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let mut params = Vec::new();

    for line in stdout.lines() {
        let line = line.trim();
        if !line.is_empty() && !line.starts_with(node_name) {
            params.push(ParamInfo {
                node_name: node_name.to_string(),
                param_name: line.to_string(),
                value: None,
                param_type: String::new(),
            });
        }
    }

    Ok(params)
}

// New improved implementation using ros2 param dump
pub async fn get_node_list() -> Result<Vec<String>, ParamError> {
    crate::debug_log("Executing: ros2 node list");

    let output = crate::common::run_with_timeout(
        "ros2",
        &["node", "list"],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);

    crate::debug_log(&format!("ros2 node list exit status: {}", output.status));
    crate::debug_log(&format!("ros2 node list stderr: '{}'", stderr));
    crate::debug_log(&format!("ros2 node list stdout: '{}'", stdout));

    if !output.status.success() {
        return Err(ParamError::ParseError(format!(
            "ros2 node list failed: {} - stderr: {}",
            output.status, stderr
        )));
    }

    let nodes: Vec<String> = stdout
        .lines()
        .map(|line| line.trim())
        .filter(|line| !line.is_empty() && line.starts_with('/'))
        .map(|line| line.to_string())
        .collect();

    crate::debug_log(&format!("Found {} nodes", nodes.len()));
    Ok(nodes)
}

pub async fn get_node_params_dump(node_name: &str) -> Result<Vec<ParamInfo>, ParamError> {
    crate::debug_log(&format!("Executing: ros2 param dump {}", node_name));

    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "dump", node_name],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);

    crate::debug_log(&format!("ros2 param dump exit status: {}", output.status));
    crate::debug_log(&format!("ros2 param dump stderr: '{}'", stderr));
    crate::debug_log(&format!("ros2 param dump stdout length: {}", stdout.len()));

    if !output.status.success() {
        // Node might not have parameters, that's OK
        crate::debug_log(&format!(
            "Node {} has no parameters or dump failed: {}",
            node_name, stderr
        ));
        return Ok(Vec::new());
    }

    parse_param_dump_yaml(node_name, &stdout)
}

fn parse_param_dump_yaml(
    node_name: &str,
    yaml_content: &str,
) -> Result<Vec<ParamInfo>, ParamError> {
    crate::debug_log(&format!("Parsing YAML dump for node: {}", node_name));

    if yaml_content.trim().is_empty() {
        crate::debug_log("Empty YAML content");
        return Ok(Vec::new());
    }

    // Parse YAML
    let yaml_value: Value = serde_yaml_ng::from_str(yaml_content)
        .map_err(|e| ParamError::ParseError(format!("Failed to parse YAML: {}", e)))?;

    let mut params = Vec::new();

    // The YAML structure is: /node_name: ros__parameters: { param1: value1, param2: value2, ... }
    if let Some(node_data) = yaml_value.get(node_name) {
        if let Some(ros_params) = node_data.get("ros__parameters") {
            if let Some(param_map) = ros_params.as_mapping() {
                parse_nested_params(&mut params, node_name, "", param_map);
            }
        }
    }

    crate::debug_log(&format!(
        "Parsed {} parameters from YAML dump",
        params.len()
    ));
    Ok(params)
}

fn parse_nested_params(
    params: &mut Vec<ParamInfo>,
    node_name: &str,
    prefix: &str,
    param_map: &serde_yaml_ng::Mapping,
) {
    for (param_name, param_value) in param_map {
        if let Some(param_name_str) = param_name.as_str() {
            let full_param_name = if prefix.is_empty() {
                param_name_str.to_string()
            } else {
                format!("{}.{}", prefix, param_name_str)
            };

            if let Some(nested_map) = param_value.as_mapping() {
                // This is a nested namespace - recurse into it
                crate::debug_log(&format!("Found namespace: {}", full_param_name));
                parse_nested_params(params, node_name, &full_param_name, nested_map);
            } else {
                // This is an actual parameter value
                let (value_str, type_str) = extract_value_and_type(param_value);
                crate::debug_log(&format!(
                    "Parsed param: {} = {} ({})",
                    full_param_name, value_str, type_str
                ));

                params.push(ParamInfo {
                    node_name: node_name.to_string(),
                    param_name: full_param_name,
                    value: Some(value_str),
                    param_type: type_str,
                });
            }
        }
    }
}

/// Detect the element type of an array to match ROS2's individual param get behavior
/// Returns the same type names that ROS2 uses: "Boolean", "Integer", "Double", "String"
pub fn detect_array_element_type(seq: &[Value]) -> String {
    if seq.is_empty() {
        return "Array".to_string();
    }

    // Check the first element to determine array type
    // ROS2 arrays are homogeneous, so all elements should be the same type
    match &seq[0] {
        Value::Bool(_) => "Boolean".to_string(),
        Value::Number(n) => {
            if n.is_i64() {
                "Integer".to_string()
            } else {
                "Double".to_string()
            }
        }
        Value::String(_) => "String".to_string(),
        _ => "Array".to_string(), // Fallback for complex types
    }
}

fn format_array_value(seq: &[Value]) -> String {
    if seq.is_empty() {
        return "[]".to_string();
    }

    // Check if we should truncate for very long arrays
    const MAX_DISPLAY_ITEMS: usize = 8;
    const MAX_TOTAL_LENGTH: usize = 100;

    let mut elements = Vec::new();
    let mut total_length = 2; // For the brackets []
    let mut truncated = false;

    for (i, item) in seq.iter().enumerate() {
        if i >= MAX_DISPLAY_ITEMS {
            truncated = true;
            break;
        }

        let item_str = match item {
            Value::Bool(b) => b.to_string(),
            Value::Number(n) => n.to_string(),
            Value::String(s) => format!("\"{}\"", s),
            _ => format!("{:?}", item),
        };

        // Check if adding this item would exceed our length limit
        let separator_len = if i == 0 { 0 } else { 2 }; // ", "
        if total_length + separator_len + item_str.len() > MAX_TOTAL_LENGTH {
            truncated = true;
            break;
        }

        elements.push(item_str);
        total_length += separator_len + elements.last().unwrap().len();
    }

    let mut result = format!("[{}]", elements.join(", "));
    if truncated {
        // Remove the closing bracket and add ellipsis
        result.pop();
        result.push_str(", ...]");
    }

    result
}

fn extract_value_and_type(value: &Value) -> (String, String) {
    match value {
        Value::Bool(b) => (b.to_string(), "Boolean".to_string()),
        Value::Number(n) => {
            if n.is_i64() {
                (n.as_i64().unwrap().to_string(), "Integer".to_string())
            } else if n.is_f64() {
                let f_val = n.as_f64().unwrap();
                let f_str = f_val.to_string();
                // Ensure doubles always show with decimal point for type clarity
                if f_str.contains('.') {
                    (f_str, "Double".to_string())
                } else {
                    (format!("{}.0", f_str), "Double".to_string())
                }
            } else {
                (n.to_string(), "Number".to_string())
            }
        }
        Value::String(s) => (s.clone(), "String".to_string()),
        Value::Sequence(seq) => {
            let formatted_array = format_array_value(seq);
            let array_type = detect_array_element_type(seq);
            (formatted_array, array_type)
        }
        Value::Mapping(_) => (
            serde_yaml_ng::to_string(value)
                .unwrap_or_default()
                .trim()
                .to_string(),
            "Object".to_string(),
        ),
        Value::Null => ("null".to_string(), "Null".to_string()),
        _ => (format!("{:?}", value), "Unknown".to_string()),
    }
}

// New improved get_param_list_with_values that gets all params with values in fewer ROS calls
pub async fn get_param_list_with_values() -> Result<Vec<ParamInfo>, ParamError> {
    crate::debug_log("Starting improved parameter fetch using ros2 param dump");

    // Step 1: Get list of all nodes
    let nodes = get_node_list().await?;
    crate::debug_log(&format!("Found {} nodes to query", nodes.len()));

    // Step 2: Get parameters for each node using dump
    let mut all_params = Vec::new();

    for node_name in nodes {
        match get_node_params_dump(&node_name).await {
            Ok(mut node_params) => {
                crate::debug_log(&format!(
                    "Node {} has {} parameters",
                    node_name,
                    node_params.len()
                ));
                all_params.append(&mut node_params);
            }
            Err(e) => {
                crate::debug_log(&format!(
                    "Failed to get parameters for node {}: {}",
                    node_name, e
                ));
                // Continue with other nodes
            }
        }
    }

    crate::debug_log(&format!("Total parameters collected: {}", all_params.len()));
    Ok(all_params)
}

/// Get parameters for a single node using ros2 param dump (more efficient than individual gets)
pub async fn get_node_params_with_values(node_name: &str) -> Result<Vec<ParamInfo>, ParamError> {
    crate::debug_log(&format!(
        "Getting parameters for node {} using ros2 param dump",
        node_name
    ));

    // Use ros2 param dump to get all parameters for this specific node
    let output = crate::common::run_with_timeout(
        "ros2",
        &["param", "dump", node_name],
        crate::common::ROS2_COMMAND_TIMEOUT,
    )
    .await?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        return Err(ParamError::ParseError(format!(
            "ros2 param dump failed for {}: {}",
            node_name, stderr
        )));
    }

    // Parse the YAML output
    let yaml_value: Value = serde_yaml_ng::from_str(&stdout).map_err(|e| {
        ParamError::ParseError(format!("Failed to parse YAML for {}: {}", node_name, e))
    })?;

    let mut params = Vec::new();

    // Process the YAML structure to extract parameters
    if let Value::Mapping(node_map) = yaml_value {
        if let Some((_, node_params)) = node_map.iter().next() {
            if let Some(nested_map) = node_params.as_mapping() {
                // Check if there's a ros__parameters key (single node dump format)
                if let Some(ros_params) = nested_map.get("ros__parameters") {
                    if let Some(ros_params_map) = ros_params.as_mapping() {
                        // Parse from ros__parameters, stripping the prefix
                        parse_nested_params(&mut params, node_name, "", ros_params_map);
                    }
                } else {
                    // Direct parameter format (multi-node dump format)
                    parse_nested_params(&mut params, node_name, "", nested_map);
                }
            }
        }
    }

    crate::debug_log(&format!(
        "Node {} has {} parameters",
        node_name,
        params.len()
    ));
    Ok(params)
}

/// Get a single parameter value using ros2 param dump (more reliable than ros2 param get)
pub async fn get_single_param_value(
    node_name: &str,
    param_name: &str,
) -> Result<(String, String), ParamError> {
    crate::debug_log(&format!(
        "Getting parameter {}/{} using ros2 param dump",
        node_name, param_name
    ));

    // Get all parameters for the node
    let params = get_node_params_with_values(node_name).await?;

    // Find the specific parameter
    for param in params {
        if param.param_name == param_name {
            if let Some(value) = param.value {
                return Ok((value, param.param_type));
            }
        }
    }

    Err(ParamError::ParseError(format!(
        "Parameter {}/{} not found",
        node_name, param_name
    )))
}
