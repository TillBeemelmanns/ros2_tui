use tokio::process::Command;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub msg_type: String,
    pub publisher_count: usize,
    pub subscriber_count: usize,
    pub hz: Option<f64>,
    pub delay: Option<f64>,
    pub watched: bool,
    pub hz_status: MeasurementStatus,
    pub delay_status: MeasurementStatus,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MeasurementStatus {
    NotMeasuring,      // Topic not being watched
    Loading(usize),    // Loading with animation frame (0, 1, 2 for dots)
    HasValue,          // Successfully measured
    NoStamp,          // Message has no header/timestamp
}

impl Default for TopicInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            msg_type: String::new(),
            publisher_count: 0,
            subscriber_count: 0,
            hz: None,
            delay: None,
            watched: false,
            hz_status: MeasurementStatus::NotMeasuring,
            delay_status: MeasurementStatus::NotMeasuring,
        }
    }
}

#[derive(Debug)]
pub enum TopicError {
    Io(std::io::Error),
    ParseError(String),
}

impl fmt::Display for TopicError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TopicError::Io(e) => write!(f, "IO error: {}", e),
            TopicError::ParseError(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for TopicError {}

impl From<std::io::Error> for TopicError {
    fn from(error: std::io::Error) -> Self {
        TopicError::Io(error)
    }
}

pub async fn get_topic_list() -> Result<Vec<TopicInfo>, TopicError> {
    crate::debug_log("Executing: ros2 topic list -v --spin-time 0.5");
    let start = std::time::Instant::now();
    let output = Command::new("ros2")
        .arg("topic")
        .arg("list")
        .arg("-v")
        .arg("--spin-time")
        .arg("0.5")
        .output()
        .await?;
    let duration = start.elapsed();
    crate::debug_log(&format!("ros2 topic list -v took {:?}", duration));

    if !output.status.success() {
        return Err(TopicError::ParseError(format!("ros2 topic list failed: {}", String::from_utf8_lossy(&output.stderr))));
    }

    let output_str = String::from_utf8_lossy(&output.stdout);
    let topic_infos = parse_topic_list_verbose(&output_str);

    crate::debug_log(&format!("Parsed {} topics with publisher/subscriber counts", topic_infos.len()));

    Ok(topic_infos)
}



pub async fn start_topic_hz_stream(topic_name: &str) -> Result<tokio::process::Child, TopicError> {
    crate::debug_log(&format!("Starting continuous hz stream for topic: '{}'", topic_name));
    
    let child = Command::new("ros2")
        .arg("topic")
        .arg("hz")
        .arg(topic_name)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;
    
    Ok(child)
}

pub async fn parse_hz_stream_line(line: &str) -> (Option<f64>, MeasurementStatus) {
    parse_topic_hz(line)
}

pub async fn start_topic_delay_stream(topic_name: &str) -> Result<tokio::process::Child, TopicError> {
    crate::debug_log(&format!("Starting continuous delay stream for topic: '{}'", topic_name));
    
    let child = Command::new("ros2")
        .arg("topic")
        .arg("delay")
        .arg(topic_name)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;
    
    Ok(child)
}

pub async fn parse_delay_stream_line(line: &str) -> (Option<f64>, MeasurementStatus) {
    parse_topic_delay(line)
}


fn parse_topic_list_verbose(output: &str) -> Vec<TopicInfo> {
    use std::collections::HashMap;
    
    let mut topics: HashMap<String, TopicInfo> = HashMap::new();
    let mut current_section = "";
    
    for line in output.lines() {
        let line = line.trim();
        
        // Skip empty lines
        if line.is_empty() {
            continue;
        }
        
        // Check for section headers
        if line.starts_with("Published topics:") {
            current_section = "published";
            continue;
        } else if line.starts_with("Subscribed topics:") {
            current_section = "subscribed";
            continue;
        }
        
        // Skip lines that don't start with "* /"
        if !line.starts_with("* /") {
            continue;
        }
        
        // Parse format: " * /topic_name [msg/Type] X publisher(s)"
        let line_without_bullet = &line[2..].trim(); // Remove "* " prefix
        
        if let Some(bracket_start) = line_without_bullet.find('[') {
            let topic_name = line_without_bullet[..bracket_start].trim();
            if let Some(bracket_end) = line_without_bullet.find(']') {
                let msg_type = line_without_bullet[bracket_start + 1..bracket_end].trim();
                
                // Extract count after the bracket
                let after_bracket = &line_without_bullet[bracket_end + 1..].trim();
                
                // Parse count using regex
                let count_regex = Regex::new(r"(\d+)\s+(?:publisher|subscriber)s?").unwrap();
                let count = if let Some(caps) = count_regex.captures(after_bracket) {
                    if let Some(count_match) = caps.get(1) {
                        count_match.as_str().parse().unwrap_or(0)
                    } else {
                        0
                    }
                } else {
                    0
                };
            
                // Get or create topic info
                let topic_info = topics.entry(topic_name.to_string()).or_insert_with(|| TopicInfo {
                    name: topic_name.to_string(),
                    msg_type: msg_type.to_string(),
                    ..Default::default()
                });
                
                // Update counts based on current section
                match current_section {
                    "published" => topic_info.publisher_count = count,
                    "subscribed" => topic_info.subscriber_count = count,
                    _ => {}
                }
            }
        }
    }
    
    let mut topic_list: Vec<TopicInfo> = topics.into_values().collect();
    topic_list.sort_by(|a, b| a.name.cmp(&b.name)); // Sort alphabetically by topic name
    topic_list
}


fn parse_topic_hz(output: &str) -> (Option<f64>, MeasurementStatus) {
    // Check if topic is no longer being published
    if output.contains("does not appear to be published yet") || 
       output.contains("WARNING: topic") {
        crate::debug_log(&format!("Detected unpublished topic in Hz output: '{}'", output));
        return (None, MeasurementStatus::NotMeasuring);
    }
    
    let re = Regex::new(r"average rate: ([\d.]+)").unwrap();
    if let Some(caps) = re.captures(output) {
        if let Some(rate) = caps.get(1) {
            if let Ok(hz_val) = rate.as_str().parse() {
                return (Some(hz_val), MeasurementStatus::HasValue);
            }
        }
    }
    (None, MeasurementStatus::Loading(0)) // Still loading if no match
}

fn parse_topic_delay(output: &str) -> (Option<f64>, MeasurementStatus) {
    crate::debug_log(&format!("Parsing delay output: '{}'", output));
    
    // Check if topic is no longer being published
    if output.contains("does not appear to be published yet") || 
       output.contains("WARNING: topic") {
        crate::debug_log(&format!("Detected unpublished topic in Delay output: '{}'", output));
        return (None, MeasurementStatus::NotMeasuring);
    }
    
    // Check for "msg does not have header" case first
    if output.contains("msg does not have header") {
        crate::debug_log("Detected 'msg does not have header' - returning NoStamp");
        return (None, MeasurementStatus::NoStamp);
    }
    
    let re = Regex::new(r"average delay: ([\d.]+)").unwrap();
    if let Some(caps) = re.captures(output) {
        if let Some(delay) = caps.get(1) {
            if let Ok(delay_val) = delay.as_str().parse() {
                crate::debug_log(&format!("Parsed delay value: {}", delay_val));
                return (Some(delay_val), MeasurementStatus::HasValue);
            }
        }
    }
    crate::debug_log("No delay match found - returning Loading");
    (None, MeasurementStatus::Loading(0)) // Still loading if no match
}


#[cfg(test)]
mod tests {
    use super::*;


    #[test]
    fn test_parse_topic_hz() {
        assert_eq!(
            parse_topic_hz("average rate: 10.5"),
            (Some(10.5), MeasurementStatus::HasValue)
        );
        
        assert_eq!(
            parse_topic_hz("no data"),
            (None, MeasurementStatus::Loading(0))
        );
        
        assert_eq!(
            parse_topic_hz("WARNING: topic [/test] does not appear to be published yet"),
            (None, MeasurementStatus::NotMeasuring)
        );
    }

    #[test]
    fn test_parse_topic_delay() {
        assert_eq!(
            parse_topic_delay("average delay: 0.025"),
            (Some(0.025), MeasurementStatus::HasValue)
        );
        
        assert_eq!(
            parse_topic_delay("msg does not have header"),
            (None, MeasurementStatus::NoStamp)
        );
        
        assert_eq!(
            parse_topic_delay("no delay data"),
            (None, MeasurementStatus::Loading(0))
        );
        
        assert_eq!(
            parse_topic_delay("WARNING: topic [/test] does not appear to be published yet"),
            (None, MeasurementStatus::NotMeasuring)
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
}