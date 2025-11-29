use once_cell::sync::Lazy;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt;
use tokio::process::Command;

// Pre-compiled regexes for better performance
static COUNT_REGEX: Lazy<Regex> =
    Lazy::new(|| Regex::new(r"(\d+)\s+(?:publisher|subscriber)s?").unwrap());
static HZ_REGEX: Lazy<Regex> = Lazy::new(|| Regex::new(r"average rate: ([\d.]+)").unwrap());
static DELAY_REGEX: Lazy<Regex> = Lazy::new(|| Regex::new(r"average delay: ([\d.]+)").unwrap());
static STD_DEV_REGEX: Lazy<Regex> = Lazy::new(|| Regex::new(r"std dev: ([\d.]+)s").unwrap());

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub msg_type: String,
    pub publisher_count: usize,
    pub subscriber_count: usize,
    pub hz: Option<f64>,
    pub hz_std_dev: Option<f64>,
    pub delay: Option<f64>,
    pub delay_std_dev: Option<f64>,
    pub watched: bool,
    pub hz_status: MeasurementStatus,
    pub delay_status: MeasurementStatus,
    pub hz_history: VecDeque<f64>,
    pub hz_std_dev_history: VecDeque<f64>,
    pub delay_history: VecDeque<f64>,
    pub delay_std_dev_history: VecDeque<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MeasurementStatus {
    NotMeasuring,   // Topic not being watched
    Loading(usize), // Loading with animation frame (0, 1, 2 for dots)
    HasValue,       // Successfully measured
    NoStamp,        // Message has no header/timestamp
}

impl Default for TopicInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            msg_type: String::new(),
            publisher_count: 0,
            subscriber_count: 0,
            hz: None,
            hz_std_dev: None,
            delay: None,
            delay_std_dev: None,
            watched: false,
            hz_status: MeasurementStatus::NotMeasuring,
            delay_status: MeasurementStatus::NotMeasuring,
            hz_history: VecDeque::new(),
            hz_std_dev_history: VecDeque::new(),
            delay_history: VecDeque::new(),
            delay_std_dev_history: VecDeque::new(),
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
        return Err(TopicError::ParseError(format!(
            "ros2 topic list failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )));
    }

    let output_str = String::from_utf8_lossy(&output.stdout);
    let topic_infos = parse_topic_list_verbose(&output_str);

    crate::debug_log(&format!(
        "Parsed {} topics with publisher/subscriber counts",
        topic_infos.len()
    ));

    Ok(topic_infos)
}

pub async fn start_topic_hz_stream(topic_name: &str) -> Result<tokio::process::Child, TopicError> {
    crate::debug_log(&format!(
        "Starting continuous hz stream for topic: '{}'",
        topic_name
    ));

    let child = Command::new("ros2")
        .arg("topic")
        .arg("hz")
        .arg(topic_name)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;

    Ok(child)
}

pub async fn parse_hz_stream_line(line: &str) -> (Option<f64>, Option<f64>, MeasurementStatus) {
    parse_topic_hz(line)
}

pub async fn start_topic_delay_stream(
    topic_name: &str,
    use_sim_time: bool,
) -> Result<tokio::process::Child, TopicError> {
    crate::debug_log(&format!(
        "Starting continuous delay stream for topic: '{}' (sim_time={})",
        topic_name, use_sim_time
    ));

    let mut command = Command::new("ros2");
    command.arg("topic").arg("delay");

    if use_sim_time {
        command.arg("--use-sim-time");
    }

    let child = command
        .arg(topic_name)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;

    Ok(child)
}

pub async fn parse_delay_stream_line(line: &str) -> (Option<f64>, Option<f64>, MeasurementStatus) {
    parse_topic_delay(line)
}

pub async fn start_topic_echo_stream(
    topic_name: &str,
) -> Result<tokio::process::Child, TopicError> {
    crate::debug_log(&format!("Starting echo stream for topic: '{}'", topic_name));

    let child = Command::new("ros2")
        .arg("topic")
        .arg("echo")
        .arg(topic_name)
        .arg("--no-arr") // Don't print array contents
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;

    Ok(child)
}

pub fn parse_topic_list_verbose(output: &str) -> Vec<TopicInfo> {
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

                // Parse count using pre-compiled regex
                let count = if let Some(caps) = COUNT_REGEX.captures(after_bracket) {
                    if let Some(count_match) = caps.get(1) {
                        count_match.as_str().parse().unwrap_or(0)
                    } else {
                        0
                    }
                } else {
                    0
                };

                // Get or create topic info
                let topic_info =
                    topics
                        .entry(topic_name.to_string())
                        .or_insert_with(|| TopicInfo {
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

pub fn parse_topic_hz(output: &str) -> (Option<f64>, Option<f64>, MeasurementStatus) {
    // Check if topic is no longer being published
    if output.contains("does not appear to be published yet") || output.contains("WARNING: topic") {
        crate::debug_log(&format!(
            "Detected unpublished topic in Hz output: '{}'",
            output
        ));
        return (None, None, MeasurementStatus::NotMeasuring);
    }

    let mut hz_val = None;
    let mut std_dev_val = None;

    // Parse Hz value using pre-compiled regex
    if let Some(caps) = HZ_REGEX.captures(output) {
        if let Some(rate) = caps.get(1) {
            if let Ok(hz) = rate.as_str().parse() {
                hz_val = Some(hz);
            }
        }
    }

    // Parse std dev value - this might be on the same line or a different line
    if let Some(caps) = STD_DEV_REGEX.captures(output) {
        if let Some(std_dev) = caps.get(1) {
            if let Ok(std_dev) = std_dev.as_str().parse() {
                std_dev_val = Some(std_dev);
            }
        }
    }

    // Return Hz data when available, std dev when available
    // We'll handle the multi-line case at the watcher level
    if hz_val.is_some() {
        (hz_val, std_dev_val, MeasurementStatus::HasValue)
    } else if std_dev_val.is_some() {
        // This is a std dev line without Hz, return it separately
        (None, std_dev_val, MeasurementStatus::Loading(0))
    } else {
        (None, None, MeasurementStatus::Loading(0)) // Still loading if no match
    }
}

pub fn parse_topic_delay(output: &str) -> (Option<f64>, Option<f64>, MeasurementStatus) {
    crate::debug_log(&format!("Parsing delay output: '{}'", output));

    // Check if topic is no longer being published
    if output.contains("does not appear to be published yet") || output.contains("WARNING: topic") {
        crate::debug_log(&format!(
            "Detected unpublished topic in Delay output: '{}'",
            output
        ));
        return (None, None, MeasurementStatus::NotMeasuring);
    }

    // Check for "msg does not have header" case first
    if output.contains("msg does not have header") {
        crate::debug_log("Detected 'msg does not have header' - returning NoStamp");
        return (None, None, MeasurementStatus::NoStamp);
    }

    let mut delay_val = None;
    let mut std_dev_val = None;

    // Parse delay value using pre-compiled regex
    if let Some(caps) = DELAY_REGEX.captures(output) {
        if let Some(delay) = caps.get(1) {
            if let Ok(delay) = delay.as_str().parse() {
                delay_val = Some(delay);
                crate::debug_log(&format!("Parsed delay value: {}", delay));
            }
        }
    }

    // Parse std dev value - this might be on the same line or a different line
    if let Some(caps) = STD_DEV_REGEX.captures(output) {
        if let Some(std_dev) = caps.get(1) {
            if let Ok(std_dev) = std_dev.as_str().parse() {
                std_dev_val = Some(std_dev);
                crate::debug_log(&format!("Parsed delay std dev value: {}", std_dev));
            }
        }
    }

    // Return delay data when available, std dev when available
    if delay_val.is_some() {
        (delay_val, std_dev_val, MeasurementStatus::HasValue)
    } else if std_dev_val.is_some() {
        // This is a std dev line without delay, return it separately
        (None, std_dev_val, MeasurementStatus::Loading(0))
    } else {
        crate::debug_log("No delay match found - returning Loading");
        (None, None, MeasurementStatus::Loading(0)) // Still loading if no match
    }
}
