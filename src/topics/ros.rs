//! Topic data shared by the topics app, its watcher and its UI. All ROS
//! access goes through the helper in `watcher.rs`.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

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
