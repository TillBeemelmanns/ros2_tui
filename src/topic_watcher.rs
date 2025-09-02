use std::{thread, time::Duration};
use crossbeam::channel::Sender;
use crate::ros::{self, TopicInfo};
use std::sync::Arc;
use once_cell::sync::Lazy;

// Shared tokio runtime to avoid expensive runtime creation
static SHARED_RUNTIME: Lazy<Arc<tokio::runtime::Runtime>> = Lazy::new(|| {
    Arc::new(tokio::runtime::Runtime::new().expect("Failed to create tokio runtime"))
});

use crate::ros::MeasurementStatus;

pub enum TopicMessage {
    TopicList(Vec<TopicInfo>),
    TopicDetails {
        topic_name: String,
        hz: Option<f64>,
        delay: Option<f64>,
    },
    TopicStatus {
        topic_name: String,
        hz_status: Option<MeasurementStatus>,
        delay_status: Option<MeasurementStatus>,
    },
    Error(String),
}

pub enum WatchMessage {
    UpdateWatchList(Vec<String>),
    StartContinuousMetrics(String),
    StopContinuousMetrics(String),
}

pub struct TopicWatcherHandle {
    _handle: thread::JoinHandle<()>,
}

impl TopicWatcherHandle {
    pub fn new(sender: Sender<TopicMessage>, refresh_interval: Duration) -> Self {
        let handle = thread::spawn(move || {
            let mut topic_watcher = TopicWatcher::new(sender, refresh_interval);
            topic_watcher.run();
        });
        
        Self { _handle: handle }
    }
}

struct TopicWatcher {
    sender: Sender<TopicMessage>,
    refresh_interval: Duration,
}

impl TopicWatcher {
    fn new(sender: Sender<TopicMessage>, refresh_interval: Duration) -> Self {
        Self {
            sender,
            refresh_interval,
        }
    }

    fn run(&mut self) {
        crate::debug_log("Topic watcher thread started");
        
        // Initial fetch
        crate::debug_log("Initial topic list fetch...");
        match self.fetch_topics() {
            Ok(topics) => {
                crate::debug_log(&format!("Initial fetch: {} topics successfully", topics.len()));
                if let Err(_) = self.sender.send(TopicMessage::TopicList(topics)) {
                    crate::debug_log("Topic receiver dropped, exiting topic watcher");
                    return;
                }
            }
            Err(e) => {
                crate::debug_log(&format!("Initial fetch failed: {}", e));
                if let Err(_) = self.sender.send(TopicMessage::Error(format!("Failed to fetch topics: {}", e))) {
                    return;
                }
            }
        }
        
        // Check if we should disable automatic refresh
        if self.refresh_interval.as_secs() == 0 {
            crate::debug_log("Topic refresh disabled (interval = 0), thread will sleep indefinitely");
            loop {
                thread::sleep(Duration::from_secs(3600)); // Sleep for 1 hour at a time
            }
        }
        
        // Regular refresh loop
        loop {
            crate::debug_log(&format!("Topic watcher sleeping for {:?}", self.refresh_interval));
            thread::sleep(self.refresh_interval);
            
            crate::debug_log("Fetching topic list...");
            match self.fetch_topics() {
                Ok(topics) => {
                    crate::debug_log(&format!("Fetched {} topics successfully", topics.len()));
                    if let Err(_) = self.sender.send(TopicMessage::TopicList(topics)) {
                        crate::debug_log("Topic receiver dropped, exiting topic watcher");
                        break;
                    }
                }
                Err(e) => {
                    crate::debug_log(&format!("Failed to fetch topics: {}", e));
                    if let Err(_) = self.sender.send(TopicMessage::Error(format!("Failed to fetch topics: {}", e))) {
                        break;
                    }
                }
            }
        }
    }

    fn fetch_topics(&self) -> Result<Vec<TopicInfo>, ros::TopicError> {
        // Use shared runtime to avoid expensive runtime creation
        crate::debug_log("Using shared tokio runtime for topic list fetch");
        let start_time = std::time::Instant::now();
        let result = SHARED_RUNTIME.block_on(ros::get_topic_list());
        let duration = start_time.elapsed();
        crate::debug_log(&format!("Topic list fetch took {:?}", duration));
        result
    }
}

pub struct TopicDetailWatcherHandle {
    _handle: thread::JoinHandle<()>,
    cleanup_sender: Sender<()>,
}

impl TopicDetailWatcherHandle {
    pub fn new(
        msg_sender: Sender<TopicMessage>,
        detail_refresh_interval: Duration,
    ) -> (Self, Sender<WatchMessage>) {
        let (watch_sender, watch_receiver) = crossbeam::channel::unbounded::<WatchMessage>();
        let (cleanup_sender, cleanup_receiver) = crossbeam::channel::bounded::<()>(1);
        
        let handle = thread::spawn(move || {
            let mut detail_watcher = TopicDetailWatcher::new(msg_sender, detail_refresh_interval);
            detail_watcher.run(watch_receiver, cleanup_receiver);
        });
        
        let watcher_handle = Self {
            _handle: handle,
            cleanup_sender,
        };
        
        (watcher_handle, watch_sender)
    }
    
    pub fn shutdown(&self) {
        crate::debug_log("Requesting TopicDetailWatcher shutdown");
        let _ = self.cleanup_sender.try_send(());
    }
}

use std::collections::HashMap;
use tokio::task::JoinHandle;

struct TopicDetailWatcher {
    sender: Sender<TopicMessage>,
    active_tasks: HashMap<String, (JoinHandle<()>, JoinHandle<()>)>, // Store task handles for Hz and Delay
}

impl TopicDetailWatcher {
    fn new(sender: Sender<TopicMessage>, _detail_refresh_interval: Duration) -> Self {
        Self {
            sender,
            active_tasks: HashMap::new(),
        }
    }

    fn run(&mut self, watch_receiver: crossbeam::channel::Receiver<WatchMessage>, cleanup_receiver: crossbeam::channel::Receiver<()>) {
        crate::debug_log("Detail watcher thread started with continuous streaming support");
        
        loop {
            // Check for cleanup signal
            if let Ok(_) = cleanup_receiver.try_recv() {
                crate::debug_log("Received cleanup signal, aborting all monitoring tasks");
                for (topic_name, (hz_handle, delay_handle)) in self.active_tasks.drain() {
                    crate::debug_log(&format!("Aborting monitoring tasks for topic: {}", topic_name));
                    hz_handle.abort();
                    delay_handle.abort();
                }
                break;
            }
            
            // Check for watch list updates
            while let Ok(watch_msg) = watch_receiver.try_recv() {
                match watch_msg {
                    WatchMessage::UpdateWatchList(new_watch_list) => {
                        crate::debug_log(&format!("Updating watched topics list, new list: {:?}", new_watch_list));
                        self.update_active_topics(new_watch_list);
                    }
                    WatchMessage::StartContinuousMetrics(topic_name) => {
                        crate::debug_log(&format!("Starting continuous metrics for topic: {}", topic_name));
                        if !self.active_tasks.contains_key(&topic_name) {
                            self.start_continuous_monitoring(topic_name);
                        }
                    }
                    WatchMessage::StopContinuousMetrics(topic_name) => {
                        crate::debug_log(&format!("Stopping continuous metrics for topic: {}", topic_name));
                        if let Some((hz_handle, delay_handle)) = self.active_tasks.remove(&topic_name) {
                            hz_handle.abort();
                            delay_handle.abort();
                        }
                    }
                }
            }

            thread::sleep(Duration::from_millis(100));
        }
        
        crate::debug_log("TopicDetailWatcher thread exiting");
    }

    fn update_active_topics(&mut self, new_watch_list: Vec<String>) {
        // Remove topics no longer being watched and abort their tasks
        let topics_to_remove: Vec<String> = self.active_tasks
            .keys()
            .filter(|topic| !new_watch_list.contains(topic))
            .cloned()
            .collect();
        
        for topic in topics_to_remove {
            if let Some((hz_handle, delay_handle)) = self.active_tasks.remove(&topic) {
                crate::debug_log(&format!("Aborting monitoring tasks for unwatched topic: {}", topic));
                hz_handle.abort();
                delay_handle.abort();
            }
        }
        
        // Start monitoring new topics
        for topic in new_watch_list {
            if !self.active_tasks.contains_key(&topic) {
                self.start_continuous_monitoring(topic);
            }
        }
    }

    fn start_continuous_monitoring(&mut self, topic_name: String) {
        crate::debug_log(&format!("Starting continuous monitoring for topic: {}", topic_name));
        
        // Spawn async task for Hz monitoring
        let topic_name_hz = topic_name.clone();
        let sender_hz = self.sender.clone();
        let hz_handle = SHARED_RUNTIME.spawn(async move {
            Self::monitor_topic_hz(topic_name_hz, sender_hz).await;
        });
        
        // Spawn async task for Delay monitoring  
        let topic_name_delay = topic_name.clone();
        let sender_delay = self.sender.clone();
        let delay_handle = SHARED_RUNTIME.spawn(async move {
            Self::monitor_topic_delay(topic_name_delay, sender_delay).await;
        });
        
        // Store the task handles so we can cancel them later
        self.active_tasks.insert(topic_name, (hz_handle, delay_handle));
    }

    async fn monitor_topic_hz(topic_name: String, sender: Sender<TopicMessage>) {
        use tokio::io::{AsyncBufReadExt, BufReader};
        
        crate::debug_log(&format!("Starting Hz monitoring task for: {}", topic_name));
        
        loop {
            // Start Hz process
            if let Ok(mut child) = ros::start_topic_hz_stream(&topic_name).await {
                if let Some(stdout) = child.stdout.take() {
                    let mut reader = BufReader::new(stdout).lines();
                    
                    while let Ok(Some(line)) = reader.next_line().await {
                        let (hz_value, hz_status) = ros::parse_hz_stream_line(&line).await;
                        
                        if let Some(hz) = hz_value {
                            crate::debug_log(&format!("Continuous Hz for {}: {:.2}", topic_name, hz));
                            
                            // Send both value and status together when we have a value
                            let _ = sender.send(TopicMessage::TopicDetails {
                                topic_name: topic_name.clone(),
                                hz: Some(hz),
                                delay: None,
                            });
                        } else {
                            // Check if topic is no longer published and exit completely
                            if matches!(hz_status, ros::MeasurementStatus::NotMeasuring) {
                                crate::debug_log(&format!("Topic {} is no longer being published, stopping Hz monitoring task", topic_name));
                                let _ = sender.send(TopicMessage::TopicStatus {
                                    topic_name: topic_name.clone(),
                                    hz_status: Some(hz_status),
                                    delay_status: None,
                                });
                                // Kill the process and exit the function completely
                                let _ = child.kill().await;
                                let _ = child.wait().await;
                                return; // Exit the entire monitoring task
                            }
                            
                            // Only send status updates when we don't have a value
                            let _ = sender.send(TopicMessage::TopicStatus {
                                topic_name: topic_name.clone(),
                                hz_status: Some(hz_status),
                                delay_status: None,
                            });
                        }
                    }
                }
                
                // Kill the process when done with reading
                let _ = child.kill().await;
                // Wait for process to fully terminate
                let _ = child.wait().await;
            }
            
            // Longer pause before restarting to avoid RCL context conflicts
            tokio::time::sleep(tokio::time::Duration::from_millis(2000)).await;
        }
    }

    async fn monitor_topic_delay(topic_name: String, sender: Sender<TopicMessage>) {
        use tokio::io::{AsyncBufReadExt, BufReader};
        
        crate::debug_log(&format!("Starting Delay monitoring task for: {}", topic_name));
        
        loop {
            // Start Delay process
            if let Ok(mut child) = ros::start_topic_delay_stream(&topic_name).await {
                // Read from both stdout and stderr, but sequentially to avoid complexity
                if let Some(stdout) = child.stdout.take() {
                    let mut reader = BufReader::new(stdout).lines();
                    
                    // First check stderr quickly for immediate error detection
                    if let Some(stderr) = child.stderr.take() {
                        let mut stderr_reader = BufReader::new(stderr).lines();
                        
                        // Try to read one line from stderr to check for immediate errors
                        match tokio::time::timeout(tokio::time::Duration::from_millis(500), stderr_reader.next_line()).await {
                            Ok(Ok(Some(line))) if line.contains("msg does not have header") => {
                                crate::debug_log(&format!("Detected no header for {} from stderr: '{}'", topic_name, line));
                                let _ = sender.send(TopicMessage::TopicStatus {
                                    topic_name: topic_name.clone(),
                                    hz_status: None,
                                    delay_status: Some(ros::MeasurementStatus::NoStamp),
                                });
                                // Kill the process and continue to next iteration
                                let _ = child.kill().await;
                                continue;
                            }
                            _ => {
                                // No immediate error, continue with stdout reading
                            }
                        }
                    }
                    
                    while let Ok(Some(line)) = reader.next_line().await {
                        let (delay_value, delay_status) = ros::parse_delay_stream_line(&line).await;
                        
                        if let Some(delay) = delay_value {
                            crate::debug_log(&format!("Continuous Delay for {}: {:.2}ms", topic_name, delay * 1000.0));
                            
                            // Send both value and status together when we have a value
                            let _ = sender.send(TopicMessage::TopicDetails {
                                topic_name: topic_name.clone(),
                                hz: None,
                                delay: Some(delay),
                            });
                        } else {
                            // Check if topic is no longer published and exit completely
                            if matches!(delay_status, ros::MeasurementStatus::NotMeasuring) {
                                crate::debug_log(&format!("Topic {} is no longer being published, stopping Delay monitoring task", topic_name));
                                let _ = sender.send(TopicMessage::TopicStatus {
                                    topic_name: topic_name.clone(),
                                    hz_status: None,
                                    delay_status: Some(delay_status),
                                });
                                // Kill the process and exit the function completely
                                let _ = child.kill().await;
                                let _ = child.wait().await;
                                return; // Exit the entire monitoring task
                            }
                            
                            // Only send status updates when we don't have a value
                            let _ = sender.send(TopicMessage::TopicStatus {
                                topic_name: topic_name.clone(),
                                hz_status: None,
                                delay_status: Some(delay_status),
                            });
                        }
                    }
                }
                
                // Kill the process when done
                let _ = child.kill().await;
                // Wait for process to fully terminate
                let _ = child.wait().await;
            }
            
            // Longer pause before restarting to avoid RCL context conflicts
            tokio::time::sleep(tokio::time::Duration::from_millis(2000)).await;
        }
    }
}