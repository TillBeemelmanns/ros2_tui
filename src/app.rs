use crate::ros;
use crate::topic_watcher::{TopicMessage, TopicWatcherHandle, TopicDetailWatcherHandle, WatchMessage};
use crate::ros::MeasurementStatus;
use std::time::Duration;
use crossbeam::channel::{Receiver, Sender};

pub struct App {
    pub should_quit: bool,
    pub topics: Vec<ros::TopicInfo>,
    pub selected_topic_index: Option<usize>,
    pub scroll_offset: usize,
    pub error_message: Option<String>,
    pub message_receiver: Receiver<TopicMessage>,
    pub watch_sender: Sender<WatchMessage>,
    _topic_watcher: TopicWatcherHandle,
    detail_watcher: TopicDetailWatcherHandle,
}

impl App {
    pub fn new(refresh_interval: Duration, detail_refresh_interval: Duration) -> Self {
        crate::debug_log(&format!("Creating App with intervals: topic_refresh={:?}, detail_refresh={:?}", 
            refresh_interval, detail_refresh_interval));
        
        let (topic_sender, topic_receiver) = crossbeam::channel::unbounded();
        
        crate::debug_log("Creating topic watcher...");
        let topic_watcher = TopicWatcherHandle::new(topic_sender.clone(), refresh_interval);
        
        crate::debug_log("Creating detail watcher...");
        let (detail_watcher, watch_sender) = TopicDetailWatcherHandle::new(
            topic_sender,
            detail_refresh_interval,
        );
        
        Self {
            should_quit: false,
            topics: Vec::new(),
            selected_topic_index: None,
            scroll_offset: 0,
            error_message: None,
            message_receiver: topic_receiver,
            watch_sender,
            _topic_watcher: topic_watcher,
            detail_watcher,
        }
    }

    pub fn handle_message(&mut self, msg: TopicMessage) {
        match msg {
            TopicMessage::TopicList(mut topics) => {
                self.error_message = None;
                
                // Preserve watched status when updating topics
                for new_topic in &mut topics {
                    if let Some(existing) = self.topics.iter().find(|t| t.name == new_topic.name) {
                        new_topic.watched = existing.watched;
                        new_topic.hz = existing.hz;
                        new_topic.delay = existing.delay;
                        new_topic.hz_status = existing.hz_status.clone();
                        new_topic.delay_status = existing.delay_status.clone();
                    }
                }
                
                self.topics = topics;
                
                // If no topic is selected and we have topics, select the first one
                if self.selected_topic_index.is_none() && !self.topics.is_empty() {
                    self.selected_topic_index = Some(0);
                }
                
                // If selected index is out of bounds, adjust it
                if let Some(index) = self.selected_topic_index {
                    if index >= self.topics.len() {
                        self.selected_topic_index = if self.topics.is_empty() { None } else { Some(self.topics.len() - 1) };
                    }
                }
                
                // Update scroll position if needed
                self.update_scroll();
                
                // Send updated watched topics list to maintain monitoring
                self.send_watched_topics_list();
            }
            TopicMessage::TopicDetails { topic_name, hz, delay } => {
                // Find the topic and update its details (merge values instead of overwriting)
                if let Some(topic) = self.topics.iter_mut().find(|t| t.name == topic_name) {
                    if let Some(new_hz) = hz {
                        topic.hz = Some(new_hz);
                        topic.hz_status = MeasurementStatus::HasValue;
                    }
                    if let Some(new_delay) = delay {
                        topic.delay = Some(new_delay);
                        topic.delay_status = MeasurementStatus::HasValue;
                    }
                }
            }
            TopicMessage::TopicStatus { topic_name, hz_status, delay_status } => {
                // Find the topic and update its status (but don't override HasValue status)
                if let Some(topic) = self.topics.iter_mut().find(|t| t.name == topic_name) {
                    if let Some(new_hz_status) = hz_status {
                        // Only update status if it's not already HasValue, but allow NoStamp to override Loading
                        match (&topic.hz_status, &new_hz_status) {
                            (MeasurementStatus::HasValue, _) => {
                                // Don't override HasValue with anything
                            }
                            _ => {
                                // Update for all other cases (Loading -> NoStamp, Loading -> Loading, etc.)
                                topic.hz_status = new_hz_status;
                            }
                        }
                    }
                    if let Some(new_delay_status) = delay_status {
                        // Only update status if it's not already HasValue, but allow NoStamp to override Loading
                        match (&topic.delay_status, &new_delay_status) {
                            (MeasurementStatus::HasValue, _) => {
                                // Don't override HasValue with anything
                            }
                            _ => {
                                // Update for all other cases (Loading -> NoStamp, Loading -> Loading, etc.)
                                if matches!(new_delay_status, MeasurementStatus::NoStamp) {
                                    crate::debug_log(&format!("Setting delay status to NoStamp for topic: {}", topic_name));
                                }
                                topic.delay_status = new_delay_status;
                            }
                        }
                    }
                }
            }
            TopicMessage::Error(error) => {
                self.error_message = Some(error);
            }
        }
    }

    pub fn try_receive_messages(&mut self) {
        let mut message_count = 0;
        while let Ok(msg) = self.message_receiver.try_recv() {
            message_count += 1;
            match &msg {
                TopicMessage::TopicList(topics) => {
                    crate::debug_log(&format!("Received topic list with {} topics", topics.len()));
                }
                TopicMessage::TopicDetails { topic_name, hz, delay } => {
                    crate::debug_log(&format!("Received details for {}: hz={:?}, delay={:?}", topic_name, hz, delay));
                }
                TopicMessage::TopicStatus { topic_name, hz_status, delay_status } => {
                    crate::debug_log(&format!("Received status for {}: hz_status={:?}, delay_status={:?}", topic_name, hz_status, delay_status));
                }
                TopicMessage::Error(error) => {
                    crate::debug_log(&format!("Received error: {}", error));
                }
            }
            self.handle_message(msg);
        }
        if message_count > 0 {
            crate::debug_log(&format!("Processed {} messages in this cycle", message_count));
        }
    }

    pub fn on_key(&mut self, c: char) {
        match c {
            'q' => self.should_quit = true,
            'r' => {
                // Force refresh - the worker threads will handle this automatically
                // We could add a force refresh message here if needed
            }
            _ => {}
        }
    }

    pub fn select_next(&mut self) {
        if let Some(index) = self.selected_topic_index {
            if index < self.topics.len().saturating_sub(1) {
                self.selected_topic_index = Some(index + 1);
            }
        } else if !self.topics.is_empty() {
            self.selected_topic_index = Some(0);
        }
        self.update_scroll();
    }

    pub fn select_previous(&mut self) {
        if let Some(index) = self.selected_topic_index {
            if index > 0 {
                self.selected_topic_index = Some(index - 1);
            }
        } else if !self.topics.is_empty() {
            self.selected_topic_index = Some(self.topics.len() - 1);
        }
        self.update_scroll();
    }
    
    pub fn toggle_watch_current_topic(&mut self) {
        if let Some(index) = self.selected_topic_index {
            if let Some(topic) = self.topics.get_mut(index) {
                topic.watched = !topic.watched;
                
                if !topic.watched {
                    // Clear the metrics when unwatching and stop continuous measurements
                    topic.hz = None;
                    topic.delay = None;
                    topic.hz_status = MeasurementStatus::NotMeasuring;
                    topic.delay_status = MeasurementStatus::NotMeasuring;
                    crate::debug_log(&format!("Topic {} unwatched, stopping continuous measurements", topic.name));
                    let _ = self.watch_sender.send(WatchMessage::StopContinuousMetrics(topic.name.clone()));
                } else {
                    // Set loading state and start continuous Hz/Delay measurements when watching
                    topic.hz_status = MeasurementStatus::Loading(0);
                    topic.delay_status = MeasurementStatus::Loading(0);
                    crate::debug_log(&format!("Topic {} marked as watched, starting continuous measurements", topic.name));
                    let _ = self.watch_sender.send(WatchMessage::StartContinuousMetrics(topic.name.clone()));
                }
                
                // Send the complete list of watched topics
                self.send_watched_topics_list();
            }
        }
    }
    
    
    fn send_watched_topics_list(&self) {
        let watched_topics: Vec<String> = self.topics
            .iter()
            .filter(|topic| topic.watched)
            .map(|topic| topic.name.clone())
            .collect();
        
        let _ = self.watch_sender.send(WatchMessage::UpdateWatchList(watched_topics));
    }
    
    fn update_scroll(&mut self) {
        // This will be used by the UI to determine visible range
        // The scroll offset doesn't need to be updated here as the UI will handle it
    }
    
    pub fn update_loading_animation(&mut self) {
        // Update loading animation only for watched topics that are still in Loading state
        for topic in &mut self.topics {
            if topic.watched {
                // Only animate if status is specifically Loading (not HasValue or NoStamp)
                if let MeasurementStatus::Loading(frame) = &topic.hz_status {
                    topic.hz_status = MeasurementStatus::Loading((*frame + 1) % 4);
                }
                if let MeasurementStatus::Loading(frame) = &topic.delay_status {
                    topic.delay_status = MeasurementStatus::Loading((*frame + 1) % 4);
                }
            }
        }
    }
    
    pub fn shutdown(&self) {
        crate::debug_log("App shutdown requested - cleaning up background tasks");
        self.detail_watcher.shutdown();
    }
    
    pub fn get_visible_topics(&self, table_height: usize) -> (usize, Vec<&ros::TopicInfo>) {
        if self.topics.is_empty() {
            return (0, Vec::new());
        }
        
        let selected_index = self.selected_topic_index.unwrap_or(0);
        let total_topics = self.topics.len();
        
        // Calculate scroll offset to keep selected item visible
        let scroll_offset = if selected_index < table_height {
            0
        } else if selected_index < total_topics.saturating_sub(table_height) {
            selected_index.saturating_sub(table_height / 2)
        } else {
            total_topics.saturating_sub(table_height)
        };
        
        let visible_topics: Vec<&ros::TopicInfo> = self.topics
            .iter()
            .skip(scroll_offset)
            .take(table_height)
            .collect();
            
        (scroll_offset, visible_topics)
    }
}
