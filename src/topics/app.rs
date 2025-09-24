use super::ros;
use super::ros::MeasurementStatus;
use super::watcher::{TopicDetailWatcherHandle, TopicMessage, TopicWatcherHandle, WatchMessage};
use crate::common::{TopicTree, TopicTreeItem};
use crossbeam::channel::{Receiver, Sender};
use std::collections::HashMap;
use std::time::Duration;

pub const CHARTS_MAX_DATA_POINTS: usize = 100;

#[derive(PartialEq, Debug)]
pub enum AppMode {
    TopicList,
    TopicDetail,
    Search,
    Help,
}

#[derive(PartialEq, Debug)]
pub enum DetailPaneFocus {
    Info,
    Echo,
}

pub struct App {
    pub should_quit: bool,
    pub mode: AppMode,

    pub topic_map: HashMap<String, ros::TopicInfo>,

    pub master_tree: TopicTree,
    pub visible_items: Vec<TopicTreeItem>,

    pub selected_index: usize,
    pub scroll_offset: usize,

    pub filter_text: String,
    pub search_text: String,

    pub selected_topic_name: Option<String>,
    pub detail_scroll_offset: usize,
    pub detail_focus: DetailPaneFocus,

    pub echo_content: Vec<String>,
    pub is_echoing: bool,
    pub echo_scroll_offset: usize,

    pub error_message: Option<String>,
    pub last_animation_update: std::time::Instant,

    pub expansion_state: HashMap<String, bool>,
    pub use_sim_time: bool,

    message_receiver: Receiver<TopicMessage>,
    watch_sender: Sender<WatchMessage>,
    _topic_watcher: TopicWatcherHandle,
    detail_watcher: TopicDetailWatcherHandle,
}

impl App {
    pub fn new(refresh_interval: Duration, _detail_refresh_interval: Duration) -> Self {
        let (topic_sender, topic_receiver) = crossbeam::channel::unbounded();

        let topic_watcher = TopicWatcherHandle::new(topic_sender.clone(), refresh_interval);
        let (detail_watcher, watch_sender) = TopicDetailWatcherHandle::new(topic_sender);

        Self {
            should_quit: false,
            mode: AppMode::TopicList,
            topic_map: HashMap::new(),
            master_tree: TopicTree::new(),
            visible_items: Vec::new(),
            selected_index: 0,
            scroll_offset: 0,
            filter_text: String::new(),
            search_text: String::new(),
            selected_topic_name: None,
            detail_scroll_offset: 0,
            detail_focus: DetailPaneFocus::Info,
            echo_content: Vec::new(),
            is_echoing: false,
            echo_scroll_offset: 0,
            error_message: None,
            last_animation_update: std::time::Instant::now(),
            expansion_state: HashMap::new(),
            use_sim_time: false,
            message_receiver: topic_receiver,
            watch_sender,
            _topic_watcher: topic_watcher,
            detail_watcher,
        }
    }

    pub fn handle_message(&mut self, msg: TopicMessage) {
        match msg {
            TopicMessage::TopicList(topics) => {
                self.error_message = None;
                let mut new_map = topics
                    .into_iter()
                    .map(|t| (t.name.clone(), t))
                    .collect::<HashMap<_, _>>();
                for (name, new_topic) in new_map.iter_mut() {
                    if let Some(existing) = self.topic_map.get(name) {
                        new_topic.watched = existing.watched;
                        new_topic.hz = existing.hz;
                        new_topic.hz_std_dev = existing.hz_std_dev;
                        new_topic.delay = existing.delay;
                        new_topic.delay_std_dev = existing.delay_std_dev;
                        new_topic.hz_status = existing.hz_status.clone();
                        new_topic.delay_status = existing.delay_status.clone();
                        new_topic.hz_history = existing.hz_history.clone();
                        new_topic.hz_std_dev_history = existing.hz_std_dev_history.clone();
                        new_topic.delay_history = existing.delay_history.clone();
                        new_topic.delay_std_dev_history = existing.delay_std_dev_history.clone();
                    }
                }
                self.topic_map = new_map;
                let all_topics: Vec<ros::TopicInfo> = self.topic_map.values().cloned().collect();
                // Convert ros::TopicInfo to tree::TopicInfo
                let tree_topics: Vec<crate::tree::TopicInfo> = all_topics
                    .iter()
                    .map(|t| crate::tree::TopicInfo {
                        name: t.name.clone(),
                    })
                    .collect();
                self.master_tree.build_from_topics(&tree_topics);

                for (path, is_expanded) in &self.expansion_state {
                    if let Some(node) = self.master_tree.find_node_by_path(path) {
                        node.is_expanded = *is_expanded;
                    }
                }

                self.rebuild_visible_items();
            }
            TopicMessage::TopicHzUpdate {
                topic_name,
                hz,
                std_dev,
            } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    topic.hz = Some(hz);
                    topic.hz_std_dev = std_dev; // Store current std dev for display
                    topic.hz_status = MeasurementStatus::HasValue;
                    topic.hz_history.push(hz);
                    if topic.hz_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.hz_history.remove(0);
                    }

                    // Store std dev for Bollinger bands, use 0.0 if no std dev available
                    topic.hz_std_dev_history.push(std_dev.unwrap_or(0.0));
                    if topic.hz_std_dev_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.hz_std_dev_history.remove(0);
                    }
                }
            }
            TopicMessage::TopicDelayUpdate {
                topic_name,
                delay,
                std_dev,
            } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    topic.delay = Some(delay);
                    topic.delay_std_dev = std_dev; // Store current std dev for display
                    topic.delay_status = MeasurementStatus::HasValue;
                    topic.delay_history.push(delay * 1000.0); // Store in ms
                    if topic.delay_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.delay_history.remove(0);
                    }

                    // Store std dev for potential Bollinger bands, use 0.0 if no std dev available
                    topic
                        .delay_std_dev_history
                        .push(std_dev.map(|x| x * 1000.0).unwrap_or(0.0)); // Convert to ms
                    if topic.delay_std_dev_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.delay_std_dev_history.remove(0);
                    }
                }
            }
            TopicMessage::TopicHzStatusUpdate { topic_name, status } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    if !matches!(topic.hz_status, MeasurementStatus::HasValue) {
                        topic.hz_status = status;
                    }
                }
            }
            TopicMessage::TopicDelayStatusUpdate { topic_name, status } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    if !matches!(topic.delay_status, MeasurementStatus::HasValue) {
                        topic.delay_status = status;
                    }
                }
            }
            TopicMessage::TopicHzStdDevUpdate {
                topic_name,
                std_dev,
            } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    // Update the most recent std_dev value in history without adding new Hz data
                    if !topic.hz_std_dev_history.is_empty() {
                        let last_idx = topic.hz_std_dev_history.len() - 1;
                        topic.hz_std_dev_history[last_idx] = std_dev;
                        topic.hz_std_dev = Some(std_dev); // Update current std dev for display
                    }
                }
            }
            TopicMessage::TopicDelayStdDevUpdate {
                topic_name,
                std_dev,
            } => {
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    // Update the most recent std_dev value in history without adding new delay data
                    if !topic.delay_std_dev_history.is_empty() {
                        let last_idx = topic.delay_std_dev_history.len() - 1;
                        topic.delay_std_dev_history[last_idx] = std_dev * 1000.0; // Convert to ms
                        topic.delay_std_dev = Some(std_dev); // Update current std dev for display
                    }
                }
            }
            TopicMessage::TopicEcho(line) => {
                self.echo_content.push(line);
                const MAX_ECHO_LINES: usize = 1000; // Increased from 200 for better history

                // Remove old messages to prevent memory issues
                if self.echo_content.len() > MAX_ECHO_LINES {
                    // Remove multiple lines at once for efficiency
                    let remove_count = self.echo_content.len() - MAX_ECHO_LINES + 100; // Keep some buffer
                    self.echo_content.drain(0..remove_count);

                    // Adjust scroll offset if needed
                    if self.echo_scroll_offset >= remove_count {
                        self.echo_scroll_offset -= remove_count;
                    } else {
                        self.echo_scroll_offset = 0;
                    }
                }

                // Keep scroll at top by default - don't auto-scroll when new messages arrive
                // User can manually scroll down if needed
            }
            TopicMessage::Error(error) => {
                self.error_message = Some(error);
            }
        }
    }

    pub fn try_receive_messages(&mut self) {
        while let Ok(msg) = self.message_receiver.try_recv() {
            self.handle_message(msg);
        }
    }

    pub fn rebuild_visible_items(&mut self) {
        let mut target_tree;
        if self.filter_text.is_empty() {
            // No filter, use the master tree which respects manual expansion state
            target_tree = self.master_tree.clone();
        } else {
            // Filter is active, build a temporary tree from only matching topics
            let filtered_topics: Vec<ros::TopicInfo> = self
                .topic_map
                .values()
                .filter(|topic| {
                    topic
                        .name
                        .to_lowercase()
                        .contains(&self.filter_text.to_lowercase())
                })
                .cloned()
                .collect();
            target_tree = TopicTree::new();
            // Convert ros::TopicInfo to tree::TopicInfo
            let tree_topics: Vec<crate::tree::TopicInfo> = filtered_topics
                .iter()
                .map(|t| crate::tree::TopicInfo {
                    name: t.name.clone(),
                })
                .collect();
            target_tree.build_from_topics(&tree_topics);

            // Automatically expand all groups in the filtered results for visibility
            for group in target_tree.root.values_mut() {
                group.is_expanded = true;
            }
        }
        self.visible_items = target_tree.get_flattened_view();
        self.clamp_selection();
    }

    pub fn on_key(&mut self, c: char) {
        if self.mode == AppMode::Search {
            self.search_text.push(c);
        } else {
            match c {
                'q' => {
                    match self.mode {
                        AppMode::TopicDetail | AppMode::Help => {
                            self.mode = AppMode::TopicList;
                        }
                        AppMode::TopicList => {
                            self.should_quit = true;
                        }
                        _ => {} // Search mode is handled above
                    }
                }
                'd' => self.enter_detail_view(),
                's' => self.toggle_sim_time(),
                '?' => self.mode = AppMode::Help,
                _ => {}
            }
        }
    }

    pub fn on_backspace(&mut self) {
        if self.mode == AppMode::Search {
            self.search_text.pop();
        }
    }

    pub fn on_f4(&mut self) {
        self.mode = AppMode::Search;
        self.search_text = self.filter_text.clone();
    }

    pub fn on_enter(&mut self) {
        match self.mode {
            AppMode::Search => {
                self.filter_text = self.search_text.clone();
                self.mode = AppMode::TopicList;
                self.rebuild_visible_items();
            }
            AppMode::TopicList => {
                let selected_path = self
                    .get_selected_item()
                    .map(|item| item.node.full_path.clone());
                let is_group = self.get_selected_item().is_some_and(|item| item.is_group());

                if let Some(path) = selected_path {
                    if is_group {
                        self.toggle_group_expansion(&path);
                    } else {
                        self.toggle_watch_current_topic();
                    }
                }
            }
            AppMode::TopicDetail => {
                self.toggle_watch_current_topic();
            }
            _ => {}
        }
    }

    pub fn on_tab(&mut self) {
        if self.mode == AppMode::TopicDetail {
            self.detail_focus = match self.detail_focus {
                DetailPaneFocus::Info => DetailPaneFocus::Echo,
                DetailPaneFocus::Echo => DetailPaneFocus::Info,
            };
        }
    }

    pub fn on_space(&mut self) {}

    pub fn on_left(&mut self) {
        if self.mode == AppMode::TopicDetail {
            // No action for left/right in detail view with combined panes
        } else if let Some(item) = self.get_selected_item() {
            if item.is_group() && item.node.is_expanded {
                let path_to_toggle = item.node.full_path.clone();
                self.toggle_group_expansion(&path_to_toggle);
            }
        }
    }

    pub fn on_right(&mut self) {
        if self.mode == AppMode::TopicDetail {
            // No action for left/right in detail view with combined panes
        } else if let Some(item) = self.get_selected_item() {
            if item.is_group() && !item.node.is_expanded {
                let path_to_toggle = item.node.full_path.clone();
                self.toggle_group_expansion(&path_to_toggle);
            }
        }
    }

    pub fn on_escape(&mut self) {
        match self.mode {
            AppMode::Search | AppMode::Help | AppMode::TopicDetail => {
                if self.is_echoing {
                    self.is_echoing = false;
                    let _ = self.watch_sender.send(WatchMessage::StopEcho);
                }
                self.mode = AppMode::TopicList;
            }
            AppMode::TopicList => {
                if !self.filter_text.is_empty() {
                    self.filter_text.clear();
                    self.search_text.clear();
                    self.rebuild_visible_items();
                } else {
                    self.should_quit = true;
                }
            }
        }
    }

    pub fn is_in_search_mode(&self) -> bool {
        matches!(self.mode, AppMode::Search)
    }

    fn enter_detail_view(&mut self) {
        if let Some(item) = self.get_selected_item() {
            if item.is_topic() {
                self.selected_topic_name = Some(item.node.full_path.clone());
                self.detail_focus = DetailPaneFocus::Info;
                self.detail_scroll_offset = 0;
                self.echo_content.clear();
                self.is_echoing = false;
                self.echo_scroll_offset = 0;
                self.mode = AppMode::TopicDetail;
            }
        }
    }

    fn toggle_group_expansion(&mut self, group_path: &str) {
        if let Some(node) = self.master_tree.find_node_by_path(group_path) {
            node.toggle_expanded();
            self.expansion_state
                .insert(group_path.to_string(), node.is_expanded);
            self.rebuild_visible_items();
        }
    }

    pub fn update_loading_animation(&mut self) {
        if self.last_animation_update.elapsed() < Duration::from_millis(150) {
            return;
        }
        for topic in self.topic_map.values_mut() {
            if topic.watched {
                if let MeasurementStatus::Loading(frame) = &mut topic.hz_status {
                    *frame = (*frame + 1) % 3;
                }
                if let MeasurementStatus::Loading(frame) = &mut topic.delay_status {
                    *frame = (*frame + 1) % 3;
                }
            }
        }
        self.last_animation_update = std::time::Instant::now();
    }

    pub fn toggle_echo(&mut self) {
        if self.mode == AppMode::TopicDetail {
            if let Some(topic_name) = &self.selected_topic_name {
                self.is_echoing = !self.is_echoing;
                if self.is_echoing {
                    self.echo_content.clear();
                    self.echo_scroll_offset = 0;
                    let _ = self
                        .watch_sender
                        .send(WatchMessage::StartEcho(topic_name.clone()));
                } else {
                    let _ = self.watch_sender.send(WatchMessage::StopEcho);
                }
            }
        }
    }

    pub fn on_up(&mut self) {
        if self.mode == AppMode::TopicDetail {
            if self.detail_focus == DetailPaneFocus::Echo {
                self.echo_scroll_offset = self.echo_scroll_offset.saturating_sub(1);
            }
        } else {
            self.select_previous();
        }
    }

    pub fn on_down(&mut self) {
        if self.mode == AppMode::TopicDetail {
            if self.detail_focus == DetailPaneFocus::Echo {
                // Constrain scroll to prevent excess whitespace
                // Assume a reasonable viewport height (will be properly calculated in UI)
                let max_scroll = self.echo_content.len().saturating_sub(20); // Rough estimate
                self.echo_scroll_offset = (self.echo_scroll_offset + 1).min(max_scroll);
            }
        } else {
            self.select_next();
        }
    }

    pub fn echo_page_up(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            let page_size = 10; // Scroll 10 lines at a time
            self.echo_scroll_offset = self.echo_scroll_offset.saturating_sub(page_size);
        }
    }

    pub fn echo_page_down(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            let page_size = 10; // Scroll 10 lines at a time
            self.echo_scroll_offset = (self.echo_scroll_offset + page_size)
                .min(self.echo_content.len().saturating_sub(1));
        }
    }

    pub fn echo_home(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            self.echo_scroll_offset = 0;
        }
    }

    pub fn echo_end(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            self.echo_scroll_offset = self.echo_content.len().saturating_sub(1);
        }
    }

    pub fn shutdown(&self) {
        self.detail_watcher.shutdown();
    }

    fn get_selected_item(&self) -> Option<&TopicTreeItem> {
        self.visible_items.get(self.selected_index)
    }

    fn clamp_selection(&mut self) {
        if self.visible_items.is_empty() {
            self.selected_index = 0;
        } else if self.selected_index >= self.visible_items.len() {
            self.selected_index = self.visible_items.len() - 1;
        }
    }

    pub fn select_next(&mut self) {
        if !self.visible_items.is_empty() {
            self.selected_index = (self.selected_index + 1) % self.visible_items.len();
        }
    }

    pub fn select_previous(&mut self) {
        if !self.visible_items.is_empty() {
            self.selected_index =
                (self.selected_index + self.visible_items.len() - 1) % self.visible_items.len();
        }
    }

    pub fn toggle_watch_current_topic(&mut self) {
        if let Some(item) = self.get_selected_item() {
            if let Some(topic_info) = item.get_topic_info() {
                let topic_name = topic_info.name.clone();
                if let Some(topic) = self.topic_map.get_mut(&topic_name) {
                    topic.watched = !topic.watched;
                    if topic.watched {
                        topic.hz_status = MeasurementStatus::Loading(0);
                        topic.delay_status = MeasurementStatus::Loading(0);
                        topic.hz_history.clear();
                        topic.hz_std_dev_history.clear();
                        topic.delay_history.clear();
                        topic.delay_std_dev_history.clear();
                        let _ = self
                            .watch_sender
                            .send(WatchMessage::StartContinuousMetrics(topic.name.clone()));
                    } else {
                        topic.hz = None;
                        topic.hz_std_dev = None;
                        topic.delay = None;
                        topic.delay_std_dev = None;
                        topic.hz_status = MeasurementStatus::NotMeasuring;
                        topic.delay_status = MeasurementStatus::NotMeasuring;
                        let _ = self
                            .watch_sender
                            .send(WatchMessage::StopContinuousMetrics(topic.name.clone()));
                    }
                }
            }
        }
    }

    fn toggle_sim_time(&mut self) {
        match self.mode {
            AppMode::TopicList | AppMode::TopicDetail => {}
            _ => return,
        }

        self.use_sim_time = !self.use_sim_time;

        for topic in self.topic_map.values_mut().filter(|t| t.watched) {
            topic.delay_status = MeasurementStatus::Loading(0);
            topic.delay = None;
            topic.delay_std_dev = None;
            topic.delay_history.clear();
            topic.delay_std_dev_history.clear();
        }

        let _ = self
            .watch_sender
            .send(WatchMessage::SetUseSimTime(self.use_sim_time));
    }

    pub fn toggle_collapse_all(&mut self) {
        // Check if any groups are currently expanded
        let any_expanded = self.check_any_expanded(&self.master_tree.root);

        if any_expanded {
            self.collapse_all();
        } else {
            self.expand_all();
        }

        // Rebuild the visible items after changing expansion states
        self.rebuild_visible_items();
    }

    #[allow(clippy::only_used_in_recursion)]
    fn check_any_expanded(
        &self,
        nodes: &std::collections::HashMap<String, crate::tree::TopicTreeNode>,
    ) -> bool {
        for node in nodes.values() {
            if !node.is_leaf && node.is_expanded {
                return true;
            }
            if self.check_any_expanded(&node.children) {
                return true;
            }
        }
        false
    }

    fn collapse_all(&mut self) {
        Self::set_all_expanded_static(&mut self.master_tree.root, false);
        // Update expansion state tracking
        for key in self.expansion_state.keys().cloned().collect::<Vec<_>>() {
            self.expansion_state.insert(key, false);
        }
    }

    fn expand_all(&mut self) {
        Self::set_all_expanded_static(&mut self.master_tree.root, true);
        // Update expansion state tracking
        for key in self.expansion_state.keys().cloned().collect::<Vec<_>>() {
            self.expansion_state.insert(key, true);
        }
    }

    fn set_all_expanded_static(
        nodes: &mut std::collections::HashMap<String, crate::tree::TopicTreeNode>,
        expanded: bool,
    ) {
        for node in nodes.values_mut() {
            if !node.is_leaf {
                node.is_expanded = expanded;
                Self::set_all_expanded_static(&mut node.children, expanded);
            }
        }
    }
}
