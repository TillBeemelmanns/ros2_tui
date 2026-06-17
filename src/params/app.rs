use super::ros;
use super::watcher::{
    ParamMessage, ParamValueWatcherHandle, ParamWatchMessage, ParamWatcherHandle,
};
use crate::common::{ParamTree, ParamTreeItem};
use crossbeam::channel::{Receiver, Sender};
use std::collections::HashMap;
use std::time::Duration;

#[derive(PartialEq, Debug)]
pub enum AppMode {
    ParamList,
    ParamDetail,
    Search,
    Help,
    SetParameter,   // Mode for setting parameter values
    DumpParameters, // Mode for dumping parameters
    LoadParameters, // Mode for loading parameters
    Warning,        // Mode for displaying warning messages
}

#[derive(PartialEq, Debug)]
pub enum DetailPaneFocus {
    Info,
    Value,
}

#[derive(Clone, Debug)]
pub struct ParameterEditState {
    pub node_name: String,
    pub param_name: String,
    pub current_value: String,
    pub param_type: String,
    pub new_value: String,
    pub is_editing: bool,
}

pub struct ParamsApp {
    pub should_quit: bool,
    pub mode: AppMode,
    pub previous_mode: Option<AppMode>,

    pub param_map: HashMap<String, ros::ParamInfo>, // Key: node_name/param_name

    pub master_tree: ParamTree,
    pub visible_items: Vec<ParamTreeItem>,

    pub selected_index: usize,
    pub scroll_offset: usize,

    pub filter_text: String,
    pub search_text: String,

    pub selected_param_key: Option<String>, // node_name/param_name
    pub detail_scroll_offset: usize,
    pub detail_focus: DetailPaneFocus,

    pub error_message: Option<String>,
    pub success_message: Option<String>,
    pub error_message_time: Option<std::time::Instant>,
    pub success_message_time: Option<std::time::Instant>,

    pub expansion_state: HashMap<String, bool>,

    // Parameter editing state
    pub edit_state: Option<ParameterEditState>,
    pub file_input: String, // For load/dump operations

    // Input cursor positions for better UX
    pub edit_cursor: usize,
    pub file_cursor: usize,

    // Warning message display
    pub warning_message: String,

    message_receiver: Receiver<ParamMessage>,
    watch_sender: Sender<ParamWatchMessage>,
    _param_watcher: ParamWatcherHandle,
    #[allow(dead_code)]
    value_watcher: ParamValueWatcherHandle,
}

impl ParamsApp {
    pub fn new(refresh_interval: Duration) -> Self {
        let (param_sender, param_receiver) = crossbeam::channel::unbounded();

        let param_watcher = ParamWatcherHandle::new(param_sender.clone(), refresh_interval);
        let (value_watcher, watch_sender) = ParamValueWatcherHandle::new(param_sender);

        Self {
            should_quit: false,
            mode: AppMode::ParamList,
            previous_mode: None,
            param_map: HashMap::new(),
            master_tree: ParamTree::new(),
            visible_items: Vec::new(),
            selected_index: 0,
            scroll_offset: 0,
            filter_text: String::new(),
            search_text: String::new(),
            selected_param_key: None,
            detail_scroll_offset: 0,
            detail_focus: DetailPaneFocus::Info,
            error_message: None,
            success_message: None,
            error_message_time: None,
            success_message_time: None,
            expansion_state: HashMap::new(),
            edit_state: None,
            file_input: String::new(),
            edit_cursor: 0,
            file_cursor: 0,
            warning_message: String::new(),
            message_receiver: param_receiver,
            watch_sender,
            _param_watcher: param_watcher,
            value_watcher,
        }
    }

    pub fn handle_message(&mut self, msg: ParamMessage) {
        match msg {
            ParamMessage::ParamList(params) => {
                self.error_message = None;

                let new_params_map = params
                    .into_iter()
                    .map(|p| (format!("{}/{}", p.node_name, p.param_name), p))
                    .collect::<HashMap<_, _>>();

                // Update existing params and add new ones
                for (key, new_param) in new_params_map {
                    self.param_map
                        .entry(key)
                        .and_modify(|existing_param| {
                            // Update fields that can change, but preserve state
                            existing_param.param_name = new_param.param_name.clone();
                            existing_param.node_name = new_param.node_name.clone();
                            // Do NOT overwrite value, type, etc.
                        })
                        .or_insert(new_param);
                }

                // Build tree structure (node -> parameters)
                let param_list: Vec<ros::ParamInfo> = self.param_map.values().cloned().collect();
                // Convert ros::ParamInfo to tree::ParamInfo
                let tree_params: Vec<crate::common::param_tree::ParamInfo> = param_list
                    .iter()
                    .map(|p| crate::common::param_tree::ParamInfo {
                        node_name: p.node_name.clone(),
                        param_name: p.param_name.clone(),
                        param_type: p.param_type.clone(),
                        is_namespace: p.param_type == "Namespace",
                    })
                    .collect();
                self.master_tree.build_from_data(&tree_params);

                // Apply expansion state
                for (group_name, is_expanded) in &self.expansion_state {
                    if let Some(group) = self.master_tree.root.get_mut(group_name) {
                        group.is_expanded = *is_expanded;
                    }
                }

                self.rebuild_visible_items();
            }
            ParamMessage::ParamValue {
                node_name,
                param_name,
                value,
                param_type,
            } => {
                let key = format!("{}/{}", node_name, param_name);
                if let Some(param) = self.param_map.get_mut(&key) {
                    param.value = Some(value.clone());
                    param.param_type = param_type.clone();
                    self.rebuild_visible_items(); // Rebuild visible items to reflect updated value
                }
            }
            ParamMessage::ParamSetSuccess {
                node_name,
                param_name,
                new_value,
            } => {
                self.success_message = Some(format!(
                    "Successfully set {}/{} to {}",
                    node_name, param_name, new_value
                ));
                self.success_message_time = Some(std::time::Instant::now());
                // Use delayed refresh to allow ROS2 parameter set to take effect (100ms delay)
                let _ = self
                    .watch_sender
                    .send(ParamWatchMessage::RefreshNodeDelayed {
                        node_name: node_name.clone(),
                        delay_ms: 100,
                    });
                // Also refresh the specific parameter after a longer delay as backup
                // This runs after the delayed node refresh to ensure consistency
            }
            ParamMessage::ParamSetError {
                node_name,
                param_name,
                error,
            } => {
                // Show ROS2's direct error message to the user
                self.error_message = Some(format!("{}/{}: {}", node_name, param_name, error));
                self.error_message_time = Some(std::time::Instant::now());
            }
            ParamMessage::DumpSuccess {
                node_name,
                file_path,
            } => {
                self.success_message = Some(format!(
                    "Successfully dumped parameters from {} to {}",
                    node_name, file_path
                ));
                self.success_message_time = Some(std::time::Instant::now());
            }
            ParamMessage::LoadSuccess {
                node_name,
                file_path,
            } => {
                self.success_message = Some(format!(
                    "Successfully loaded parameters to {} from {}",
                    node_name, file_path
                ));
                self.success_message_time = Some(std::time::Instant::now());
                // Refresh all parameters for this node
                let _ = self
                    .watch_sender
                    .send(ParamWatchMessage::RefreshNode { node_name });
            }
            ParamMessage::Error(error) => {
                self.error_message = Some(error);
                self.error_message_time = Some(std::time::Instant::now());
            }
        }
    }

    pub fn try_receive_messages(&mut self) {
        while let Ok(msg) = self.message_receiver.try_recv() {
            self.handle_message(msg);
        }

        // Auto-hide messages after 15 seconds
        self.check_and_clear_expired_messages();
    }

    pub fn check_and_clear_expired_messages(&mut self) {
        let now = std::time::Instant::now();
        let message_duration = Duration::from_secs(15);

        // Clear expired success message
        if let Some(success_time) = self.success_message_time {
            if now.duration_since(success_time) >= message_duration {
                self.success_message = None;
                self.success_message_time = None;
            }
        }

        // Clear expired error message
        if let Some(error_time) = self.error_message_time {
            if now.duration_since(error_time) >= message_duration {
                self.error_message = None;
                self.error_message_time = None;
            }
        }
    }

    pub fn rebuild_visible_items(&mut self) {
        let mut target_tree;
        if self.filter_text.is_empty() {
            target_tree = self.master_tree.clone();
        } else {
            let filtered_params: Vec<ros::ParamInfo> = self
                .param_map
                .values()
                .filter(|param| {
                    param
                        .node_name
                        .to_lowercase()
                        .contains(&self.filter_text.to_lowercase())
                        || param
                            .param_name
                            .to_lowercase()
                            .contains(&self.filter_text.to_lowercase())
                })
                .cloned()
                .collect();
            target_tree = ParamTree::new();
            // Convert params_ros::ParamInfo to tree::ParamInfo
            let tree_params: Vec<crate::common::param_tree::ParamInfo> = filtered_params
                .iter()
                .map(|p| crate::common::param_tree::ParamInfo {
                    node_name: p.node_name.clone(),
                    param_name: p.param_name.clone(),
                    param_type: p.param_type.clone(),
                    is_namespace: p.param_type == "Namespace",
                })
                .collect();
            target_tree.build_from_data(&tree_params);

            // Auto-expand filtered results
            for group in target_tree.root.values_mut() {
                group.is_expanded = true;
            }
        }
        self.visible_items = target_tree.get_flattened_view();
        self.clamp_selection();
    }

    pub fn on_key(&mut self, c: char) {
        match self.mode {
            AppMode::Search => {
                self.search_text.push(c);
                // Dynamic filtering as you type
                self.filter_text = self.search_text.clone();
                self.rebuild_visible_items();
            }
            AppMode::SetParameter => {
                if let Some(ref mut edit_state) = self.edit_state {
                    let byte_index = edit_state
                        .new_value
                        .char_indices()
                        .map(|(i, _)| i)
                        .nth(self.edit_cursor)
                        .unwrap_or(edit_state.new_value.len());
                    edit_state.new_value.insert(byte_index, c);
                    self.move_edit_cursor_right();
                }
            }
            AppMode::DumpParameters | AppMode::LoadParameters => {
                let byte_index = self.byte_index_file();
                self.file_input.insert(byte_index, c);
                self.move_file_cursor_right();
            }
            _ => match c {
                'q' => self.should_quit = true,
                'c' => self.toggle_collapse_all(),
                'r' => self.refresh_param_list(),
                '\n' | '\r' => match self.mode {
                    AppMode::SetParameter => self.confirm_set_parameter(),
                    AppMode::DumpParameters => self.confirm_dump_parameters(),
                    AppMode::LoadParameters => self.confirm_load_parameters(),
                    _ => {}
                },
                _ => {}
            },
        }
    }

    pub fn on_up(&mut self) {
        if !self.visible_items.is_empty() {
            if self.selected_index > 0 {
                self.selected_index -= 1;
            } else {
                self.selected_index = self.visible_items.len() - 1;
            }
            self.adjust_scroll();
        }
    }

    pub fn on_down(&mut self) {
        if !self.visible_items.is_empty() {
            self.selected_index = (self.selected_index + 1) % self.visible_items.len();
            self.adjust_scroll();
        }
    }

    pub fn on_left(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() {
                let group_name = item.node.name.clone();
                self.collapse_group(&group_name);
            }
        }
    }

    pub fn on_right(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() {
                let group_name = item.node.name.clone();
                self.expand_group(&group_name);
            }
        }
    }

    pub fn on_enter(&mut self) {
        match self.mode {
            AppMode::Search => {
                // In search mode, Enter should confirm the search pattern and exit search mode
                self.filter_text = self.search_text.clone();
                self.mode = AppMode::ParamList;
                self.rebuild_visible_items();
            }
            AppMode::ParamList => {
                // In parameter list mode, handle group expansion only
                if let Some(item) = self.visible_items.get(self.selected_index) {
                    if item.is_group() {
                        let group_name = item.node.name.clone();
                        self.toggle_group(&group_name);
                    }
                    // Parameters can no longer be watched - they're loaded automatically via YAML dump
                }
            }
            _ => {
                // Handle other modes if needed (SetParameter, DumpParameters, etc.)
                // These are handled by the main.rs key event handler
            }
        }
    }

    pub fn on_tab(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() {
                let group_name = item.node.name.clone();
                self.toggle_group(&group_name);
            }
        }
    }

    pub fn on_space(&mut self) {
        match self.mode {
            AppMode::ParamList => self.refresh_param_list(),
            AppMode::ParamDetail => {
                self.mode = AppMode::ParamList;
                self.selected_param_key = None;
            }
            _ => {}
        }
    }

    pub fn on_f4(&mut self) {
        match self.mode {
            AppMode::Search => {
                self.filter_text = self.search_text.clone();
                self.search_text.clear();
                self.mode = AppMode::ParamList;
                self.rebuild_visible_items();
            }
            _ => {
                self.mode = AppMode::Search;
                self.search_text = self.filter_text.clone();
                self.expand_all();
            }
        }
    }

    pub fn on_escape(&mut self) {
        match self.mode {
            AppMode::Search => {
                self.search_text.clear();
                self.filter_text.clear();
                self.mode = AppMode::ParamList;
                self.rebuild_visible_items();
            }
            AppMode::SetParameter => {
                self.mode = AppMode::ParamList;
                self.edit_state = None;
            }
            AppMode::DumpParameters | AppMode::LoadParameters => {
                self.mode = AppMode::ParamList;
                self.file_input.clear();
                self.file_cursor = 0;
            }
            AppMode::Warning => {
                // Return to previous mode if we have one, otherwise go to ParamList
                if let Some(prev_mode) = self.previous_mode.take() {
                    self.mode = prev_mode;
                } else {
                    self.mode = AppMode::ParamList;
                }
                self.warning_message.clear();
            }
            AppMode::Help => {
                self.mode = AppMode::ParamList;
            }
            _ => {
                self.should_quit = true;
            }
        }
    }

    pub fn on_backspace(&mut self) {
        match self.mode {
            AppMode::Search => {
                self.search_text.pop();
                // Dynamic filtering as you delete characters
                self.filter_text = self.search_text.clone();
                self.rebuild_visible_items();
            }
            AppMode::SetParameter => {
                if let Some(ref mut edit_state) = self.edit_state {
                    if self.edit_cursor > 0 {
                        // Find the previous char boundary without borrowing
                        let chars: Vec<char> = edit_state.new_value.chars().collect();
                        let char_index = self.edit_cursor - 1;
                        edit_state.new_value = chars
                            .into_iter()
                            .enumerate()
                            .filter(|(i, _)| *i != char_index)
                            .map(|(_, c)| c)
                            .collect();
                        self.move_edit_cursor_left();
                    }
                }
            }
            AppMode::DumpParameters | AppMode::LoadParameters if self.file_cursor > 0 => {
                let byte_index = self.byte_index_file();
                if byte_index > 0 {
                    // Find the previous char boundary
                    let chars: Vec<char> = self.file_input.chars().collect();
                    let char_index = self.file_cursor - 1;
                    self.file_input = chars
                        .into_iter()
                        .enumerate()
                        .filter(|(i, _)| *i != char_index)
                        .map(|(_, c)| c)
                        .collect();
                    self.move_file_cursor_left();
                }
            }
            _ => {}
        }
    }

    pub fn on_set_parameter(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if !item.is_group() {
                let key = &item.node.full_path;
                if let Some(param) = self.param_map.get(key) {
                    let parts: Vec<&str> = key.split('/').collect();
                    if parts.len() >= 3 {
                        let node_name = format!("/{}", parts[1]); // Add back the leading slash
                        let param_name = parts[2..].join("/");
                        let current_value = param.value.clone().unwrap_or_default();
                        self.edit_state = Some(ParameterEditState {
                            node_name,
                            param_name,
                            current_value: current_value.clone(),
                            param_type: param.param_type.clone(),
                            new_value: current_value.clone(),
                            is_editing: true,
                        });
                        self.edit_cursor = current_value.chars().count(); // Set cursor at end of current value
                        self.mode = AppMode::SetParameter;
                    }
                }
            }
        }
    }

    pub fn on_dump_parameters(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() {
                // Dump parameters for the selected node
                self.file_input = format!("{}.yaml", item.node.name);
                self.file_cursor = self.file_input.chars().count(); // Set cursor at end
                self.mode = AppMode::DumpParameters;
            } else {
                // Show warning for single parameter
                self.warning_message = "Parameter dumping only works with nodes, not with single parameters. Navigate to a specific node to execute parameter dumping.".to_string();
                self.previous_mode = Some(AppMode::ParamList);
                self.mode = AppMode::Warning;
            }
        }
    }

    pub fn on_load_parameters(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() {
                // Load parameters for the selected node
                self.file_input = format!("{}.yaml", item.node.name);
                self.file_cursor = self.file_input.chars().count(); // Set cursor at end
                self.mode = AppMode::LoadParameters;
            } else {
                // Show warning for single parameter
                self.warning_message = "Parameter loading only works with nodes, not with single parameters. Navigate to a specific node to execute parameter loading.".to_string();
                self.previous_mode = Some(AppMode::ParamList);
                self.mode = AppMode::Warning;
            }
        }
    }

    pub fn confirm_set_parameter(&mut self) {
        if let Some(edit_state) = &self.edit_state {
            // Send the value directly to ROS2 - let it handle validation and provide feedback
            let _ = self.watch_sender.send(ParamWatchMessage::SetParam {
                node_name: edit_state.node_name.clone(),
                param_name: edit_state.param_name.clone(),
                value: edit_state.new_value.clone(),
            });
            self.mode = AppMode::ParamList;
            self.edit_state = None;
        } else {
            self.mode = AppMode::ParamList;
            self.edit_state = None;
        }
    }

    pub fn confirm_dump_parameters(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() && !self.file_input.is_empty() {
                let _ = self.watch_sender.send(ParamWatchMessage::DumpParam {
                    node_name: item.node.name.clone(),
                    file_path: self.file_input.clone(),
                });
            }
        }
        self.mode = AppMode::ParamList;
        self.file_input.clear();
    }

    pub fn confirm_load_parameters(&mut self) {
        if let Some(item) = self.visible_items.get(self.selected_index) {
            if item.is_group() && !self.file_input.is_empty() {
                let _ = self.watch_sender.send(ParamWatchMessage::LoadParam {
                    node_name: item.node.name.clone(),
                    file_path: self.file_input.clone(),
                });
            }
        }
        self.mode = AppMode::ParamList;
        self.file_input.clear();
    }

    pub fn is_in_search_mode(&self) -> bool {
        self.mode == AppMode::Search
    }

    pub fn toggle_collapse_all(&mut self) {
        let all_collapsed = self.expansion_state.values().all(|&expanded| !expanded);
        if all_collapsed {
            self.expand_all();
        } else {
            self.collapse_all();
        }
    }

    pub fn toggle_expand_all(&mut self) {
        self.expand_all();
    }

    pub fn collapse_all(&mut self) {
        let group_names: Vec<_> = self.master_tree.root.keys().cloned().collect();
        for group_name in group_names {
            self.expansion_state.insert(group_name.clone(), false);
            if let Some(group) = self.master_tree.root.get_mut(&group_name) {
                group.is_expanded = false;
            }
        }
        self.rebuild_visible_items();
    }

    pub fn expand_all(&mut self) {
        let group_names: Vec<_> = self.master_tree.root.keys().cloned().collect();
        for group_name in group_names {
            self.expansion_state.insert(group_name.clone(), true);
            if let Some(group) = self.master_tree.root.get_mut(&group_name) {
                group.is_expanded = true;
            }
        }
        self.rebuild_visible_items();
    }

    pub fn expand_group(&mut self, group_name: &str) {
        self.expansion_state.insert(group_name.to_string(), true);
        if let Some(group) = self.master_tree.root.get_mut(group_name) {
            group.is_expanded = true;
        }
        self.rebuild_visible_items();
    }

    pub fn collapse_group(&mut self, group_name: &str) {
        self.expansion_state.insert(group_name.to_string(), false);
        if let Some(group) = self.master_tree.root.get_mut(group_name) {
            group.is_expanded = false;
        }
        self.rebuild_visible_items();
    }

    pub fn toggle_group(&mut self, group_name: &str) {
        let is_expanded = self
            .expansion_state
            .get(group_name)
            .copied()
            .unwrap_or(false);
        if is_expanded {
            self.collapse_group(group_name);
        } else {
            self.expand_group(group_name);
        }
    }

    pub fn refresh_param_list(&mut self) {
        self.error_message = None;
        let _ = self.watch_sender.send(ParamWatchMessage::RefreshList);
    }

    pub fn clamp_selection(&mut self) {
        if self.visible_items.is_empty() {
            self.selected_index = 0;
        } else if self.selected_index >= self.visible_items.len() {
            self.selected_index = self.visible_items.len() - 1;
        }
        self.adjust_scroll();
    }

    pub fn adjust_scroll(&mut self) {
        const VISIBLE_HEIGHT: usize = 20; // Approximate visible height

        if self.selected_index < self.scroll_offset {
            self.scroll_offset = self.selected_index;
        } else if self.selected_index >= self.scroll_offset + VISIBLE_HEIGHT {
            self.scroll_offset = self.selected_index - VISIBLE_HEIGHT + 1;
        }
    }

    pub fn shutdown(&mut self) {
        // Send shutdown message to background threads
        let _ = self.watch_sender.send(ParamWatchMessage::Shutdown);
    }

    // Cursor handling methods for better UX
    pub fn move_edit_cursor_left(&mut self) {
        self.edit_cursor = self.edit_cursor.saturating_sub(1);
    }

    pub fn move_edit_cursor_right(&mut self) {
        if let Some(edit_state) = &self.edit_state {
            let max_pos = edit_state.new_value.chars().count();
            self.edit_cursor = (self.edit_cursor + 1).min(max_pos);
        }
    }

    pub fn move_file_cursor_left(&mut self) {
        self.file_cursor = self.file_cursor.saturating_sub(1);
    }

    pub fn move_file_cursor_right(&mut self) {
        let max_pos = self.file_input.chars().count();
        self.file_cursor = (self.file_cursor + 1).min(max_pos);
    }

    fn byte_index_file(&self) -> usize {
        self.file_input
            .char_indices()
            .map(|(i, _)| i)
            .nth(self.file_cursor)
            .unwrap_or(self.file_input.len())
    }
}
