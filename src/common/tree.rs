use std::collections::HashMap;

// We need to define both types in all contexts since the tree can hold either
#[derive(Debug, Clone, Default)]
pub struct ParamInfo {
    pub node_name: String,
    pub param_name: String,
}

#[derive(Debug, Clone, Default)]
pub struct TopicInfo {
    pub name: String,
}

#[derive(Debug, Clone)]
pub struct TopicTreeNode {
    pub name: String,
    pub full_path: String,
    pub is_leaf: bool,
    pub topic_info: Option<TopicInfo>,
    pub param_info: Option<ParamInfo>,
    pub children: HashMap<String, TopicTreeNode>,
    pub is_expanded: bool,
}

impl TopicTreeNode {
    pub fn new(name: String, full_path: String, is_leaf: bool) -> Self {
        Self {
            name,
            full_path,
            is_leaf,
            topic_info: None,
            param_info: None,
            children: HashMap::new(),
            is_expanded: false, // Groups should start collapsed by default.
        }
    }

    pub fn new_with_topic(topic_info: TopicInfo) -> Self {
        let name = topic_info.name.clone();
        Self {
            name: name.clone(),
            full_path: name,
            is_leaf: true,
            topic_info: Some(topic_info),
            param_info: None,
            children: HashMap::new(),
            is_expanded: false,
        }
    }

    #[allow(dead_code)]
    pub fn new_with_param(param_info: ParamInfo) -> Self {
        let name = format!("{}/{}", param_info.node_name, param_info.param_name);
        Self {
            name: name.clone(),
            full_path: name,
            is_leaf: true,
            topic_info: None,
            param_info: Some(param_info),
            children: HashMap::new(),
            is_expanded: false,
        }
    }

    pub fn toggle_expanded(&mut self) {
        if !self.is_leaf {
            self.is_expanded = !self.is_expanded;
        }
    }
}

#[derive(Debug, Clone)]
pub struct TopicTree {
    pub root: HashMap<String, TopicTreeNode>,
}

impl Default for TopicTree {
    fn default() -> Self {
        Self::new()
    }
}

impl TopicTree {
    pub fn new() -> Self {
        Self {
            root: HashMap::new(),
        }
    }

    pub fn build_from_topics(&mut self, topics: &[TopicInfo]) {
        self.root.clear();
        for topic in topics {
            self.insert_topic(topic.clone());
        }
    }

    #[allow(dead_code)]
    pub fn build_from_params(&mut self, params: &[ParamInfo]) {
        self.root.clear();
        for param in params {
            self.insert_param(param.clone());
        }
    }

    fn insert_topic(&mut self, topic: TopicInfo) {
        let topic_name = topic.name.clone();
        let parts: Vec<&str> = topic_name.split('/').filter(|s| !s.is_empty()).collect();
        if parts.is_empty() {
            return;
        }

        let group_name = parts[0].to_string();
        let group_path = format!("/{}", group_name);

        let group_node = self
            .root
            .entry(group_name.clone())
            .or_insert_with(|| TopicTreeNode::new(group_name, group_path, false));

        let leaf_node = TopicTreeNode::new_with_topic(topic);
        group_node.children.insert(topic_name, leaf_node);
    }

    #[allow(dead_code)]
    fn insert_param(&mut self, param: ParamInfo) {
        let node_name = param.node_name.clone();
        let node_name_for_path = node_name.clone();
        let param_key = format!("{}/{}", param.node_name, param.param_name);

        // Remove leading slash if present
        let clean_node_name = if let Some(stripped) = node_name.strip_prefix('/') {
            stripped.to_string()
        } else {
            node_name
        };

        let group_node = self.root.entry(clean_node_name.clone()).or_insert_with(|| {
            TopicTreeNode::new(clean_node_name.clone(), node_name_for_path, false)
        });

        let leaf_node = TopicTreeNode::new_with_param(param);
        group_node.children.insert(param_key, leaf_node);
    }

    pub fn get_flattened_view(&self) -> Vec<TopicTreeItem> {
        let mut items = Vec::new();
        let mut sorted_groups: Vec<_> = self.root.values().collect();
        sorted_groups.sort_by(|a, b| a.name.cmp(&b.name));

        // This is the new, correct logic.
        // It processes one group and its children completely before moving to the next group.
        for group_node in sorted_groups {
            items.push(TopicTreeItem {
                node: group_node.clone(),
                indent_level: 0,
            });

            if group_node.is_expanded {
                let mut sorted_children: Vec<_> = group_node.children.values().collect();
                sorted_children.sort_by(|a, b| a.name.cmp(&b.name));
                for topic_node in sorted_children {
                    items.push(TopicTreeItem {
                        node: topic_node.clone(),
                        indent_level: 1,
                    });
                }
            }
        }
        items
    }

    pub fn find_node_by_path(&mut self, path: &str) -> Option<&mut TopicTreeNode> {
        let parts: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
        if parts.is_empty() {
            return None;
        }

        let group_name = parts[0];
        if let Some(group_node) = self.root.get_mut(group_name) {
            if parts.len() == 1 {
                return Some(group_node);
            } else {
                return group_node.children.get_mut(path);
            }
        }
        None
    }
}

#[derive(Debug, Clone)]
pub struct TopicTreeItem {
    pub node: TopicTreeNode,
    pub indent_level: usize,
}

impl TopicTreeItem {
    pub fn is_group(&self) -> bool {
        !self.node.is_leaf
    }
    pub fn is_topic(&self) -> bool {
        self.node.is_leaf
    }

    pub fn get_display_name(&self) -> String {
        let prefix = "  ".repeat(self.indent_level);
        let icon = if self.is_group() {
            if self.node.is_expanded {
                "▼ "
            } else {
                "▶ "
            }
        } else {
            "  "
        };

        let display_name = if self.is_topic() {
            let parts: Vec<&str> = self
                .node
                .name
                .split('/')
                .filter(|s| !s.is_empty())
                .collect();
            if parts.len() > 1 {
                format!("/{}", parts[1..].join("/"))
            } else {
                self.node.name.clone()
            }
        } else {
            self.node.name.clone()
        };

        format!("{}{}{}", prefix, icon, display_name)
    }

    pub fn get_topic_info(&self) -> Option<&TopicInfo> {
        self.node.topic_info.as_ref()
    }
}
