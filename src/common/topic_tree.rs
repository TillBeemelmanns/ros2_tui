use super::abstract_tree::{GenericTree, TreeData, TreeItem};

/// Topic information for the tree
#[derive(Debug, Clone, Default)]
pub struct TopicInfo {
    pub name: String,
}

impl TreeData for TopicInfo {
    fn get_display_name(&self) -> String {
        self.name.clone()
    }

    fn get_full_path(&self) -> String {
        self.name.clone()
    }

    fn is_leaf(&self) -> bool {
        true // Topics are always leaf nodes
    }

    fn get_parent_path(&self) -> Option<String> {
        // Extract parent path from topic name for hierarchical organization
        // For example: "/sensor_msgs/msg/Image" -> "/sensor_msgs/msg" -> "/sensor_msgs"
        let parts: Vec<&str> = self.name.trim_start_matches('/').split('/').collect();
        if parts.len() > 1 {
            let parent_parts = &parts[..parts.len() - 1];
            Some(format!("/{}", parent_parts.join("/")))
        } else {
            None // Top-level topic
        }
    }
}

/// Type aliases for topic-specific tree structures
pub type TopicTree = GenericTree<TopicInfo>;
pub type TopicTreeNode = super::abstract_tree::GenericTreeNode<TopicInfo>;
pub type TopicTreeItem = TreeItem<TopicInfo>;
