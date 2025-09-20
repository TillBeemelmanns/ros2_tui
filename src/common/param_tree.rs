use super::abstract_tree::{GenericTree, TreeData, TreeItem};

/// Parameter information for the tree
#[derive(Debug, Clone, Default)]
pub struct ParamInfo {
    pub node_name: String,
    pub param_name: String,
    pub param_type: String,
    pub is_namespace: bool,
}

impl TreeData for ParamInfo {
    fn get_display_name(&self) -> String {
        // For namespaced parameters, show only the last part of the name
        if self.param_name.contains('.') {
            self.param_name
                .split('.')
                .next_back()
                .unwrap_or(&self.param_name)
                .to_string()
        } else {
            self.param_name.clone()
        }
    }

    fn get_full_path(&self) -> String {
        format!("{}/{}", self.node_name, self.param_name)
    }

    fn is_leaf(&self) -> bool {
        !self.is_namespace
    }

    fn get_parent_path(&self) -> Option<String> {
        if self.param_name.contains('.') {
            // For namespaced parameters, the parent is either another namespace or the node
            let parts: Vec<&str> = self.param_name.split('.').collect();
            if parts.len() > 1 {
                let parent_param = parts[0..parts.len() - 1].join(".");
                Some(format!("{}/{}", self.node_name, parent_param))
            } else {
                Some(self.node_name.clone())
            }
        } else {
            // Top-level parameters belong directly to the node
            Some(self.node_name.clone())
        }
    }
}

/// Type aliases for parameter-specific tree structures
pub type ParamTree = GenericTree<ParamInfo>;
pub type ParamTreeNode = super::abstract_tree::GenericTreeNode<ParamInfo>;
pub type ParamTreeItem = TreeItem<ParamInfo>;
