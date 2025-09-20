use std::collections::HashMap;

/// Trait that defines what data can be stored in the tree
pub trait TreeData: Clone + std::fmt::Debug {
    /// Returns the display name for this data item
    fn get_display_name(&self) -> String;

    /// Returns the full path/key for this data item
    fn get_full_path(&self) -> String;

    /// Returns whether this item should be treated as a leaf (no children)
    fn is_leaf(&self) -> bool;

    /// Returns the parent path for hierarchical organization
    fn get_parent_path(&self) -> Option<String>;
}

/// A generic tree node that can store any type of data implementing TreeData
#[derive(Debug, Clone)]
pub struct GenericTreeNode<T: TreeData> {
    pub name: String,
    pub full_path: String,
    pub is_leaf: bool,
    pub data: Option<T>,
    pub children: HashMap<String, GenericTreeNode<T>>,
    pub is_expanded: bool,
}

impl<T: TreeData> GenericTreeNode<T> {
    pub fn new(name: String, full_path: String, is_leaf: bool) -> Self {
        Self {
            name,
            full_path,
            is_leaf,
            data: None,
            children: HashMap::new(),
            is_expanded: false, // Groups should start collapsed by default
        }
    }

    pub fn new_with_data(data: T) -> Self {
        let name = data.get_display_name();
        let full_path = data.get_full_path();
        let is_leaf = data.is_leaf();

        Self {
            name,
            full_path,
            is_leaf,
            data: Some(data),
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

/// Generic tree structure that can work with any TreeData type
#[derive(Debug, Clone)]
pub struct GenericTree<T: TreeData> {
    pub root: HashMap<String, GenericTreeNode<T>>,
}

impl<T: TreeData> Default for GenericTree<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: TreeData> GenericTree<T> {
    pub fn new() -> Self {
        Self {
            root: HashMap::new(),
        }
    }

    pub fn build_from_data(&mut self, data_items: &[T]) {
        self.root.clear();
        for item in data_items {
            self.insert_data(item.clone());
        }
    }

    fn insert_data(&mut self, data: T) {
        if let Some(parent_path) = data.get_parent_path() {
            // This is a hierarchical item - insert under parent
            self.ensure_parent_exists(&parent_path);

            if let Some(parent_node) = self.find_node_mut(&parent_path) {
                let leaf_node = GenericTreeNode::new_with_data(data.clone());
                let key = data.get_full_path();
                parent_node.children.insert(key, leaf_node);
            }
        } else {
            // This is a top-level item
            let node = GenericTreeNode::new_with_data(data.clone());
            let key = data.get_full_path();
            self.root.insert(key, node);
        }
    }

    fn ensure_parent_exists(&mut self, parent_path: &str) {
        if !self.root.contains_key(parent_path) {
            let parent_node = GenericTreeNode::new(
                parent_path.to_string(),
                parent_path.to_string(),
                false, // Groups are not leaves
            );
            self.root.insert(parent_path.to_string(), parent_node);
        }
    }

    fn find_node_mut(&mut self, path: &str) -> Option<&mut GenericTreeNode<T>> {
        self.root.get_mut(path)
    }

    pub fn get_flattened_view(&self) -> Vec<TreeItem<T>> {
        let mut items = Vec::new();
        let mut sorted_keys: Vec<_> = self.root.keys().collect();
        sorted_keys.sort();

        for key in sorted_keys {
            if let Some(node) = self.root.get(key) {
                self.add_node_to_flattened(&mut items, node, 0);
            }
        }
        items
    }

    #[allow(clippy::only_used_in_recursion)]
    fn add_node_to_flattened(
        &self,
        items: &mut Vec<TreeItem<T>>,
        node: &GenericTreeNode<T>,
        depth: usize,
    ) {
        items.push(TreeItem {
            node: node.clone(),
            depth,
        });

        if node.is_expanded && !node.children.is_empty() {
            let mut sorted_children: Vec<_> = node.children.values().collect();
            sorted_children.sort_by(|a, b| a.name.cmp(&b.name));

            for child in sorted_children {
                self.add_node_to_flattened(items, child, depth + 1);
            }
        }
    }
}

/// Represents an item in the flattened tree view
#[derive(Debug, Clone)]
pub struct TreeItem<T: TreeData> {
    pub node: GenericTreeNode<T>,
    pub depth: usize,
}

impl<T: TreeData> TreeItem<T> {
    pub fn is_group(&self) -> bool {
        !self.node.is_leaf
    }

    pub fn get_display_name(&self) -> String {
        if self.is_group() {
            if self.node.is_expanded {
                format!("▼ {}", self.node.name)
            } else {
                format!("▶ {}", self.node.name)
            }
        } else {
            self.node.name.clone()
        }
    }
}
