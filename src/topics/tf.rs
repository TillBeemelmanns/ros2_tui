//! Decoding of `tf2_msgs/msg/TFMessage` and the transform tree the TF
//! viewer shows, with the problems tf2 users run into: frames that stopped
//! updating, frames published with more than one parent, cycles, and trees
//! that are not connected.

use super::cdr::{CdrError, CdrReader};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::time::{Duration, Instant};

pub const TF_TYPE: &str = "tf2_msgs/msg/TFMessage";

/// Span over which update rates are measured.
const RATE_WINDOW: Duration = Duration::from_secs(5);
/// A frame published with two parents within this span has both.
const PARENT_WINDOW: Duration = Duration::from_secs(5);
/// A dynamic frame is stale after this many of its periods without an
/// update, and at least after `MIN_STALE_AGE`.
const STALE_PERIODS: f64 = 5.0;
const MIN_STALE_AGE: f64 = 1.0;

pub fn is_tf_type(msg_type: &str) -> bool {
    msg_type == TF_TYPE
}

/// The other topic of a TF pair: `/tf` for `/tf_static` and the other way
/// round, in the same namespace. The tree needs both.
pub fn companion_topic(topic: &str) -> Option<String> {
    if let Some(base) = topic.strip_suffix("tf_static") {
        Some(format!("{}tf", base))
    } else if topic == "/tf" || topic.ends_with("/tf") {
        Some(format!("{}_static", topic))
    } else {
        None
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    pub stamp: f64,
    pub translation: [f64; 3],
    /// Quaternion (x, y, z, w).
    pub rotation: [f64; 4],
}

pub fn decode_message(payload: &[u8]) -> Result<Vec<Transform>, CdrError> {
    let mut r = CdrReader::new(payload)?;
    // A transform takes at least a stamp, two strings and seven doubles.
    let count = r.sequence_len(8 + 8 + 56)?;
    let mut transforms = Vec::with_capacity(count);
    for _ in 0..count {
        let (stamp, parent) = r.header()?;
        let child = r.string()?;
        let translation = r.f64_array::<3>()?;
        let rotation = r.f64_array::<4>()?;
        // tf2 ignores a leading slash in frame names.
        transforms.push(Transform {
            parent: parent.trim_start_matches('/').to_string(),
            child: child.trim_start_matches('/').to_string(),
            stamp,
            translation,
            rotation,
        });
    }
    Ok(transforms)
}

/// The latest transform of a frame to its parent, and how it is updated.
#[derive(Debug, Clone)]
pub struct FrameEntry {
    pub parent: String,
    pub is_static: bool,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub stamp: f64,
    /// Receive time minus stamp of the latest update, in seconds.
    pub delay: f64,
    pub updated: Instant,
    arrivals: VecDeque<Instant>,
    /// Parents the frame was published with recently, and when.
    parents: BTreeMap<String, Instant>,
}

impl FrameEntry {
    /// Updates per second over the last seconds, if there were two or more.
    pub fn rate(&self, now: Instant) -> Option<f64> {
        let recent: Vec<&Instant> = self
            .arrivals
            .iter()
            .filter(|at| now.duration_since(**at) <= RATE_WINDOW)
            .collect();
        let span = recent
            .last()?
            .duration_since(**recent.first()?)
            .as_secs_f64();
        (recent.len() >= 2 && span > 0.0).then(|| (recent.len() - 1) as f64 / span)
    }

    pub fn age(&self, now: Instant) -> f64 {
        now.duration_since(self.updated).as_secs_f64()
    }

    /// Seconds without an update, if that is too long for a dynamic frame.
    pub fn stale_for(&self, now: Instant) -> Option<f64> {
        if self.is_static {
            return None;
        }
        // The rate of the arrivals before the last one, as the gap since
        // then must not count against it.
        let period = self
            .arrivals
            .iter()
            .zip(self.arrivals.iter().skip(1))
            .map(|(a, b)| b.duration_since(*a).as_secs_f64())
            .sum::<f64>()
            / (self.arrivals.len().max(2) - 1) as f64;
        let age = self.age(now);
        (age > (period * STALE_PERIODS).max(MIN_STALE_AGE)).then_some(age)
    }

    /// Parents seen within the last seconds, if more than one.
    pub fn conflicting_parents(&self, now: Instant) -> Vec<&str> {
        let recent: Vec<&str> = self
            .parents
            .iter()
            .filter(|(_, at)| now.duration_since(**at) <= PARENT_WINDOW)
            .map(|(name, _)| name.as_str())
            .collect();
        if recent.len() > 1 {
            recent
        } else {
            vec![]
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Issue {
    Stale(f64),
    MultipleParents(Vec<String>),
    Cycle,
}

impl Issue {
    pub fn is_error(&self) -> bool {
        !matches!(self, Issue::Stale(_))
    }

    pub fn text(&self) -> String {
        match self {
            Issue::Stale(age) => format!("no update for {:.1}s", age),
            Issue::MultipleParents(parents) => format!("parents {}", parents.join(", ")),
            Issue::Cycle => "in a cycle".to_string(),
        }
    }
}

/// A frame as a row of the tree view.
#[derive(Debug, Clone)]
pub struct TreeRow {
    pub frame: String,
    /// Box drawing before the name, like "│  ├─ ".
    pub prefix: String,
    pub depth: usize,
    pub issues: Vec<Issue>,
}

/// Frames merged from all TF messages received.
#[derive(Debug, Default)]
pub struct TfTree {
    pub frames: BTreeMap<String, FrameEntry>,
}

impl TfTree {
    /// Add transforms received at `now`; `wall_now` is the current time in
    /// seconds since the epoch, to tell how old their stamps are.
    pub fn update(
        &mut self,
        transforms: Vec<Transform>,
        is_static: bool,
        now: Instant,
        wall_now: f64,
    ) {
        for t in transforms {
            let entry = self
                .frames
                .entry(t.child.clone())
                .or_insert_with(|| FrameEntry {
                    parent: t.parent.clone(),
                    is_static,
                    translation: t.translation,
                    rotation: t.rotation,
                    stamp: t.stamp,
                    delay: 0.0,
                    updated: now,
                    arrivals: VecDeque::new(),
                    parents: BTreeMap::new(),
                });
            entry.parent = t.parent.clone();
            entry.is_static = is_static;
            entry.translation = t.translation;
            entry.rotation = t.rotation;
            entry.stamp = t.stamp;
            entry.delay = wall_now - t.stamp;
            entry.updated = now;
            entry.parents.insert(t.parent, now);
            entry.arrivals.push_back(now);
            while entry.arrivals.len() > 2 && now.duration_since(entry.arrivals[0]) > RATE_WINDOW {
                entry.arrivals.pop_front();
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Frames that are parents but have none: the roots of the trees.
    pub fn roots(&self) -> Vec<&str> {
        let parents: BTreeSet<&str> = self.frames.values().map(|e| e.parent.as_str()).collect();
        parents
            .into_iter()
            .filter(|p| !self.frames.contains_key(*p))
            .collect()
    }

    /// The frame and its ancestors up to the root, stopping at a cycle.
    pub fn chain(&self, frame: &str) -> Vec<String> {
        let mut chain = vec![frame.to_string()];
        let mut current = frame;
        while let Some(entry) = self.frames.get(current) {
            if chain.contains(&entry.parent) {
                break;
            }
            chain.push(entry.parent.clone());
            current = &entry.parent;
        }
        chain
    }

    /// The pose of `frame` in the root of its tree: translation and
    /// rotation, composed along the chain.
    pub fn in_root(&self, frame: &str) -> ([f64; 3], [f64; 4]) {
        let mut translation = [0.0; 3];
        let mut rotation = [0.0, 0.0, 0.0, 1.0];
        for name in self.chain(frame) {
            if let Some(e) = self.frames.get(&name) {
                // p_parent = q_e * p_child + t_e
                let rotated = rotate(e.rotation, translation);
                translation = [
                    rotated[0] + e.translation[0],
                    rotated[1] + e.translation[1],
                    rotated[2] + e.translation[2],
                ];
                rotation = multiply(e.rotation, rotation);
            }
        }
        (translation, rotation)
    }

    /// Issues of one frame at `now`.
    pub fn issues(&self, frame: &str, now: Instant) -> Vec<Issue> {
        let Some(entry) = self.frames.get(frame) else {
            return vec![];
        };
        let mut issues = vec![];
        let parents = entry.conflicting_parents(now);
        if !parents.is_empty() {
            issues.push(Issue::MultipleParents(
                parents.into_iter().map(String::from).collect(),
            ));
        }
        if let Some(age) = entry.stale_for(now) {
            issues.push(Issue::Stale(age));
        }
        issues
    }

    /// All frames as tree rows: each tree from its root, children sorted by
    /// name, then frames in cycles, which no root reaches.
    pub fn rows(&self, now: Instant) -> Vec<TreeRow> {
        let mut children: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for (child, entry) in &self.frames {
            children.entry(&entry.parent).or_default().push(child);
        }
        let mut rows = vec![];
        let mut visited = BTreeSet::new();
        for root in self.roots() {
            self.push_rows(root, "", "", 0, &children, &mut visited, &mut rows, now);
        }
        for frame in self.frames.keys() {
            if !visited.contains(frame.as_str()) {
                visited.insert(frame.as_str());
                let mut issues = self.issues(frame, now);
                issues.push(Issue::Cycle);
                rows.push(TreeRow {
                    frame: frame.clone(),
                    prefix: String::new(),
                    depth: 0,
                    issues,
                });
            }
        }
        rows
    }

    #[allow(clippy::too_many_arguments)]
    fn push_rows<'a>(
        &'a self,
        frame: &'a str,
        prefix: &str,
        child_prefix: &str,
        depth: usize,
        children: &BTreeMap<&'a str, Vec<&'a str>>,
        visited: &mut BTreeSet<&'a str>,
        rows: &mut Vec<TreeRow>,
        now: Instant,
    ) {
        if !visited.insert(frame) {
            return;
        }
        rows.push(TreeRow {
            frame: frame.to_string(),
            prefix: prefix.to_string(),
            depth,
            issues: self.issues(frame, now),
        });
        let kids = children.get(frame).map(Vec::as_slice).unwrap_or_default();
        for (i, kid) in kids.iter().enumerate() {
            let last = i + 1 == kids.len();
            let (branch, next) = if last {
                ("└─ ", "   ")
            } else {
                ("├─ ", "│  ")
            };
            self.push_rows(
                kid,
                &format!("{}{}", child_prefix, branch),
                &format!("{}{}", child_prefix, next),
                depth + 1,
                children,
                visited,
                rows,
                now,
            );
        }
    }
}

/// Hamilton product of quaternions (x, y, z, w).
pub fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// Rotate `v` by the quaternion `q` (x, y, z, w).
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let p = multiply(
        multiply(q, [v[0], v[1], v[2], 0.0]),
        [-q[0], -q[1], -q[2], q[3]],
    );
    [p[0], p[1], p[2]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn tf_msg(transforms: &[(&str, &str, [f64; 3], [f64; 4])]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.u32(transforms.len() as u32);
        for (parent, child, t, q) in transforms {
            w.header(parent);
            w.string(child);
            t.iter().for_each(|v| w.f64(*v));
            q.iter().for_each(|v| w.f64(*v));
        }
        w.0
    }

    const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

    fn yaw(degrees: f64) -> [f64; 4] {
        let half = degrees.to_radians() / 2.0;
        [0.0, 0.0, half.sin(), half.cos()]
    }

    fn tree(now: Instant) -> TfTree {
        let mut tree = TfTree::default();
        let dynamic = decode_message(&tf_msg(&[
            ("map", "odom", [1.0, 0.0, 0.0], IDENTITY),
            ("/odom", "base_link", [2.0, 0.0, 0.0], yaw(90.0)),
        ]))
        .unwrap();
        let fixed = decode_message(&tf_msg(&[
            ("base_link", "lidar", [1.0, 0.0, 0.5], IDENTITY),
            ("base_link", "imu", [0.0, 0.0, 0.1], IDENTITY),
        ]))
        .unwrap();
        tree.update(dynamic, false, now, 12.6);
        tree.update(fixed, true, now, 12.6);
        tree
    }

    #[test]
    fn decodes_transforms_and_strips_leading_slashes() {
        let transforms =
            decode_message(&tf_msg(&[("/map", "odom", [1.0, 2.0, 3.0], IDENTITY)])).unwrap();
        assert_eq!(transforms[0].parent, "map");
        assert_eq!(transforms[0].child, "odom");
        assert_eq!(transforms[0].translation, [1.0, 2.0, 3.0]);
        assert_eq!(transforms[0].stamp, 12.5);
    }

    #[test]
    fn rows_draw_the_tree_from_its_root() {
        let now = Instant::now();
        let rows: Vec<String> = tree(now)
            .rows(now)
            .iter()
            .map(|r| format!("{}{}", r.prefix, r.frame))
            .collect();
        assert_eq!(
            rows,
            [
                "map",
                "└─ odom",
                "   └─ base_link",
                "      ├─ imu",
                "      └─ lidar"
            ]
        );
    }

    #[test]
    fn composes_the_pose_in_the_root() {
        let now = Instant::now();
        let tree = tree(now);
        assert_eq!(tree.chain("lidar"), ["lidar", "base_link", "odom", "map"]);
        // base_link is turned 90° left, so the lidar's 1 m forward is +y.
        let (t, q) = tree.in_root("lidar");
        assert!(
            (t[0] - 3.0).abs() < 1e-9 && (t[1] - 1.0).abs() < 1e-9,
            "{t:?}"
        );
        assert!((t[2] - 0.5).abs() < 1e-9);
        assert!((q[2] - yaw(90.0)[2]).abs() < 1e-9);
    }

    #[test]
    fn finds_stale_frames_conflicting_parents_and_cycles() {
        let start = Instant::now();
        let mut tree = tree(start);
        let later = start + Duration::from_secs(3);
        // Static frames never go stale; the dynamic ones did not update.
        assert_eq!(tree.issues("lidar", later), vec![]);
        assert!(matches!(
            tree.issues("base_link", later)[..],
            [Issue::Stale(_)]
        ));

        let conflict = decode_message(&tf_msg(&[("map", "base_link", [0.0; 3], IDENTITY)]));
        tree.update(conflict.unwrap(), false, later, 0.0);
        assert_eq!(
            tree.issues("base_link", later),
            vec![Issue::MultipleParents(vec!["map".into(), "odom".into()])]
        );

        let cycle = decode_message(&tf_msg(&[
            ("a", "b", [0.0; 3], IDENTITY),
            ("b", "a", [0.0; 3], IDENTITY),
        ]));
        tree.update(cycle.unwrap(), true, later, 0.0);
        let rows = tree.rows(later);
        let cyclic: Vec<&str> = rows
            .iter()
            .filter(|r| r.issues.contains(&Issue::Cycle))
            .map(|r| r.frame.as_str())
            .collect();
        assert_eq!(cyclic, ["a", "b"]);
        assert_eq!(tree.chain("a"), ["a", "b"]);
    }

    #[test]
    fn pairs_tf_topics() {
        assert_eq!(companion_topic("/tf").as_deref(), Some("/tf_static"));
        assert_eq!(companion_topic("/tf_static").as_deref(), Some("/tf"));
        assert_eq!(
            companion_topic("/robot1/tf").as_deref(),
            Some("/robot1/tf_static")
        );
        assert_eq!(companion_topic("/my_tf_topic"), None);
    }
}
