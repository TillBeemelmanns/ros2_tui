//! Decoding of `sensor_msgs/msg/JointState` and the joint table the joint
//! viewer shows, merged by name, as several sources often publish partial
//! states (e.g. an arm and its gripper) on one topic.

use super::cdr::{CdrError, CdrReader};
use std::collections::{BTreeMap, VecDeque};
use std::time::{Duration, Instant};

pub const JOINT_STATE_TYPE: &str = "sensor_msgs/msg/JointState";

/// Span of the history kept per joint for the plot.
pub const JOINT_HISTORY: Duration = Duration::from_secs(10);
/// A joint without an update for this long is shown as stale.
pub const JOINT_STALE_AFTER: Duration = Duration::from_secs(2);

pub fn is_joint_state_type(msg_type: &str) -> bool {
    msg_type == JOINT_STATE_TYPE
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointStates {
    pub stamp: f64,
    pub names: Vec<String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}

pub fn decode_message(payload: &[u8]) -> Result<JointStates, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, _) = r.header()?;
    Ok(JointStates {
        stamp,
        names: r.string_sequence()?,
        position: r.f64_sequence()?,
        velocity: r.f64_sequence()?,
        effort: r.f64_sequence()?,
    })
}

/// The latest state of one joint and what was seen of it.
#[derive(Debug, Clone)]
pub struct Joint {
    pub position: Option<f64>,
    pub velocity: Option<f64>,
    pub effort: Option<f64>,
    /// Lowest and highest position seen.
    pub range: Option<(f64, f64)>,
    pub updated: Instant,
    /// (time, position, velocity) of the last seconds.
    pub history: VecDeque<(Instant, Option<f64>, Option<f64>)>,
}

#[derive(Debug, Default)]
pub struct JointTable {
    pub joints: BTreeMap<String, Joint>,
    /// A message whose arrays do not match its names, if the last one did.
    pub malformed: Option<String>,
}

impl JointTable {
    pub fn update(&mut self, states: JointStates, now: Instant) {
        let n = states.names.len();
        let fits = |len: usize| len == 0 || len == n;
        self.malformed = (!(fits(states.position.len())
            && fits(states.velocity.len())
            && fits(states.effort.len())))
        .then(|| {
            format!(
                "{} names but {} positions, {} velocities, {} efforts",
                n,
                states.position.len(),
                states.velocity.len(),
                states.effort.len()
            )
        });
        for (i, name) in states.names.into_iter().enumerate() {
            let position = states.position.get(i).copied();
            let velocity = states.velocity.get(i).copied();
            let joint = self.joints.entry(name).or_insert_with(|| Joint {
                position: None,
                velocity: None,
                effort: None,
                range: None,
                updated: now,
                history: VecDeque::new(),
            });
            joint.position = position;
            joint.velocity = velocity;
            joint.effort = states.effort.get(i).copied();
            joint.updated = now;
            if let Some(p) = position.filter(|p| p.is_finite()) {
                joint.range = Some(match joint.range {
                    Some((lo, hi)) => (lo.min(p), hi.max(p)),
                    None => (p, p),
                });
            }
            joint.history.push_back((now, position, velocity));
            while joint
                .history
                .front()
                .is_some_and(|(at, _, _)| now.duration_since(*at) > JOINT_HISTORY)
            {
                joint.history.pop_front();
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.joints.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn msg(names: &[&str], position: &[f64], velocity: &[f64]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("");
        w.string_sequence(names);
        w.f64_sequence(position);
        w.f64_sequence(velocity);
        w.f64_sequence(&[]);
        w.0
    }

    #[test]
    fn merges_partial_states_and_tracks_ranges() {
        let now = Instant::now();
        let mut table = JointTable::default();
        table.update(
            decode_message(&msg(&["shoulder", "elbow"], &[0.5, -1.0], &[])).unwrap(),
            now,
        );
        table.update(
            decode_message(&msg(&["gripper"], &[0.02], &[0.1])).unwrap(),
            now,
        );
        table.update(
            decode_message(&msg(&["shoulder", "elbow"], &[1.5, -1.0], &[])).unwrap(),
            now,
        );
        assert_eq!(table.joints.len(), 3);
        let shoulder = &table.joints["shoulder"];
        assert_eq!(shoulder.range, Some((0.5, 1.5)));
        assert_eq!(shoulder.velocity, None);
        assert_eq!(table.joints["gripper"].velocity, Some(0.1));
        assert_eq!(table.malformed, None);
    }

    #[test]
    fn reports_arrays_that_do_not_match_the_names() {
        let mut table = JointTable::default();
        table.update(
            decode_message(&msg(&["a", "b"], &[1.0], &[])).unwrap(),
            Instant::now(),
        );
        assert_eq!(
            table.malformed.as_deref(),
            Some("2 names but 1 positions, 0 velocities, 0 efforts")
        );
    }
}
