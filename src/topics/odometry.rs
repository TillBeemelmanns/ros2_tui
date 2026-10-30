//! Decoding of `nav_msgs/msg/Odometry` and `nav_msgs/msg/Path`, and the
//! trail of recent odometry poses the odometry viewer plots.

use super::cdr::{CdrError, CdrReader};
use std::collections::VecDeque;

pub const ODOMETRY_TYPE: &str = "nav_msgs/msg/Odometry";
pub const PATH_TYPE: &str = "nav_msgs/msg/Path";

/// Poses kept for the trail.
const TRAIL_LENGTH: usize = 3000;
/// A step longer than this, and longer than the reported speed explains,
/// is a jump: a reset or a relocalization.
const MIN_JUMP: f64 = 0.5;

pub fn is_odometry_type(msg_type: &str) -> bool {
    msg_type == ODOMETRY_TYPE
}

pub fn is_path_type(msg_type: &str) -> bool {
    msg_type == PATH_TYPE
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: [f64; 3],
    /// Quaternion (x, y, z, w).
    pub orientation: [f64; 4],
}

impl Pose {
    fn read(r: &mut CdrReader) -> Result<Self, CdrError> {
        Ok(Self {
            position: r.f64_array::<3>()?,
            orientation: r.f64_array::<4>()?,
        })
    }

    /// Heading around z in radians.
    pub fn yaw(&self) -> f64 {
        super::imu::euler(self.orientation)[2]
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Odometry {
    pub frame_id: String,
    pub child_frame_id: String,
    pub stamp: f64,
    pub pose: Pose,
    /// Row-major 6x6 over x, y, z, roll, pitch, yaw.
    pub pose_covariance: [f64; 36],
    pub linear: [f64; 3],
    pub angular: [f64; 3],
    pub twist_covariance: [f64; 36],
}

impl Odometry {
    pub fn speed(&self) -> f64 {
        let [x, y, z] = self.linear;
        (x * x + y * y + z * z).sqrt()
    }

    /// Standard deviations of x, y and yaw, if a covariance is given.
    pub fn pose_std_dev(&self) -> Option<[f64; 3]> {
        let c = &self.pose_covariance;
        let diagonal = [c[0], c[7], c[35]];
        diagonal
            .iter()
            .any(|v| *v > 0.0)
            .then(|| diagonal.map(|v| v.max(0.0).sqrt()))
    }
}

pub fn decode_odometry(payload: &[u8]) -> Result<Odometry, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, frame_id) = r.header()?;
    let child_frame_id = r.string()?;
    let pose = Pose::read(&mut r)?;
    let pose_covariance = r.f64_array::<36>()?;
    let linear = r.f64_array::<3>()?;
    let angular = r.f64_array::<3>()?;
    let twist_covariance = r.f64_array::<36>()?;
    Ok(Odometry {
        frame_id,
        child_frame_id,
        stamp,
        pose,
        pose_covariance,
        linear,
        angular,
        twist_covariance,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub frame_id: String,
    pub stamp: f64,
    pub poses: Vec<Pose>,
}

impl Path {
    /// Length along the poses in meters, in the plane.
    pub fn length(&self) -> f64 {
        self.poses
            .windows(2)
            .map(|w| planar_distance(&w[0], &w[1]))
            .sum()
    }
}

pub fn decode_path(payload: &[u8]) -> Result<Path, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, frame_id) = r.header()?;
    // A pose takes at least a stamp, a frame name and seven doubles.
    let count = r.sequence_len(8 + 4 + 56)?;
    let mut poses = Vec::with_capacity(count);
    for _ in 0..count {
        r.header()?;
        poses.push(Pose::read(&mut r)?);
    }
    Ok(Path {
        frame_id,
        stamp,
        poses,
    })
}

fn planar_distance(a: &Pose, b: &Pose) -> f64 {
    (b.position[0] - a.position[0]).hypot(b.position[1] - a.position[1])
}

/// Recent odometry positions, the distance travelled and the jumps seen.
#[derive(Debug, Default)]
pub struct Trail {
    pub points: VecDeque<(f64, f64)>,
    pub distance: f64,
    pub jumps: usize,
    last: Option<(Pose, f64)>,
}

impl Trail {
    pub fn push(&mut self, odom: &Odometry) {
        let pose = odom.pose;
        if let Some((last, last_stamp)) = self.last {
            let step = planar_distance(&last, &pose);
            let dt = (odom.stamp - last_stamp).abs();
            if step > MIN_JUMP && step > 3.0 * odom.speed() * dt {
                self.jumps += 1;
            } else {
                self.distance += step;
            }
        }
        self.last = Some((pose, odom.stamp));
        self.points.push_back((pose.position[0], pose.position[1]));
        if self.points.len() > TRAIL_LENGTH {
            self.points.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn pose(w: &mut CdrWriter, x: f64, y: f64) {
        [x, y, 0.0, 0.0, 0.0, 0.0, 1.0]
            .iter()
            .for_each(|v| w.f64(*v));
    }

    fn odom_msg(x: f64, y: f64, vx: f64) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("odom");
        w.string("base_link");
        pose(&mut w, x, y);
        (0..36).for_each(|i| w.f64(if i == 0 { 0.04 } else { 0.0 }));
        [vx, 0.0, 0.0, 0.0, 0.0, 0.3].iter().for_each(|v| w.f64(*v));
        (0..36).for_each(|_| w.f64(0.0));
        w.0
    }

    #[test]
    fn decodes_odometry() {
        let odom = decode_odometry(&odom_msg(1.0, 2.0, 0.5)).unwrap();
        assert_eq!(odom.child_frame_id, "base_link");
        assert_eq!(odom.pose.position, [1.0, 2.0, 0.0]);
        assert_eq!(odom.linear, [0.5, 0.0, 0.0]);
        assert_eq!(odom.angular[2], 0.3);
        assert_eq!(odom.pose_std_dev(), Some([0.2, 0.0, 0.0]));
    }

    #[test]
    fn trail_sums_distance_and_counts_jumps() {
        let mut trail = Trail::default();
        for x in [0.0, 0.1, 0.2, 5.0, 5.1] {
            // Same stamp for all: the 4.8 m step cannot be explained.
            trail.push(&decode_odometry(&odom_msg(x, 0.0, 1.0)).unwrap());
        }
        assert_eq!(trail.jumps, 1);
        assert!((trail.distance - 0.3).abs() < 1e-9);
        assert_eq!(trail.points.len(), 5);
    }

    #[test]
    fn decodes_path_and_its_length() {
        let mut w = CdrWriter::new();
        w.header("map");
        w.u32(3);
        for (x, y) in [(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)] {
            w.header("map");
            pose(&mut w, x, y);
        }
        let path = decode_path(&w.0).unwrap();
        assert_eq!(path.poses.len(), 3);
        assert!((path.length() - 6.0).abs() < 1e-9);
    }
}
