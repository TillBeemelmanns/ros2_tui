//! Decoding of `sensor_msgs/msg/Imu` and the sanity checks the IMU viewer
//! shows: orientation as angles and which axis gravity is measured on.

use super::cdr::{CdrError, CdrReader};

pub const IMU_TYPE: &str = "sensor_msgs/msg/Imu";

pub fn is_imu_type(msg_type: &str) -> bool {
    msg_type == IMU_TYPE
}

#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    /// Quaternion (x, y, z, w); `None` when the covariance marks it as not
    /// provided (first element -1).
    pub orientation: Option<[f64; 4]>,
    /// rad/s
    pub angular_velocity: Option<[f64; 3]>,
    /// m/s²
    pub linear_acceleration: Option<[f64; 3]>,
    pub frame_id: String,
    pub stamp: f64,
}

pub fn decode_message(payload: &[u8]) -> Result<ImuSample, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, frame_id) = r.header()?;
    let orientation = r.f64_array::<4>()?;
    let orientation_cov = r.f64_array::<9>()?;
    let angular_velocity = r.f64_array::<3>()?;
    let angular_velocity_cov = r.f64_array::<9>()?;
    let linear_acceleration = r.f64_array::<3>()?;
    let linear_acceleration_cov = r.f64_array::<9>()?;
    // REP 145: a covariance starting with -1 means "not provided".
    let provided = |cov: &[f64; 9]| cov[0] != -1.0;
    Ok(ImuSample {
        orientation: provided(&orientation_cov).then_some(orientation),
        angular_velocity: provided(&angular_velocity_cov).then_some(angular_velocity),
        linear_acceleration: provided(&linear_acceleration_cov).then_some(linear_acceleration),
        frame_id,
        stamp,
    })
}

/// Roll, pitch and yaw in radians from a quaternion (x, y, z, w).
pub fn euler([x, y, z, w]: [f64; 4]) -> [f64; 3] {
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw]
}

/// Roll and pitch in radians implied by the acceleration, assuming it is
/// mostly gravity: what the orientation should roughly agree with at rest.
pub fn tilt_from_gravity([ax, ay, az]: [f64; 3]) -> [f64; 2] {
    [ay.atan2(az), (-ax).atan2(ay.hypot(az))]
}

/// The axis gravity is measured on, like "+z": the largest component of the
/// acceleration and its sign.
pub fn gravity_axis(acceleration: [f64; 3]) -> &'static str {
    let (axis, value) = acceleration
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .unwrap();
    match (axis, *value >= 0.0) {
        (0, true) => "+x",
        (0, false) => "-x",
        (1, true) => "+y",
        (1, false) => "-y",
        (_, true) => "+z",
        (_, false) => "-z",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn imu_msg(
        orientation: [f64; 4],
        gyro: [f64; 3],
        accel: [f64; 3],
        orientation_cov0: f64,
    ) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("imu_link");
        let cov = |w: &mut CdrWriter, first: f64| {
            w.f64(first);
            for _ in 0..8 {
                w.f64(0.0);
            }
        };
        orientation.iter().for_each(|v| w.f64(*v));
        cov(&mut w, orientation_cov0);
        gyro.iter().for_each(|v| w.f64(*v));
        cov(&mut w, 0.01);
        accel.iter().for_each(|v| w.f64(*v));
        cov(&mut w, 0.01);
        w.0
    }

    #[test]
    fn decodes_and_honors_not_provided_orientation() {
        let msg = imu_msg([0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3], [0.0, 0.0, 9.81], 0.0);
        let imu = decode_message(&msg).unwrap();
        assert_eq!(imu.orientation, Some([0.0, 0.0, 0.0, 1.0]));
        assert_eq!(imu.angular_velocity, Some([0.1, 0.2, 0.3]));
        assert_eq!(imu.linear_acceleration, Some([0.0, 0.0, 9.81]));
        let msg = imu_msg([0.0; 4], [0.0; 3], [0.0; 3], -1.0);
        assert_eq!(decode_message(&msg).unwrap().orientation, None);
    }

    #[test]
    fn euler_angles_of_a_yaw_rotation() {
        let half = 45f64.to_radians();
        let [roll, pitch, yaw] = euler([0.0, 0.0, half.sin(), half.cos()]);
        assert!(roll.abs() < 1e-9 && pitch.abs() < 1e-9);
        assert!((yaw.to_degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn gravity_checks() {
        assert_eq!(gravity_axis([0.1, -0.2, 9.8]), "+z");
        assert_eq!(gravity_axis([-9.8, 0.0, 0.3]), "-x");
        let [roll, pitch] = tilt_from_gravity([0.0, 0.0, 9.81]);
        assert!(roll.abs() < 1e-9 && pitch.abs() < 1e-9);
        let [roll, _] = tilt_from_gravity([0.0, 9.81 * 0.5, 9.81 * 0.75f64.sqrt()]);
        assert!((roll.to_degrees() - 30.0).abs() < 1e-6);
    }
}
