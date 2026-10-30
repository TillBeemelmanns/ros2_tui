//! Decoding of `sensor_msgs/msg/BatteryState` for the battery viewer.

use super::cdr::{CdrError, CdrReader};

pub const BATTERY_STATE_TYPE: &str = "sensor_msgs/msg/BatteryState";

pub fn is_battery_state_type(msg_type: &str) -> bool {
    msg_type == BATTERY_STATE_TYPE
}

/// Values are NaN when not measured, as the message defines.
#[derive(Debug, Clone, PartialEq)]
pub struct BatteryState {
    pub stamp: f64,
    pub voltage: f32,
    pub temperature: f32,
    /// Negative while discharging.
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    /// 0 to 1.
    pub percentage: f32,
    pub status: u8,
    pub health: u8,
    pub technology: u8,
    pub present: bool,
    pub cell_voltage: Vec<f32>,
    pub cell_temperature: Vec<f32>,
    pub location: String,
    pub serial_number: String,
}

pub fn decode_message(payload: &[u8]) -> Result<BatteryState, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, _) = r.header()?;
    Ok(BatteryState {
        stamp,
        voltage: r.f32()?,
        temperature: r.f32()?,
        current: r.f32()?,
        charge: r.f32()?,
        capacity: r.f32()?,
        design_capacity: r.f32()?,
        percentage: r.f32()?,
        status: r.u8()?,
        health: r.u8()?,
        technology: r.u8()?,
        present: r.bool()?,
        cell_voltage: r.f32_sequence()?,
        cell_temperature: r.f32_sequence()?,
        location: r.string()?,
        serial_number: r.string()?,
    })
}

/// A value if measured.
pub fn measured(v: f32) -> Option<f64> {
    v.is_finite().then_some(v as f64)
}

impl BatteryState {
    pub fn status_name(&self) -> &'static str {
        match self.status {
            1 => "charging",
            2 => "discharging",
            3 => "not charging",
            4 => "full",
            _ => "unknown",
        }
    }

    pub fn health_name(&self) -> &'static str {
        match self.health {
            1 => "good",
            2 => "overheat",
            3 => "dead",
            4 => "overvoltage",
            5 => "unspecified failure",
            6 => "cold",
            7 => "watchdog timer expired",
            8 => "safety timer expired",
            _ => "unknown",
        }
    }

    pub fn health_is_bad(&self) -> bool {
        self.health >= 2
    }

    pub fn technology_name(&self) -> &'static str {
        match self.technology {
            1 => "NiMH",
            2 => "Li-ion",
            3 => "LiPo",
            4 => "LiFe",
            5 => "NiCd",
            6 => "LiMn",
            7 => "Ternary",
            8 => "VRLA",
            _ => "unknown",
        }
    }

    /// Charge left in 0..1: the percentage, or charge over capacity.
    pub fn fraction(&self) -> Option<f64> {
        measured(self.percentage)
            .or_else(|| Some(measured(self.charge)? / measured(self.capacity)?))
            .map(|f| f.clamp(0.0, 1.0))
    }

    /// Hours until empty at the present current, while discharging.
    pub fn hours_left(&self) -> Option<f64> {
        let current = measured(self.current)?;
        let charge = measured(self.charge)?;
        (current < -0.01).then(|| charge / -current)
    }

    /// Lowest and highest cell voltage, if cells are reported.
    pub fn cell_spread(&self) -> Option<(f64, f64)> {
        let cells: Vec<f64> = self
            .cell_voltage
            .iter()
            .filter_map(|v| measured(*v))
            .collect();
        let lo = cells.iter().copied().reduce(f64::min)?;
        let hi = cells.iter().copied().reduce(f64::max)?;
        Some((lo, hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn msg(percentage: f32, charge: f32, current: f32) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("battery");
        for v in [24.1, 31.5, current, charge, 20.0, 22.0, percentage] {
            w.f32(v);
        }
        for v in [2, 1, 2, 1] {
            w.u8(v);
        }
        w.u32(3);
        for v in [4.01f32, 3.98, 3.71] {
            w.0.extend_from_slice(&v.to_le_bytes());
        }
        w.u32(0);
        w.string("base");
        w.string("SN1");
        w.0
    }

    #[test]
    fn decodes_state_and_derived_values() {
        let b = decode_message(&msg(0.62, 12.4, -6.2)).unwrap();
        assert_eq!(b.status_name(), "discharging");
        assert_eq!(b.health_name(), "good");
        assert_eq!(b.technology_name(), "Li-ion");
        assert!(b.present);
        assert_eq!(b.location, "base");
        assert!((b.fraction().unwrap() - 0.62).abs() < 1e-6);
        assert!((b.hours_left().unwrap() - 2.0).abs() < 1e-6);
        let (lo, hi) = b.cell_spread().unwrap();
        assert!((lo - 3.71).abs() < 1e-6 && (hi - 4.01).abs() < 1e-6);
    }

    #[test]
    fn falls_back_to_charge_over_capacity() {
        let b = decode_message(&msg(f32::NAN, 5.0, 1.0)).unwrap();
        assert!((b.fraction().unwrap() - 0.25).abs() < 1e-6);
        // Charging: no time left to show.
        assert_eq!(b.hours_left(), None);
    }
}
