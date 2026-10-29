//! Decoding of `sensor_msgs/msg/NavSatFix` and the track of recent fixes
//! the GNSS viewer plots.

use super::cdr::{CdrError, CdrReader};
use std::collections::VecDeque;

pub const NAVSATFIX_TYPE: &str = "sensor_msgs/msg/NavSatFix";

/// Fixes kept for the track plot.
const TRACK_LENGTH: usize = 2000;
const EARTH_RADIUS: f64 = 6_371_000.0;

pub fn is_navsatfix_type(msg_type: &str) -> bool {
    msg_type == NAVSATFIX_TYPE
}

#[derive(Debug, Clone, PartialEq)]
pub struct NavSatFix {
    pub status: i8,
    pub service: u16,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub covariance: [f64; 9],
    pub covariance_type: u8,
    pub frame_id: String,
    pub stamp: f64,
}

impl NavSatFix {
    pub fn has_fix(&self) -> bool {
        self.status >= 0 && self.latitude.is_finite() && self.longitude.is_finite()
    }

    pub fn status_name(&self) -> String {
        match self.status {
            -1 => "NO FIX".into(),
            0 => "FIX".into(),
            1 => "SBAS FIX".into(),
            2 => "GBAS FIX".into(),
            other => format!("status {}", other),
        }
    }

    /// Satellite systems in use, from the service bit mask.
    pub fn services(&self) -> String {
        let names = [(1, "GPS"), (2, "GLONASS"), (4, "BeiDou"), (8, "Galileo")];
        let used: Vec<&str> = names
            .iter()
            .filter(|(bit, _)| self.service & bit != 0)
            .map(|(_, name)| *name)
            .collect();
        if used.is_empty() {
            "none".into()
        } else {
            used.join(" ")
        }
    }

    pub fn covariance_type_name(&self) -> &'static str {
        match self.covariance_type {
            1 => "approximated",
            2 => "diagonal known",
            3 => "known",
            _ => "unknown",
        }
    }

    /// Horizontal and vertical standard deviation in meters, if reported.
    pub fn accuracy(&self) -> Option<(f64, f64)> {
        if self.covariance_type == 0 {
            return None;
        }
        let c = &self.covariance;
        Some((((c[0] + c[4]) / 2.0).max(0.0).sqrt(), c[8].max(0.0).sqrt()))
    }
}

pub fn decode_message(payload: &[u8]) -> Result<NavSatFix, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, frame_id) = r.header()?;
    let status = r.i8()?;
    let service = r.u16()?;
    let latitude = r.f64()?;
    let longitude = r.f64()?;
    let altitude = r.f64()?;
    let covariance = r.f64_array::<9>()?;
    let covariance_type = r.u8()?;
    Ok(NavSatFix {
        status,
        service,
        latitude,
        longitude,
        altitude,
        covariance,
        covariance_type,
        frame_id,
        stamp,
    })
}

/// Recent fixes as meters east and north of the first one.
#[derive(Debug, Default)]
pub struct Track {
    origin: Option<(f64, f64)>,
    pub points: VecDeque<(f64, f64)>,
}

impl Track {
    pub fn push(&mut self, fix: &NavSatFix) {
        if !fix.has_fix() {
            return;
        }
        let (lat0, lon0) = *self.origin.get_or_insert((fix.latitude, fix.longitude));
        let north = (fix.latitude - lat0).to_radians() * EARTH_RADIUS;
        let east = (fix.longitude - lon0).to_radians() * EARTH_RADIUS * lat0.to_radians().cos();
        self.points.push_back((east, north));
        if self.points.len() > TRACK_LENGTH {
            self.points.pop_front();
        }
    }

    /// Distance from the first fix to the latest, in meters.
    pub fn distance_from_start(&self) -> Option<f64> {
        self.points.back().map(|(e, n)| e.hypot(*n))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn fix_msg(status: i8, lat: f64, lon: f64, cov_type: u8) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("gps");
        w.u8(status as u8);
        w.u16(1 | 8);
        for v in [lat, lon, 120.5] {
            w.f64(v);
        }
        for i in 0..9 {
            w.f64(if i % 4 == 0 { 4.0 } else { 0.0 });
        }
        w.u8(cov_type);
        w.0
    }

    #[test]
    fn decodes_fix_status_services_and_accuracy() {
        let fix = decode_message(&fix_msg(2, 50.77, 6.08, 2)).unwrap();
        assert_eq!(fix.status_name(), "GBAS FIX");
        assert_eq!(fix.services(), "GPS Galileo");
        assert_eq!(
            (fix.latitude, fix.longitude, fix.altitude),
            (50.77, 6.08, 120.5)
        );
        assert_eq!(fix.accuracy(), Some((2.0, 2.0)));
        assert!(fix.has_fix());
        let no_fix = decode_message(&fix_msg(-1, 50.77, 6.08, 0)).unwrap();
        assert!(!no_fix.has_fix());
        assert_eq!(no_fix.accuracy(), None);
    }

    #[test]
    fn track_is_in_meters_from_the_first_fix() {
        let mut track = Track::default();
        track.push(&decode_message(&fix_msg(0, 50.0, 6.0, 0)).unwrap());
        // 0.001 degrees of latitude is about 111 m north.
        track.push(&decode_message(&fix_msg(0, 50.001, 6.0, 0)).unwrap());
        track.push(&decode_message(&fix_msg(-1, 0.0, 0.0, 0)).unwrap());
        assert_eq!(track.points.len(), 2);
        let (east, north) = track.points[1];
        assert!(
            east.abs() < 1e-6 && (north - 111.19).abs() < 0.1,
            "{east} {north}"
        );
        assert!((track.distance_from_start().unwrap() - 111.19).abs() < 0.1);
    }
}
