//! Decoding of `sensor_msgs/msg/PointCloud2` from its serialized (CDR) form
//! into points and the per-point values the viewer colors by.

use super::cdr::{CdrError, CdrReader};
use super::image::display_range;
use std::fmt;

pub const POINTCLOUD_TYPE: &str = "sensor_msgs/msg/PointCloud2";
pub const LASERSCAN_TYPE: &str = "sensor_msgs/msg/LaserScan";

/// Fields read as the intensity channel, in order of preference.
const INTENSITY_FIELDS: [&str; 3] = ["intensity", "reflectivity", "i"];

pub fn is_pointcloud_type(msg_type: &str) -> bool {
    msg_type == POINTCLOUD_TYPE
}

pub fn is_laserscan_type(msg_type: &str) -> bool {
    msg_type == LASERSCAN_TYPE
}

#[derive(Debug, Clone, PartialEq)]
pub enum CloudError {
    Cdr(CdrError),
    MissingXyz,
    UnsupportedField { name: String, datatype: u8 },
    InvalidLayout(String),
}

impl From<CdrError> for CloudError {
    fn from(e: CdrError) -> Self {
        CloudError::Cdr(e)
    }
}

impl fmt::Display for CloudError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CloudError::Cdr(e) => write!(f, "{}", e),
            CloudError::MissingXyz => write!(f, "cloud has no x, y and z fields"),
            CloudError::UnsupportedField { name, datatype } => {
                write!(f, "field '{}' has unknown datatype {}", name, datatype)
            }
            CloudError::InvalidLayout(why) => write!(f, "invalid cloud layout: {}", why),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    /// Points with finite coordinates, from at most the requested number of
    /// sampled points.
    pub points: Vec<[f32; 3]>,
    /// Intensity per point, if the cloud has such a field.
    pub intensity: Option<Vec<f32>>,
    pub intensity_field: Option<String>,
    /// Color per point, from a packed `rgb`/`rgba` field.
    pub rgb: Option<Vec<[u8; 3]>>,
    pub width: u32,
    pub height: u32,
    /// Points in the message.
    pub total_points: usize,
    /// Points examined; less than `total_points` for large clouds.
    pub sampled_points: usize,
    /// "name:type" of every field, e.g. "intensity:f32".
    pub fields: Vec<String>,
    pub frame_id: String,
    pub stamp: f64,
    /// Min/max of x, y and z over the valid points.
    pub bounds: Option<[(f32, f32); 3]>,
    /// Display ranges (2nd to 98th percentile) of the color channels.
    pub z_range: Option<(f32, f32)>,
    pub intensity_range: Option<(f32, f32)>,
    pub distance_range: Option<(f32, f32)>,
    /// Set when the cloud was made from a `LaserScan`.
    pub scan: Option<ScanInfo>,
}

/// What a `LaserScan` says about itself, beyond its points.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanInfo {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    /// Smallest and largest valid range measured.
    pub measured: Option<(f32, f32)>,
}

impl PointCloud {
    /// Share of examined points with finite coordinates.
    pub fn valid_ratio(&self) -> f64 {
        if self.sampled_points == 0 {
            0.0
        } else {
            self.points.len() as f64 / self.sampled_points as f64
        }
    }
}

#[derive(Debug, Clone)]
struct Field {
    name: String,
    offset: u32,
    datatype: u8,
}

/// Size in bytes and short name of a `sensor_msgs/PointField` datatype.
fn datatype_info(datatype: u8) -> Option<(usize, &'static str)> {
    Some(match datatype {
        1 => (1, "i8"),
        2 => (1, "u8"),
        3 => (2, "i16"),
        4 => (2, "u16"),
        5 => (4, "i32"),
        6 => (4, "u32"),
        7 => (4, "f32"),
        8 => (8, "f64"),
        _ => return None,
    })
}

fn read_bytes<const N: usize>(data: &[u8], offset: usize) -> [u8; N] {
    data[offset..offset + N].try_into().unwrap()
}

fn read_scalar(data: &[u8], offset: usize, datatype: u8, big_endian: bool) -> f32 {
    macro_rules! num {
        ($t:ty) => {{
            let b = read_bytes(data, offset);
            (if big_endian {
                <$t>::from_be_bytes(b)
            } else {
                <$t>::from_le_bytes(b)
            }) as f32
        }};
    }
    match datatype {
        1 => data[offset] as i8 as f32,
        2 => data[offset] as f32,
        3 => num!(i16),
        4 => num!(u16),
        5 => num!(i32),
        6 => num!(u32),
        7 => num!(f32),
        _ => num!(f64),
    }
}

/// Decode a serialized `PointCloud2`, examining at most `max_points` evenly
/// spaced points.
pub fn decode_message(payload: &[u8], max_points: usize) -> Result<PointCloud, CloudError> {
    let mut reader = CdrReader::new(payload)?;
    let (stamp, frame_id) = reader.header()?;
    let height = reader.u32()?;
    let width = reader.u32()?;
    let field_count = reader.u32()?;
    let mut fields = Vec::new();
    for _ in 0..field_count {
        let name = reader.string()?;
        let offset = reader.u32()?;
        let datatype = reader.u8()?;
        let _count = reader.u32()?;
        fields.push(Field {
            name,
            offset,
            datatype,
        });
    }
    let big_endian = reader.bool()?;
    let point_step = reader.u32()? as usize;
    let row_step = reader.u32()? as usize;
    let data = reader.byte_sequence()?;

    for field in &fields {
        let size = datatype_info(field.datatype)
            .ok_or_else(|| CloudError::UnsupportedField {
                name: field.name.clone(),
                datatype: field.datatype,
            })?
            .0;
        if field.offset as usize + size > point_step {
            return Err(CloudError::InvalidLayout(format!(
                "field '{}' ends past point_step {}",
                field.name, point_step
            )));
        }
    }
    let find = |name: &str| fields.iter().find(|f| f.name == name);
    let (Some(x), Some(y), Some(z)) = (find("x"), find("y"), find("z")) else {
        return Err(CloudError::MissingXyz);
    };
    let intensity = INTENSITY_FIELDS.iter().find_map(|name| find(name));
    // Packed colors are 4 bytes; rgb fields of other types are not packed.
    let rgb = find("rgb")
        .or_else(|| find("rgba"))
        .filter(|f| matches!(f.datatype, 6 | 7));

    let total_points = width as usize * height as usize;
    if total_points > 0 {
        let needed = row_step * (height as usize - 1) + width as usize * point_step;
        if point_step == 0 || row_step < width as usize * point_step || data.len() < needed {
            return Err(CloudError::InvalidLayout(format!(
                "{}x{} points of {} bytes (row_step {}) need {} bytes, data has {}",
                width,
                height,
                point_step,
                row_step,
                needed,
                data.len()
            )));
        }
    }

    let stride = total_points.div_ceil(max_points.max(1)).max(1);
    let mut points = Vec::with_capacity(total_points / stride + 1);
    let mut intensities = Vec::new();
    let mut colors = Vec::new();
    let mut sampled_points = 0;
    for index in (0..total_points).step_by(stride) {
        sampled_points += 1;
        let (row, col) = (index / width as usize, index % width as usize);
        let base = row * row_step + col * point_step;
        let value = |f: &Field| read_scalar(data, base + f.offset as usize, f.datatype, big_endian);
        let p = [value(x), value(y), value(z)];
        if !p.iter().all(|v| v.is_finite()) {
            continue;
        }
        points.push(p);
        if let Some(f) = intensity {
            intensities.push(value(f));
        }
        if let Some(f) = rgb {
            let b = read_bytes::<4>(data, base + f.offset as usize);
            let packed = if big_endian {
                u32::from_be_bytes(b)
            } else {
                u32::from_le_bytes(b)
            };
            colors.push([(packed >> 16) as u8, (packed >> 8) as u8, packed as u8]);
        }
    }

    let axis = |i: usize| -> Vec<f32> { points.iter().map(|p| p[i]).collect() };
    let bounds = (!points.is_empty()).then(|| {
        let min_max = |i: usize| {
            points.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
                (lo.min(p[i]), hi.max(p[i]))
            })
        };
        [min_max(0), min_max(1), min_max(2)]
    });
    let distances: Vec<f32> = points
        .iter()
        .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
        .collect();

    Ok(PointCloud {
        z_range: display_range(&axis(2)),
        distance_range: display_range(&distances),
        intensity_range: intensity.and_then(|_| display_range(&intensities)),
        intensity: intensity.map(|_| intensities),
        intensity_field: intensity.map(|f| f.name.clone()),
        rgb: rgb.map(|_| colors),
        points,
        width,
        height,
        total_points,
        sampled_points,
        fields: fields
            .iter()
            .map(|f| format!("{}:{}", f.name, datatype_info(f.datatype).unwrap().1))
            .collect(),
        frame_id,
        stamp,
        bounds,
        scan: None,
    })
}

/// Decode a serialized `LaserScan` into a cloud of its valid returns in the
/// scan plane (z = 0). Ranges outside [range_min, range_max] are invalid.
pub fn decode_laserscan(payload: &[u8]) -> Result<PointCloud, CloudError> {
    let mut reader = CdrReader::new(payload)?;
    let (stamp, frame_id) = reader.header()?;
    let angle_min = reader.f32()?;
    let angle_max = reader.f32()?;
    let angle_increment = reader.f32()?;
    let _time_increment = reader.f32()?;
    let _scan_time = reader.f32()?;
    let range_min = reader.f32()?;
    let range_max = reader.f32()?;
    let ranges = reader.f32_sequence()?;
    let intensities = reader.f32_sequence()?;
    // Drivers without intensities send an empty array.
    let has_intensity = intensities.len() == ranges.len() && !ranges.is_empty();

    let mut points = Vec::with_capacity(ranges.len());
    let mut kept_intensity = Vec::new();
    for (i, &range) in ranges.iter().enumerate() {
        if !range.is_finite() || range < range_min || range > range_max {
            continue;
        }
        let angle = angle_min + i as f32 * angle_increment;
        points.push([range * angle.cos(), range * angle.sin(), 0.0]);
        if has_intensity {
            kept_intensity.push(intensities[i]);
        }
    }

    let distances: Vec<f32> = points.iter().map(|p| p[0].hypot(p[1])).collect();
    let measured = distances.iter().fold(None, |acc: Option<(f32, f32)>, &d| {
        Some(acc.map_or((d, d), |(lo, hi)| (lo.min(d), hi.max(d))))
    });
    let bounds = (!points.is_empty()).then(|| {
        let min_max = |i: usize| {
            points.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
                (lo.min(p[i]), hi.max(p[i]))
            })
        };
        [min_max(0), min_max(1), (0.0, 0.0)]
    });

    Ok(PointCloud {
        z_range: Some((0.0, 1.0)),
        distance_range: display_range(&distances),
        intensity_range: has_intensity
            .then(|| display_range(&kept_intensity))
            .flatten(),
        intensity: has_intensity.then_some(kept_intensity),
        intensity_field: has_intensity.then(|| "intensities".to_string()),
        rgb: None,
        width: ranges.len() as u32,
        height: 1,
        total_points: ranges.len(),
        sampled_points: ranges.len(),
        fields: vec![],
        frame_id,
        stamp,
        bounds,
        points,
        scan: Some(ScanInfo {
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
            measured,
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    /// A cloud message with the given fields (name, offset, datatype) and
    /// raw point data.
    fn cloud_msg(
        width: u32,
        height: u32,
        fields: &[(&str, u32, u8)],
        point_step: u32,
        row_step: u32,
        data: &[u8],
    ) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("lidar");
        w.u32(height);
        w.u32(width);
        w.u32(fields.len() as u32);
        for (name, offset, datatype) in fields {
            w.string(name);
            w.u32(*offset);
            w.u8(*datatype);
            w.u32(1);
        }
        w.u8(0); // is_bigendian
        w.u32(point_step);
        w.u32(row_step);
        w.bytes(data);
        w.u8(1); // is_dense
        w.0
    }

    fn xyzi(points: &[[f32; 4]]) -> Vec<u8> {
        points
            .iter()
            .flat_map(|p| p.iter().flat_map(|v| v.to_le_bytes()))
            .collect()
    }

    const XYZI: [(&str, u32, u8); 4] =
        [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7)];

    #[test]
    fn decodes_points_intensity_and_stats() {
        let data = xyzi(&[[1.0, 2.0, 3.0, 10.0], [-1.0, 0.0, 0.5, 20.0]]);
        let msg = cloud_msg(2, 1, &XYZI, 16, 32, &data);
        let cloud = decode_message(&msg, 1000).unwrap();
        assert_eq!(cloud.frame_id, "lidar");
        assert_eq!(cloud.points, vec![[1.0, 2.0, 3.0], [-1.0, 0.0, 0.5]]);
        assert_eq!(cloud.intensity, Some(vec![10.0, 20.0]));
        assert_eq!(cloud.intensity_field.as_deref(), Some("intensity"));
        assert_eq!(cloud.bounds.unwrap()[0], (-1.0, 1.0));
        assert_eq!(cloud.fields[3], "intensity:f32");
        assert!(cloud.rgb.is_none());
    }

    #[test]
    fn skips_invalid_points_and_honors_row_padding() {
        // Organized 1x2 rows with 4 bytes of padding per row.
        let mut data = xyzi(&[[1.0, 1.0, 1.0, 0.0]]);
        data.extend([0; 4]);
        data.extend(xyzi(&[[f32::NAN, 0.0, 0.0, 0.0]]));
        data.extend([0; 4]);
        let msg = cloud_msg(1, 2, &XYZI, 16, 20, &data);
        let cloud = decode_message(&msg, 1000).unwrap();
        assert_eq!(cloud.points.len(), 1);
        assert_eq!(cloud.sampled_points, 2);
        assert_eq!(cloud.valid_ratio(), 0.5);
    }

    #[test]
    fn samples_large_clouds_evenly() {
        let points: Vec<[f32; 4]> = (0..1000).map(|i| [i as f32, 0.0, 0.0, 0.0]).collect();
        let msg = cloud_msg(1000, 1, &XYZI, 16, 16_000, &xyzi(&points));
        let cloud = decode_message(&msg, 100).unwrap();
        assert_eq!(cloud.total_points, 1000);
        assert_eq!(cloud.points.len(), 100);
        assert_eq!(cloud.points[1][0], 10.0);
    }

    #[test]
    fn unpacks_rgb_and_reads_integer_fields() {
        let mut data = Vec::new();
        for v in [1.5f32, 2.5, 3.5] {
            data.extend(v.to_le_bytes());
        }
        data.extend(0x00ff_8000u32.to_le_bytes()); // r=255 g=128 b=0
        data.extend(7u16.to_le_bytes()); // ring
        data.extend([0; 2]);
        let fields = [
            ("x", 0, 7),
            ("y", 4, 7),
            ("z", 8, 7),
            ("rgb", 12, 7),
            ("ring", 16, 4),
        ];
        let msg = cloud_msg(1, 1, &fields, 20, 20, &data);
        let cloud = decode_message(&msg, 10).unwrap();
        assert_eq!(cloud.rgb, Some(vec![[255, 128, 0]]));
        assert_eq!(cloud.fields[4], "ring:u16");
    }

    #[test]
    fn rejects_clouds_without_xyz_or_with_short_data() {
        let msg = cloud_msg(1, 1, &[("x", 0, 7)], 4, 4, &[0; 4]);
        assert_eq!(
            decode_message(&msg, 10).unwrap_err(),
            CloudError::MissingXyz
        );
        let msg = cloud_msg(4, 1, &XYZI, 16, 64, &[0; 32]);
        assert!(matches!(
            decode_message(&msg, 10),
            Err(CloudError::InvalidLayout(_))
        ));
    }

    fn scan_msg(ranges: &[f32], intensities: &[f32]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("laser");
        let increment = std::f32::consts::FRAC_PI_2;
        for v in [
            -std::f32::consts::FRAC_PI_2,
            std::f32::consts::FRAC_PI_2,
            increment,
            0.0,
            0.1,
            0.5,
            10.0,
        ] {
            w.f32(v);
        }
        w.f32_sequence(ranges);
        w.f32_sequence(intensities);
        w.0
    }

    #[test]
    fn decodes_laserscan_returns_in_the_scan_plane() {
        // Beams at -90, 0 and +90 degrees; the last is out of range.
        let msg = scan_msg(&[2.0, 3.0, 20.0], &[1.0, 2.0, 3.0]);
        let cloud = decode_laserscan(&msg).unwrap();
        assert_eq!(cloud.total_points, 3);
        assert_eq!(cloud.points.len(), 2);
        assert!(
            (cloud.points[0][1] + 2.0).abs() < 1e-5,
            "{:?}",
            cloud.points[0]
        );
        assert!(
            (cloud.points[1][0] - 3.0).abs() < 1e-5,
            "{:?}",
            cloud.points[1]
        );
        assert_eq!(cloud.intensity, Some(vec![1.0, 2.0]));
        let scan = cloud.scan.unwrap();
        assert_eq!(scan.range_max, 10.0);
        assert_eq!(scan.measured, Some((2.0, 3.0)));
    }

    #[test]
    fn laserscan_without_intensities() {
        let msg = scan_msg(&[1.0, f32::INFINITY, 1.0], &[]);
        let cloud = decode_laserscan(&msg).unwrap();
        assert_eq!(cloud.points.len(), 2);
        assert!(cloud.intensity.is_none());
        assert!((cloud.valid_ratio() - 2.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn empty_cloud_is_valid() {
        let msg = cloud_msg(0, 1, &XYZI, 16, 0, &[]);
        let cloud = decode_message(&msg, 10).unwrap();
        assert!(cloud.points.is_empty());
        assert!(cloud.bounds.is_none());
    }
}
