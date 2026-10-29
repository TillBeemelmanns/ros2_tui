//! Decoding of `nav_msgs/msg/OccupancyGrid` (maps and costmaps) into an
//! image with north up, plus the map's geometry and coverage.

use super::cdr::{CdrError, CdrReader};
use super::image::{fit_within, DecodedImage};
use image::{imageops, RgbImage};

pub const OCCUPANCY_GRID_TYPE: &str = "nav_msgs/msg/OccupancyGrid";

/// Cells at or above this probability count as occupied, at or below
/// `FREE_THRESHOLD` as free, like map_server's defaults.
const OCCUPIED_THRESHOLD: i8 = 65;
const FREE_THRESHOLD: i8 = 25;
const UNKNOWN_COLOR: [u8; 3] = [40, 48, 64];

pub fn is_occupancy_grid_type(msg_type: &str) -> bool {
    msg_type == OCCUPANCY_GRID_TYPE
}

/// Free is white, occupied black, in between gray; unknown is slate.
fn cell_color(value: i8) -> [u8; 3] {
    if !(0..=100).contains(&value) {
        return UNKNOWN_COLOR;
    }
    let v = 255 - (value as u16 * 255 / 100) as u8;
    [v, v, v]
}

struct Grid<'a> {
    stamp: f64,
    frame_id: String,
    resolution: f32,
    width: u32,
    height: u32,
    position: [f64; 3],
    /// Rotation of the map about z, from the origin quaternion.
    yaw: f64,
    data: &'a [u8],
}

fn read_grid(payload: &[u8]) -> Result<Grid<'_>, CdrError> {
    let mut r = CdrReader::new(payload)?;
    let (stamp, frame_id) = r.header()?;
    let _map_load_time = (r.i32()?, r.u32()?);
    let resolution = r.f32()?;
    let width = r.u32()?;
    let height = r.u32()?;
    let position = r.f64_array::<3>()?;
    let [_, _, qz, qw] = r.f64_array::<4>()?;
    let data = r.byte_sequence()?;
    Ok(Grid {
        stamp,
        frame_id,
        resolution,
        width,
        height,
        position,
        yaw: 2.0 * qz.atan2(qw),
        data,
    })
}

/// Decode a serialized `OccupancyGrid`, downscaled to fit `max_width` x
/// `max_height`. Grid row 0 is the map's bottom, so rows are flipped.
pub fn decode_message(
    payload: &[u8],
    max_width: u32,
    max_height: u32,
) -> Result<DecodedImage, String> {
    let Grid {
        stamp,
        frame_id,
        resolution,
        width,
        height,
        position,
        yaw,
        data,
    } = read_grid(payload).map_err(|e| e.to_string())?;

    let cells = width as usize * height as usize;
    if width == 0 || height == 0 {
        return Err("map is empty".into());
    }
    if data.len() < cells {
        return Err(format!(
            "{}x{} map needs {} cells, data has {}",
            width,
            height,
            cells,
            data.len()
        ));
    }

    let mut rgb = Vec::with_capacity(cells * 3);
    let (mut known, mut occupied) = (0usize, 0usize);
    for row in (0..height as usize).rev() {
        for &cell in &data[row * width as usize..(row + 1) * width as usize] {
            let value = cell as i8;
            if (0..=100).contains(&value) {
                known += 1;
                if value >= OCCUPIED_THRESHOLD {
                    occupied += 1;
                }
            }
            rgb.extend_from_slice(&cell_color(value));
        }
    }
    let image = RgbImage::from_raw(width, height, rgb).unwrap();
    let (w, h) = fit_within(width, height, max_width, max_height);
    let image = if (w, h) == (width, height) {
        image
    } else {
        imageops::thumbnail(&image, w, h)
    };

    let percent = |n: usize| format!("{:.1}%", n as f64 * 100.0 / cells as f64);
    Ok(DecodedImage {
        rgb: image,
        source_width: width,
        source_height: height,
        encoding: "cells".into(),
        frame_id,
        stamp,
        extra: vec![
            ("Resolution", format!("{} m", resolution)),
            (
                "Size",
                format!(
                    "{:.1}x{:.1} m",
                    width as f32 * resolution,
                    height as f32 * resolution
                ),
            ),
            (
                "Origin",
                format!(
                    "({:.2}, {:.2}) {:.1}°",
                    position[0],
                    position[1],
                    yaw.to_degrees()
                ),
            ),
            ("Known", percent(known)),
            ("Occupied", percent(occupied)),
            ("Free", percent(known - occupied - unclear(data, cells))),
        ],
    })
}

/// Known cells between the free and occupied thresholds.
fn unclear(data: &[u8], cells: usize) -> usize {
    data[..cells]
        .iter()
        .filter(|&&c| {
            let v = c as i8;
            v > FREE_THRESHOLD && v < OCCUPIED_THRESHOLD
        })
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn grid_msg(width: u32, height: u32, data: &[i8]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("map");
        w.u32(0);
        w.u32(0);
        w.f32(0.05);
        w.u32(width);
        w.u32(height);
        for v in [-10.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0] {
            w.f64(v);
        }
        w.bytes(&data.iter().map(|&v| v as u8).collect::<Vec<_>>());
        w.0
    }

    #[test]
    fn colors_cells_and_puts_row_zero_at_the_bottom() {
        // Bottom row: free, occupied. Top row: unknown, 50%.
        let msg = grid_msg(2, 2, &[0, 100, -1, 50]);
        let map = decode_message(&msg, 100, 100).unwrap();
        assert_eq!(map.rgb.get_pixel(0, 1).0, [255, 255, 255]);
        assert_eq!(map.rgb.get_pixel(1, 1).0, [0, 0, 0]);
        assert_eq!(map.rgb.get_pixel(0, 0).0, UNKNOWN_COLOR);
        assert_eq!(map.rgb.get_pixel(1, 0).0, [128, 128, 128]);
        let extra = |label| {
            map.extra
                .iter()
                .find(|(l, _)| *l == label)
                .unwrap()
                .1
                .clone()
        };
        assert_eq!(extra("Known"), "75.0%");
        assert_eq!(extra("Occupied"), "25.0%");
        assert_eq!(extra("Free"), "25.0%");
        assert_eq!(extra("Origin"), "(-10.00, -5.00) 0.0°");
        assert_eq!(extra("Size"), "0.1x0.1 m");
    }

    #[test]
    fn rejects_short_data() {
        let msg = grid_msg(4, 4, &[0; 3]);
        assert!(decode_message(&msg, 100, 100).is_err());
    }
}
