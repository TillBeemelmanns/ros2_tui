//! Decoding of `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage`
//! from their serialized (CDR) form into RGB pixels for display.

use super::cdr::{CdrError, CdrReader};
use image::{imageops, DynamicImage, ImageFormat, RgbImage};
use std::fmt;

pub const IMAGE_TYPE: &str = "sensor_msgs/msg/Image";
pub const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/msg/CompressedImage";

/// Whether the image viewer can display messages of `msg_type`.
pub fn is_image_type(msg_type: &str) -> bool {
    msg_type == IMAGE_TYPE || msg_type == COMPRESSED_IMAGE_TYPE
}

#[derive(Debug, Clone)]
pub struct DecodedImage {
    /// Pixels, already downscaled to fit the requested display size.
    pub rgb: RgbImage,
    pub source_width: u32,
    pub source_height: u32,
    /// `encoding` of a raw image, `format` of a compressed one.
    pub encoding: String,
    pub frame_id: String,
    /// Header stamp in seconds.
    pub stamp: f64,
    /// Further (label, value) facts for the info line, e.g. of a map.
    pub extra: Vec<(&'static str, String)>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ImageError {
    Truncated,
    UnsupportedEncapsulation(u16),
    UnsupportedMessageType(String),
    UnsupportedEncoding(String),
    InvalidDimensions { width: u32, height: u32, step: u32 },
    Decode(String),
}

impl From<CdrError> for ImageError {
    fn from(e: CdrError) -> Self {
        match e {
            CdrError::Truncated => ImageError::Truncated,
            CdrError::UnsupportedEncapsulation(kind) => ImageError::UnsupportedEncapsulation(kind),
        }
    }
}

impl fmt::Display for ImageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImageError::Truncated => write!(f, "message is truncated"),
            ImageError::UnsupportedEncapsulation(kind) => {
                write!(f, "unsupported CDR encapsulation 0x{:04x}", kind)
            }
            ImageError::UnsupportedMessageType(t) => write!(f, "no image viewer for {}", t),
            ImageError::UnsupportedEncoding(e) => write!(f, "unsupported encoding '{}'", e),
            ImageError::InvalidDimensions {
                width,
                height,
                step,
            } => write!(
                f,
                "invalid dimensions {}x{} (step {}) for the data size",
                width, height, step
            ),
            ImageError::Decode(e) => write!(f, "decode failed: {}", e),
        }
    }
}

/// Decode a serialized image message and downscale it to fit within
/// `max_width` x `max_height` (aspect ratio kept, never upscaled).
pub fn decode_message(
    msg_type: &str,
    payload: &[u8],
    max_width: u32,
    max_height: u32,
) -> Result<DecodedImage, ImageError> {
    let mut reader = CdrReader::new(payload)?;
    let (stamp, frame_id) = reader.header()?;

    let (rgb, encoding) = match msg_type {
        IMAGE_TYPE => {
            let height = reader.u32()?;
            let width = reader.u32()?;
            let encoding = reader.string()?;
            let big_endian = reader.bool()?;
            let step = reader.u32()?;
            let data = reader.byte_sequence()?;
            let raw = RawImage {
                width,
                height,
                step,
                big_endian,
                data,
            };
            (decode_raw(&raw, &encoding)?, encoding)
        }
        COMPRESSED_IMAGE_TYPE => {
            let format = reader.string()?;
            let data = reader.byte_sequence()?;
            (decode_compressed(&format, data)?, format)
        }
        other => return Err(ImageError::UnsupportedMessageType(other.to_string())),
    };

    let (source_width, source_height) = rgb.dimensions();
    let (width, height) = fit_within(source_width, source_height, max_width, max_height);
    let rgb = if (width, height) == (source_width, source_height) {
        rgb
    } else {
        imageops::thumbnail(&rgb, width, height)
    };

    Ok(DecodedImage {
        rgb,
        source_width,
        source_height,
        encoding,
        frame_id,
        stamp,
        extra: Vec::new(),
    })
}

/// Largest size with the aspect ratio of `width` x `height` that fits within
/// `max_width` x `max_height`, without upscaling.
pub fn fit_within(width: u32, height: u32, max_width: u32, max_height: u32) -> (u32, u32) {
    if width == 0 || height == 0 {
        return (width, height);
    }
    let scale = (max_width as f64 / width as f64)
        .min(max_height as f64 / height as f64)
        .min(1.0);
    (
        ((width as f64 * scale).round() as u32).max(1),
        ((height as f64 * scale).round() as u32).max(1),
    )
}

struct RawImage<'a> {
    width: u32,
    height: u32,
    step: u32,
    big_endian: bool,
    data: &'a [u8],
}

impl RawImage<'_> {
    /// Row `y`, checked to hold `bytes_per_pixel` * width bytes.
    fn row(&self, y: u32) -> &[u8] {
        let start = y as usize * self.step as usize;
        &self.data[start..start + self.step as usize]
    }

    fn u16_at(&self, row: &[u8], i: usize) -> u16 {
        let b = [row[2 * i], row[2 * i + 1]];
        if self.big_endian {
            u16::from_be_bytes(b)
        } else {
            u16::from_le_bytes(b)
        }
    }

    fn f32_at(&self, row: &[u8], i: usize) -> f32 {
        let b: [u8; 4] = row[4 * i..4 * i + 4].try_into().unwrap();
        if self.big_endian {
            f32::from_be_bytes(b)
        } else {
            f32::from_le_bytes(b)
        }
    }
}

fn decode_raw(raw: &RawImage, encoding: &str) -> Result<RgbImage, ImageError> {
    let encoding_lc = encoding.to_ascii_lowercase();
    let bytes_per_pixel = match encoding_lc.as_str() {
        "rgb8" | "bgr8" | "8uc3" => 3,
        "rgba8" | "bgra8" | "8uc4" => 4,
        "rgb16" | "bgr16" | "16uc3" => 6,
        "rgba16" | "bgra16" | "16uc4" => 8,
        "mono8" | "8uc1" => 1,
        "mono16" | "16uc1" => 2,
        "32fc1" => 4,
        "yuv422" | "uyvy" | "yuv422_yuy2" | "yuyv" => 2,
        e if e.starts_with("bayer_") && e.ends_with('8') => 1,
        _ => return Err(ImageError::UnsupportedEncoding(encoding.to_string())),
    };

    let (width, height, step) = (raw.width, raw.height, raw.step);
    let fits = width > 0
        && height > 0
        && (step as u64) >= width as u64 * bytes_per_pixel
        && (step as u64) * (height as u64) <= raw.data.len() as u64;
    if !fits {
        return Err(ImageError::InvalidDimensions {
            width,
            height,
            step,
        });
    }

    let w = width as usize;
    let mut out = Vec::with_capacity(w * height as usize * 3);
    match encoding_lc.as_str() {
        "rgb8" | "bgr8" | "8uc3" | "rgba8" | "bgra8" | "8uc4" => {
            let n = bytes_per_pixel as usize;
            // Like OpenCV, the generic 8UCn encodings are taken as BGR(A).
            let bgr = !encoding_lc.starts_with("rgb");
            for y in 0..height {
                for px in raw.row(y)[..w * n].chunks_exact(n) {
                    if bgr {
                        out.extend_from_slice(&[px[2], px[1], px[0]]);
                    } else {
                        out.extend_from_slice(&px[..3]);
                    }
                }
            }
        }
        "rgb16" | "bgr16" | "16uc3" | "rgba16" | "bgra16" | "16uc4" => {
            let channels = (bytes_per_pixel / 2) as usize;
            let bgr = !encoding_lc.starts_with("rgb");
            for y in 0..height {
                let row = raw.row(y);
                for x in 0..w {
                    let c = |k: usize| (raw.u16_at(row, x * channels + k) >> 8) as u8;
                    if bgr {
                        out.extend_from_slice(&[c(2), c(1), c(0)]);
                    } else {
                        out.extend_from_slice(&[c(0), c(1), c(2)]);
                    }
                }
            }
        }
        "mono8" | "8uc1" => {
            for y in 0..height {
                for &v in &raw.row(y)[..w] {
                    out.extend_from_slice(&[v, v, v]);
                }
            }
        }
        "mono16" => {
            let values = collect_values(raw, |row, x| raw.u16_at(row, x) as f32);
            return Ok(normalize_to_rgb(&values, width, height, |t| {
                let v = (t * 255.0) as u8;
                [v, v, v]
            }));
        }
        "16uc1" | "32fc1" => {
            let values = if encoding_lc == "16uc1" {
                // 16-bit depth is in millimeters with 0 meaning "no reading".
                collect_values(raw, |row, x| match raw.u16_at(row, x) {
                    0 => f32::NAN,
                    v => v as f32,
                })
            } else {
                collect_values(raw, |row, x| raw.f32_at(row, x))
            };
            return Ok(colorize_depth(&values, width, height));
        }
        "yuv422" | "uyvy" | "yuv422_yuy2" | "yuyv" => {
            let uyvy = matches!(encoding_lc.as_str(), "yuv422" | "uyvy");
            for y in 0..height {
                let row = &raw.row(y)[..w * 2];
                for x in 0..w {
                    let pair = &row[(x / 2) * 4..(x / 2) * 4 + 4];
                    let (luma, u, v) = if uyvy {
                        (row[x * 2 + 1], pair[0], pair[2])
                    } else {
                        (row[x * 2], pair[1], pair[3])
                    };
                    out.extend_from_slice(&yuv_to_rgb(luma, u, v));
                }
            }
        }
        bayer => {
            // Demosaic each 2x2 cell into one pixel: half resolution, but
            // cheap, artifact free and plenty for a terminal preview.
            let pattern = &bayer["bayer_".len()..bayer.len() - 1];
            let (cw, ch) = ((width / 2).max(1), (height / 2).max(1));
            if width < 2 || height < 2 {
                return Err(ImageError::InvalidDimensions {
                    width,
                    height,
                    step,
                });
            }
            for cy in 0..ch {
                let top = raw.row(cy * 2);
                let bottom = raw.row(cy * 2 + 1);
                for cx in 0..cw as usize {
                    let q = [
                        top[cx * 2],
                        top[cx * 2 + 1],
                        bottom[cx * 2],
                        bottom[cx * 2 + 1],
                    ];
                    let (r, g, b) = match pattern {
                        "rggb" => (q[0], avg(q[1], q[2]), q[3]),
                        "bggr" => (q[3], avg(q[1], q[2]), q[0]),
                        "gbrg" => (q[2], avg(q[0], q[3]), q[1]),
                        "grbg" => (q[1], avg(q[0], q[3]), q[2]),
                        _ => return Err(ImageError::UnsupportedEncoding(encoding.to_string())),
                    };
                    out.extend_from_slice(&[r, g, b]);
                }
            }
            return Ok(RgbImage::from_raw(cw, ch, out).unwrap());
        }
    }
    Ok(RgbImage::from_raw(width, height, out).unwrap())
}

fn avg(a: u8, b: u8) -> u8 {
    ((a as u16 + b as u16) / 2) as u8
}

fn yuv_to_rgb(y: u8, u: u8, v: u8) -> [u8; 3] {
    // BT.601 limited range.
    let c = (y as f32 - 16.0) * 1.164;
    let d = u as f32 - 128.0;
    let e = v as f32 - 128.0;
    let clamp = |x: f32| x.round().clamp(0.0, 255.0) as u8;
    [
        clamp(c + 1.596 * e),
        clamp(c - 0.392 * d - 0.813 * e),
        clamp(c + 2.017 * d),
    ]
}

fn collect_values(raw: &RawImage, value: impl Fn(&[u8], usize) -> f32) -> Vec<f32> {
    let w = raw.width as usize;
    let mut values = Vec::with_capacity(w * raw.height as usize);
    for y in 0..raw.height {
        let row = raw.row(y);
        values.extend((0..w).map(|x| value(row, x)));
    }
    values
}

/// Robust display range of `values`: the 2nd to 98th percentile of the finite
/// ones, estimated from an evenly spaced sample.
pub(crate) fn display_range(values: &[f32]) -> Option<(f32, f32)> {
    const SAMPLES: usize = 4096;
    let stride = (values.len() / SAMPLES).max(1);
    let mut sample: Vec<f32> = values
        .iter()
        .step_by(stride)
        .copied()
        .filter(|v| v.is_finite())
        .collect();
    if sample.is_empty() {
        return None;
    }
    sample.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let at = |p: f32| sample[((sample.len() - 1) as f32 * p) as usize];
    let (lo, hi) = (at(0.02), at(0.98));
    Some((lo, if hi > lo { hi } else { lo + 1.0 }))
}

/// Map `values` into colors over their display range; non-finite values,
/// which mark missing readings, become black.
fn normalize_to_rgb(
    values: &[f32],
    width: u32,
    height: u32,
    color: impl Fn(f32) -> [u8; 3],
) -> RgbImage {
    let mut out = Vec::with_capacity(values.len() * 3);
    match display_range(values) {
        Some((lo, hi)) => {
            for &v in values {
                if v.is_finite() {
                    out.extend_from_slice(&color(((v - lo) / (hi - lo)).clamp(0.0, 1.0)));
                } else {
                    out.extend_from_slice(&[0, 0, 0]);
                }
            }
        }
        None => out.resize(values.len() * 3, 0),
    }
    RgbImage::from_raw(width, height, out).unwrap()
}

/// Depth colored with the turbo map, near red and far blue.
fn colorize_depth(values: &[f32], width: u32, height: u32) -> RgbImage {
    normalize_to_rgb(values, width, height, |t| turbo(1.0 - t))
}

/// Polynomial approximation of Google's turbo colormap.
pub(crate) fn turbo(t: f32) -> [u8; 3] {
    let x = t.clamp(0.0, 1.0);
    let poly = |c: [f32; 6]| {
        let v = c[0] + x * (c[1] + x * (c[2] + x * (c[3] + x * (c[4] + x * c[5]))));
        (v.clamp(0.0, 1.0) * 255.0) as u8
    };
    [
        poly([
            0.135_721_38,
            4.615_392_6,
            -42.660_324,
            132.131_08,
            -152.942_4,
            59.286_38,
        ]),
        poly([
            0.091_402_61,
            2.194_188_4,
            4.842_966_6,
            -14.185_033,
            4.277_298_6,
            2.829_566,
        ]),
        poly([
            0.106_673_3,
            12.641_946,
            -60.582_05,
            110.362_77,
            -89.903_11,
            27.348_25,
        ]),
    ]
}

fn decode_compressed(format: &str, data: &[u8]) -> Result<RgbImage, ImageError> {
    let format_lc = format.to_ascii_lowercase();
    if format_lc.contains("compresseddepth") {
        return decode_compressed_depth(&format_lc, data);
    }
    image::load_from_memory(data)
        .map(|img| img.to_rgb8())
        .map_err(|e| ImageError::Decode(e.to_string()))
}

/// `compressed_depth_image_transport` output: a 12-byte config header
/// (format enum, two depth quantization floats) followed by a 16-bit PNG.
fn decode_compressed_depth(format_lc: &str, data: &[u8]) -> Result<RgbImage, ImageError> {
    const HEADER_LEN: usize = 12;
    if data.len() < HEADER_LEN {
        return Err(ImageError::Truncated);
    }
    let float_at = |i: usize| f32::from_le_bytes(data[i..i + 4].try_into().unwrap());
    let (quant_a, quant_b) = (float_at(4), float_at(8));

    let png = image::load_from_memory_with_format(&data[HEADER_LEN..], ImageFormat::Png)
        .map_err(|e| ImageError::Decode(e.to_string()))?;
    let depth = match png {
        DynamicImage::ImageLuma16(buf) => buf,
        other => other.to_luma16(),
    };
    let (width, height) = depth.dimensions();
    // 32FC1 depth is stored as quantized inverse depth, 16UC1 as is.
    let inverse_depth = format_lc.starts_with("32fc1");
    let values: Vec<f32> = depth
        .pixels()
        .map(|p| match p.0[0] {
            0 => f32::NAN,
            v if inverse_depth => quant_a / (v as f32 - quant_b),
            v => v as f32,
        })
        .collect();
    Ok(colorize_depth(&values, width, height))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    fn image_msg(width: u32, height: u32, encoding: &str, step: u32, data: &[u8]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("cam");
        w.u32(height);
        w.u32(width);
        w.string(encoding);
        w.0.push(0); // is_bigendian
        w.u32(step);
        w.bytes(data);
        w.0
    }

    #[test]
    fn decodes_rgb8_with_header() {
        let data = [255, 0, 0, 0, 255, 0, 0, 0, 255, 10, 20, 30];
        let msg = image_msg(2, 2, "rgb8", 6, &data);
        let img = decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap();
        assert_eq!(img.frame_id, "cam");
        assert!((img.stamp - 12.5).abs() < 1e-9);
        assert_eq!((img.source_width, img.source_height), (2, 2));
        assert_eq!(img.rgb.get_pixel(0, 0).0, [255, 0, 0]);
        assert_eq!(img.rgb.get_pixel(1, 1).0, [10, 20, 30]);
    }

    #[test]
    fn swaps_bgr8_and_honors_row_padding() {
        // One pixel per row, padded to a 4-byte step.
        let data = [1, 2, 3, 0, 4, 5, 6, 0];
        let msg = image_msg(1, 2, "bgr8", 4, &data);
        let img = decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap();
        assert_eq!(img.rgb.get_pixel(0, 0).0, [3, 2, 1]);
        assert_eq!(img.rgb.get_pixel(0, 1).0, [6, 5, 4]);
    }

    #[test]
    fn colorizes_depth_and_blacks_out_missing_readings() {
        let depth: Vec<u8> = [1.0f32, 2.0, f32::NAN, 4.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let msg = image_msg(4, 1, "32FC1", 16, &depth);
        let img = decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap();
        assert_eq!(img.rgb.get_pixel(2, 0).0, [0, 0, 0]);
        assert_ne!(img.rgb.get_pixel(0, 0).0, img.rgb.get_pixel(3, 0).0);
    }

    #[test]
    fn decodes_yuyv() {
        // Two gray pixels: Y=235 (white), U=V=128 (no chroma).
        let msg = image_msg(2, 1, "yuv422_yuy2", 4, &[235, 128, 235, 128]);
        let img = decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap();
        assert_eq!(img.rgb.get_pixel(0, 0).0, [255, 255, 255]);
    }

    #[test]
    fn demosaics_bayer_to_half_resolution() {
        let msg = image_msg(2, 2, "bayer_rggb8", 2, &[200, 100, 100, 50]);
        let img = decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap();
        assert_eq!(img.rgb.dimensions(), (1, 1));
        assert_eq!(img.rgb.get_pixel(0, 0).0, [200, 100, 50]);
    }

    #[test]
    fn rejects_data_shorter_than_dimensions() {
        let msg = image_msg(4, 4, "rgb8", 12, &[0; 10]);
        assert!(matches!(
            decode_message(IMAGE_TYPE, &msg, 100, 100),
            Err(ImageError::InvalidDimensions { .. })
        ));
    }

    #[test]
    fn rejects_unknown_encoding() {
        let msg = image_msg(1, 1, "nv12", 1, &[0; 4]);
        assert_eq!(
            decode_message(IMAGE_TYPE, &msg, 100, 100).unwrap_err(),
            ImageError::UnsupportedEncoding("nv12".into())
        );
    }

    #[test]
    fn decodes_compressed_png_and_downscales() {
        let src = RgbImage::from_pixel(64, 32, image::Rgb([9, 99, 199]));
        let mut png = Vec::new();
        DynamicImage::ImageRgb8(src)
            .write_to(&mut std::io::Cursor::new(&mut png), ImageFormat::Png)
            .unwrap();
        let mut w = CdrWriter::new();
        w.header("cam");
        w.string("png");
        w.bytes(&png);
        let img = decode_message(COMPRESSED_IMAGE_TYPE, &w.0, 16, 16).unwrap();
        assert_eq!((img.source_width, img.source_height), (64, 32));
        assert_eq!(img.rgb.dimensions(), (16, 8));
        assert_eq!(img.encoding, "png");
    }

    #[test]
    fn truncated_message_is_an_error() {
        let msg = image_msg(2, 2, "rgb8", 6, &[0; 12]);
        assert_eq!(
            decode_message(IMAGE_TYPE, &msg[..20], 100, 100).unwrap_err(),
            ImageError::Truncated
        );
    }

    #[test]
    fn fit_within_keeps_aspect_and_never_upscales() {
        assert_eq!(fit_within(1920, 1080, 640, 640), (640, 360));
        assert_eq!(fit_within(100, 50, 640, 640), (100, 50));
        assert_eq!(fit_within(1000, 10, 10, 10), (10, 1));
    }
}
