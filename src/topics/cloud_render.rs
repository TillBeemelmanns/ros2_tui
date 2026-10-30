//! Software rendering of point clouds into RGB images: a top-down, side or
//! orbiting perspective view, colored by a per-point value, with a grid.

use super::image::turbo;
use super::pointcloud::PointCloud;
use image::RgbImage;
use std::f32::consts::{FRAC_PI_2, PI};

/// Vertical field of view of the perspective camera.
const FOV: f32 = 60.0 * PI / 180.0;
const MIN_RADIUS: f32 = 0.1;
const MAX_RADIUS: f32 = 10_000.0;
const GRID_COLOR: [u8; 3] = [55, 60, 70];

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraPreset {
    /// Orthographic, looking down with x pointing up: a bird's eye view.
    Top,
    /// Perspective from behind and above the sensor.
    Orbit,
    /// Orthographic from the right, x pointing right.
    Side,
}

impl CameraPreset {
    pub fn name(self) -> &'static str {
        match self {
            CameraPreset::Top => "top",
            CameraPreset::Orbit => "orbit",
            CameraPreset::Side => "side",
        }
    }

    pub fn next(self) -> Self {
        match self {
            CameraPreset::Top => CameraPreset::Orbit,
            CameraPreset::Orbit => CameraPreset::Side,
            CameraPreset::Side => CameraPreset::Top,
        }
    }
}

/// Camera orbiting `target`, looking at it from `yaw`/`pitch`, with `radius`
/// meters around the target in view.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CloudCamera {
    pub preset: CameraPreset,
    pub yaw: f32,
    pub pitch: f32,
    pub radius: f32,
    pub target: [f32; 3],
    pub perspective: bool,
}

impl CloudCamera {
    pub fn new(preset: CameraPreset, radius: f32) -> Self {
        let (yaw, pitch, perspective) = match preset {
            CameraPreset::Top => (PI, FRAC_PI_2, false),
            CameraPreset::Orbit => (PI, 0.5, true),
            CameraPreset::Side => (-FRAC_PI_2, 0.0, false),
        };
        Self {
            preset,
            yaw,
            pitch,
            radius: radius.clamp(MIN_RADIUS, MAX_RADIUS),
            target: [0.0; 3],
            perspective,
        }
    }

    pub fn orbit(&mut self, yaw: f32, pitch: f32) {
        self.yaw = (self.yaw + yaw).rem_euclid(2.0 * PI);
        self.pitch = (self.pitch + pitch).clamp(-FRAC_PI_2, FRAC_PI_2);
    }

    pub fn zoom(&mut self, factor: f32) {
        self.radius = (self.radius * factor).clamp(MIN_RADIUS, MAX_RADIUS);
    }

    /// Move the target in the ground plane, in steps of a tenth of the view.
    pub fn pan(&mut self, forward: f32, right: f32) {
        let step = self.radius * 0.1;
        let (sin, cos) = self.yaw.sin_cos();
        self.target[0] += step * (-cos * forward - sin * right);
        self.target[1] += step * (-sin * forward + cos * right);
    }

    /// Unit vectors from the target towards the camera, to screen right and
    /// to screen up.
    fn basis(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let back = [cp * cy, cp * sy, sp];
        let right = [-sy, cy, 0.0];
        let up = cross(back, right);
        (back, right, up)
    }
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ColorMode {
    Height,
    Intensity,
    Distance,
    Rgb,
}

impl ColorMode {
    pub fn name(self) -> &'static str {
        match self {
            ColorMode::Height => "height",
            ColorMode::Intensity => "intensity",
            ColorMode::Distance => "distance",
            ColorMode::Rgb => "rgb",
        }
    }

    /// Modes `cloud` has data for, the most informative first. Scans are
    /// flat, so height says nothing about them.
    pub fn available(cloud: &PointCloud) -> Vec<ColorMode> {
        let mut modes = Vec::new();
        if cloud.rgb.is_some() {
            modes.push(ColorMode::Rgb);
        }
        if cloud.intensity.is_some() {
            modes.push(ColorMode::Intensity);
        }
        if cloud.scan.is_none() {
            modes.push(ColorMode::Height);
        }
        modes.push(ColorMode::Distance);
        modes
    }

    /// Value range the colors span, for display.
    pub fn range(self, cloud: &PointCloud) -> Option<(f32, f32)> {
        match self {
            ColorMode::Height => cloud.z_range,
            ColorMode::Intensity => cloud.intensity_range,
            ColorMode::Distance => cloud.distance_range,
            ColorMode::Rgb => None,
        }
    }
}

/// World to pixel projection for one render.
struct Projection {
    back: [f32; 3],
    right: [f32; 3],
    up: [f32; 3],
    target: [f32; 3],
    perspective: bool,
    /// Pixels per meter (orthographic) or focal length in pixels.
    scale: f32,
    /// Horizontal stretch for pixels that are not square.
    x_scale: f32,
    camera_distance: f32,
    center: (f32, f32),
}

impl Projection {
    fn new(camera: &CloudCamera, width: u32, height: u32, pixel_aspect: f32) -> Self {
        let (back, right, up) = camera.basis();
        // Fit the view radius into the shorter side, measured in square units.
        let half = (width as f32 * pixel_aspect).min(height as f32) / 2.0;
        let tan = (FOV / 2.0).tan();
        Self {
            back,
            right,
            up,
            target: camera.target,
            perspective: camera.perspective,
            scale: if camera.perspective {
                half / tan
            } else {
                half / camera.radius
            },
            x_scale: 1.0 / pixel_aspect,
            camera_distance: camera.radius / tan,
            center: (width as f32 / 2.0, height as f32 / 2.0),
        }
    }

    /// Pixel position and depth (larger is farther) of world point `p`.
    fn project(&self, p: [f32; 3]) -> Option<(f32, f32, f32)> {
        let d = [
            p[0] - self.target[0],
            p[1] - self.target[1],
            p[2] - self.target[2],
        ];
        let (x, y, z) = (dot(d, self.right), dot(d, self.up), dot(d, self.back));
        let (factor, depth) = if self.perspective {
            let depth = self.camera_distance - z;
            if depth < 0.05 {
                return None;
            }
            (self.scale / depth, depth)
        } else {
            (self.scale, -z)
        };
        Some((
            self.center.0 + x * factor * self.x_scale,
            self.center.1 - y * factor,
            depth,
        ))
    }
}

struct Canvas {
    width: u32,
    height: u32,
    rgb: Vec<u8>,
    depth: Vec<f32>,
}

impl Canvas {
    fn plot(&mut self, x: f32, y: f32, depth: f32, color: [u8; 3], size: u32) {
        if x < 0.0 || y < 0.0 {
            return;
        }
        let (x0, y0) = (x as u32, y as u32);
        for py in y0..(y0 + size).min(self.height) {
            for px in x0..(x0 + size).min(self.width) {
                let i = (py * self.width + px) as usize;
                if depth < self.depth[i] {
                    self.depth[i] = depth;
                    self.rgb[i * 3..i * 3 + 3].copy_from_slice(&color);
                }
            }
        }
    }

    /// Line from `a` to `b` in world space, sampled so it has no gaps.
    fn line(
        &mut self,
        proj: &Projection,
        a: [f32; 3],
        b: [f32; 3],
        color: [u8; 3],
        depth: Option<f32>,
    ) {
        let samples = match (proj.project(a), proj.project(b)) {
            (Some(pa), Some(pb)) => {
                ((pa.0 - pb.0).abs().max((pa.1 - pb.1).abs()) as usize).clamp(2, 4000)
            }
            _ => 400,
        };
        for i in 0..=samples {
            let t = i as f32 / samples as f32;
            let p = [
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ];
            if let Some((x, y, d)) = proj.project(p) {
                self.plot(x, y, depth.unwrap_or(d), color, 1);
            }
        }
    }
}

/// Grid spacing that puts a handful of lines across `radius`.
pub fn grid_step(radius: f32) -> f32 {
    let raw = radius / 4.0;
    let magnitude = 10f32.powf(raw.log10().floor());
    [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * magnitude)
        .find(|step| *step >= raw)
        .unwrap_or(10.0 * magnitude)
}

pub struct RenderOptions {
    pub camera: CloudCamera,
    pub color: ColorMode,
    pub grid: bool,
    pub width: u32,
    pub height: u32,
    /// Width over height of one pixel, e.g. of a braille dot.
    pub pixel_aspect: f32,
}

/// Render `cloud`; empty pixels stay black.
pub fn render(cloud: &PointCloud, options: &RenderOptions) -> RgbImage {
    let (width, height) = (options.width.max(1), options.height.max(1));
    let proj = Projection::new(&options.camera, width, height, options.pixel_aspect);
    let mut canvas = Canvas {
        width,
        height,
        rgb: vec![0; (width * height * 3) as usize],
        depth: vec![f32::INFINITY; (width * height) as usize],
    };

    if options.grid {
        // On the ground: roughly the lowest points of the cloud. Grid lines
        // sit behind everything so points are never hidden by them.
        let ground = cloud.z_range.map_or(0.0, |(lo, _)| lo);
        let step = grid_step(options.camera.radius);
        let lines = (options.camera.radius * 2.0 / step).ceil() as i32;
        let [tx, ty, _] = options.camera.target;
        let (cx, cy) = ((tx / step).round() * step, (ty / step).round() * step);
        let extent = lines as f32 * step;
        for i in -lines..=lines {
            let o = i as f32 * step;
            let behind = Some(f32::MAX);
            canvas.line(
                &proj,
                [cx + o, cy - extent, ground],
                [cx + o, cy + extent, ground],
                GRID_COLOR,
                behind,
            );
            canvas.line(
                &proj,
                [cx - extent, cy + o, ground],
                [cx + extent, cy + o, ground],
                GRID_COLOR,
                behind,
            );
        }
    }

    let size = if width >= 640 { 2 } else { 1 };
    let range = options.color.range(cloud);
    let normalize = |v: f32| range.map_or(0.5, |(lo, hi)| ((v - lo) / (hi - lo)).clamp(0.0, 1.0));
    for (i, &p) in cloud.points.iter().enumerate() {
        let Some((x, y, depth)) = proj.project(p) else {
            continue;
        };
        let color = match options.color {
            ColorMode::Rgb => match &cloud.rgb {
                // Black would read as "no point".
                Some(rgb) => {
                    let [r, g, b] = rgb[i];
                    [r.max(24), g.max(24), b.max(24)]
                }
                None => turbo(normalize(p[2])),
            },
            ColorMode::Intensity => match &cloud.intensity {
                Some(values) => turbo(normalize(values[i])),
                None => turbo(normalize(p[2])),
            },
            ColorMode::Height => turbo(normalize(p[2])),
            ColorMode::Distance => {
                turbo(1.0 - normalize((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt()))
            }
        };
        canvas.plot(x, y, depth, color, size);
    }

    // Sensor origin axes: x red, y green, z blue.
    let axis = options.camera.radius * 0.08;
    let origin = [0.0; 3];
    canvas.line(&proj, origin, [axis, 0.0, 0.0], [230, 60, 60], None);
    canvas.line(&proj, origin, [0.0, axis, 0.0], [60, 200, 60], None);
    canvas.line(&proj, origin, [0.0, 0.0, axis], [70, 110, 240], None);

    RgbImage::from_raw(width, height, canvas.rgb).unwrap()
}

/// View radius that frames most of `cloud`: a little more than the 90th
/// percentile of the points' horizontal distance from the sensor.
pub fn fit_radius(cloud: &PointCloud) -> f32 {
    const SAMPLES: usize = 2048;
    let stride = (cloud.points.len() / SAMPLES).max(1);
    let mut distances: Vec<f32> = cloud
        .points
        .iter()
        .step_by(stride)
        .map(|p| (p[0] * p[0] + p[1] * p[1]).sqrt())
        .collect();
    if distances.is_empty() {
        return 10.0;
    }
    distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p90 = distances[(distances.len() - 1) * 9 / 10];
    (p90 * 1.1).clamp(1.0, MAX_RADIUS)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(points: Vec<[f32; 3]>) -> PointCloud {
        PointCloud {
            z_range: Some((0.0, 1.0)),
            distance_range: Some((0.0, 10.0)),
            intensity_range: None,
            intensity: None,
            intensity_field: None,
            rgb: None,
            width: points.len() as u32,
            height: 1,
            total_points: points.len(),
            sampled_points: points.len(),
            fields: vec![],
            frame_id: String::new(),
            stamp: 0.0,
            bounds: None,
            points,
            scan: None,
        }
    }

    fn options(preset: CameraPreset) -> RenderOptions {
        RenderOptions {
            camera: CloudCamera::new(preset, 10.0),
            color: ColorMode::Height,
            grid: false,
            width: 101,
            height: 101,
            pixel_aspect: 1.0,
        }
    }

    fn lit(img: &RgbImage, x: u32, y: u32) -> bool {
        img.get_pixel(x, y).0 != [0, 0, 0]
    }

    #[test]
    fn top_view_puts_forward_up_and_left_left() {
        // 5 m ahead and 5 m to the left of the sensor.
        let img = render(
            &cloud(vec![[5.0, 0.0, 0.0], [0.0, 5.0, 0.0]]),
            &options(CameraPreset::Top),
        );
        // 50 px per 10 m radius: 5 m is 25 px from the center.
        assert!(
            lit(&img, 50, 25),
            "forward point should be above the center"
        );
        assert!(lit(&img, 25, 50), "left point should be left of the center");
    }

    #[test]
    fn side_view_shows_height_up_and_forward_right() {
        let img = render(
            &cloud(vec![[5.0, 0.0, 0.0], [0.0, 0.0, 5.0]]),
            &options(CameraPreset::Side),
        );
        assert!(lit(&img, 75, 50));
        assert!(lit(&img, 50, 25));
    }

    #[test]
    fn nearer_point_wins_the_depth_test() {
        let mut c = cloud(vec![[5.0, 0.0, 1.0], [5.0, 0.0, 0.0]]);
        c.z_range = Some((0.0, 1.0));
        let img = render(&c, &options(CameraPreset::Top));
        // From above, z=1 is nearer: it gets the color for height 1.
        assert_eq!(img.get_pixel(50, 25).0, turbo(1.0));
    }

    #[test]
    fn perspective_skips_points_behind_the_camera() {
        let opts = options(CameraPreset::Orbit);
        let img = render(&cloud(vec![[-1000.0, 0.0, 0.0]]), &opts);
        let lit_count = img.pixels().filter(|p| p.0 != [0, 0, 0]).count();
        // Only the origin axes are drawn.
        assert!(lit_count < 30, "{lit_count}");
    }

    #[test]
    fn grid_step_is_a_round_number() {
        assert_eq!(grid_step(10.0), 5.0);
        assert_eq!(grid_step(40.0), 10.0);
        assert_eq!(grid_step(3.0), 1.0);
    }

    #[test]
    fn fit_radius_frames_most_points() {
        let points = (0..100).map(|i| [i as f32, 0.0, 0.0]).collect();
        let r = fit_radius(&cloud(points));
        assert!((r - 89.0 * 1.1).abs() < 0.01, "{r}");
    }

    #[test]
    fn pan_moves_forward_in_the_view_direction() {
        let mut cam = CloudCamera::new(CameraPreset::Top, 10.0);
        cam.pan(1.0, 0.0);
        // Top view looks along +x, so forward is +x.
        assert!((cam.target[0] - 1.0).abs() < 1e-5 && cam.target[1].abs() < 1e-5);
        cam.pan(0.0, 1.0);
        // Screen right in the top view is -y.
        assert!((cam.target[1] + 1.0).abs() < 1e-5);
    }
}
