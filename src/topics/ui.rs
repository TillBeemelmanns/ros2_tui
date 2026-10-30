use ratatui::{
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{
        Axis, Block, Borders, Cell, Chart, Dataset, GraphType, Paragraph, Row, Scrollbar,
        ScrollbarOrientation, ScrollbarState, Table, Wrap,
    },
    Frame,
};

use super::app::{
    has_viewer, App, AppMode, BatteryView, CloudView, DetailPaneFocus, DiagnosticsView, GnssView,
    ImuPoint, ImuView, JointsView, OdometryView, PathView, TfView, TopicView, ViewContent,
    BATTERY_HISTORY, CHARTS_MAX_DATA_POINTS, IMU_HISTORY,
};
use super::battery::measured;
use super::cloud_render::{grid_step, ColorMode};
use super::diagnostics::Level;
use super::imu;
use super::joints::JOINT_STALE_AFTER;
use super::pointcloud::PointCloud;
use super::ros;
use super::ros::MeasurementStatus;
use super::tf::Issue;
use crate::common::graphics::{Braille, GraphicsProtocol, Halfblocks};
use crate::common::{TopicTree, TopicTreeItem};

pub fn ui(f: &mut Frame, app: &App) {
    match app.mode {
        AppMode::TopicList => render_topic_list(f, app),
        AppMode::TopicDetail => render_topic_detail(f, app),
        AppMode::Search => render_search_mode(f, app),
        AppMode::Help => render_help(f),
        AppMode::TopicView => render_view(f, app),
    }
}

fn create_block(title: &str) -> Block<'_> {
    Block::default().borders(Borders::ALL).title(Span::styled(
        title,
        Style::default().add_modifier(Modifier::BOLD),
    ))
}

fn render_help(f: &mut Frame) {
    let help_text = vec![
        Line::from(""),
        Line::from(Span::styled(
            " Topics Help",
            Style::default()
                .add_modifier(Modifier::BOLD)
                .fg(Color::Cyan),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Navigation",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ↑ / k       - Move selection up"),
        Line::from("    ↓ / j       - Move selection down"),
        Line::from("    ← / h       - Collapse selected group"),
        Line::from("    → / l       - Expand selected group"),
        Line::from(""),
        Line::from(Span::styled(
            "  Actions",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    <Enter>     - Toggle watch on topic / expand group"),
        Line::from("    d           - View details for selected topic"),
        Line::from(
            "    v           - View topic marked ◉ (images, maps, clouds, scans, GNSS, IMU, diagnostics)",
        ),
        Line::from("    c           - Toggle collapse/uncollapse all groups"),
        Line::from("    s           - Toggle sim time for delay metrics"),
        Line::from("    F4          - Enter search/filter mode"),
        Line::from(""),
        Line::from(Span::styled(
            "  Topic Viewer",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    + / -       - Raise / lower the frame rate limit"),
        Line::from("    p           - Pause / resume"),
        Line::from("    q / <Esc>   - Back"),
        Line::from(""),
        Line::from(Span::styled(
            "  Point Cloud Viewer",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ←→↑↓ / hjkl - Orbit the camera"),
        Line::from("    w a s d     - Pan"),
        Line::from("    mouse wheel - Zoom in / out"),
        Line::from("    <Tab>       - Next view: top, orbit, side"),
        Line::from("    c           - Next coloring: rgb, intensity, height, distance"),
        Line::from("    g           - Toggle the ground grid"),
        Line::from("    r           - Reset the camera"),
        Line::from(""),
        Line::from(Span::styled(
            "  Diagnostics Viewer",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ↑↓ / jk     - Select a status to see its values"),
        Line::from("    f           - Show only problems (WARN, ERROR, STALE)"),
        Line::from(""),
        Line::from(Span::styled(
            "  TF and Joint State Viewers",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ↑↓ / jk     - Select a frame or joint to see its details"),
        Line::from(""),
        Line::from(Span::styled(
            "  General",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ?           - Show this help screen"),
        Line::from("    q           - Quit the application"),
        Line::from("    <Esc>       - Clear filter, or exit current view, or quit"),
        Line::from(""),
        Line::from(Span::styled(
            "  Press <Esc> or 'q' to close this help window.",
            Style::default().fg(Color::Yellow),
        )),
    ];
    let paragraph = Paragraph::new(help_text).block(create_block("Help"));
    f.render_widget(paragraph, f.area());
}

fn render_topic_list(f: &mut Frame, app: &App) {
    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    render_topic_table(f, app, main_chunks[0]);
    render_status_bar(f, app, main_chunks[1]);
}

fn render_status_bar(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let filter_display = if !app.filter_text.is_empty() {
        format!(" | Filter: '{}'", app.filter_text)
    } else {
        String::new()
    };

    let sim_time_span = Span::styled(
        format!("Sim time: {}", if app.use_sim_time { "ON" } else { "OFF" }),
        Style::default().fg(if app.use_sim_time {
            Color::Green
        } else {
            Color::DarkGray
        }),
    );

    let mut spans = if let Some(error) = &app.error_message {
        vec![Span::styled(
            format!("Error: {}", error),
            Style::default().fg(Color::Red),
        )]
    } else if app.mode == AppMode::TopicView {
        let bold = Style::default().add_modifier(Modifier::BOLD);
        let mut keys = vec![];
        let content = app.view.as_ref().map(|v| &v.content);
        let diagnostics = matches!(content, Some(ViewContent::Diagnostics(_)));
        let selectable = matches!(
            content,
            Some(ViewContent::Diagnostics(_) | ViewContent::Tf(_) | ViewContent::Joints(_))
        );
        if selectable {
            keys.extend([Span::styled("↑↓/jk", bold), Span::raw(": Select | ")]);
        }
        if diagnostics {
            keys.extend([Span::styled("f", bold), Span::raw(": Problems Only | ")]);
        }
        if matches!(content, Some(ViewContent::Cloud(_))) {
            keys.extend([
                Span::styled("←→↑↓/hjkl", bold),
                Span::raw(": Orbit | "),
                Span::styled("wasd", bold),
                Span::raw(": Pan | "),
                Span::styled("wheel", bold),
                Span::raw(": Zoom | "),
                Span::styled("Tab", bold),
                Span::raw(": View | "),
                Span::styled("c", bold),
                Span::raw(": Color | "),
                Span::styled("g", bold),
                Span::raw(": Grid | "),
                Span::styled("r", bold),
                Span::raw(": Reset | "),
            ]);
        }
        if !content.is_some_and(ViewContent::is_unthrottled) {
            keys.extend([
                Span::styled("+/-", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(": Max FPS | "),
            ]);
        }
        keys.extend([
            Span::styled("p", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Pause | "),
            Span::styled("q/Esc", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Back"),
        ]);
        keys
    } else if app.mode == AppMode::TopicDetail {
        vec![
            Span::styled("v", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": View | "),
            Span::raw("↑↓/jk: Scroll | "),
            Span::styled("PgUp/PgDn", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Page | "),
            Span::styled("Home/End", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Top/Bottom | "),
            Span::styled("Tab", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Switch Pane | "),
            Span::styled("e", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Toggle Echo | "),
            Span::styled("s", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Sim Time | "),
            Span::styled("q/Esc", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Back"),
        ]
    } else {
        let mut base = vec![
            Span::raw("↑↓/jk Navigate | "),
            Span::styled("Enter", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Watch/Expand | "),
            Span::styled("d", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Details | "),
            Span::styled("v", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": View | "),
            Span::styled("c", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Un/Collapse | "),
            Span::styled("F4", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Search | "),
            Span::styled("s", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Sim Time | "),
            Span::styled("r", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Refresh | "),
            Span::styled("?", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Help | "),
            Span::styled("q", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Quit"),
        ];
        if !filter_display.is_empty() {
            base.push(Span::styled(
                filter_display,
                Style::default().fg(Color::Yellow),
            ));
        }
        base
    };

    let content_width: usize = spans.iter().map(|span| span.content.len()).sum();
    let indicator_width = sim_time_span.content.len();
    let available_width = area.width.saturating_sub(1) as usize; // Leave room for border rendering quirks
    let padding = available_width
        .saturating_sub(content_width + indicator_width)
        .max(1);
    spans.push(Span::raw(" ".repeat(padding)));
    spans.push(sim_time_span);

    let status = Paragraph::new(Line::from(spans)).block(Block::default().borders(Borders::TOP));
    f.render_widget(status, area);
}

fn render_view(f: &mut Frame, app: &App) {
    let Some(view) = app.view.as_ref() else {
        return;
    };
    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    let block = create_block(&view.topic_name).title(
        Line::from(Span::styled(
            format!(" {} ", app.graphics.protocol_name()),
            Style::default().fg(Color::DarkGray),
        ))
        .right_aligned(),
    );
    let inner = block.inner(main_chunks[0]);
    f.render_widget(block, main_chunks[0]);

    let info = view_info_lines(app, view);
    let inner_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(info.len() as u16), Constraint::Min(0)])
        .split(inner);
    f.render_widget(Paragraph::new(info), inner_chunks[0]);
    let content_area = inner_chunks[1];

    // Kitty images are drawn by `App::sync_graphics` after the frame, at the
    // placement recorded here.
    let kitty = app.graphics.protocol == GraphicsProtocol::Kitty;
    let shown = match &view.content {
        ViewContent::Image(frame) => {
            let (width, height) = app.graphics.target_pixels(content_area);
            view.target.set(width, height);
            frame.as_ref().map(|frame| {
                let (width, height) = frame.rgb.dimensions();
                if !kitty {
                    f.render_widget(Halfblocks(&frame.rgb), content_area);
                }
                app.graphics.placement(content_area, width, height)
            })
        }
        ViewContent::Cloud(cloud) => {
            let (width, height, pixel_aspect) = app.graphics.cloud_pixels(content_area);
            cloud
                .render(view.frame_seq, width, height, pixel_aspect)
                .map(|rendered| {
                    if !kitty {
                        f.render_widget(Braille(&rendered.image), content_area);
                    }
                    app.graphics.placement(content_area, width, height)
                })
        }
        ViewContent::Diagnostics(diag) => {
            if !diag.table.is_empty() {
                render_diagnostics(f, diag, content_area);
            }
            None
        }
        ViewContent::Gnss(gnss) => {
            if gnss.latest.is_some() {
                render_gnss(f, app, gnss, content_area);
            }
            None
        }
        ViewContent::Imu(imu_view) => {
            if imu_view.latest.is_some() {
                render_imu(f, imu_view, content_area);
            }
            None
        }
        ViewContent::Tf(tf_view) => {
            if !tf_view.tree.is_empty() {
                render_tf(f, tf_view, content_area);
            }
            None
        }
        ViewContent::Odometry(odom) => {
            if odom.latest.is_some() {
                render_odometry(f, app, odom, content_area);
            }
            None
        }
        ViewContent::Path(path) => {
            if path.latest.is_some() {
                render_path(f, app, path, content_area);
            }
            None
        }
        ViewContent::Joints(joints) => {
            if !joints.table.is_empty() {
                render_joints(f, joints, content_area);
            }
            None
        }
        ViewContent::Battery(battery) => {
            if battery.latest.is_some() {
                render_battery(f, battery, content_area);
            }
            None
        }
    };
    view.placement.set(shown);

    if !view_has_content(view) {
        let text = match &view.error {
            Some(error) => Span::styled(error.clone(), Style::default().fg(Color::Red)),
            None => Span::styled(
                format!("Waiting for messages on {} ...", view.topic_name),
                Style::default().fg(Color::DarkGray),
            ),
        };
        let message_area = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage(50),
                Constraint::Length(1),
                Constraint::Min(0),
            ])
            .split(content_area)[1];
        f.render_widget(
            Paragraph::new(Line::from(text)).alignment(ratatui::layout::Alignment::Center),
            message_area,
        );
    }

    render_status_bar(f, app, main_chunks[1]);
}

fn view_has_content(view: &TopicView) -> bool {
    match &view.content {
        ViewContent::Image(frame) => frame.is_some(),
        ViewContent::Cloud(cloud) => cloud.cloud.is_some(),
        ViewContent::Diagnostics(diag) => !diag.table.is_empty(),
        ViewContent::Gnss(gnss) => gnss.latest.is_some(),
        ViewContent::Imu(imu_view) => imu_view.latest.is_some(),
        ViewContent::Tf(tf_view) => !tf_view.tree.is_empty(),
        ViewContent::Odometry(odom) => odom.latest.is_some(),
        ViewContent::Path(path) => path.latest.is_some(),
        ViewContent::Joints(joints) => !joints.table.is_empty(),
        ViewContent::Battery(battery) => battery.latest.is_some(),
    }
}

/// Text rows of label and value, the labels padded to one width.
fn labeled_lines<'a>(rows: Vec<(&'a str, Vec<Span<'a>>)>) -> Vec<Line<'a>> {
    let width = rows.iter().map(|(label, _)| label.len()).max().unwrap_or(0) + 2;
    rows.into_iter()
        .map(|(label, mut value)| {
            let mut spans = vec![Span::styled(
                format!("{:<width$}", label),
                Style::default().fg(Color::Cyan),
            )];
            spans.append(&mut value);
            Line::from(spans)
        })
        .collect()
}

/// Fix details beside a track of recent fixes, north up, in meters.
fn render_gnss(f: &mut Frame, app: &App, gnss: &GnssView, area: ratatui::layout::Rect) {
    let Some(fix) = &gnss.latest else {
        return;
    };
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Length(48), Constraint::Min(10)])
        .split(area);

    let status_color = match fix.status {
        s if s < 0 => Color::Red,
        0 => Color::Yellow,
        _ => Color::Green,
    };
    let hemisphere = |v: f64, pos: char, neg: char| {
        format!("{:.7}° {}", v.abs(), if v >= 0.0 { pos } else { neg })
    };
    let mut rows = vec![
        (
            "Status",
            vec![Span::styled(
                fix.status_name(),
                Style::default()
                    .fg(status_color)
                    .add_modifier(Modifier::BOLD),
            )],
        ),
        ("Satellites", vec![Span::raw(fix.services())]),
        (
            "Latitude",
            vec![Span::raw(hemisphere(fix.latitude, 'N', 'S'))],
        ),
        (
            "Longitude",
            vec![Span::raw(hemisphere(fix.longitude, 'E', 'W'))],
        ),
        (
            "Altitude",
            vec![Span::raw(format!("{:.2} m", fix.altitude))],
        ),
        (
            "Accuracy",
            vec![Span::raw(match fix.accuracy() {
                Some((h, v)) => format!("±{:.2} m horiz, ±{:.2} m vert", h, v),
                None => "not reported".to_string(),
            })],
        ),
        ("Covariance", vec![Span::raw(fix.covariance_type_name())]),
        ("Fixes", vec![Span::raw(gnss.fixes.to_string())]),
    ];
    if let Some(distance) = gnss.track.distance_from_start() {
        rows.push(("From start", vec![Span::raw(format!("{:.2} m", distance))]));
    }
    f.render_widget(
        Paragraph::new(labeled_lines(rows)).block(create_block("Fix")),
        chunks[0],
    );

    let block = create_block("Track (m, north up)");
    let inner = block.inner(chunks[1]);
    let points: Vec<(f64, f64)> = gnss.track.points.iter().copied().collect();
    let (x_bounds, y_bounds) = equal_aspect_bounds(app, &points, inner);
    let latest = points.last().copied();
    let accuracy = fix.accuracy().map(|(h, _)| h);
    let canvas = ratatui::widgets::canvas::Canvas::default()
        .block(block)
        .marker(symbols::Marker::Braille)
        .x_bounds(x_bounds)
        .y_bounds(y_bounds)
        .paint(move |ctx| {
            ctx.draw(&ratatui::widgets::canvas::Points {
                coords: &points,
                color: Color::Cyan,
            });
            if let Some((x, y)) = latest {
                if let Some(radius) = accuracy.filter(|r| *r > 0.0) {
                    ctx.draw(&ratatui::widgets::canvas::Circle {
                        x,
                        y,
                        radius,
                        color: Color::Yellow,
                    });
                }
                ctx.print(x, y, Span::styled("◆", Style::default().fg(Color::Yellow)));
            }
        });
    f.render_widget(canvas, chunks[1]);
}

/// Canvas bounds around `points` where a meter is as long across as up,
/// given braille dots of 2x4 per cell of the terminal's cell size.
fn equal_aspect_bounds(
    app: &App,
    points: &[(f64, f64)],
    area: ratatui::layout::Rect,
) -> ([f64; 2], [f64; 2]) {
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &(x, y) in points {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    if points.is_empty() {
        (min_x, max_x, min_y, max_y) = (0.0, 0.0, 0.0, 0.0);
    }
    let width_px = area.width.max(1) as f64 * app.graphics.cell_width as f64;
    let height_px = area.height.max(1) as f64 * app.graphics.cell_height as f64;
    // At least 10 m across, plus a margin around the track.
    let meters_per_px = ((max_x - min_x) / width_px)
        .max((max_y - min_y) / height_px)
        .max(10.0 / width_px.min(height_px))
        * 1.2;
    let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
    let (half_w, half_h) = (
        meters_per_px * width_px / 2.0,
        meters_per_px * height_px / 2.0,
    );
    ([cx - half_w, cx + half_w], [cy - half_h, cy + half_h])
}

/// Latest values and gravity checks above plots of the last seconds.
fn render_imu(f: &mut Frame, view: &ImuView, area: ratatui::layout::Rect) {
    let Some(sample) = &view.latest else {
        return;
    };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(7), Constraint::Min(6)])
        .split(area);

    let xyz = |v: [f64; 3], digits: usize| {
        format!(
            "x {:>8.digits$}  y {:>8.digits$}  z {:>8.digits$}",
            v[0], v[1], v[2]
        )
    };
    let not_provided = || {
        vec![Span::styled(
            "not provided",
            Style::default().fg(Color::DarkGray),
        )]
    };
    let orientation = sample.orientation.map(imu::euler);
    let mut rows = vec![
        (
            "Orientation",
            match orientation {
                Some([r, p, y]) => vec![Span::raw(format!(
                    "roll {:>7.2}°  pitch {:>7.2}°  yaw {:>7.2}°",
                    r.to_degrees(),
                    p.to_degrees(),
                    y.to_degrees()
                ))],
                None => not_provided(),
            },
        ),
        (
            "Turn rate",
            match sample.angular_velocity {
                Some(w) => vec![Span::raw(format!(
                    "{} rad/s  |ω| {:.1}°/s",
                    xyz(w, 3),
                    (w[0] * w[0] + w[1] * w[1] + w[2] * w[2])
                        .sqrt()
                        .to_degrees()
                ))],
                None => not_provided(),
            },
        ),
        (
            "Acceleration",
            match sample.linear_acceleration {
                Some(a) => vec![Span::raw(format!(
                    "{} m/s²  |a| {:.2} m/s²",
                    xyz(a, 2),
                    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
                ))],
                None => not_provided(),
            },
        ),
    ];
    if let Some(a) = sample.linear_acceleration {
        let [roll, pitch] = imu::tilt_from_gravity(a);
        let mut value = vec![Span::raw(format!(
            "on {}  tilt roll {:.1}° pitch {:.1}°",
            imu::gravity_axis(a),
            roll.to_degrees(),
            pitch.to_degrees()
        ))];
        // At rest the orientation should roughly agree with gravity.
        if let Some([r, p, _]) = orientation {
            let off = (r - roll).abs().max((p - pitch).abs()).to_degrees();
            let (text, color) = if off < 5.0 {
                ("  matches orientation".to_string(), Color::Green)
            } else {
                (format!("  {:.0}° off orientation", off), Color::Yellow)
            };
            value.push(Span::styled(text, Style::default().fg(color)));
        }
        rows.push(("Gravity", value));
    }
    rows.push(("Frame", vec![Span::raw(sample.frame_id.clone())]));
    f.render_widget(Paragraph::new(labeled_lines(rows)), chunks[0]);

    let plots = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[1]);
    let now = std::time::Instant::now();
    let series = |pick: fn(&ImuPoint) -> Option<[f64; 3]>| {
        let mut axes: [Vec<(f64, f64)>; 3] = Default::default();
        for entry in &view.history {
            if let Some(v) = pick(entry) {
                let t = -now.duration_since(entry.at).as_secs_f64();
                for (axis, values) in axes.iter_mut().enumerate() {
                    values.push((t, v[axis]));
                }
            }
        }
        axes
    };
    render_xyz_chart(
        f,
        "Acceleration (m/s²)",
        &series(|p| p.acceleration),
        plots[0],
    );
    render_xyz_chart(f, "Turn rate (rad/s)", &series(|p| p.turn_rate), plots[1]);
}

/// Plot x, y and z over the last seconds, the y axis fitted to the data.
fn render_xyz_chart(
    f: &mut Frame,
    title: &str,
    axes: &[Vec<(f64, f64)>; 3],
    area: ratatui::layout::Rect,
) {
    let series = [
        ("x", Color::Red, axes[0].as_slice()),
        ("y", Color::Green, axes[1].as_slice()),
        ("z", Color::LightBlue, axes[2].as_slice()),
    ];
    render_series_chart(f, title, &series, IMU_HISTORY, area);
}

/// A named, colored series of (seconds before now, value).
type Series<'a> = (&'a str, Color, &'a [(f64, f64)]);

/// Plot named series of (seconds before now, value) over the last `span`,
/// the y axis fitted to the data.
fn render_series_chart(
    f: &mut Frame,
    title: &str,
    series: &[Series],
    span: std::time::Duration,
    area: ratatui::layout::Rect,
) {
    let (mut lo, mut hi) = series
        .iter()
        .flat_map(|(_, _, data)| data.iter())
        .filter(|(_, v)| v.is_finite())
        .fold((f64::MAX, f64::MIN), |(lo, hi), (_, v)| {
            (lo.min(*v), hi.max(*v))
        });
    if lo > hi {
        (lo, hi) = (-1.0, 1.0);
    }
    let pad = ((hi - lo) * 0.1).max(0.05);
    let (lo, hi) = (lo - pad, hi + pad);
    let datasets = series
        .iter()
        .map(|(name, color, data)| {
            Dataset::default()
                .name(*name)
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(*color))
                .data(data)
        })
        .collect();
    let seconds = span.as_secs_f64();
    let span_label = if seconds >= 120.0 {
        format!("-{:.0}m", seconds / 60.0)
    } else {
        format!("-{:.0}s", seconds)
    };
    let chart = Chart::new(datasets)
        .block(create_block(title))
        .x_axis(
            Axis::default()
                .bounds([-seconds, 0.0])
                .labels([span_label, "now".to_string()]),
        )
        .y_axis(
            Axis::default()
                .bounds([lo, hi])
                .labels([format!("{:.2}", lo), format!("{:.2}", hi)]),
        );
    f.render_widget(chart, area);
}

fn level_style(level: Level) -> Style {
    let color = match level {
        Level::Ok => Color::Green,
        Level::Warn => Color::Yellow,
        Level::Error => Color::Red,
        Level::Stale => Color::DarkGray,
    };
    Style::default().fg(color).add_modifier(Modifier::BOLD)
}

/// Status table, worst first, above the details of the selected status.
fn render_diagnostics(f: &mut Frame, diag: &DiagnosticsView, area: ratatui::layout::Rect) {
    let now = std::time::Instant::now();
    let entries = diag.table.sorted(now, diag.problems_only);
    let selected = diag.selected.min(entries.len().saturating_sub(1));
    let details_height = entries
        .get(selected)
        .map_or(0, |e| e.status.values.len() as u16 + 4)
        .min(area.height / 2);
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(3), Constraint::Length(details_height)])
        .split(area);

    let header = Row::new(["Level", "Name", "Message", "Hardware ID", "Age"]).style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );
    let rows = entries.iter().map(|entry| {
        let level = entry.level(now);
        Row::new(vec![
            Cell::from(Span::styled(level.name(), level_style(level))),
            Cell::from(entry.status.name.clone()),
            Cell::from(entry.status.message.clone()),
            Cell::from(entry.status.hardware_id.clone()),
            Cell::from(format!(
                "{:.1}s",
                now.duration_since(entry.updated).as_secs_f64()
            )),
        ])
    });
    let table = Table::new(
        rows,
        [
            Constraint::Length(6),
            Constraint::Percentage(35),
            Constraint::Percentage(40),
            Constraint::Percentage(15),
            Constraint::Length(7),
        ],
    )
    .header(header)
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");
    let mut state = ratatui::widgets::TableState::default();
    state.select((!entries.is_empty()).then_some(selected));
    f.render_stateful_widget(table, chunks[0], &mut state);

    if let Some(entry) = entries.get(selected) {
        let label = Style::default().fg(Color::Cyan);
        let mut lines = vec![Line::from(vec![
            Span::styled("Message ", label),
            Span::raw(entry.status.message.clone()),
        ])];
        lines.extend(entry.status.values.iter().map(|(key, value)| {
            Line::from(vec![
                Span::styled(format!("{}: ", key), label),
                Span::raw(value.clone()),
            ])
        }));
        let details = Paragraph::new(lines)
            .wrap(Wrap { trim: false })
            .block(create_block(&entry.status.name));
        f.render_widget(details, chunks[1]);
    }
}

/// Lines describing what is shown and the stream: one for images, two for
/// point clouds, whose second line has the view and cloud statistics.
fn view_info_lines<'a>(app: &App, view: &'a TopicView) -> Vec<Line<'a>> {
    let label = Style::default().fg(Color::Cyan);
    let mut spans = vec![];
    let mut second = None;
    match &view.content {
        ViewContent::Image(Some(frame)) => {
            let (width, height) = frame.rgb.dimensions();
            spans.extend([
                Span::styled("Source ", label),
                Span::raw(format!(
                    "{}x{} {}",
                    frame.source_width, frame.source_height, frame.encoding
                )),
                Span::styled("  Shown ", label),
                Span::raw(format!("{}x{}", width, height)),
            ]);
            if !frame.frame_id.is_empty() {
                spans.push(Span::styled("  Frame ", label));
                spans.push(Span::raw(frame.frame_id.clone()));
            }
            if !frame.extra.is_empty() {
                let mut extra = vec![];
                for (i, (name, value)) in frame.extra.iter().enumerate() {
                    let gap = if i == 0 { "" } else { "  " };
                    extra.push(Span::styled(format!("{}{} ", gap, name), label));
                    extra.push(Span::raw(value.clone()));
                }
                second = Some(Line::from(extra));
            }
        }
        ViewContent::Cloud(cloud_view) if cloud_view.cloud.is_some() => {
            let cloud = cloud_view.cloud.as_ref().unwrap();
            let shape = if cloud.height > 1 {
                format!("{}x{}", cloud.width, cloud.height)
            } else {
                "unordered".to_string()
            };
            if cloud.scan.is_some() {
                spans.extend([
                    Span::styled("Beams ", label),
                    Span::raw(cloud.total_points.to_string()),
                ]);
            } else {
                spans.extend([
                    Span::styled("Points ", label),
                    Span::raw(format!("{} ({})", cloud.total_points, shape)),
                ]);
            }
            if cloud.sampled_points < cloud.total_points {
                spans.push(Span::styled("  Shown ", label));
                spans.push(Span::raw(format!(
                    "every {}th",
                    cloud.total_points / cloud.sampled_points.max(1)
                )));
            }
            spans.extend([
                Span::styled("  Valid ", label),
                Span::raw(format!("{:.1}%", cloud.valid_ratio() * 100.0)),
            ]);
            if !cloud.frame_id.is_empty() {
                spans.push(Span::styled("  Frame ", label));
                spans.push(Span::raw(cloud.frame_id.clone()));
            }
            second = Some(cloud_stats_line(cloud_view, cloud));
        }
        ViewContent::Diagnostics(diag) if !diag.table.is_empty() => {
            let counts = diag.table.counts(std::time::Instant::now());
            spans.push(Span::styled("Statuses ", label));
            for (level, count) in [Level::Ok, Level::Warn, Level::Error, Level::Stale]
                .into_iter()
                .zip(counts)
            {
                let style = if count > 0 {
                    level_style(level)
                } else {
                    Style::default().fg(Color::DarkGray)
                };
                spans.push(Span::styled(format!("{} {}  ", count, level.name()), style));
            }
            if diag.problems_only {
                spans.push(Span::styled(
                    "(problems only)",
                    Style::default().fg(Color::Yellow),
                ));
            }
        }
        ViewContent::Tf(tf_view) if !tf_view.tree.is_empty() => {
            let now = std::time::Instant::now();
            let tree = &tf_view.tree;
            let statics = tree.frames.values().filter(|e| e.is_static).count();
            let rows = tree.rows(now);
            let problems = rows.iter().filter(|r| !r.issues.is_empty()).count();
            let trees = tree.roots().len();
            spans.extend([
                Span::styled("Frames ", label),
                Span::raw(format!(
                    "{} ({} static, {} dynamic)",
                    tree.frames.len(),
                    statics,
                    tree.frames.len() - statics
                )),
                Span::styled("  Trees ", label),
                Span::styled(
                    trees.to_string(),
                    if trees > 1 {
                        Style::default().fg(Color::Yellow)
                    } else {
                        Style::default()
                    },
                ),
                Span::styled("  Problems ", label),
                Span::styled(
                    problems.to_string(),
                    if problems > 0 {
                        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
                    } else {
                        Style::default().fg(Color::Green)
                    },
                ),
            ]);
        }
        ViewContent::Joints(joints) if !joints.table.is_empty() => {
            spans.extend([
                Span::styled("Joints ", label),
                Span::raw(joints.table.joints.len().to_string()),
            ]);
        }
        _ => {
            spans.push(Span::styled("Type ", label));
            spans.push(Span::raw(view.msg_type.as_str()));
        }
    }

    let rate = |hz: Option<f64>| hz.map_or("-".to_string(), |hz| format!("{:.1}", hz));
    spans.extend([
        Span::styled("  Topic ", label),
        Span::raw(format!("{} Hz", rate(view.topic_hz))),
    ]);
    // Unthrottled views have no display rate to show.
    if !view.content.is_unthrottled() {
        spans.extend([
            Span::styled("  Display ", label),
            Span::raw(format!("{} / {} fps", rate(view.display_fps), view.max_fps)),
        ]);
    }
    let graphical = matches!(view.content, ViewContent::Image(_) | ViewContent::Cloud(_));
    if graphical && app.graphics.protocol == GraphicsProtocol::Kitty {
        spans.push(Span::styled("  Link ", label));
        spans.push(Span::raw(format!(
            "{:.1} MB/s",
            app.graphics_bytes_per_second() as f64 / 1e6
        )));
    }

    let has_content = view_has_content(view);
    if view.paused {
        spans.push(Span::styled(
            "  PAUSED",
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ));
    } else if let Some(error) = view.error.as_ref().filter(|_| has_content) {
        spans.push(Span::styled(
            format!("  {}", error),
            Style::default().fg(Color::Red),
        ));
    } else if let Some(age) = view.last_frame_at.map(|at| at.elapsed()) {
        if age.as_secs_f64() > 2.0 {
            // A single message so far is likely a latched topic such as a
            // map, which is fine; a stream that stopped is not.
            let (text, style) = if view.topic_hz.is_none() {
                (
                    format!("  Last message {:.0}s ago", age.as_secs_f64()),
                    Style::default().fg(Color::DarkGray),
                )
            } else {
                (
                    format!("  No message for {:.0}s", age.as_secs_f64()),
                    Style::default().fg(Color::Yellow),
                )
            };
            spans.push(Span::styled(text, style));
        }
    }
    let mut lines = vec![Line::from(spans)];
    lines.extend(second);
    lines
}

/// View settings and cloud statistics: camera, coloring with its value
/// range, grid spacing, fields and the bounding box.
fn cloud_stats_line<'a>(view: &CloudView, cloud: &PointCloud) -> Line<'a> {
    let label = Style::default().fg(Color::Cyan);
    let mut spans = vec![
        Span::styled("View ", label),
        Span::raw(view.camera.preset.name()),
        Span::styled("  Color ", label),
        Span::raw(match view.color {
            ColorMode::Intensity => cloud
                .intensity_field
                .clone()
                .unwrap_or_else(|| "intensity".into()),
            mode => mode.name().to_string(),
        }),
    ];
    if let Some((lo, hi)) = view.color.range(cloud) {
        spans.push(Span::raw(format!(" {:.1}..{:.1}", lo, hi)));
    }
    if view.grid {
        spans.push(Span::styled("  Grid ", label));
        spans.push(Span::raw(format!("{} m", grid_step(view.camera.radius))));
    }
    if let Some(scan) = &cloud.scan {
        spans.extend([
            Span::styled("  Angles ", label),
            Span::raw(format!(
                "{:.1}..{:.1}° step {:.2}°",
                scan.angle_min.to_degrees(),
                scan.angle_max.to_degrees(),
                scan.angle_increment.to_degrees()
            )),
            Span::styled("  Limits ", label),
            Span::raw(format!("{:.2}..{:.1} m", scan.range_min, scan.range_max)),
        ]);
        if let Some((lo, hi)) = scan.measured {
            spans.push(Span::styled("  Measured ", label));
            spans.push(Span::raw(format!("{:.2}..{:.2} m", lo, hi)));
        }
        return Line::from(spans);
    }
    spans.push(Span::styled("  Fields ", label));
    spans.push(Span::raw(
        cloud
            .fields
            .iter()
            .map(|f| f.split(':').next().unwrap_or(f))
            .collect::<Vec<_>>()
            .join(" "),
    ));
    if let Some([x, y, z]) = cloud.bounds {
        spans.push(Span::styled("  Bounds ", label));
        spans.push(Span::raw(format!(
            "x {:.1}..{:.1} y {:.1}..{:.1} z {:.1}..{:.1}",
            x.0, x.1, y.0, y.1, z.0, z.1
        )));
    }
    Line::from(spans)
}

fn render_topic_detail(f: &mut Frame, app: &App) {
    let topic_name = app.selected_topic_name.as_deref().unwrap_or("Unknown");
    let topic_info = app.topic_map.get(topic_name);

    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    // Split the main area into top and bottom halves
    let top_bottom_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(main_chunks[0]);

    let top_chunk = top_bottom_chunks[0];
    let echo_pane_chunk = top_bottom_chunks[1];

    // Split the top half into left and right columns
    let top_left_right_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(top_chunk);

    let detail_info_chunk = top_left_right_chunks[0];
    let chart_chunk = top_left_right_chunks[1];

    // Split the right column into two for the charts
    let chart_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chart_chunk);

    let hz_chart_chunk = chart_chunks[0];
    let delay_chart_chunk = chart_chunks[1];

    // Render Topic Info Pane
    let mut info_content = vec![];
    if let Some(topic) = topic_info {
        info_content.extend(vec![
            Line::from(vec![
                Span::styled("Topic: ", Style::default().fg(Color::Cyan)),
                Span::raw(&topic.name),
            ]),
            Line::from(vec![
                Span::styled("Message Type: ", Style::default().fg(Color::Cyan)),
                Span::raw(&topic.msg_type),
            ]),
            Line::from(vec![
                Span::styled("Publishers:   ", Style::default().fg(Color::Cyan)),
                Span::raw(topic.publisher_count.to_string()),
            ]),
            Line::from(vec![
                Span::styled("Subscribers:  ", Style::default().fg(Color::Cyan)),
                Span::raw(topic.subscriber_count.to_string()),
            ]),
            Line::from(vec![
                Span::styled("Watched:      ", Style::default().fg(Color::Cyan)),
                Span::raw(if topic.watched { "Yes" } else { "No" }),
            ]),
            Line::from(vec![
                Span::styled("Echo:         ", Style::default().fg(Color::Cyan)),
                Span::raw(if app.is_echoing { "Yes" } else { "No" }),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled("Frequency (Hz): ", Style::default().fg(Color::Cyan)),
                Span::raw(match (topic.hz, topic.hz_std_dev) {
                    (Some(hz), Some(std_dev)) if std_dev > 0.0 => {
                        format!("{:.2} ± {:.3}", hz, std_dev)
                    }
                    (Some(hz), _) => format!("{:.2}", hz),
                    _ => "N/A".to_string(),
                }),
            ]),
            Line::from(vec![
                Span::styled("Delay (ms):     ", Style::default().fg(Color::Cyan)),
                Span::raw(match (topic.delay, topic.delay_std_dev) {
                    (Some(delay), Some(std_dev)) if std_dev > 0.0 => {
                        format!("{:.2} ± {:.3}", delay * 1000.0, std_dev * 1000.0)
                    }
                    (Some(delay), _) => format!("{:.2}", delay * 1000.0),
                    _ => "N/A".to_string(),
                }),
            ]),
        ]);
    } else {
        info_content.push(Line::from(Span::raw(format!(
            "Topic '{}' not found.",
            topic_name
        ))));
    }
    let paragraph = Paragraph::new(info_content.clone())
        .block(create_block("Topic Details"))
        .wrap(Wrap { trim: true });
    f.render_widget(paragraph, detail_info_chunk);

    if let Some(topic) = topic_info {
        if topic.watched {
            // Hz Chart with Bollinger Bands - Simple FIFO approach
            // Both histories should ALWAYS have same length since they come from same ROS2 output

            let mut hz_chart_data: Vec<(f64, f64)> = Vec::new();
            let mut upper_band_data: Vec<(f64, f64)> = Vec::new();
            let mut lower_band_data: Vec<(f64, f64)> = Vec::new();

            // Simple approach: both queues should always have identical length
            let data_len = topic.hz_history.len().min(topic.hz_std_dev_history.len());

            for i in 0..data_len {
                let hz_val = topic.hz_history[i];
                let std_dev_val = topic.hz_std_dev_history[i];

                // Main Hz line
                hz_chart_data.push((i as f64, hz_val));

                // Bollinger bands
                let upper_val = hz_val + std_dev_val;
                let lower_val = (hz_val - std_dev_val).max(0.0);
                upper_band_data.push((i as f64, upper_val));
                lower_band_data.push((i as f64, lower_val));
            }

            // Calculate y bounds including Bollinger bands
            let all_values: Vec<f64> = hz_chart_data
                .iter()
                .map(|(_, v)| *v)
                .chain(upper_band_data.iter().map(|(_, v)| *v))
                .chain(lower_band_data.iter().map(|(_, v)| *v))
                .collect();
            let hz_min_val = all_values.iter().cloned().fold(f64::INFINITY, f64::min);
            let hz_max_val = all_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let hz_y_bounds =
                if hz_min_val.is_finite() && hz_max_val.is_finite() && hz_min_val != hz_max_val {
                    [
                        hz_min_val - (hz_max_val - hz_min_val) * 0.1,
                        hz_max_val + (hz_max_val - hz_min_val) * 0.1,
                    ]
                } else if hz_min_val.is_finite() {
                    [hz_min_val * 0.9, hz_min_val * 1.1]
                } else {
                    [0.0, 1.0]
                };

            let hz_x_bounds = [0.0, CHARTS_MAX_DATA_POINTS as f64];

            let mut datasets = vec![];

            // Simple: always add all three datasets if we have any data
            if data_len > 0 {
                // Upper Bollinger band (Hz + std_dev)
                datasets.push(
                    Dataset::default()
                        .name("Upper Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&upper_band_data),
                );

                // Lower Bollinger band (Hz - std_dev)
                datasets.push(
                    Dataset::default()
                        .name("Lower Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&lower_band_data),
                );

                // Main Hz line (should be drawn on top)
                datasets.push(
                    Dataset::default()
                        .name("Hz")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Green))
                        .graph_type(GraphType::Line)
                        .data(&hz_chart_data),
                );
            }

            let hz_chart = Chart::new(datasets)
                .block(create_block("Frequency (Hz)"))
                .x_axis(
                    Axis::default()
                        .title("Time")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(hz_x_bounds)
                        .labels(vec![
                            Span::raw("0"),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS / 2)),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS)),
                        ]),
                )
                .y_axis(
                    Axis::default()
                        .title("Hz")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(hz_y_bounds)
                        .labels(vec![
                            Span::raw(format!("{:.1}", hz_y_bounds[0])),
                            Span::raw(format!("{:.1}", hz_y_bounds[1])),
                        ]),
                );
            f.render_widget(hz_chart, hz_chart_chunk);

            // Delay Chart with Bollinger Bands - Simple FIFO approach
            // Both histories should ALWAYS have same length since they come from same ROS2 output

            let mut delay_chart_data: Vec<(f64, f64)> = Vec::new();
            let mut delay_upper_band_data: Vec<(f64, f64)> = Vec::new();
            let mut delay_lower_band_data: Vec<(f64, f64)> = Vec::new();

            // Simple approach: both queues should always have identical length
            let delay_data_len = topic
                .delay_history
                .len()
                .min(topic.delay_std_dev_history.len());

            for i in 0..delay_data_len {
                let delay_val = topic.delay_history[i];
                let std_dev_val = topic.delay_std_dev_history[i];

                // Main delay line
                delay_chart_data.push((i as f64, delay_val));

                // Bollinger bands
                let upper_val = delay_val + std_dev_val;
                let lower_val = (delay_val - std_dev_val).max(0.0);
                delay_upper_band_data.push((i as f64, upper_val));
                delay_lower_band_data.push((i as f64, lower_val));
            }

            // Calculate y bounds including Bollinger bands
            let all_delay_values: Vec<f64> = delay_chart_data
                .iter()
                .map(|(_, v)| *v)
                .chain(delay_upper_band_data.iter().map(|(_, v)| *v))
                .chain(delay_lower_band_data.iter().map(|(_, v)| *v))
                .collect();
            let delay_min_val = all_delay_values
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);
            let delay_max_val = all_delay_values
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let delay_y_bounds = if delay_min_val.is_finite()
                && delay_max_val.is_finite()
                && delay_min_val != delay_max_val
            {
                [
                    delay_min_val - (delay_max_val - delay_min_val) * 0.1,
                    delay_max_val + (delay_max_val - delay_min_val) * 0.1,
                ]
            } else if delay_min_val.is_finite() {
                [delay_min_val * 0.9, delay_min_val * 1.1]
            } else {
                [0.0, 1.0]
            };

            let delay_x_bounds = [0.0, CHARTS_MAX_DATA_POINTS as f64];

            let mut delay_datasets = vec![];

            // Simple: always add all three datasets if we have any data
            if delay_data_len > 0 {
                // Upper Bollinger band (Delay + std_dev)
                delay_datasets.push(
                    Dataset::default()
                        .name("Upper Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&delay_upper_band_data),
                );

                // Lower Bollinger band (Delay - std_dev)
                delay_datasets.push(
                    Dataset::default()
                        .name("Lower Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&delay_lower_band_data),
                );

                // Main delay line (should be drawn on top)
                delay_datasets.push(
                    Dataset::default()
                        .name("Delay")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Green))
                        .graph_type(GraphType::Line)
                        .data(&delay_chart_data),
                );
            }

            let delay_chart = Chart::new(delay_datasets)
                .block(create_block("Delay (ms)"))
                .x_axis(
                    Axis::default()
                        .title("Time")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(delay_x_bounds)
                        .labels(vec![
                            Span::raw("0"),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS / 2)),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS)),
                        ]),
                )
                .y_axis(
                    Axis::default()
                        .title("ms")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(delay_y_bounds)
                        .labels(vec![
                            Span::raw(format!("{:.1}", delay_y_bounds[0])),
                            Span::raw(format!("{:.1}", delay_y_bounds[1])),
                        ]),
                );
            f.render_widget(delay_chart, delay_chart_chunk);
        } else {
            // Show helpful text in chart areas when topic is not being watched
            let hz_placeholder = Paragraph::new(vec![
                Line::from(""),
                Line::from(""),
                Line::from(Span::styled(
                    "Press 'Enter' to start watching this topic.",
                    Style::default().fg(Color::Yellow),
                )),
            ])
            .block(create_block("Frequency (Hz)"))
            .alignment(ratatui::layout::Alignment::Center);
            f.render_widget(hz_placeholder, hz_chart_chunk);

            let delay_placeholder = Paragraph::new(vec![
                Line::from(""),
                Line::from(""),
                Line::from(Span::styled(
                    "Press 'Enter' to start watching this topic.",
                    Style::default().fg(Color::Yellow),
                )),
            ])
            .block(create_block("Delay (ms)"))
            .alignment(ratatui::layout::Alignment::Center);
            f.render_widget(delay_placeholder, delay_chart_chunk);
        }
    } else {
        // Show placeholder when topic doesn't exist
        let hz_placeholder = Paragraph::new(vec![
            Line::from(""),
            Line::from(""),
            Line::from(Span::styled(
                "Topic not found",
                Style::default().fg(Color::Red),
            )),
        ])
        .block(create_block("Frequency (Hz)"))
        .alignment(ratatui::layout::Alignment::Center);
        f.render_widget(hz_placeholder, hz_chart_chunk);

        let delay_placeholder = Paragraph::new(vec![
            Line::from(""),
            Line::from(""),
            Line::from(Span::styled(
                "Topic not found",
                Style::default().fg(Color::Red),
            )),
        ])
        .block(create_block("Delay (ms)"))
        .alignment(ratatui::layout::Alignment::Center);
        f.render_widget(delay_placeholder, delay_chart_chunk);
    }

    // Render Echo Pane
    let echo_block = Block::default()
        .title("Echo")
        .borders(Borders::ALL)
        .border_style(if app.detail_focus == DetailPaneFocus::Echo {
            Style::default().fg(Color::Cyan)
        } else {
            Style::default()
        });

    let mut lines: Vec<Line> = app
        .echo_content
        .iter()
        .map(|s| Line::from(s.clone()))
        .collect();
    if !app.is_echoing && app.echo_content.is_empty() {
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(
            "Press 'e' to start echoing this topic.",
            Style::default().fg(Color::Yellow),
        )));
    } else if app.is_echoing && app.echo_content.is_empty() {
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(
            "Waiting for messages...",
            Style::default().fg(Color::Gray),
        )));
    }
    let paragraph = Paragraph::new(lines)
        .block(echo_block)
        .scroll((app.echo_scroll_offset as u16, 0));
    f.render_widget(paragraph, echo_pane_chunk);

    // Render scrollbar for echo area
    let echo_area_height = echo_pane_chunk.height.saturating_sub(2) as usize; // Account for borders
    let content_length = app.echo_content.len();
    let max_scroll = content_length.saturating_sub(echo_area_height);
    let scroll_position = app.echo_scroll_offset.min(max_scroll);

    let mut scrollbar_state = ScrollbarState::default()
        .content_length(max_scroll.max(1)) // Ensure minimum of 1 to show scrollbar
        .position(scroll_position);

    let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight)
        .begin_symbol(Some("↑"))
        .end_symbol(Some("↓"));
    f.render_stateful_widget(scrollbar, echo_pane_chunk, &mut scrollbar_state);

    render_status_bar(f, app, main_chunks[1]);
}

fn render_topic_table(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let table_height = area.height.saturating_sub(2) as usize;
    let mut scroll_offset = app.scroll_offset;
    if app.selected_index >= scroll_offset.saturating_add(table_height) {
        scroll_offset = app
            .selected_index
            .saturating_sub(table_height)
            .saturating_add(1);
    }
    if app.selected_index < scroll_offset {
        scroll_offset = app.selected_index;
    }

    let visible_items: Vec<&TopicTreeItem> = app
        .visible_items
        .iter()
        .skip(scroll_offset)
        .take(table_height)
        .collect();

    let header_cells = [
        "W",
        "V",
        "Topic / Group",
        "Type",
        "Pub",
        "Sub",
        "Hz",
        "Delay (ms)",
    ];
    let header = Row::new(header_cells).style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );

    let rows: Vec<Row> = visible_items
        .iter()
        .map(|item| {
            if item.is_group() {
                let style = Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD);
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(item.get_display_name()),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                ])
                .style(style)
            } else if let Some(topic_info) = app.topic_map.get(&item.node.full_path) {
                let watch_indicator = if topic_info.watched { "●" } else { " " };

                let (hz_display, delay_display) = if topic_info.watched {
                    (
                        format_measurement(&topic_info.hz, &topic_info.hz_status, 1.0),
                        format_measurement(&topic_info.delay, &topic_info.delay_status, 1000.0),
                    )
                } else {
                    ("".to_string(), "".to_string())
                };

                let style = if topic_info.watched {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(watch_indicator),
                    viewer_cell(&topic_info.msg_type),
                    Cell::from(item.get_display_name()),
                    Cell::from(topic_info.msg_type.clone()),
                    Cell::from(topic_info.publisher_count.to_string()),
                    Cell::from(topic_info.subscriber_count.to_string()),
                    Cell::from(hz_display),
                    Cell::from(delay_display),
                ])
                .style(style)
            } else {
                // Topic not found in map - show placeholder row
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(item.get_display_name()),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                ])
                .style(Style::default().fg(Color::DarkGray))
            }
        })
        .collect();

    let title = "ROS2 Topics";
    let table = Table::new(
        rows,
        &[
            Constraint::Length(2),
            Constraint::Length(2),
            Constraint::Percentage(45),
            Constraint::Percentage(25),
            Constraint::Length(5),
            Constraint::Length(5),
            Constraint::Length(8),
            Constraint::Length(12),
        ],
    )
    .header(header)
    .block(create_block(title))
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");

    let mut state = ratatui::widgets::TableState::default();
    state.select(Some(app.selected_index.saturating_sub(scroll_offset)));
    f.render_stateful_widget(table, area, &mut state);
}

/// Marks topics that `v` can open in a viewer.
fn viewer_cell(msg_type: &str) -> Cell<'static> {
    if has_viewer(msg_type) {
        Cell::from(Span::styled("◉", Style::default().fg(Color::Cyan)))
    } else {
        Cell::from("")
    }
}

fn format_measurement(val: &Option<f64>, status: &MeasurementStatus, multiplier: f64) -> String {
    match status {
        MeasurementStatus::NotMeasuring => " ".to_string(),
        MeasurementStatus::Loading(frame) => {
            format!("{}{}", ".".repeat(*frame + 1), " ".repeat(2 - *frame))
        }
        MeasurementStatus::HasValue => {
            val.map_or(" ".to_string(), |v| format!("{:.1}", v * multiplier))
        }
        MeasurementStatus::NoStamp => "-".to_string(),
    }
}

fn render_search_mode(f: &mut Frame, app: &App) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    // Build filtered tree directly without creating expensive App instance
    let search_text = &app.search_text;
    let filtered_topics: Vec<ros::TopicInfo> = app
        .topic_map
        .values()
        .filter(|topic| {
            topic
                .name
                .to_lowercase()
                .contains(&search_text.to_lowercase())
        })
        .cloned()
        .collect();

    let mut filtered_tree = TopicTree::new();
    // Convert ros::TopicInfo to tree::TopicInfo
    let tree_topics: Vec<crate::tree::TopicInfo> = filtered_topics
        .iter()
        .map(|t| crate::tree::TopicInfo {
            name: t.name.clone(),
        })
        .collect();
    filtered_tree.build_from_topics(&tree_topics);

    // Automatically expand all groups in the live search preview
    for group in filtered_tree.root.values_mut() {
        group.is_expanded = true;
    }

    let filtered_visible_items = filtered_tree.get_flattened_view();
    let selected_index = app
        .selected_index
        .min(filtered_visible_items.len().saturating_sub(1));

    // Render the filtered search results table
    render_search_table(f, app, &filtered_visible_items, selected_index, chunks[0]);

    let search_prompt = Paragraph::new(Line::from(vec![
        Span::styled("Search: ", Style::default().fg(Color::Yellow)),
        Span::raw(&app.search_text),
        Span::styled(
            "█",
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::SLOW_BLINK),
        ),
    ]))
    .block(
        Block::default()
            .borders(Borders::TOP)
            .title("[Search Mode - Enter to confirm, Esc to cancel]"),
    );

    f.render_widget(search_prompt, chunks[1]);
}

fn render_search_table(
    f: &mut Frame,
    app: &App,
    visible_items: &[TopicTreeItem],
    selected_index: usize,
    area: ratatui::layout::Rect,
) {
    let table_height = area.height.saturating_sub(2) as usize;
    let mut scroll_offset: usize = 0;
    if selected_index >= scroll_offset.saturating_add(table_height) {
        scroll_offset = selected_index
            .saturating_sub(table_height)
            .saturating_add(1);
    }
    if selected_index < scroll_offset {
        scroll_offset = selected_index;
    }

    let visible_slice: Vec<&TopicTreeItem> = visible_items
        .iter()
        .skip(scroll_offset)
        .take(table_height)
        .collect();

    let header_cells = [
        "W",
        "V",
        "Topic / Group",
        "Type",
        "Pub",
        "Sub",
        "Hz",
        "Delay (ms)",
    ];
    let header = Row::new(header_cells).style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );

    let rows: Vec<Row> = visible_slice
        .iter()
        .map(|item| {
            if item.is_group() {
                let style = Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD);
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(item.get_display_name()),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                ])
                .style(style)
            } else if let Some(topic_info) = app.topic_map.get(&item.node.full_path) {
                let watch_indicator = if topic_info.watched { "●" } else { " " };

                let (hz_display, delay_display) = if topic_info.watched {
                    (
                        format_measurement(&topic_info.hz, &topic_info.hz_status, 1.0),
                        format_measurement(&topic_info.delay, &topic_info.delay_status, 1000.0),
                    )
                } else {
                    ("".to_string(), "".to_string())
                };

                let style = if topic_info.watched {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(watch_indicator),
                    viewer_cell(&topic_info.msg_type),
                    Cell::from(item.get_display_name()),
                    Cell::from(topic_info.msg_type.clone()),
                    Cell::from(topic_info.publisher_count.to_string()),
                    Cell::from(topic_info.subscriber_count.to_string()),
                    Cell::from(hz_display),
                    Cell::from(delay_display),
                ])
                .style(style)
            } else {
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(item.get_display_name()),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                ])
            }
        })
        .collect();

    let title = "ROS2 Topics (Search Results)";
    let table = Table::new(
        rows,
        &[
            Constraint::Length(2),
            Constraint::Length(2),
            Constraint::Percentage(45),
            Constraint::Percentage(25),
            Constraint::Length(5),
            Constraint::Length(5),
            Constraint::Length(8),
            Constraint::Length(12),
        ],
    )
    .header(header)
    .block(create_block(title))
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");

    let mut state = ratatui::widgets::TableState::default();
    state.select(Some(selected_index.saturating_sub(scroll_offset)));
    f.render_stateful_widget(table, area, &mut state);
}

/// Format seconds as "1.2s", or "-" for none.
fn seconds_text(value: Option<f64>) -> String {
    value.map_or("-".to_string(), |v| format!("{:.1}s", v))
}

/// Delay of a stamp in milliseconds, or seconds when large; stamps far off
/// the wall clock (sim time, unset stamps) show as "-".
fn delay_text(delay: f64) -> String {
    if delay.abs() > 86_400.0 {
        "-".to_string()
    } else if delay.abs() >= 1.0 {
        format!("{:.1}s", delay)
    } else {
        format!("{:.1}ms", delay * 1000.0)
    }
}

/// Roll, pitch and yaw in degrees.
fn rpy_text(q: [f64; 4]) -> String {
    let [r, p, y] = imu::euler(q);
    format!(
        "{:>6.1}° {:>6.1}° {:>6.1}°",
        r.to_degrees(),
        p.to_degrees(),
        y.to_degrees()
    )
}

fn xyz_text(v: [f64; 3]) -> String {
    format!("{:>7.3} {:>7.3} {:>7.3}", v[0], v[1], v[2])
}

/// The frame tree with rates, ages and problems, above the details of the
/// selected frame.
fn render_tf(f: &mut Frame, view: &TfView, area: ratatui::layout::Rect) {
    let now = std::time::Instant::now();
    let tree = &view.tree;
    let rows = tree.rows(now);
    let selected = view.selected.min(rows.len().saturating_sub(1));
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(4), Constraint::Length(7)])
        .split(area);

    let header = Row::new([
        "Frame",
        "Kind",
        "Rate",
        "Age",
        "Delay",
        "Translation (m)",
        "Roll Pitch Yaw",
        "Status",
    ])
    .style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );
    let table_rows = rows.iter().map(|row| {
        let entry = tree.frames.get(&row.frame);
        let status = if row.issues.is_empty() {
            Span::styled("OK", Style::default().fg(Color::Green))
        } else {
            let text: Vec<String> = row.issues.iter().map(Issue::text).collect();
            let color = if row.issues.iter().any(Issue::is_error) {
                Color::Red
            } else {
                Color::Yellow
            };
            Span::styled(text.join("; "), Style::default().fg(color))
        };
        let name = Span::raw(format!("{}{}", row.prefix, row.frame));
        match entry {
            None => Row::new(vec![
                Cell::from(name),
                Cell::from(Span::styled("root", Style::default().fg(Color::DarkGray))),
                Cell::from(""),
                Cell::from(""),
                Cell::from(""),
                Cell::from(""),
                Cell::from(""),
                Cell::from(status),
            ]),
            Some(e) => Row::new(vec![
                Cell::from(name),
                Cell::from(if e.is_static { "static" } else { "dynamic" }),
                // A stopped frame has no rate, whatever it had before.
                Cell::from(if e.is_static || e.stale_for(now).is_some() {
                    "-".to_string()
                } else {
                    e.rate(now)
                        .map_or("-".to_string(), |hz| format!("{:.1} Hz", hz))
                }),
                Cell::from(if e.is_static {
                    "-".to_string()
                } else {
                    seconds_text(Some(e.age(now)))
                }),
                Cell::from(if e.is_static {
                    "-".to_string()
                } else {
                    delay_text(e.delay)
                }),
                Cell::from(xyz_text(e.translation)),
                Cell::from(rpy_text(e.rotation)),
                Cell::from(status),
            ]),
        }
    });
    let table = Table::new(
        table_rows,
        [
            Constraint::Min(24),
            Constraint::Length(7),
            Constraint::Length(9),
            Constraint::Length(7),
            Constraint::Length(9),
            Constraint::Length(23),
            Constraint::Length(23),
            Constraint::Min(14),
        ],
    )
    .header(header)
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");
    let mut state = ratatui::widgets::TableState::default();
    state.select((!rows.is_empty()).then_some(selected));
    f.render_stateful_widget(table, chunks[0], &mut state);

    let Some(row) = rows.get(selected) else {
        return;
    };
    let chain = tree.chain(&row.frame);
    let root = chain.last().cloned().unwrap_or_default();
    let (translation, rotation) = tree.in_root(&row.frame);
    let mut details = vec![
        ("Chain", vec![Span::raw(chain.join(" → "))]),
        (
            "In root",
            vec![Span::raw(format!(
                "{}: xyz {} m, rpy {}",
                root,
                xyz_text(translation),
                rpy_text(rotation)
            ))],
        ),
    ];
    // Static transforms are valid at any time, so their stamps say nothing.
    if let Some(e) = tree.frames.get(&row.frame).filter(|e| !e.is_static) {
        details.push((
            "Stamp",
            vec![Span::raw(format!(
                "{:.3} (delay {})",
                e.stamp,
                delay_text(e.delay)
            ))],
        ));
    }
    for issue in &row.issues {
        let color = if issue.is_error() {
            Color::Red
        } else {
            Color::Yellow
        };
        let text = match issue {
            Issue::MultipleParents(parents) => format!(
                "published with parents {}: two broadcasters disagree",
                parents.join(" and ")
            ),
            Issue::Stale(age) => format!("no update for {:.1}s", age),
            Issue::Cycle => "the frame is its own ancestor".to_string(),
        };
        details.push((
            "Problem",
            vec![Span::styled(text, Style::default().fg(color))],
        ));
    }
    f.render_widget(
        Paragraph::new(labeled_lines(details))
            .wrap(Wrap { trim: false })
            .block(create_block(&row.frame)),
        chunks[1],
    );
}

/// Pose, velocity and trail checks beside the trail, above velocity plots.
fn render_odometry(f: &mut Frame, app: &App, view: &OdometryView, area: ratatui::layout::Rect) {
    let Some(odom) = &view.latest else {
        return;
    };
    let rows_area = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
        .split(area);
    let top = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Length(52), Constraint::Min(10)])
        .split(rows_area[0]);

    let [px, py, pz] = odom.pose.position;
    let mut rows = vec![
        (
            "Frames",
            vec![Span::raw(format!(
                "{} → {}",
                odom.frame_id, odom.child_frame_id
            ))],
        ),
        (
            "Position",
            vec![Span::raw(format!(
                "x {:.3}  y {:.3}  z {:.3} m",
                px, py, pz
            ))],
        ),
        (
            "Heading",
            vec![Span::raw(format!("{:.1}°", odom.pose.yaw().to_degrees()))],
        ),
        (
            "Speed",
            vec![Span::raw(format!(
                "{:.2} m/s  (x {:.2} y {:.2} z {:.2})",
                odom.speed(),
                odom.linear[0],
                odom.linear[1],
                odom.linear[2]
            ))],
        ),
        (
            "Turn rate",
            vec![Span::raw(format!(
                "{:.1}°/s around z",
                odom.angular[2].to_degrees()
            ))],
        ),
        (
            "Travelled",
            vec![Span::raw(format!("{:.2} m", view.trail.distance))],
        ),
        (
            "Jumps",
            vec![if view.trail.jumps > 0 {
                Span::styled(
                    format!(
                        "{} (pose moved further than the speed explains)",
                        view.trail.jumps
                    ),
                    Style::default().fg(Color::Yellow),
                )
            } else {
                Span::styled("none", Style::default().fg(Color::Green))
            }],
        ),
        (
            "Std dev",
            vec![match odom.pose_std_dev() {
                Some([x, y, yaw]) => Span::raw(format!(
                    "x {:.3} m  y {:.3} m  yaw {:.2}°",
                    x,
                    y,
                    yaw.to_degrees()
                )),
                None => Span::styled("no covariance", Style::default().fg(Color::DarkGray)),
            }],
        ),
    ];
    if odom.child_frame_id.is_empty() {
        rows.push((
            "Warning",
            vec![Span::styled(
                "child_frame_id is empty",
                Style::default().fg(Color::Yellow),
            )],
        ));
    }
    f.render_widget(
        Paragraph::new(labeled_lines(rows)).block(create_block("Odometry")),
        top[0],
    );

    let title = format!("Trail (m, {} frame)", odom.frame_id);
    let block = create_block(&title);
    let inner = block.inner(top[1]);
    let points: Vec<(f64, f64)> = view.trail.points.iter().copied().collect();
    let (x_bounds, y_bounds) = equal_aspect_bounds(app, &points, inner);
    let yaw = odom.pose.yaw();
    let arrow = (x_bounds[1] - x_bounds[0]).min(y_bounds[1] - y_bounds[0]) * 0.06;
    let canvas = ratatui::widgets::canvas::Canvas::default()
        .block(block)
        .marker(symbols::Marker::Braille)
        .x_bounds(x_bounds)
        .y_bounds(y_bounds)
        .paint(move |ctx| {
            ctx.draw(&ratatui::widgets::canvas::Points {
                coords: &points,
                color: Color::Cyan,
            });
            ctx.draw(&ratatui::widgets::canvas::Line {
                x1: px,
                y1: py,
                x2: px + arrow * yaw.cos(),
                y2: py + arrow * yaw.sin(),
                color: Color::Yellow,
            });
            ctx.print(
                px,
                py,
                Span::styled("◆", Style::default().fg(Color::Yellow)),
            );
        });
    f.render_widget(canvas, top[1]);

    let plots = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(rows_area[1]);
    let now = std::time::Instant::now();
    let series = |pick: fn(&super::app::OdometryPoint) -> [f64; 3]| {
        let mut axes: [Vec<(f64, f64)>; 3] = Default::default();
        for entry in &view.history {
            let t = -now.duration_since(entry.at).as_secs_f64();
            for (axis, values) in axes.iter_mut().enumerate() {
                values.push((t, pick(entry)[axis]));
            }
        }
        axes
    };
    render_xyz_chart(f, "Linear velocity (m/s)", &series(|p| p.linear), plots[0]);
    render_xyz_chart(
        f,
        "Angular velocity (rad/s)",
        &series(|p| p.angular),
        plots[1],
    );
}

/// Path details beside the path, start green, end red.
fn render_path(f: &mut Frame, app: &App, view: &PathView, area: ratatui::layout::Rect) {
    let Some(path) = &view.latest else {
        return;
    };
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Length(44), Constraint::Min(10)])
        .split(area);
    let point = |p: Option<&super::odometry::Pose>| {
        p.map_or("-".to_string(), |p| {
            format!("x {:.2}  y {:.2} m", p.position[0], p.position[1])
        })
    };
    let mut rows = vec![
        ("Frame", vec![Span::raw(path.frame_id.clone())]),
        ("Poses", vec![Span::raw(path.poses.len().to_string())]),
        ("Length", vec![Span::raw(format!("{:.2} m", path.length()))]),
        ("Start", vec![Span::raw(point(path.poses.first()))]),
        ("End", vec![Span::raw(point(path.poses.last()))]),
        ("Updates", vec![Span::raw(view.updates.to_string())]),
    ];
    if path.poses.is_empty() {
        rows.push((
            "Warning",
            vec![Span::styled(
                "empty path",
                Style::default().fg(Color::Yellow),
            )],
        ));
    }
    f.render_widget(
        Paragraph::new(labeled_lines(rows)).block(create_block("Path")),
        chunks[0],
    );

    let title = format!("Path (m, {} frame)", path.frame_id);
    let block = create_block(&title);
    let inner = block.inner(chunks[1]);
    let points: Vec<(f64, f64)> = path
        .poses
        .iter()
        .map(|p| (p.position[0], p.position[1]))
        .collect();
    let (x_bounds, y_bounds) = equal_aspect_bounds(app, &points, inner);
    let canvas = ratatui::widgets::canvas::Canvas::default()
        .block(block)
        .marker(symbols::Marker::Braille)
        .x_bounds(x_bounds)
        .y_bounds(y_bounds)
        .paint(move |ctx| {
            for w in points.windows(2) {
                ctx.draw(&ratatui::widgets::canvas::Line {
                    x1: w[0].0,
                    y1: w[0].1,
                    x2: w[1].0,
                    y2: w[1].1,
                    color: Color::Cyan,
                });
            }
            if let Some(&(x, y)) = points.first() {
                ctx.print(x, y, Span::styled("●", Style::default().fg(Color::Green)));
            }
            if let Some(&(x, y)) = points.last() {
                ctx.print(x, y, Span::styled("◆", Style::default().fg(Color::Red)));
            }
        });
    f.render_widget(canvas, chunks[1]);
}

/// A bar of `width` cells with a marker where `value` lies in `lo..hi`.
fn range_bar(value: f64, lo: f64, hi: f64, width: usize) -> String {
    if hi - lo < 1e-9 || !value.is_finite() {
        return format!("{:─^width$}", "");
    }
    let at = (((value - lo) / (hi - lo)) * (width - 1) as f64).round() as usize;
    (0..width)
        .map(|i| match i.cmp(&at.min(width - 1)) {
            std::cmp::Ordering::Less => '━',
            std::cmp::Ordering::Equal => '●',
            std::cmp::Ordering::Greater => '─',
        })
        .collect()
}

/// The joint table, above plots of the selected joint.
fn render_joints(f: &mut Frame, view: &JointsView, area: ratatui::layout::Rect) {
    let now = std::time::Instant::now();
    let table = &view.table;
    let names: Vec<&String> = table.joints.keys().collect();
    let selected = view.selected.min(names.len().saturating_sub(1));
    let warning_height = u16::from(table.malformed.is_some());
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(warning_height),
            Constraint::Min(4),
            Constraint::Length(12),
        ])
        .split(area);
    if let Some(malformed) = &table.malformed {
        f.render_widget(
            Paragraph::new(Span::styled(
                format!("Malformed message: {}", malformed),
                Style::default().fg(Color::Yellow),
            )),
            chunks[0],
        );
    }

    let number = |v: Option<f64>, digits: usize| match v {
        Some(v) if v.is_finite() => format!("{:>9.digits$}", v),
        Some(_) => "      NaN".to_string(),
        None => "        -".to_string(),
    };
    let header = Row::new([
        "Joint",
        "Position",
        "deg",
        "Seen range",
        "Velocity",
        "Effort",
        "Age",
    ])
    .style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );
    let rows = table.joints.iter().map(|(name, joint)| {
        let age = now.duration_since(joint.updated);
        let bar = match (joint.position, joint.range) {
            (Some(p), Some((lo, hi))) => {
                format!("{:>7.2} {} {:<7.2}", lo, range_bar(p, lo, hi, 16), hi)
            }
            _ => String::new(),
        };
        let style = if age > JOINT_STALE_AFTER {
            Style::default().fg(Color::DarkGray)
        } else if [joint.position, joint.velocity, joint.effort]
            .iter()
            .flatten()
            .any(|v| !v.is_finite())
        {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        };
        Row::new(vec![
            Cell::from(name.clone()),
            Cell::from(number(joint.position, 4)),
            Cell::from(number(joint.position.map(f64::to_degrees), 1)),
            Cell::from(bar),
            Cell::from(number(joint.velocity, 3)),
            Cell::from(number(joint.effort, 2)),
            Cell::from(seconds_text(Some(age.as_secs_f64()))),
        ])
        .style(style)
    });
    let widget = Table::new(
        rows,
        [
            Constraint::Min(16),
            Constraint::Length(10),
            Constraint::Length(10),
            Constraint::Length(34),
            Constraint::Length(10),
            Constraint::Length(10),
            Constraint::Length(7),
        ],
    )
    .header(header)
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");
    let mut state = ratatui::widgets::TableState::default();
    state.select((!names.is_empty()).then_some(selected));
    f.render_stateful_widget(widget, chunks[1], &mut state);

    let Some(joint) = names.get(selected).map(|name| &table.joints[*name]) else {
        return;
    };
    let mut position = vec![];
    let mut velocity = vec![];
    for (at, p, v) in &joint.history {
        let t = -now.duration_since(*at).as_secs_f64();
        position.extend(p.map(|p| (t, p)));
        velocity.extend(v.map(|v| (t, v)));
    }
    let plots = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[2]);
    let name = names[selected];
    render_series_chart(
        f,
        &format!("{} position", name),
        &[("position", Color::Cyan, &position)],
        IMU_HISTORY,
        plots[0],
    );
    render_series_chart(
        f,
        &format!("{} velocity", name),
        &[("velocity", Color::Magenta, &velocity)],
        IMU_HISTORY,
        plots[1],
    );
}

/// Charge gauge and battery details beside the cells, above voltage and
/// current plots of the last minutes.
fn render_battery(f: &mut Frame, view: &BatteryView, area: ratatui::layout::Rect) {
    let Some(b) = &view.latest else {
        return;
    };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(8),
            Constraint::Length(10),
        ])
        .split(area);

    let fraction = b.fraction();
    let color = match fraction {
        Some(f) if f < 0.15 => Color::Red,
        Some(f) if f < 0.3 => Color::Yellow,
        _ => Color::Green,
    };
    let mut label = match fraction {
        Some(f) => format!("{:.0}% · {}", f * 100.0, b.status_name()),
        None => format!("charge not reported · {}", b.status_name()),
    };
    if let Some(hours) = b.hours_left() {
        label.push_str(&format!(
            " · {}h {:02}m left",
            hours.floor() as u64,
            (hours.fract() * 60.0) as u64
        ));
    }
    f.render_widget(
        ratatui::widgets::Gauge::default()
            .block(create_block("Charge"))
            .gauge_style(Style::default().fg(color))
            .ratio(fraction.unwrap_or(0.0))
            .label(Span::styled(
                label,
                Style::default().add_modifier(Modifier::BOLD),
            )),
        chunks[0],
    );

    let middle = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Length(54), Constraint::Min(10)])
        .split(chunks[1]);
    let value = |v: f32, unit: &str| match measured(v) {
        Some(v) => Span::raw(format!("{:.2} {}", v, unit)),
        None => Span::styled("not measured", Style::default().fg(Color::DarkGray)),
    };
    let mut current = vec![value(b.current, "A")];
    if let (Some(v), Some(i)) = (measured(b.voltage), measured(b.current)) {
        current.push(Span::raw(format!(
            "  ({} {:.0} W)",
            if i >= 0.0 { "charging" } else { "drawing" },
            (v * i).abs()
        )));
    }
    let mut capacity = vec![
        value(b.charge, "Ah"),
        Span::raw(" of "),
        value(b.capacity, "Ah"),
    ];
    if let Some(design) = measured(b.design_capacity) {
        capacity.push(Span::raw(format!(" (design {:.1} Ah)", design)));
    }
    let health_style = if b.health_is_bad() {
        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
    } else {
        Style::default()
    };
    let rows = vec![
        ("Voltage", vec![value(b.voltage, "V")]),
        ("Current", current),
        ("Charge", capacity),
        ("Temperature", vec![value(b.temperature, "°C")]),
        ("Health", vec![Span::styled(b.health_name(), health_style)]),
        ("Technology", vec![Span::raw(b.technology_name())]),
        (
            "Present",
            vec![if b.present {
                Span::raw("yes")
            } else {
                Span::styled("no", Style::default().fg(Color::Red))
            }],
        ),
        (
            "Location",
            vec![Span::raw(format!("{} {}", b.location, b.serial_number))],
        ),
    ];
    f.render_widget(
        Paragraph::new(labeled_lines(rows)).block(create_block("Battery")),
        middle[0],
    );

    // Cells, the weakest highlighted when they drift apart.
    let mut cells = vec![];
    if let Some((lo, hi)) = b.cell_spread() {
        let unbalanced = hi - lo > 0.05;
        cells.push(Line::from(vec![
            Span::styled("Spread ", Style::default().fg(Color::Cyan)),
            Span::styled(
                format!("{:.0} mV", (hi - lo) * 1000.0),
                Style::default().fg(if unbalanced {
                    Color::Yellow
                } else {
                    Color::Green
                }),
            ),
        ]));
        for (i, v) in b.cell_voltage.iter().enumerate() {
            let text = match measured(*v) {
                Some(v) => {
                    let bar = range_bar(v, lo - 0.05, hi, 20);
                    let style = if unbalanced && (v - lo).abs() < 1e-6 {
                        Style::default().fg(Color::Yellow)
                    } else {
                        Style::default()
                    };
                    Span::styled(format!("{:>2}  {:.3} V  {}", i + 1, v, bar), style)
                }
                None => Span::styled(
                    format!("{:>2}  not measured", i + 1),
                    Style::default().fg(Color::DarkGray),
                ),
            };
            cells.push(Line::from(text));
        }
    } else {
        cells.push(Line::from(Span::styled(
            "No cell voltages reported",
            Style::default().fg(Color::DarkGray),
        )));
    }
    f.render_widget(
        Paragraph::new(cells).block(create_block("Cells")),
        middle[1],
    );

    let now = std::time::Instant::now();
    let mut voltage = vec![];
    let mut current = vec![];
    for (at, v, i) in &view.history {
        let t = -now.duration_since(*at).as_secs_f64();
        voltage.extend(v.map(|v| (t, v)));
        current.extend(i.map(|i| (t, i)));
    }
    let plots = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[2]);
    render_series_chart(
        f,
        "Voltage (V)",
        &[("voltage", Color::Yellow, &voltage)],
        BATTERY_HISTORY,
        plots[0],
    );
    render_series_chart(
        f,
        "Current (A)",
        &[("current", Color::Magenta, &current)],
        BATTERY_HISTORY,
        plots[1],
    );
}
