use super::cloud_render::{self, CameraPreset, CloudCamera, ColorMode, RenderOptions};
use super::diagnostics::{self, DiagnosticsTable};
use super::image::{self, DecodedImage};
use super::imu::{self, ImuSample};
use super::navsat::{self, NavSatFix, Track};
use super::occupancy;
use super::pointcloud::{self, PointCloud};
use super::ros;
use super::ros::MeasurementStatus;
use super::watcher::{
    FrameContent, RosBridgeHandle, TopicMessage, ViewFrame, ViewTarget, WatchMessage,
};
use crate::common::graphics::{GraphicsProtocol, KittyPlacement, TerminalGraphics};
use crate::common::{TopicTree, TopicTreeItem};
use ::image::RgbImage;
use crossbeam::channel::Receiver;
use ratatui::layout::Rect;
use std::cell::{Cell, Ref, RefCell};
use std::collections::{HashMap, VecDeque};
use std::io::{self, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

pub const CHARTS_MAX_DATA_POINTS: usize = 100;

/// Selectable viewer frame rate limits.
pub const VIEW_FPS_STEPS: [u32; 6] = [1, 2, 5, 10, 15, 30];
const DEFAULT_VIEW_FPS: u32 = 10;
/// IMUs publish fast; a higher default keeps their plots detailed.
const DEFAULT_IMU_FPS: u32 = 30;
/// Span of the IMU plots.
pub const IMU_HISTORY: Duration = Duration::from_secs(10);
/// Rate limit for diagnostics, high enough to never drop a message: every
/// driver publishes its own partial array, and each one matters.
const UNTHROTTLED_FPS: u32 = 1000;
/// Span over which the viewer measures rates.
const VIEW_RATE_WINDOW: Duration = Duration::from_secs(2);
/// Point cloud orbit step per arrow key press.
const ORBIT_STEP: f32 = 2.5 * std::f32::consts::PI / 180.0;
/// Zoom per scroll wheel event; trackpads send many small ones.
const WHEEL_ZOOM_STEP: f32 = 1.1;

#[derive(PartialEq, Debug)]
pub enum AppMode {
    TopicList,
    TopicDetail,
    Search,
    Help,
    TopicView,
}

#[derive(PartialEq, Debug)]
pub enum DetailPaneFocus {
    Info,
    Echo,
}

/// Whether `v` can open a viewer for messages of `msg_type`.
pub fn has_viewer(msg_type: &str) -> bool {
    image::is_image_type(msg_type)
        || pointcloud::is_pointcloud_type(msg_type)
        || pointcloud::is_laserscan_type(msg_type)
        || diagnostics::is_diagnostics_type(msg_type)
        || occupancy::is_occupancy_grid_type(msg_type)
        || navsat::is_navsatfix_type(msg_type)
        || imu::is_imu_type(msg_type)
}

/// Empty view content for a topic of `msg_type`.
fn view_content(msg_type: &str) -> ViewContent {
    if pointcloud::is_pointcloud_type(msg_type) || pointcloud::is_laserscan_type(msg_type) {
        ViewContent::Cloud(Box::new(CloudView::new()))
    } else if diagnostics::is_diagnostics_type(msg_type) {
        ViewContent::Diagnostics(DiagnosticsView::default())
    } else if navsat::is_navsatfix_type(msg_type) {
        ViewContent::Gnss(GnssView::default())
    } else if imu::is_imu_type(msg_type) {
        ViewContent::Imu(ImuView::default())
    } else {
        // Images and occupancy grids.
        ViewContent::Image(None)
    }
}

/// What a topic view shows; `None` until the first message arrives.
pub enum ViewContent {
    Image(Option<DecodedImage>),
    Cloud(Box<CloudView>),
    Diagnostics(DiagnosticsView),
    Gnss(GnssView),
    Imu(ImuView),
}

/// GNSS viewer state: the latest fix and the track of recent ones.
#[derive(Default)]
pub struct GnssView {
    pub latest: Option<NavSatFix>,
    pub track: Track,
    pub fixes: usize,
}

/// IMU viewer state: the latest sample and the recent history the plots show.
#[derive(Default)]
pub struct ImuView {
    pub latest: Option<ImuSample>,
    pub history: VecDeque<ImuPoint>,
}

pub struct ImuPoint {
    pub at: Instant,
    pub acceleration: Option<[f64; 3]>,
    pub turn_rate: Option<[f64; 3]>,
}

/// Diagnostics viewer state: statuses merged from all arrays received.
#[derive(Default)]
pub struct DiagnosticsView {
    pub table: DiagnosticsTable,
    /// Row selected in the status table, clamped when drawn.
    pub selected: usize,
    pub problems_only: bool,
}

/// Point cloud viewer state: the latest cloud and how it is looked at.
pub struct CloudView {
    pub cloud: Option<PointCloud>,
    pub camera: CloudCamera,
    pub color: ColorMode,
    pub grid: bool,
    /// View radius fitted to the first cloud, restored by a reset.
    fitted_radius: Option<f32>,
    rendered: RefCell<Option<RenderedCloud>>,
}

/// A rendered cloud and what it was rendered from, to skip re-rendering
/// while nothing changes.
pub struct RenderedCloud {
    pub image: RgbImage,
    /// Distinguishes renders, like `TopicView::frame_seq` frames.
    pub seq: u64,
    key: RenderKey,
}

#[derive(PartialEq)]
struct RenderKey {
    frame_seq: u64,
    camera: CloudCamera,
    color: ColorMode,
    grid: bool,
    size: (u32, u32),
}

impl CloudView {
    fn new() -> Self {
        Self {
            cloud: None,
            camera: CloudCamera::new(CameraPreset::Top, 10.0),
            color: ColorMode::Height,
            grid: true,
            fitted_radius: None,
            rendered: RefCell::new(None),
        }
    }

    fn set_cloud(&mut self, cloud: PointCloud) {
        if self.fitted_radius.is_none() && !cloud.points.is_empty() {
            let radius = cloud_render::fit_radius(&cloud);
            self.fitted_radius = Some(radius);
            self.camera = CloudCamera::new(self.camera.preset, radius);
            self.color = ColorMode::available(&cloud)[0];
        }
        self.cloud = Some(cloud);
    }

    fn reset_camera(&mut self, preset: CameraPreset) {
        self.camera = CloudCamera::new(preset, self.fitted_radius.unwrap_or(10.0));
    }

    fn cycle_color(&mut self) {
        if let Some(cloud) = &self.cloud {
            let modes = ColorMode::available(cloud);
            let next = modes
                .iter()
                .position(|m| *m == self.color)
                .map_or(0, |i| (i + 1) % modes.len());
            self.color = modes[next];
        }
    }

    /// The cloud rendered at `width` x `height`, re-rendered only when the
    /// cloud, the camera or the size changed since the last call.
    pub fn render(
        &self,
        frame_seq: u64,
        width: u32,
        height: u32,
        pixel_aspect: f32,
    ) -> Option<Ref<'_, RenderedCloud>> {
        let cloud = self.cloud.as_ref()?;
        let key = RenderKey {
            frame_seq,
            camera: self.camera,
            color: self.color,
            grid: self.grid,
            size: (width, height),
        };
        let stale = self.rendered.borrow().as_ref().is_none_or(|r| r.key != key);
        if stale {
            let options = RenderOptions {
                camera: self.camera,
                color: self.color,
                grid: self.grid,
                width,
                height,
                pixel_aspect,
            };
            let seq = self.rendered.borrow().as_ref().map_or(0, |r| r.seq + 1);
            *self.rendered.borrow_mut() = Some(RenderedCloud {
                image: cloud_render::render(cloud, &options),
                seq,
                key,
            });
        }
        Ref::filter_map(self.rendered.borrow(), |r| r.as_ref()).ok()
    }
}

/// State of the viewer for one topic.
pub struct TopicView {
    pub topic_name: String,
    pub msg_type: String,
    pub content: ViewContent,
    /// Incremented for every new message, to tell when to redraw.
    pub frame_seq: u64,
    pub error: Option<String>,
    pub max_fps: u32,
    pub paused: bool,
    /// Rate of the topic itself, from the helper's message counter.
    pub topic_hz: Option<f64>,
    /// Rate at which frames reach the display.
    pub display_fps: Option<f64>,
    pub last_frame_at: Option<Instant>,
    /// Cells the image was laid out in by the last draw.
    pub placement: Cell<Option<Rect>>,
    pub target: Arc<ViewTarget>,
    /// (messages received by the helper, arrival time) of recent frames.
    rate_samples: VecDeque<(u32, Instant)>,
    return_mode: AppMode,
}

impl TopicView {
    fn on_frame(&mut self, frame: ViewFrame) {
        let now = Instant::now();
        // Rates over a window rather than averaged per-frame rates, which
        // overstate the rate whenever frame intervals jitter.
        self.rate_samples.push_back((frame.received_count, now));
        while self.rate_samples.len() > 2
            && now.duration_since(self.rate_samples[0].1) > VIEW_RATE_WINDOW
        {
            self.rate_samples.pop_front();
        }
        if let (Some(&(first_count, first_at)), Some(&(last_count, last_at))) =
            (self.rate_samples.front(), self.rate_samples.back())
        {
            let span = last_at.duration_since(first_at).as_secs_f64();
            if span > 0.0 {
                let received = last_count.wrapping_sub(first_count) as f64;
                self.topic_hz = Some(received / span);
                self.display_fps = Some((self.rate_samples.len() - 1) as f64 / span);
            }
        }
        self.last_frame_at = Some(now);
        match (&mut self.content, frame.content) {
            (ViewContent::Image(image), FrameContent::Image(new)) => *image = Some(new),
            (ViewContent::Cloud(view), FrameContent::Cloud(cloud)) => view.set_cloud(cloud),
            (ViewContent::Diagnostics(view), FrameContent::Diagnostics(statuses)) => {
                view.table.update(statuses, now)
            }
            (ViewContent::Gnss(view), FrameContent::Gnss(fix)) => {
                view.track.push(&fix);
                view.fixes += 1;
                view.latest = Some(fix);
            }
            (ViewContent::Imu(view), FrameContent::Imu(sample)) => {
                view.history.push_back(ImuPoint {
                    at: now,
                    acceleration: sample.linear_acceleration,
                    turn_rate: sample.angular_velocity,
                });
                while view
                    .history
                    .front()
                    .is_some_and(|p| now.duration_since(p.at) > IMU_HISTORY)
                {
                    view.history.pop_front();
                }
                view.latest = Some(sample);
            }
            _ => return,
        }
        self.frame_seq += 1;
        self.error = None;
    }
}

pub struct App {
    pub should_quit: bool,
    pub mode: AppMode,

    pub topic_map: HashMap<String, ros::TopicInfo>,

    pub master_tree: TopicTree,
    pub visible_items: Vec<TopicTreeItem>,

    pub selected_index: usize,
    pub scroll_offset: usize,

    pub filter_text: String,
    pub search_text: String,

    pub selected_topic_name: Option<String>,
    pub detail_scroll_offset: usize,
    pub detail_focus: DetailPaneFocus,

    pub echo_content: Vec<String>,
    pub is_echoing: bool,
    pub echo_scroll_offset: usize,

    pub error_message: Option<String>,
    pub last_animation_update: std::time::Instant,

    pub expansion_state: HashMap<String, bool>,
    pub use_sim_time: bool,

    pub view: Option<TopicView>,
    pub graphics: TerminalGraphics,
    kitty: KittyPlacement,
    /// Redraw the kitty image even if unchanged, e.g. after a resize cleared it.
    force_graphics_redraw: bool,

    message_receiver: Receiver<TopicMessage>,
    bridge: RosBridgeHandle,
}

impl App {
    pub fn new(
        refresh_interval: Duration,
        _detail_refresh_interval: Duration,
        initial_fetch: bool,
    ) -> Self {
        let (topic_sender, topic_receiver) = crossbeam::channel::unbounded();

        let bridge = RosBridgeHandle::new(topic_sender, refresh_interval, initial_fetch);

        Self {
            should_quit: false,
            mode: AppMode::TopicList,
            topic_map: HashMap::new(),
            master_tree: TopicTree::new(),
            visible_items: Vec::new(),
            selected_index: 0,
            scroll_offset: 0,
            filter_text: String::new(),
            search_text: String::new(),
            selected_topic_name: None,
            detail_scroll_offset: 0,
            detail_focus: DetailPaneFocus::Info,
            echo_content: Vec::new(),
            is_echoing: false,
            echo_scroll_offset: 0,
            error_message: None,
            last_animation_update: std::time::Instant::now(),
            expansion_state: HashMap::new(),
            use_sim_time: false,
            view: None,
            graphics: TerminalGraphics::default(),
            kitty: KittyPlacement::default(),
            force_graphics_redraw: false,
            message_receiver: topic_receiver,
            bridge,
        }
    }

    pub fn handle_message(&mut self, msg: TopicMessage) {
        match msg {
            TopicMessage::TopicList(topics) => {
                self.error_message = None;
                let mut new_map = topics
                    .into_iter()
                    .map(|t| (t.name.clone(), t))
                    .collect::<HashMap<_, _>>();
                for (name, new_topic) in new_map.iter_mut() {
                    if let Some(existing) = self.topic_map.get(name) {
                        new_topic.watched = existing.watched;
                        new_topic.hz = existing.hz;
                        new_topic.hz_std_dev = existing.hz_std_dev;
                        new_topic.delay = existing.delay;
                        new_topic.delay_std_dev = existing.delay_std_dev;
                        new_topic.hz_status = existing.hz_status.clone();
                        new_topic.delay_status = existing.delay_status.clone();
                        new_topic.hz_history = existing.hz_history.clone();
                        new_topic.hz_std_dev_history = existing.hz_std_dev_history.clone();
                        new_topic.delay_history = existing.delay_history.clone();
                        new_topic.delay_std_dev_history = existing.delay_std_dev_history.clone();
                    }
                }
                // Keep watched topics a list misses, e.g. one taken before a
                // restarted helper discovered everything: they are still
                // being measured, and a vanished topic shows as 0 Hz.
                for (name, existing) in &self.topic_map {
                    if existing.watched && !new_map.contains_key(name) {
                        new_map.insert(name.clone(), existing.clone());
                    }
                }
                self.topic_map = new_map;
                let all_topics: Vec<ros::TopicInfo> = self.topic_map.values().cloned().collect();
                // Convert ros::TopicInfo to tree::TopicInfo
                let tree_topics: Vec<crate::tree::TopicInfo> = all_topics
                    .iter()
                    .map(|t| crate::tree::TopicInfo {
                        name: t.name.clone(),
                    })
                    .collect();
                self.master_tree.build_from_topics(&tree_topics);

                for (path, is_expanded) in &self.expansion_state {
                    if let Some(node) = self.master_tree.find_node_by_path(path) {
                        node.is_expanded = *is_expanded;
                    }
                }

                self.rebuild_visible_items();
            }
            TopicMessage::TopicHzUpdate {
                topic_name,
                hz,
                std_dev,
            } => {
                if let Some(topic) = self.watched_topic_mut(&topic_name) {
                    topic.hz = Some(hz);
                    topic.hz_std_dev = std_dev; // Store current std dev for display
                    topic.hz_status = MeasurementStatus::HasValue;
                    topic.hz_history.push_back(hz);
                    if topic.hz_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.hz_history.pop_front();
                    }

                    // Store std dev for Bollinger bands, use 0.0 if no std dev available
                    topic.hz_std_dev_history.push_back(std_dev.unwrap_or(0.0));
                    if topic.hz_std_dev_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.hz_std_dev_history.pop_front();
                    }
                }
            }
            TopicMessage::TopicDelayUpdate {
                topic_name,
                delay,
                std_dev,
            } => {
                if let Some(topic) = self.watched_topic_mut(&topic_name) {
                    topic.delay = Some(delay);
                    topic.delay_std_dev = std_dev; // Store current std dev for display
                    topic.delay_status = MeasurementStatus::HasValue;
                    topic.delay_history.push_back(delay * 1000.0); // Store in ms
                    if topic.delay_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.delay_history.pop_front();
                    }

                    // Store std dev for potential Bollinger bands, use 0.0 if no std dev available
                    topic
                        .delay_std_dev_history
                        .push_back(std_dev.map(|x| x * 1000.0).unwrap_or(0.0)); // Convert to ms
                    if topic.delay_std_dev_history.len() > CHARTS_MAX_DATA_POINTS {
                        topic.delay_std_dev_history.pop_front();
                    }
                }
            }
            TopicMessage::TopicHzStatusUpdate { topic_name, status } => {
                if let Some(topic) = self.watched_topic_mut(&topic_name) {
                    if !matches!(topic.hz_status, MeasurementStatus::HasValue) {
                        topic.hz_status = status;
                    }
                }
            }
            TopicMessage::TopicDelayStatusUpdate { topic_name, status } => {
                if let Some(topic) = self.watched_topic_mut(&topic_name) {
                    if !matches!(topic.delay_status, MeasurementStatus::HasValue) {
                        topic.delay_status = status;
                    }
                }
            }
            TopicMessage::TopicEcho(line) => {
                self.echo_content.push(line);
                const MAX_ECHO_LINES: usize = 1000; // Increased from 200 for better history

                // Remove old messages to prevent memory issues
                if self.echo_content.len() > MAX_ECHO_LINES {
                    // Remove multiple lines at once for efficiency
                    let remove_count = self.echo_content.len() - MAX_ECHO_LINES + 100; // Keep some buffer
                    self.echo_content.drain(0..remove_count);

                    // Adjust scroll offset if needed
                    if self.echo_scroll_offset >= remove_count {
                        self.echo_scroll_offset -= remove_count;
                    } else {
                        self.echo_scroll_offset = 0;
                    }
                }

                // Keep scroll at top by default - don't auto-scroll when new messages arrive
                // User can manually scroll down if needed
            }
            TopicMessage::ViewFrame(frame) => {
                if let Some(view) = self.view.as_mut() {
                    if view.topic_name == frame.topic_name && !view.paused {
                        view.on_frame(*frame);
                    }
                }
            }
            TopicMessage::ViewError {
                topic_name,
                message,
            } => {
                if let Some(view) = self.view.as_mut() {
                    if view.topic_name == topic_name {
                        view.error = Some(message);
                    }
                }
            }
            TopicMessage::Error(error) => {
                self.error_message = Some(error);
            }
        }
    }

    /// Look up a topic only if it is being watched. Metric updates can still
    /// arrive briefly after a topic is unwatched and must not resurrect values.
    fn watched_topic_mut(&mut self, topic_name: &str) -> Option<&mut ros::TopicInfo> {
        self.topic_map.get_mut(topic_name).filter(|t| t.watched)
    }

    pub fn try_receive_messages(&mut self) {
        while let Ok(msg) = self.message_receiver.try_recv() {
            self.handle_message(msg);
        }
    }

    pub fn rebuild_visible_items(&mut self) {
        // Remember the selected row by path so it stays put when rows are
        // added or removed above it (e.g. by a periodic topic list refresh).
        let selected_path = self
            .get_selected_item()
            .map(|item| item.node.full_path.clone());

        let mut target_tree;
        if self.filter_text.is_empty() {
            // No filter, use the master tree which respects manual expansion state
            target_tree = self.master_tree.clone();
        } else {
            // Filter is active, build a temporary tree from only matching topics
            let filtered_topics: Vec<ros::TopicInfo> = self
                .topic_map
                .values()
                .filter(|topic| {
                    topic
                        .name
                        .to_lowercase()
                        .contains(&self.filter_text.to_lowercase())
                })
                .cloned()
                .collect();
            target_tree = TopicTree::new();
            // Convert ros::TopicInfo to tree::TopicInfo
            let tree_topics: Vec<crate::tree::TopicInfo> = filtered_topics
                .iter()
                .map(|t| crate::tree::TopicInfo {
                    name: t.name.clone(),
                })
                .collect();
            target_tree.build_from_topics(&tree_topics);

            // Automatically expand all groups in the filtered results for visibility
            for group in target_tree.root.values_mut() {
                group.is_expanded = true;
            }
        }
        self.visible_items = target_tree.get_flattened_view();
        if let Some(path) = selected_path {
            if let Some(index) = self
                .visible_items
                .iter()
                .position(|item| item.node.full_path == path)
            {
                self.selected_index = index;
            }
        }
        self.clamp_selection();
    }

    pub fn on_key(&mut self, c: char) {
        if self.mode == AppMode::TopicView {
            match c {
                'q' => self.close_view(),
                'p' => {
                    if let Some(view) = self.view.as_mut() {
                        view.paused = !view.paused;
                    }
                }
                '+' | '=' => self.step_view_fps(1),
                '-' => self.step_view_fps(-1),
                c => self.on_view_key(c),
            }
        } else if self.mode == AppMode::Search {
            self.search_text.push(c);
        } else {
            match c {
                'q' => {
                    match self.mode {
                        AppMode::TopicDetail | AppMode::Help => {
                            self.mode = AppMode::TopicList;
                        }
                        AppMode::TopicList => {
                            self.should_quit = true;
                        }
                        _ => {} // Search mode is handled above
                    }
                }
                'd' => self.enter_detail_view(),
                'v' => self.open_view(),
                'r' => self.refresh(),
                's' => self.toggle_sim_time(),
                '?' => self.mode = AppMode::Help,
                _ => {}
            }
        }
    }

    pub fn on_backspace(&mut self) {
        if self.mode == AppMode::Search {
            self.search_text.pop();
        }
    }

    pub fn on_f4(&mut self) {
        self.mode = AppMode::Search;
        self.search_text = self.filter_text.clone();
    }

    pub fn on_enter(&mut self) {
        match self.mode {
            AppMode::Search => {
                self.filter_text = self.search_text.clone();
                self.mode = AppMode::TopicList;
                self.rebuild_visible_items();
            }
            AppMode::TopicList => {
                let selected_path = self
                    .get_selected_item()
                    .map(|item| item.node.full_path.clone());
                let is_group = self.get_selected_item().is_some_and(|item| item.is_group());

                if let Some(path) = selected_path {
                    if is_group {
                        self.toggle_group_expansion(&path);
                    } else {
                        self.toggle_watch_current_topic();
                    }
                }
            }
            AppMode::TopicDetail => {
                // Use the topic the detail view was opened for: the list
                // selection may have moved since, e.g. after a refresh.
                if let Some(topic_name) = self.selected_topic_name.clone() {
                    self.toggle_watch(&topic_name);
                }
            }
            _ => {}
        }
    }

    pub fn on_tab(&mut self) {
        if let Some(cloud) = self.cloud_view_mut() {
            let preset = cloud.camera.preset.next();
            cloud.reset_camera(preset);
            return;
        }
        if self.mode == AppMode::TopicDetail {
            self.detail_focus = match self.detail_focus {
                DetailPaneFocus::Info => DetailPaneFocus::Echo,
                DetailPaneFocus::Echo => DetailPaneFocus::Info,
            };
        }
    }

    pub fn on_space(&mut self) {
        if self.mode != AppMode::Search {
            self.refresh();
        }
    }

    /// Fetch the topic list now instead of waiting for the next periodic refresh.
    pub fn refresh(&mut self) {
        self.bridge.refresh();
    }

    pub fn on_left(&mut self) {
        if let Some(cloud) = self.cloud_view_mut() {
            cloud.camera.orbit(-ORBIT_STEP, 0.0);
            return;
        }
        if self.mode == AppMode::TopicDetail {
            // No action for left/right in detail view with combined panes
        } else if let Some(item) = self.get_selected_item() {
            if item.is_group() && item.node.is_expanded {
                let path_to_toggle = item.node.full_path.clone();
                self.toggle_group_expansion(&path_to_toggle);
            }
        }
    }

    pub fn on_right(&mut self) {
        if let Some(cloud) = self.cloud_view_mut() {
            cloud.camera.orbit(ORBIT_STEP, 0.0);
            return;
        }
        if self.mode == AppMode::TopicDetail {
            // No action for left/right in detail view with combined panes
        } else if let Some(item) = self.get_selected_item() {
            if item.is_group() && !item.node.is_expanded {
                let path_to_toggle = item.node.full_path.clone();
                self.toggle_group_expansion(&path_to_toggle);
            }
        }
    }

    pub fn on_escape(&mut self) {
        match self.mode {
            AppMode::TopicView => self.close_view(),
            AppMode::Search | AppMode::Help | AppMode::TopicDetail => {
                if self.is_echoing {
                    self.is_echoing = false;
                    let _ = self.bridge.send(WatchMessage::StopEcho);
                }
                self.mode = AppMode::TopicList;
            }
            AppMode::TopicList => {
                if !self.filter_text.is_empty() {
                    self.filter_text.clear();
                    self.search_text.clear();
                    self.rebuild_visible_items();
                } else {
                    self.should_quit = true;
                }
            }
        }
    }

    pub fn is_in_search_mode(&self) -> bool {
        matches!(self.mode, AppMode::Search)
    }

    fn enter_detail_view(&mut self) {
        if let Some(item) = self.get_selected_item() {
            if item.is_topic() {
                self.selected_topic_name = Some(item.node.full_path.clone());
                self.detail_focus = DetailPaneFocus::Info;
                self.detail_scroll_offset = 0;
                self.echo_content.clear();
                self.is_echoing = false;
                self.echo_scroll_offset = 0;
                self.mode = AppMode::TopicDetail;
            }
        }
    }

    /// Open the image viewer on the selected topic, from the topic list or
    /// the detail view.
    pub fn open_view(&mut self) {
        let topic_name = match self.mode {
            AppMode::TopicList => self
                .get_selected_item()
                .and_then(|item| item.get_topic_info())
                .map(|topic| topic.name.clone()),
            AppMode::TopicDetail => self.selected_topic_name.clone(),
            _ => None,
        };
        let Some(topic) = topic_name.and_then(|name| self.topic_map.get(&name)) else {
            return;
        };
        if !has_viewer(&topic.msg_type) {
            self.error_message = Some(format!(
                "No viewer for {} (supported: images, point clouds, laser scans, diagnostics)",
                topic.msg_type
            ));
            return;
        }

        let target = Arc::new(ViewTarget::default());
        let view = TopicView {
            topic_name: topic.name.clone(),
            msg_type: topic.msg_type.clone(),
            content: view_content(&topic.msg_type),
            frame_seq: 0,
            error: None,
            max_fps: if diagnostics::is_diagnostics_type(&topic.msg_type) {
                UNTHROTTLED_FPS
            } else if imu::is_imu_type(&topic.msg_type) {
                DEFAULT_IMU_FPS
            } else {
                DEFAULT_VIEW_FPS
            },
            paused: false,
            topic_hz: None,
            display_fps: None,
            last_frame_at: None,
            placement: Cell::new(None),
            target: target.clone(),
            rate_samples: VecDeque::new(),
            return_mode: std::mem::replace(&mut self.mode, AppMode::TopicView),
        };
        let _ = self.bridge.send(WatchMessage::StartViewStream {
            topic_name: view.topic_name.clone(),
            msg_type: view.msg_type.clone(),
            max_fps: view.max_fps,
            target,
        });
        self.view = Some(view);
    }

    fn close_view(&mut self) {
        if let Some(view) = self.view.take() {
            let _ = self.bridge.send(WatchMessage::StopViewStream);
            self.mode = view.return_mode;
        }
    }

    fn cloud_view(&self) -> Option<&CloudView> {
        if self.mode != AppMode::TopicView {
            return None;
        }
        match self.view.as_ref().map(|v| &v.content) {
            Some(ViewContent::Cloud(cloud)) => Some(cloud),
            _ => None,
        }
    }

    /// Whether the main loop should capture the mouse: only for wheel zoom in
    /// the point cloud view, so text selection works everywhere else.
    pub fn wants_mouse(&self) -> bool {
        self.cloud_view().is_some()
    }

    /// Scroll wheel: zoom the point cloud view.
    pub fn on_scroll(&mut self, up: bool) {
        if let Some(cloud) = self.cloud_view_mut() {
            let factor = if up {
                1.0 / WHEEL_ZOOM_STEP
            } else {
                WHEEL_ZOOM_STEP
            };
            cloud.camera.zoom(factor);
        }
    }

    fn cloud_view_mut(&mut self) -> Option<&mut CloudView> {
        if self.mode != AppMode::TopicView {
            return None;
        }
        match self.view.as_mut().map(|v| &mut v.content) {
            Some(ViewContent::Cloud(cloud)) => Some(&mut **cloud),
            _ => None,
        }
    }

    fn diagnostics_view_mut(&mut self) -> Option<&mut DiagnosticsView> {
        if self.mode != AppMode::TopicView {
            return None;
        }
        match self.view.as_mut().map(|v| &mut v.content) {
            Some(ViewContent::Diagnostics(diag)) => Some(diag),
            _ => None,
        }
    }

    /// Point cloud keys: pan, color, grid and camera reset. Diagnostics:
    /// `f` toggles showing only problems.
    fn on_view_key(&mut self, c: char) {
        if let Some(diag) = self.diagnostics_view_mut() {
            if c == 'f' {
                diag.problems_only = !diag.problems_only;
                diag.selected = 0;
            }
            return;
        }
        let Some(cloud) = self.cloud_view_mut() else {
            return;
        };
        match c {
            'w' => cloud.camera.pan(1.0, 0.0),
            's' => cloud.camera.pan(-1.0, 0.0),
            'a' => cloud.camera.pan(0.0, -1.0),
            'd' => cloud.camera.pan(0.0, 1.0),
            'c' => cloud.cycle_color(),
            'g' => cloud.grid = !cloud.grid,
            'r' => cloud.reset_camera(cloud.camera.preset),
            _ => {}
        }
    }

    fn step_view_fps(&mut self, direction: isize) {
        let Some(view) = self.view.as_mut() else {
            return;
        };
        if matches!(view.content, ViewContent::Diagnostics(_)) {
            return;
        }
        let current = VIEW_FPS_STEPS
            .iter()
            .position(|&fps| fps >= view.max_fps)
            .unwrap_or(VIEW_FPS_STEPS.len() - 1);
        let next = current
            .saturating_add_signed(direction)
            .min(VIEW_FPS_STEPS.len() - 1);
        view.max_fps = VIEW_FPS_STEPS[next];
        let _ = self.bridge.send(WatchMessage::SetViewFps(view.max_fps));
    }

    /// Main loop polling interval: short while images stream, so frames are
    /// not held back waiting for input.
    pub fn poll_interval(&self) -> Duration {
        if self.mode == AppMode::TopicView {
            Duration::from_millis(10)
        } else {
            Duration::from_millis(50)
        }
    }

    pub fn on_resize(&mut self) {
        self.force_graphics_redraw = true;
    }

    /// Bytes per second of image data sent to the terminal.
    pub fn graphics_bytes_per_second(&self) -> usize {
        self.kitty.bytes_per_second()
    }

    /// Write kitty graphics after a draw: show the current frame where the
    /// UI laid it out, or remove the image once the viewer is closed.
    pub fn sync_graphics(&mut self, out: &mut impl Write) -> io::Result<()> {
        if self.graphics.protocol != GraphicsProtocol::Kitty {
            return Ok(());
        }
        let force = std::mem::take(&mut self.force_graphics_redraw);
        let view = self
            .view
            .as_ref()
            .filter(|_| self.mode == AppMode::TopicView);
        let Some((view, rect)) = view.and_then(|v| Some((v, v.placement.get()?))) else {
            return self.kitty.clear(out);
        };
        match &view.content {
            ViewContent::Image(Some(frame)) => {
                self.kitty
                    .show(out, &frame.rgb, view.frame_seq, rect, force)
            }
            ViewContent::Cloud(cloud) => match cloud.rendered.borrow().as_ref() {
                Some(rendered) => self
                    .kitty
                    .show(out, &rendered.image, rendered.seq, rect, force),
                None => self.kitty.clear(out),
            },
            ViewContent::Image(None)
            | ViewContent::Diagnostics(_)
            | ViewContent::Gnss(_)
            | ViewContent::Imu(_) => self.kitty.clear(out),
        }
    }

    /// Remove any image from the terminal, e.g. before quitting.
    pub fn clear_graphics(&mut self, out: &mut impl Write) -> io::Result<()> {
        self.kitty.clear(out)
    }

    fn toggle_group_expansion(&mut self, group_path: &str) {
        if let Some(node) = self.master_tree.find_node_by_path(group_path) {
            node.toggle_expanded();
            self.expansion_state
                .insert(group_path.to_string(), node.is_expanded);
            self.rebuild_visible_items();
        }
    }

    pub fn update_loading_animation(&mut self) {
        if self.last_animation_update.elapsed() < Duration::from_millis(150) {
            return;
        }
        for topic in self.topic_map.values_mut() {
            if topic.watched {
                if let MeasurementStatus::Loading(frame) = &mut topic.hz_status {
                    *frame = (*frame + 1) % 3;
                }
                if let MeasurementStatus::Loading(frame) = &mut topic.delay_status {
                    *frame = (*frame + 1) % 3;
                }
            }
        }
        self.last_animation_update = std::time::Instant::now();
    }

    pub fn toggle_echo(&mut self) {
        if self.mode == AppMode::TopicDetail {
            if let Some(topic_name) = &self.selected_topic_name {
                self.is_echoing = !self.is_echoing;
                if self.is_echoing {
                    self.echo_content.clear();
                    self.echo_scroll_offset = 0;
                    let _ = self
                        .bridge
                        .send(WatchMessage::StartEcho(topic_name.clone()));
                } else {
                    let _ = self.bridge.send(WatchMessage::StopEcho);
                }
            }
        }
    }

    pub fn on_up(&mut self) {
        if self.mode == AppMode::TopicView {
            if let Some(cloud) = self.cloud_view_mut() {
                cloud.camera.orbit(0.0, ORBIT_STEP);
            } else if let Some(diag) = self.diagnostics_view_mut() {
                diag.selected = diag.selected.saturating_sub(1);
            }
            return;
        }
        if self.mode == AppMode::TopicDetail {
            if self.detail_focus == DetailPaneFocus::Echo {
                self.echo_scroll_offset = self.echo_scroll_offset.saturating_sub(1);
            }
        } else {
            self.select_previous();
        }
    }

    pub fn on_down(&mut self) {
        if self.mode == AppMode::TopicView {
            if let Some(cloud) = self.cloud_view_mut() {
                cloud.camera.orbit(0.0, -ORBIT_STEP);
            } else if let Some(diag) = self.diagnostics_view_mut() {
                diag.selected += 1;
            }
            return;
        }
        if self.mode == AppMode::TopicDetail {
            if self.detail_focus == DetailPaneFocus::Echo {
                // Constrain scroll to prevent excess whitespace
                // Assume a reasonable viewport height (will be properly calculated in UI)
                let max_scroll = self.echo_content.len().saturating_sub(20); // Rough estimate
                self.echo_scroll_offset = (self.echo_scroll_offset + 1).min(max_scroll);
            }
        } else {
            self.select_next();
        }
    }

    pub fn echo_page_up(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            let page_size = 10; // Scroll 10 lines at a time
            self.echo_scroll_offset = self.echo_scroll_offset.saturating_sub(page_size);
        }
    }

    pub fn echo_page_down(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            let page_size = 10; // Scroll 10 lines at a time
            self.echo_scroll_offset = (self.echo_scroll_offset + page_size)
                .min(self.echo_content.len().saturating_sub(1));
        }
    }

    pub fn echo_home(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            self.echo_scroll_offset = 0;
        }
    }

    pub fn echo_end(&mut self) {
        if self.mode == AppMode::TopicDetail && self.detail_focus == DetailPaneFocus::Echo {
            self.echo_scroll_offset = self.echo_content.len().saturating_sub(1);
        }
    }

    pub fn shutdown(&mut self) {
        self.bridge.shutdown();
    }

    fn get_selected_item(&self) -> Option<&TopicTreeItem> {
        self.visible_items.get(self.selected_index)
    }

    fn clamp_selection(&mut self) {
        if self.visible_items.is_empty() {
            self.selected_index = 0;
        } else if self.selected_index >= self.visible_items.len() {
            self.selected_index = self.visible_items.len() - 1;
        }
        self.adjust_scroll();
    }

    fn adjust_scroll(&mut self) {
        const VISIBLE_HEIGHT: usize = 20; // Approximate visible height

        if self.selected_index < self.scroll_offset {
            self.scroll_offset = self.selected_index;
        } else if self.selected_index >= self.scroll_offset + VISIBLE_HEIGHT {
            self.scroll_offset = self.selected_index - VISIBLE_HEIGHT + 1;
        }
    }

    pub fn select_next(&mut self) {
        if !self.visible_items.is_empty() {
            self.selected_index = (self.selected_index + 1) % self.visible_items.len();
            self.adjust_scroll();
        }
    }

    pub fn select_previous(&mut self) {
        if !self.visible_items.is_empty() {
            self.selected_index =
                (self.selected_index + self.visible_items.len() - 1) % self.visible_items.len();
            self.adjust_scroll();
        }
    }

    pub fn toggle_watch_current_topic(&mut self) {
        let topic_name = self
            .get_selected_item()
            .and_then(|item| item.get_topic_info())
            .map(|topic_info| topic_info.name.clone());
        if let Some(topic_name) = topic_name {
            self.toggle_watch(&topic_name);
        }
    }

    fn toggle_watch(&mut self, topic_name: &str) {
        if let Some(topic) = self.topic_map.get_mut(topic_name) {
            topic.watched = !topic.watched;
            if topic.watched {
                topic.hz_status = MeasurementStatus::Loading(0);
                topic.delay_status = MeasurementStatus::Loading(0);
                topic.hz_history.clear();
                topic.hz_std_dev_history.clear();
                topic.delay_history.clear();
                topic.delay_std_dev_history.clear();
                let _ = self
                    .bridge
                    .send(WatchMessage::StartContinuousMetrics(topic.name.clone()));
            } else {
                topic.hz = None;
                topic.hz_std_dev = None;
                topic.delay = None;
                topic.delay_std_dev = None;
                topic.hz_status = MeasurementStatus::NotMeasuring;
                topic.delay_status = MeasurementStatus::NotMeasuring;
                let _ = self
                    .bridge
                    .send(WatchMessage::StopContinuousMetrics(topic.name.clone()));
            }
        }
    }

    fn toggle_sim_time(&mut self) {
        match self.mode {
            AppMode::TopicList | AppMode::TopicDetail => {}
            _ => return,
        }

        self.use_sim_time = !self.use_sim_time;

        for topic in self.topic_map.values_mut().filter(|t| t.watched) {
            topic.delay_status = MeasurementStatus::Loading(0);
            topic.delay = None;
            topic.delay_std_dev = None;
            topic.delay_history.clear();
            topic.delay_std_dev_history.clear();
        }

        let _ = self
            .bridge
            .send(WatchMessage::SetUseSimTime(self.use_sim_time));
    }

    pub fn toggle_collapse_all(&mut self) {
        // Check if any groups are currently expanded
        let any_expanded = self.check_any_expanded(&self.master_tree.root);

        if any_expanded {
            self.collapse_all();
        } else {
            self.expand_all();
        }

        // Rebuild the visible items after changing expansion states
        self.rebuild_visible_items();
    }

    #[allow(clippy::only_used_in_recursion)]
    fn check_any_expanded(
        &self,
        nodes: &std::collections::HashMap<String, crate::tree::TopicTreeNode>,
    ) -> bool {
        for node in nodes.values() {
            if !node.is_leaf && node.is_expanded {
                return true;
            }
            if self.check_any_expanded(&node.children) {
                return true;
            }
        }
        false
    }

    fn collapse_all(&mut self) {
        Self::set_all_expanded_static(&mut self.master_tree.root, false, &mut self.expansion_state);
    }

    fn expand_all(&mut self) {
        Self::set_all_expanded_static(&mut self.master_tree.root, true, &mut self.expansion_state);
    }

    /// Set every group's expansion and record it for every group, not just the
    /// ones toggled before, so the next topic list refresh preserves it.
    fn set_all_expanded_static(
        nodes: &mut std::collections::HashMap<String, crate::tree::TopicTreeNode>,
        expanded: bool,
        expansion_state: &mut HashMap<String, bool>,
    ) {
        for node in nodes.values_mut() {
            if !node.is_leaf {
                node.is_expanded = expanded;
                expansion_state.insert(node.full_path.clone(), expanded);
                Self::set_all_expanded_static(&mut node.children, expanded, expansion_state);
            }
        }
    }
}
