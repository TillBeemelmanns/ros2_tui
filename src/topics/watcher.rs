use super::diagnostics::{self, DiagnosticStatus};
use super::image::{self, DecodedImage};
use super::imu::{self, ImuSample};
use super::navsat::{self, NavSatFix};
use super::occupancy;
use super::pointcloud::{self, PointCloud};
use super::ros::{self, MeasurementStatus, TopicInfo};
use crossbeam::channel::{Receiver, Sender};
use once_cell::sync::Lazy;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::{thread, time::Duration};
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader, Lines};
use tokio::process::{Child, ChildStderr, ChildStdout};
use tokio::sync::watch;
use tokio::task::JoinHandle;

static SHARED_RUNTIME: Lazy<Arc<tokio::runtime::Runtime>> =
    Lazy::new(|| Arc::new(tokio::runtime::Runtime::new().expect("Failed to create tokio runtime")));

/// Delay before restarting a `ros2 topic hz`/`delay` process that exited.
const STREAM_RESTART_DELAY: Duration = Duration::from_millis(2000);

pub enum TopicMessage {
    TopicList(Vec<TopicInfo>),
    TopicHzUpdate {
        topic_name: String,
        hz: f64,
        std_dev: Option<f64>,
    },
    TopicHzStdDevUpdate {
        topic_name: String,
        std_dev: f64,
    },
    TopicDelayUpdate {
        topic_name: String,
        delay: f64,
        std_dev: Option<f64>,
    },
    TopicDelayStdDevUpdate {
        topic_name: String,
        std_dev: f64,
    },
    TopicHzStatusUpdate {
        topic_name: String,
        status: MeasurementStatus,
    },
    TopicDelayStatusUpdate {
        topic_name: String,
        status: MeasurementStatus,
    },
    TopicEcho(String),
    ViewFrame(Box<ViewFrame>),
    ViewError {
        topic_name: String,
        message: String,
    },
    Error(String),
}

/// Most points of a cloud decoded for display; larger clouds are sampled.
const MAX_CLOUD_POINTS: usize = 200_000;

pub enum FrameContent {
    Image(DecodedImage),
    Cloud(PointCloud),
    Diagnostics(Vec<DiagnosticStatus>),
    Gnss(NavSatFix),
    Imu(ImuSample),
}

/// Decode a serialized message of `msg_type` for display.
fn decode_frame(
    msg_type: &str,
    payload: &[u8],
    target: (u32, u32),
) -> Result<FrameContent, String> {
    if pointcloud::is_pointcloud_type(msg_type) {
        pointcloud::decode_message(payload, MAX_CLOUD_POINTS)
            .map(FrameContent::Cloud)
            .map_err(|e| e.to_string())
    } else if occupancy::is_occupancy_grid_type(msg_type) {
        occupancy::decode_message(payload, target.0, target.1).map(FrameContent::Image)
    } else if navsat::is_navsatfix_type(msg_type) {
        navsat::decode_message(payload)
            .map(FrameContent::Gnss)
            .map_err(|e| e.to_string())
    } else if imu::is_imu_type(msg_type) {
        imu::decode_message(payload)
            .map(FrameContent::Imu)
            .map_err(|e| e.to_string())
    } else if diagnostics::is_diagnostics_type(msg_type) {
        diagnostics::decode_message(payload)
            .map(FrameContent::Diagnostics)
            .map_err(|e| e.to_string())
    } else if pointcloud::is_laserscan_type(msg_type) {
        pointcloud::decode_laserscan(payload)
            .map(FrameContent::Cloud)
            .map_err(|e| e.to_string())
    } else {
        image::decode_message(msg_type, payload, target.0, target.1)
            .map(FrameContent::Image)
            .map_err(|e| e.to_string())
    }
}

pub struct ViewFrame {
    pub topic_name: String,
    pub content: FrameContent,
    /// Messages received on the topic so far, including throttled ones.
    pub received_count: u32,
}

/// Pixel size images are decoded to, shared with the UI, which knows the
/// display area. Packed into one atomic as width << 32 | height.
#[derive(Debug, Default)]
pub struct ViewTarget(AtomicU64);

impl ViewTarget {
    pub fn set(&self, width: u32, height: u32) {
        self.0
            .store(((width as u64) << 32) | height as u64, Ordering::Relaxed);
    }

    pub fn get(&self) -> (u32, u32) {
        let packed = self.0.load(Ordering::Relaxed);
        match ((packed >> 32) as u32, packed as u32) {
            (0, _) | (_, 0) => (640, 480),
            size => size,
        }
    }
}

pub enum WatchMessage {
    StartContinuousMetrics(String),
    StopContinuousMetrics(String),
    StartEcho(String),
    StopEcho,
    StartViewStream {
        topic_name: String,
        msg_type: String,
        max_fps: u32,
        target: Arc<ViewTarget>,
    },
    StopViewStream,
    SetViewFps(u32),
    SetUseSimTime(bool),
}

pub struct TopicWatcherHandle {
    _handle: thread::JoinHandle<()>,
    shutdown_sender: Sender<()>,
    refresh_sender: Sender<()>,
}

impl TopicWatcherHandle {
    pub fn new(
        sender: Sender<TopicMessage>,
        refresh_interval: Duration,
        initial_fetch: bool,
    ) -> Self {
        let (shutdown_sender, shutdown_receiver) = crossbeam::channel::bounded::<()>(1);
        // Capacity 1: repeated refresh requests while a fetch is running collapse
        // into a single follow-up fetch.
        let (refresh_sender, refresh_receiver) = crossbeam::channel::bounded::<()>(1);
        let handle = thread::spawn(move || {
            let mut topic_watcher = TopicWatcher::new(sender, refresh_interval);
            topic_watcher.run(shutdown_receiver, refresh_receiver, initial_fetch);
        });

        Self {
            _handle: handle,
            shutdown_sender,
            refresh_sender,
        }
    }

    /// Request an immediate topic list refresh.
    pub fn refresh(&self) {
        let _ = self.refresh_sender.try_send(());
    }

    pub fn shutdown(&self) {
        let _ = self.shutdown_sender.try_send(());
    }
}

struct TopicWatcher {
    sender: Sender<TopicMessage>,
    refresh_interval: Duration,
}

impl TopicWatcher {
    fn new(sender: Sender<TopicMessage>, refresh_interval: Duration) -> Self {
        Self {
            sender,
            refresh_interval,
        }
    }

    fn run(
        &mut self,
        shutdown_receiver: Receiver<()>,
        refresh_receiver: Receiver<()>,
        initial_fetch: bool,
    ) {
        crate::debug_log("Topic watcher thread started");

        if initial_fetch {
            self.fetch_and_send_topics();
        }

        loop {
            crossbeam::select! {
                recv(shutdown_receiver) -> _ => {
                    crate::debug_log("Topic watcher received shutdown signal");
                    break;
                }
                recv(refresh_receiver) -> _ => {
                    crate::debug_log("Topic watcher received manual refresh request");
                    self.fetch_and_send_topics();
                }
                default(self.refresh_interval) => {
                    self.fetch_and_send_topics();
                }
            }
        }
        crate::debug_log("Topic watcher thread exiting");
    }

    fn fetch_and_send_topics(&self) {
        let msg = match SHARED_RUNTIME.block_on(ros::get_topic_list()) {
            Ok(topics) => TopicMessage::TopicList(topics),
            Err(e) => {
                let error_msg = format!("Failed to fetch topics: {}", e);
                crate::debug_log(&error_msg);
                TopicMessage::Error(error_msg)
            }
        };
        // A send error only means the UI thread has exited.
        let _ = self.sender.send(msg);
    }
}

pub struct TopicDetailWatcherHandle {
    handle: Option<thread::JoinHandle<()>>,
    cleanup_sender: Sender<()>,
}

impl TopicDetailWatcherHandle {
    pub fn new(msg_sender: Sender<TopicMessage>) -> (Self, Sender<WatchMessage>) {
        let (watch_sender, watch_receiver) = crossbeam::channel::unbounded::<WatchMessage>();
        let (cleanup_sender, cleanup_receiver) = crossbeam::channel::bounded::<()>(1);

        let handle = thread::spawn(move || {
            let mut detail_watcher = TopicDetailWatcher::new(msg_sender);
            detail_watcher.run(watch_receiver, cleanup_receiver);
        });

        (
            Self {
                handle: Some(handle),
                cleanup_sender,
            },
            watch_sender,
        )
    }

    /// Stop all monitoring and wait until every spawned `ros2` process has been
    /// killed, so none outlive the application.
    pub fn shutdown(&mut self) {
        let _ = self.cleanup_sender.try_send(());
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

struct MonitoringHandles {
    hz: JoinHandle<()>,
    delay: JoinHandle<()>,
}

/// The two continuously streamed metrics of a watched topic.
#[derive(Clone, Copy)]
enum Metric {
    Hz,
    Delay { use_sim_time: bool },
}

impl Metric {
    async fn start(self, topic_name: &str) -> Result<Child, ros::TopicError> {
        match self {
            Metric::Hz => ros::start_topic_hz_stream(topic_name).await,
            Metric::Delay { use_sim_time } => {
                ros::start_topic_delay_stream(topic_name, use_sim_time).await
            }
        }
    }

    fn parse(self, line: &str) -> (Option<f64>, Option<f64>, MeasurementStatus) {
        match self {
            Metric::Hz => ros::parse_topic_hz(line),
            Metric::Delay { .. } => ros::parse_topic_delay(line),
        }
    }

    fn value_message(self, topic_name: String, value: f64, std_dev: Option<f64>) -> TopicMessage {
        match self {
            Metric::Hz => TopicMessage::TopicHzUpdate {
                topic_name,
                hz: value,
                std_dev,
            },
            Metric::Delay { .. } => TopicMessage::TopicDelayUpdate {
                topic_name,
                delay: value,
                std_dev,
            },
        }
    }

    fn std_dev_message(self, topic_name: String, std_dev: f64) -> TopicMessage {
        match self {
            Metric::Hz => TopicMessage::TopicHzStdDevUpdate {
                topic_name,
                std_dev,
            },
            Metric::Delay { .. } => TopicMessage::TopicDelayStdDevUpdate {
                topic_name,
                std_dev,
            },
        }
    }

    fn status_message(self, topic_name: String, status: MeasurementStatus) -> TopicMessage {
        match self {
            Metric::Hz => TopicMessage::TopicHzStatusUpdate { topic_name, status },
            Metric::Delay { .. } => TopicMessage::TopicDelayStatusUpdate { topic_name, status },
        }
    }
}

/// Line reader over a child's stdout and stderr, yielding lines from whichever
/// stream produces them first until both are closed.
struct OutputLines {
    stdout: Option<Lines<BufReader<ChildStdout>>>,
    stderr: Option<Lines<BufReader<ChildStderr>>>,
}

impl OutputLines {
    fn new(child: &mut Child) -> Self {
        Self {
            stdout: child.stdout.take().map(|s| BufReader::new(s).lines()),
            stderr: child.stderr.take().map(|s| BufReader::new(s).lines()),
        }
    }

    async fn next_line(&mut self) -> Option<String> {
        loop {
            let (from_stdout, line) = match (&mut self.stdout, &mut self.stderr) {
                (None, None) => return None,
                (Some(out), None) => (true, out.next_line().await),
                (None, Some(err)) => (false, err.next_line().await),
                // `next_line` is cancel-safe, so racing both streams loses no data.
                (Some(out), Some(err)) => tokio::select! {
                    line = out.next_line() => (true, line),
                    line = err.next_line() => (false, line),
                },
            };
            match line {
                Ok(Some(line)) => return Some(line),
                _ if from_stdout => self.stdout = None,
                _ => self.stderr = None,
            }
        }
    }
}

struct TopicDetailWatcher {
    sender: Sender<TopicMessage>,
    active_tasks: HashMap<String, MonitoringHandles>,
    echo_task: Option<JoinHandle<()>>,
    view_task: Option<(JoinHandle<()>, watch::Sender<u32>)>,
    use_sim_time: bool,
}

impl TopicDetailWatcher {
    fn new(sender: Sender<TopicMessage>) -> Self {
        Self {
            sender,
            active_tasks: HashMap::new(),
            echo_task: None,
            view_task: None,
            use_sim_time: false,
        }
    }

    fn run(&mut self, watch_receiver: Receiver<WatchMessage>, cleanup_receiver: Receiver<()>) {
        crate::debug_log("Detail watcher thread started");
        loop {
            crossbeam::select! {
                recv(cleanup_receiver) -> _ => break,
                recv(watch_receiver) -> msg => match msg {
                    Ok(msg) => self.handle_watch_message(msg),
                    Err(_) => break,
                },
            }
        }
        self.stop_all();
        crate::debug_log("TopicDetailWatcher thread exiting");
    }

    fn handle_watch_message(&mut self, watch_msg: WatchMessage) {
        match watch_msg {
            WatchMessage::StartContinuousMetrics(topic_name) => {
                // Always (re)start: a previous run may have ended on its own,
                // e.g. because the topic stopped being published.
                self.stop_monitoring(&topic_name);
                self.start_continuous_monitoring(topic_name);
            }
            WatchMessage::StopContinuousMetrics(topic_name) => {
                self.stop_monitoring(&topic_name);
            }
            WatchMessage::StartEcho(topic_name) => {
                if let Some(task) = self.echo_task.take() {
                    task.abort();
                }
                self.echo_task = Some(
                    SHARED_RUNTIME.spawn(Self::monitor_topic_echo(topic_name, self.sender.clone())),
                );
            }
            WatchMessage::StopEcho => {
                if let Some(task) = self.echo_task.take() {
                    task.abort();
                }
            }
            WatchMessage::StartViewStream {
                topic_name,
                msg_type,
                max_fps,
                target,
            } => {
                self.stop_view_stream();
                let (fps_sender, fps_receiver) = watch::channel(max_fps);
                let task = SHARED_RUNTIME.spawn(Self::monitor_view_stream(
                    topic_name,
                    msg_type,
                    target,
                    fps_receiver,
                    self.sender.clone(),
                ));
                self.view_task = Some((task, fps_sender));
            }
            WatchMessage::StopViewStream => self.stop_view_stream(),
            WatchMessage::SetViewFps(max_fps) => {
                if let Some((_, fps_sender)) = &self.view_task {
                    let _ = fps_sender.send(max_fps);
                }
            }
            WatchMessage::SetUseSimTime(enabled) => {
                if self.use_sim_time != enabled {
                    self.use_sim_time = enabled;
                    let metric = Metric::Delay {
                        use_sim_time: enabled,
                    };
                    for (topic, handles) in self.active_tasks.iter_mut() {
                        handles.delay.abort();
                        handles.delay = SHARED_RUNTIME.spawn(Self::monitor_metric(
                            metric,
                            topic.clone(),
                            self.sender.clone(),
                        ));
                    }
                }
            }
        }
    }

    fn start_continuous_monitoring(&mut self, topic_name: String) {
        let hz = SHARED_RUNTIME.spawn(Self::monitor_metric(
            Metric::Hz,
            topic_name.clone(),
            self.sender.clone(),
        ));
        let delay = SHARED_RUNTIME.spawn(Self::monitor_metric(
            Metric::Delay {
                use_sim_time: self.use_sim_time,
            },
            topic_name.clone(),
            self.sender.clone(),
        ));
        self.active_tasks
            .insert(topic_name, MonitoringHandles { hz, delay });
    }

    fn stop_monitoring(&mut self, topic_name: &str) {
        if let Some(handles) = self.active_tasks.remove(topic_name) {
            handles.hz.abort();
            handles.delay.abort();
        }
    }

    fn stop_view_stream(&mut self) {
        if let Some((task, _)) = self.view_task.take() {
            task.abort();
        }
    }

    /// Abort every task and wait for the aborts to complete. Dropping a task's
    /// future drops its `Child`, which (via `kill_on_drop`) kills the process.
    fn stop_all(&mut self) {
        let mut tasks: Vec<JoinHandle<()>> = self
            .active_tasks
            .drain()
            .flat_map(|(_, handles)| [handles.hz, handles.delay])
            .collect();
        tasks.extend(self.echo_task.take());
        tasks.extend(self.view_task.take().map(|(task, _)| task));
        for task in &tasks {
            task.abort();
        }
        SHARED_RUNTIME.block_on(async {
            for task in tasks {
                let _ = task.await;
            }
        });
    }

    async fn monitor_topic_echo(topic_name: String, sender: Sender<TopicMessage>) {
        if let Ok(mut child) = ros::start_topic_echo_stream(&topic_name).await {
            let mut lines = OutputLines::new(&mut child);
            while let Some(line) = lines.next_line().await {
                let _ = sender.send(TopicMessage::TopicEcho(line));
            }
            let _ = child.kill().await;
        }
    }

    /// Stream decoded images of `topic_name` until the helper exits or the
    /// task is aborted, which kills the helper.
    async fn monitor_view_stream(
        topic_name: String,
        msg_type: String,
        target: Arc<ViewTarget>,
        mut fps: watch::Receiver<u32>,
        sender: Sender<TopicMessage>,
    ) {
        let send_error = |message: String| {
            let _ = sender.send(TopicMessage::ViewError {
                topic_name: topic_name.clone(),
                message,
            });
        };
        let max_fps = *fps.borrow_and_update();
        let mut child = match ros::start_message_stream(&topic_name, &msg_type, max_fps).await {
            Ok(child) => child,
            Err(e) => return send_error(format!("cannot start python3: {}", e)),
        };
        let (Some(mut stdout), Some(mut stdin), Some(stderr)) =
            (child.stdout.take(), child.stdin.take(), child.stderr.take())
        else {
            return;
        };

        // Both end on their own once the helper is gone.
        let fps_task = tokio::spawn(async move {
            while fps.changed().await.is_ok() {
                let line = format!("{}\n", *fps.borrow_and_update());
                if stdin.write_all(line.as_bytes()).await.is_err() || stdin.flush().await.is_err() {
                    break;
                }
            }
        });
        let stderr_task = tokio::spawn(async move {
            let mut text = String::new();
            let _ = BufReader::new(stderr).read_to_string(&mut text).await;
            text
        });

        loop {
            let mut header = [0u8; 8];
            if stdout.read_exact(&mut header).await.is_err() {
                break;
            }
            let len = u32::from_le_bytes(header[..4].try_into().unwrap()) as usize;
            let received_count = u32::from_le_bytes(header[4..].try_into().unwrap());
            let mut payload = vec![0u8; len];
            if stdout.read_exact(&mut payload).await.is_err() {
                break;
            }

            let size = target.get();
            let decode_type = msg_type.clone();
            let decoded =
                tokio::task::spawn_blocking(move || decode_frame(&decode_type, &payload, size))
                    .await;
            let msg = match decoded {
                Ok(Ok(content)) => TopicMessage::ViewFrame(Box::new(ViewFrame {
                    topic_name: topic_name.clone(),
                    content,
                    received_count,
                })),
                Ok(Err(message)) => TopicMessage::ViewError {
                    topic_name: topic_name.clone(),
                    message,
                },
                Err(_) => break,
            };
            if sender.send(msg).is_err() {
                break;
            }
        }

        fps_task.abort();
        let _ = child.kill().await;
        let stderr = stderr_task.await.unwrap_or_default();
        let reason = stderr
            .lines()
            .rev()
            .find(|line| !line.trim().is_empty())
            .unwrap_or("the image helper exited");
        send_error(format!("image stream ended: {}", reason.trim()));
    }

    /// Stream one metric for `topic_name`, restarting the CLI process if it
    /// exits, until the topic is reported as no longer published or (for
    /// delay) as having no header stamp. Both are terminal: restarting would
    /// only respawn a Python process every few seconds to learn the same thing.
    async fn monitor_metric(metric: Metric, topic_name: String, sender: Sender<TopicMessage>) {
        loop {
            if let Ok(mut child) = metric.start(&topic_name).await {
                let mut lines = OutputLines::new(&mut child);
                while let Some(line) = lines.next_line().await {
                    let msg = match metric.parse(&line) {
                        // Value line, possibly with std dev on the same line
                        (Some(value), std_dev, _) => {
                            metric.value_message(topic_name.clone(), value, std_dev)
                        }
                        // Std dev on its own line - don't duplicate the value
                        (None, Some(std_dev), _) => {
                            metric.std_dev_message(topic_name.clone(), std_dev)
                        }
                        (
                            None,
                            None,
                            status @ (MeasurementStatus::NotMeasuring | MeasurementStatus::NoStamp),
                        ) => {
                            let _ = sender.send(metric.status_message(topic_name.clone(), status));
                            let _ = child.kill().await;
                            return;
                        }
                        (None, None, status) => metric.status_message(topic_name.clone(), status),
                    };
                    let _ = sender.send(msg);
                }
                let _ = child.kill().await;
            }
            tokio::time::sleep(STREAM_RESTART_DELAY).await;
        }
    }
}
