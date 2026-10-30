use super::diagnostics::{self, DiagnosticStatus};
use super::image::{self, DecodedImage};
use super::imu::{self, ImuSample};
use super::navsat::{self, NavSatFix};
use super::occupancy;
use super::pointcloud::{self, PointCloud};
use super::ros::{MeasurementStatus, TopicInfo};
use crate::common::ros_helper::{Helper, EVENT, RESTART_DELAY};
use crossbeam::channel::Sender;
use once_cell::sync::Lazy;
use serde::Deserialize;
use serde_json::json;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;

static SHARED_RUNTIME: Lazy<Arc<tokio::runtime::Runtime>> =
    Lazy::new(|| Arc::new(tokio::runtime::Runtime::new().expect("Failed to create tokio runtime")));

/// The helper process all ROS access goes through, see the script.
const ROS_BRIDGE_SCRIPT: &str = include_str!("ros_bridge.py");
/// How long shutdown waits for the helper to be killed.
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(3);

pub enum TopicMessage {
    TopicList(Vec<TopicInfo>),
    TopicHzUpdate {
        topic_name: String,
        hz: f64,
        std_dev: Option<f64>,
    },
    TopicDelayUpdate {
        topic_name: String,
        delay: f64,
        std_dev: Option<f64>,
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
    RefreshTopics,
    Shutdown,
}

/// An event from the helper, a JSON object tagged by `type`.
#[derive(Debug, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
enum Event {
    Ready,
    Topics {
        topics: Vec<ListedTopic>,
    },
    Hz {
        topic: String,
        hz: f64,
        std_dev: f64,
    },
    Delay {
        topic: String,
        delay: f64,
        std_dev: f64,
    },
    NoStamp {
        topic: String,
    },
    NotPublished {
        topic: String,
    },
    Echo {
        lines: Vec<String>,
    },
    Error {
        topic: Option<String>,
        view: Option<u32>,
        message: String,
    },
}

#[derive(Debug, Deserialize, PartialEq)]
struct ListedTopic {
    name: String,
    #[serde(rename = "type")]
    msg_type: String,
    publishers: usize,
    subscribers: usize,
}

const FRAME: u8 = 1;

/// The viewer stream currently open.
struct ActiveView {
    id: u32,
    topic: String,
    msg_type: String,
    fps: u32,
    target: Arc<ViewTarget>,
}

/// Everything asked of the helper that must survive a restart.
#[derive(Default)]
struct BridgeState {
    watched: BTreeSet<String>,
    echo: Option<String>,
    view: Option<ActiveView>,
    use_sim_time: bool,
    next_view_id: u32,
}

impl BridgeState {
    /// Record `msg` and return the helper commands it takes.
    fn apply(&mut self, msg: WatchMessage) -> Vec<serde_json::Value> {
        match msg {
            WatchMessage::StartContinuousMetrics(topic) => {
                self.watched.insert(topic.clone());
                vec![json!({"cmd": "watch", "topic": topic})]
            }
            WatchMessage::StopContinuousMetrics(topic) => {
                self.watched.remove(&topic);
                vec![json!({"cmd": "unwatch", "topic": topic})]
            }
            WatchMessage::StartEcho(topic) => {
                self.echo = Some(topic.clone());
                vec![json!({"cmd": "echo", "topic": topic})]
            }
            WatchMessage::StopEcho => {
                self.echo = None;
                vec![json!({"cmd": "unecho"})]
            }
            WatchMessage::StartViewStream {
                topic_name,
                msg_type,
                max_fps,
                target,
            } => {
                let mut commands = self.stop_view();
                self.next_view_id += 1;
                let view = ActiveView {
                    id: self.next_view_id,
                    topic: topic_name,
                    msg_type,
                    fps: max_fps,
                    target,
                };
                commands.push(view_command(&view));
                self.view = Some(view);
                commands
            }
            WatchMessage::StopViewStream => self.stop_view(),
            WatchMessage::SetViewFps(fps) => match &mut self.view {
                Some(view) => {
                    view.fps = fps;
                    vec![json!({"cmd": "fps", "id": view.id, "fps": fps})]
                }
                None => vec![],
            },
            WatchMessage::SetUseSimTime(enabled) => {
                self.use_sim_time = enabled;
                vec![json!({"cmd": "sim_time", "enabled": enabled})]
            }
            WatchMessage::RefreshTopics => vec![json!({"cmd": "list"})],
            WatchMessage::Shutdown => vec![],
        }
    }

    fn stop_view(&mut self) -> Vec<serde_json::Value> {
        self.view
            .take()
            .map(|view| json!({"cmd": "unview", "id": view.id}))
            .into_iter()
            .collect()
    }

    /// Commands that restore this state in a freshly started helper.
    fn replay(&self) -> Vec<serde_json::Value> {
        let mut commands = vec![json!({"cmd": "list"})];
        if self.use_sim_time {
            commands.push(json!({"cmd": "sim_time", "enabled": true}));
        }
        commands.extend(
            self.watched
                .iter()
                .map(|topic| json!({"cmd": "watch", "topic": topic})),
        );
        commands.extend(
            self.echo
                .iter()
                .map(|topic| json!({"cmd": "echo", "topic": topic})),
        );
        commands.extend(self.view.iter().map(view_command));
        commands
    }

    /// Messages for the UI from a helper event.
    fn event_messages(&self, event: Event) -> Vec<TopicMessage> {
        match event {
            Event::Ready => vec![],
            Event::Topics { topics } => vec![TopicMessage::TopicList(
                topics
                    .into_iter()
                    .map(|t| TopicInfo {
                        name: t.name,
                        msg_type: t.msg_type,
                        publisher_count: t.publishers,
                        subscriber_count: t.subscribers,
                        ..TopicInfo::default()
                    })
                    .collect(),
            )],
            Event::Hz { topic, hz, std_dev } => vec![TopicMessage::TopicHzUpdate {
                topic_name: topic,
                hz,
                std_dev: Some(std_dev),
            }],
            Event::Delay {
                topic,
                delay,
                std_dev,
            } => vec![TopicMessage::TopicDelayUpdate {
                topic_name: topic,
                delay,
                std_dev: Some(std_dev),
            }],
            Event::NoStamp { topic } => vec![TopicMessage::TopicDelayStatusUpdate {
                topic_name: topic,
                status: MeasurementStatus::NoStamp,
            }],
            Event::NotPublished { topic } => vec![
                TopicMessage::TopicHzStatusUpdate {
                    topic_name: topic.clone(),
                    status: MeasurementStatus::NotMeasuring,
                },
                TopicMessage::TopicDelayStatusUpdate {
                    topic_name: topic,
                    status: MeasurementStatus::NotMeasuring,
                },
            ],
            Event::Echo { lines } => lines.into_iter().map(TopicMessage::TopicEcho).collect(),
            Event::Error {
                topic,
                view,
                message,
            } => match (&self.view, view, topic) {
                (Some(active), Some(id), _) if active.id == id => vec![TopicMessage::ViewError {
                    topic_name: active.topic.clone(),
                    message,
                }],
                // An error of a view since closed.
                (_, Some(_), _) => vec![],
                (_, None, Some(topic)) => {
                    vec![TopicMessage::Error(format!("{}: {}", topic, message))]
                }
                (_, None, None) => vec![TopicMessage::Error(message)],
            },
        }
    }
}

fn view_command(view: &ActiveView) -> serde_json::Value {
    json!({
        "cmd": "view",
        "id": view.id,
        "topic": view.topic,
        "type": view.msg_type,
        "fps": view.fps,
    })
}

/// Owns the helper: forwards the app's requests, turns the helper's output
/// into `TopicMessage`s and restarts it, with its state, if it exits.
pub struct RosBridgeHandle {
    commands: mpsc::UnboundedSender<WatchMessage>,
    done: crossbeam::channel::Receiver<()>,
}

impl RosBridgeHandle {
    pub fn new(
        sender: Sender<TopicMessage>,
        refresh_interval: Duration,
        initial_fetch: bool,
    ) -> Self {
        let (commands, receiver) = mpsc::unbounded_channel();
        let (done_sender, done) = crossbeam::channel::bounded(1);
        SHARED_RUNTIME.spawn(async move {
            supervise(sender, receiver, refresh_interval, initial_fetch).await;
            let _ = done_sender.send(());
        });
        Self { commands, done }
    }

    /// Queue a request; false if the bridge is shut down.
    pub fn send(&self, msg: WatchMessage) -> bool {
        self.commands.send(msg).is_ok()
    }

    /// Fetch the topic list now instead of at the next periodic refresh.
    pub fn refresh(&self) {
        self.send(WatchMessage::RefreshTopics);
    }

    /// Stop the helper and wait until it has been killed.
    pub fn shutdown(&self) {
        if self.send(WatchMessage::Shutdown) {
            let _ = self.done.recv_timeout(SHUTDOWN_TIMEOUT);
        }
    }
}

async fn supervise(
    sender: Sender<TopicMessage>,
    mut commands: mpsc::UnboundedReceiver<WatchMessage>,
    refresh_interval: Duration,
    initial_fetch: bool,
) {
    let mut state = BridgeState::default();
    let mut first_start = true;
    loop {
        match Helper::spawn(ROS_BRIDGE_SCRIPT) {
            Ok(mut helper) => {
                let mut replay = state.replay();
                if first_start && !initial_fetch {
                    replay.remove(0);
                }
                first_start = false;
                for command in &replay {
                    let _ = helper.send(command).await;
                }
                if run(
                    &mut helper,
                    &mut state,
                    &sender,
                    &mut commands,
                    refresh_interval,
                )
                .await
                {
                    return;
                }
                let reason = helper.exit_reason().await;
                crate::debug_log(&format!("ROS bridge helper exited: {}", reason));
                let _ = sender.send(TopicMessage::Error(format!(
                    "ROS helper exited ({}), restarting",
                    reason
                )));
            }
            Err(e) => {
                let _ = sender.send(TopicMessage::Error(format!(
                    "Cannot start python3 for the ROS helper: {}",
                    e
                )));
            }
        }
        // Keep taking requests while waiting, so they are replayed.
        let restart = tokio::time::sleep(RESTART_DELAY);
        tokio::pin!(restart);
        loop {
            tokio::select! {
                _ = &mut restart => break,
                msg = commands.recv() => match msg {
                    None | Some(WatchMessage::Shutdown) => return,
                    Some(msg) => { state.apply(msg); }
                },
            }
        }
    }
}

/// Serve one helper until it exits (false) or shutdown is requested (true).
async fn run(
    helper: &mut Helper,
    state: &mut BridgeState,
    sender: &Sender<TopicMessage>,
    commands: &mut mpsc::UnboundedReceiver<WatchMessage>,
    refresh_interval: Duration,
) -> bool {
    let mut refresh = tokio::time::interval_at(
        tokio::time::Instant::now() + refresh_interval,
        refresh_interval,
    );
    refresh.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            msg = commands.recv() => {
                let Some(msg) = msg else { return true };
                if matches!(msg, WatchMessage::Shutdown) {
                    return true;
                }
                for command in state.apply(msg) {
                    if helper.send(&command).await.is_err() {
                        return false;
                    }
                }
            }
            frame = helper.frames.recv() => {
                let Some((kind, payload)) = frame else { return false };
                for msg in handle_frame(state, kind, payload).await {
                    if sender.send(msg).is_err() {
                        return true;
                    }
                }
            }
            _ = refresh.tick() => {
                if helper.send(&json!({"cmd": "list"})).await.is_err() {
                    return false;
                }
            }
        }
    }
}

async fn handle_frame(state: &BridgeState, kind: u8, payload: Vec<u8>) -> Vec<TopicMessage> {
    match kind {
        EVENT => match serde_json::from_slice::<Event>(&payload) {
            Ok(event) => state.event_messages(event),
            Err(e) => {
                crate::debug_log(&format!("Unreadable helper event: {}", e));
                vec![]
            }
        },
        FRAME if payload.len() >= 8 => {
            let id = u32::from_le_bytes(payload[..4].try_into().unwrap());
            let received_count = u32::from_le_bytes(payload[4..8].try_into().unwrap());
            let Some(view) = state.view.as_ref().filter(|v| v.id == id) else {
                return vec![];
            };
            let (msg_type, size) = (view.msg_type.clone(), view.target.get());
            let decoded =
                tokio::task::spawn_blocking(move || decode_frame(&msg_type, &payload[8..], size))
                    .await;
            match decoded {
                Ok(Ok(content)) => vec![TopicMessage::ViewFrame(Box::new(ViewFrame {
                    topic_name: view.topic.clone(),
                    content,
                    received_count,
                }))],
                Ok(Err(message)) => vec![TopicMessage::ViewError {
                    topic_name: view.topic.clone(),
                    message,
                }],
                Err(_) => vec![],
            }
        }
        _ => vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(json: &str) -> Event {
        serde_json::from_str(json).unwrap()
    }

    #[test]
    fn parses_helper_events() {
        assert_eq!(event(r#"{"type": "ready"}"#), Event::Ready);
        assert_eq!(
            event(r#"{"type": "hz", "topic": "/a", "hz": 10.0, "std_dev": 0.001}"#),
            Event::Hz {
                topic: "/a".into(),
                hz: 10.0,
                std_dev: 0.001
            }
        );
        let topics = event(
            r#"{"type": "topics", "topics": [{"name": "/a", "type": "std_msgs/msg/String", "publishers": 1, "subscribers": 0}]}"#,
        );
        let messages = BridgeState::default().event_messages(topics);
        let [TopicMessage::TopicList(list)] = messages.as_slice() else {
            panic!("expected a topic list");
        };
        assert_eq!(list[0].msg_type, "std_msgs/msg/String");
        assert_eq!(list[0].publisher_count, 1);
    }

    #[test]
    fn replays_watches_echo_and_view_after_a_restart() {
        let mut state = BridgeState::default();
        state.apply(WatchMessage::StartContinuousMetrics("/a".into()));
        state.apply(WatchMessage::StartContinuousMetrics("/b".into()));
        state.apply(WatchMessage::StopContinuousMetrics("/a".into()));
        state.apply(WatchMessage::StartEcho("/b".into()));
        state.apply(WatchMessage::StartViewStream {
            topic_name: "/cam".into(),
            msg_type: "sensor_msgs/msg/Image".into(),
            max_fps: 10,
            target: Arc::new(ViewTarget::default()),
        });
        state.apply(WatchMessage::SetViewFps(5));
        let replay: Vec<String> = state.replay().iter().map(|c| c.to_string()).collect();
        assert_eq!(
            replay,
            [
                r#"{"cmd":"list"}"#,
                r#"{"cmd":"watch","topic":"/b"}"#,
                r#"{"cmd":"echo","topic":"/b"}"#,
                r#"{"cmd":"view","fps":5,"id":1,"topic":"/cam","type":"sensor_msgs/msg/Image"}"#,
            ]
        );
    }

    #[test]
    fn a_new_view_closes_the_previous_one() {
        let mut state = BridgeState::default();
        let view = |topic: &str| WatchMessage::StartViewStream {
            topic_name: topic.into(),
            msg_type: "sensor_msgs/msg/Imu".into(),
            max_fps: 30,
            target: Arc::new(ViewTarget::default()),
        };
        state.apply(view("/imu"));
        let commands = state.apply(view("/imu2"));
        assert_eq!(commands[0], json!({"cmd": "unview", "id": 1}));
        assert_eq!(commands[1]["id"], 2);
    }

    #[test]
    fn errors_of_closed_views_are_dropped() {
        let mut state = BridgeState::default();
        state.apply(WatchMessage::StartViewStream {
            topic_name: "/cam".into(),
            msg_type: "sensor_msgs/msg/Image".into(),
            max_fps: 10,
            target: Arc::new(ViewTarget::default()),
        });
        let error = |view| Event::Error {
            topic: Some("/cam".into()),
            view: Some(view),
            message: "boom".into(),
        };
        assert!(matches!(
            state.event_messages(error(1)).as_slice(),
            [TopicMessage::ViewError { .. }]
        ));
        assert!(state.event_messages(error(7)).is_empty());
    }
}
