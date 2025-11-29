use super::ros::{self, MeasurementStatus, TopicInfo};
use crossbeam::channel::Sender;
use once_cell::sync::Lazy;
use std::sync::Arc;
use std::{thread, time::Duration};

static SHARED_RUNTIME: Lazy<Arc<tokio::runtime::Runtime>> =
    Lazy::new(|| Arc::new(tokio::runtime::Runtime::new().expect("Failed to create tokio runtime")));

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
    Error(String),
}

pub enum WatchMessage {
    StartContinuousMetrics(String),
    StopContinuousMetrics(String),
    StartEcho(String),
    StopEcho,
    SetUseSimTime(bool),
}

pub struct TopicWatcherHandle {
    _handle: thread::JoinHandle<()>,
    shutdown_sender: Sender<()>,
}

impl TopicWatcherHandle {
    pub fn new(sender: Sender<TopicMessage>, refresh_interval: Duration) -> Self {
        let (shutdown_sender, shutdown_receiver) = crossbeam::channel::bounded::<()>(1);
        let handle = thread::spawn(move || {
            let mut topic_watcher = TopicWatcher::new(sender, refresh_interval);
            topic_watcher.run(shutdown_receiver);
        });

        Self {
            _handle: handle,
            shutdown_sender,
        }
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

    fn run(&mut self, shutdown_receiver: crossbeam::channel::Receiver<()>) {
        crate::debug_log("Topic watcher thread started");

        // Initial fetch
        self.fetch_and_send_topics();

        // Loop fetch with shutdown check
        loop {
            // Use select to wait for either timeout or shutdown signal
            crossbeam::select! {
                recv(shutdown_receiver) -> _ => {
                    crate::debug_log("Topic watcher received shutdown signal");
                    break;
                }
                default(self.refresh_interval) => {
                    self.fetch_and_send_topics();
                }
            }
        }
        crate::debug_log("Topic watcher thread exiting");
    }

    // FIX: Handle both Ok and Err cases to make the Error variant live.
    fn fetch_and_send_topics(&self) {
        match self.fetch_topics() {
            Ok(topics) => {
                if self.sender.send(TopicMessage::TopicList(topics)).is_err() {
                    // Main thread has likely exited.
                }
            }
            Err(e) => {
                let error_msg = format!("Failed to fetch topics: {}", e);
                crate::debug_log(&error_msg);
                if self.sender.send(TopicMessage::Error(error_msg)).is_err() {
                    // Main thread has likely exited.
                }
            }
        }
    }

    fn fetch_topics(&self) -> Result<Vec<TopicInfo>, ros::TopicError> {
        SHARED_RUNTIME.block_on(ros::get_topic_list())
    }
}

pub struct TopicDetailWatcherHandle {
    _handle: thread::JoinHandle<()>,
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
                _handle: handle,
                cleanup_sender,
            },
            watch_sender,
        )
    }

    pub fn shutdown(&self) {
        let _ = self.cleanup_sender.try_send(());
    }
}
use std::collections::HashMap;
use tokio::task::JoinHandle;
struct MonitoringHandles {
    hz: JoinHandle<()>,
    delay: JoinHandle<()>,
}

struct TopicDetailWatcher {
    sender: Sender<TopicMessage>,
    active_tasks: HashMap<String, MonitoringHandles>,
    echo_task: Option<JoinHandle<()>>,
    use_sim_time: bool,
}
impl TopicDetailWatcher {
    fn new(sender: Sender<TopicMessage>) -> Self {
        Self {
            sender,
            active_tasks: HashMap::new(),
            echo_task: None,
            use_sim_time: false,
        }
    }
    fn run(
        &mut self,
        watch_receiver: crossbeam::channel::Receiver<WatchMessage>,
        cleanup_receiver: crossbeam::channel::Receiver<()>,
    ) {
        crate::debug_log("Detail watcher thread started");
        loop {
            if cleanup_receiver.try_recv().is_ok() {
                self.active_tasks.drain().for_each(|(_, handles)| {
                    handles.hz.abort();
                    handles.delay.abort();
                });
                if let Some(task) = self.echo_task.take() {
                    task.abort();
                }
                break;
            }
            while let Ok(watch_msg) = watch_receiver.try_recv() {
                match watch_msg {
                    WatchMessage::StartContinuousMetrics(topic_name) => {
                        if !self.active_tasks.contains_key(&topic_name) {
                            self.start_continuous_monitoring(topic_name);
                        }
                    }
                    WatchMessage::StopContinuousMetrics(topic_name) => {
                        if let Some(handles) = self.active_tasks.remove(&topic_name) {
                            handles.hz.abort();
                            handles.delay.abort();
                        }
                    }
                    WatchMessage::StartEcho(topic_name) => {
                        if let Some(task) = self.echo_task.take() {
                            task.abort();
                        }
                        self.echo_task = Some(
                            SHARED_RUNTIME
                                .spawn(Self::monitor_topic_echo(topic_name, self.sender.clone())),
                        );
                    }
                    WatchMessage::StopEcho => {
                        if let Some(task) = self.echo_task.take() {
                            task.abort();
                        }
                    }
                    WatchMessage::SetUseSimTime(enabled) => {
                        if self.use_sim_time != enabled {
                            self.use_sim_time = enabled;
                            let topics: Vec<String> = self.active_tasks.keys().cloned().collect();
                            for topic in topics {
                                if let Some(handles) = self.active_tasks.get_mut(&topic) {
                                    handles.delay.abort();
                                    let sender_delay = self.sender.clone();
                                    handles.delay =
                                        SHARED_RUNTIME.spawn(Self::monitor_topic_delay(
                                            topic.clone(),
                                            sender_delay,
                                            self.use_sim_time,
                                        ));
                                }
                            }
                        }
                    }
                }
            }
            thread::sleep(Duration::from_millis(100));
        }
        crate::debug_log("TopicDetailWatcher thread exiting");
    }
    fn start_continuous_monitoring(&mut self, topic_name: String) {
        let sender_hz = self.sender.clone();
        let hz_handle = SHARED_RUNTIME.spawn(Self::monitor_topic_hz(topic_name.clone(), sender_hz));
        let sender_delay = self.sender.clone();
        let delay_handle = SHARED_RUNTIME.spawn(Self::monitor_topic_delay(
            topic_name.clone(),
            sender_delay,
            self.use_sim_time,
        ));
        self.active_tasks.insert(
            topic_name,
            MonitoringHandles {
                hz: hz_handle,
                delay: delay_handle,
            },
        );
    }
    async fn monitor_topic_echo(topic_name: String, sender: Sender<TopicMessage>) {
        use tokio::io::{AsyncBufReadExt, BufReader};
        if let Ok(mut child) = ros::start_topic_echo_stream(&topic_name).await {
            if let Some(stdout) = child.stdout.take() {
                let mut reader = BufReader::new(stdout).lines();
                while let Ok(Some(line)) = reader.next_line().await {
                    let _ = sender.send(TopicMessage::TopicEcho(line));
                }
            }
            let _ = child.kill().await;
        }
    }
    async fn monitor_topic_hz(topic_name: String, sender: Sender<TopicMessage>) {
        use tokio::io::{AsyncBufReadExt, BufReader};
        loop {
            if let Ok(mut child) = ros::start_topic_hz_stream(&topic_name).await {
                if let Some(stdout) = child.stdout.take() {
                    let mut reader = BufReader::new(stdout).lines();

                    while let Ok(Some(line)) = reader.next_line().await {
                        let (hz_value, std_dev_value, hz_status) =
                            ros::parse_hz_stream_line(&line).await;

                        if let Some(hz) = hz_value {
                            // Got Hz value - send immediately with any std_dev on same line
                            let _ = sender.send(TopicMessage::TopicHzUpdate {
                                topic_name: topic_name.clone(),
                                hz,
                                std_dev: std_dev_value,
                            });
                        } else if let Some(std_dev) = std_dev_value {
                            // Got std_dev without Hz - send std_dev update only (don't duplicate Hz data)
                            let _ = sender.send(TopicMessage::TopicHzStdDevUpdate {
                                topic_name: topic_name.clone(),
                                std_dev,
                            });
                        } else {
                            // No Hz or std_dev on this line
                            if matches!(hz_status, ros::MeasurementStatus::NotMeasuring) {
                                let _ = sender.send(TopicMessage::TopicHzStatusUpdate {
                                    topic_name: topic_name.clone(),
                                    status: hz_status,
                                });
                                let _ = child.kill().await;
                                return;
                            }
                            let _ = sender.send(TopicMessage::TopicHzStatusUpdate {
                                topic_name: topic_name.clone(),
                                status: hz_status,
                            });
                        }
                    }
                }
                let _ = child.kill().await;
            }
            tokio::time::sleep(Duration::from_millis(2000)).await;
        }
    }
    async fn monitor_topic_delay(
        topic_name: String,
        sender: Sender<TopicMessage>,
        use_sim_time: bool,
    ) {
        use tokio::io::{AsyncBufReadExt, BufReader};
        loop {
            if let Ok(mut child) = ros::start_topic_delay_stream(&topic_name, use_sim_time).await {
                if let Some(stdout) = child.stdout.take() {
                    let mut reader = BufReader::new(stdout).lines();

                    while let Ok(Some(line)) = reader.next_line().await {
                        let (delay_value, std_dev_value, delay_status) =
                            ros::parse_delay_stream_line(&line).await;

                        if let Some(delay) = delay_value {
                            // Got delay value - send immediately with any std_dev on same line
                            let _ = sender.send(TopicMessage::TopicDelayUpdate {
                                topic_name: topic_name.clone(),
                                delay,
                                std_dev: std_dev_value,
                            });
                        } else if let Some(std_dev) = std_dev_value {
                            // Got std_dev without delay - send std_dev update only (don't duplicate delay data)
                            let _ = sender.send(TopicMessage::TopicDelayStdDevUpdate {
                                topic_name: topic_name.clone(),
                                std_dev,
                            });
                        } else {
                            // No delay or std_dev on this line
                            if matches!(delay_status, ros::MeasurementStatus::NotMeasuring) {
                                let _ = sender.send(TopicMessage::TopicDelayStatusUpdate {
                                    topic_name: topic_name.clone(),
                                    status: delay_status,
                                });
                                let _ = child.kill().await;
                                return;
                            }
                            let _ = sender.send(TopicMessage::TopicDelayStatusUpdate {
                                topic_name: topic_name.clone(),
                                status: delay_status,
                            });
                        }
                    }
                }
                let _ = child.kill().await;
            }
            tokio::time::sleep(Duration::from_millis(2000)).await;
        }
    }
}
