use super::ros::{format_value, type_name, ParamInfo};
use crate::common::ros_helper::{Helper, EVENT, RESTART_DELAY};
use crossbeam::channel::Sender;
use once_cell::sync::Lazy;
use serde::Deserialize;
use serde_json::json;
use std::time::Duration;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;

static RUNTIME: Lazy<Runtime> =
    Lazy::new(|| Runtime::new().expect("Failed to create Tokio runtime"));

/// The helper process all parameter access goes through, see the script.
const PARAM_BRIDGE_SCRIPT: &str = include_str!("param_bridge.py");
/// How long shutdown waits for the helper to be killed.
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Debug, Clone)]
pub enum ParamMessage {
    ParamList(Vec<ParamInfo>),
    ParamValue {
        node_name: String,
        param_name: String,
        value: String,
        param_type: String,
    },
    ParamSetSuccess {
        node_name: String,
        param_name: String,
        new_value: String,
    },
    ParamSetError {
        node_name: String,
        param_name: String,
        error: String,
    },
    DumpSuccess {
        node_name: String,
        file_path: String,
    },
    LoadSuccess {
        node_name: String,
        file_path: String,
    },
    Error(String),
}

#[derive(Debug, Clone)]
pub enum ParamWatchMessage {
    RefreshList,
    RefreshNode {
        node_name: String,
    },
    SetParam {
        node_name: String,
        param_name: String,
        value: String,
    },
    DumpParam {
        node_name: String,
        file_path: String,
    },
    LoadParam {
        node_name: String,
        file_path: String,
    },
    Shutdown,
}

/// An event from the helper, a JSON object tagged by `type`.
#[derive(Debug, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
enum Event {
    Ready,
    ParamList {
        nodes: Vec<NodeParams>,
    },
    NodeParams {
        node: String,
        params: Vec<Param>,
    },
    SetResult {
        node: String,
        name: String,
        value: String,
        error: Option<String>,
    },
    Dumped {
        node: String,
        path: String,
        yaml: String,
    },
    Loaded {
        node: String,
        path: String,
        count: usize,
        errors: Vec<String>,
    },
    Error {
        message: String,
    },
}

#[derive(Debug, Deserialize, PartialEq)]
struct NodeParams {
    node: String,
    params: Vec<Param>,
}

#[derive(Debug, Deserialize, PartialEq)]
struct Param {
    name: String,
    #[serde(rename = "type")]
    param_type: u8,
    value: serde_json::Value,
}

impl Param {
    fn into_info(self, node: &str) -> ParamInfo {
        ParamInfo {
            node_name: node.to_string(),
            value: Some(format_value(self.param_type, &self.value)),
            param_type: type_name(self.param_type).to_string(),
            param_name: self.name,
        }
    }
}

/// The helper command for a request, or a message for the UI if it cannot
/// be sent.
fn command(msg: ParamWatchMessage) -> Result<serde_json::Value, ParamMessage> {
    Ok(match msg {
        ParamWatchMessage::RefreshList | ParamWatchMessage::Shutdown => json!({"cmd": "list"}),
        ParamWatchMessage::RefreshNode { node_name } => json!({"cmd": "node", "node": node_name}),
        ParamWatchMessage::SetParam {
            node_name,
            param_name,
            value,
        } => json!({"cmd": "set", "node": node_name, "name": param_name, "value": value}),
        ParamWatchMessage::DumpParam {
            node_name,
            file_path,
        } => json!({"cmd": "dump", "node": node_name, "path": file_path}),
        ParamWatchMessage::LoadParam {
            node_name,
            file_path,
        } => {
            // Read here, so the error names the file rather than the helper.
            let yaml = std::fs::read_to_string(&file_path)
                .map_err(|e| ParamMessage::Error(format!("Cannot read {}: {}", file_path, e)))?;
            json!({"cmd": "load", "node": node_name, "path": file_path, "yaml": yaml})
        }
    })
}

/// Messages for the UI from a helper event.
async fn event_messages(event: Event) -> Vec<ParamMessage> {
    match event {
        Event::Ready => vec![],
        Event::ParamList { nodes } => vec![ParamMessage::ParamList(
            nodes
                .into_iter()
                .flat_map(|n| {
                    let node = n.node;
                    n.params
                        .into_iter()
                        .map(move |p| p.into_info(&node))
                        .collect::<Vec<_>>()
                })
                .collect(),
        )],
        Event::NodeParams { node, params } => params
            .into_iter()
            .map(|p| {
                let info = p.into_info(&node);
                ParamMessage::ParamValue {
                    node_name: info.node_name,
                    param_name: info.param_name,
                    value: info.value.unwrap_or_default(),
                    param_type: info.param_type,
                }
            })
            .collect(),
        Event::SetResult {
            node,
            name,
            value,
            error,
        } => vec![match error {
            None => ParamMessage::ParamSetSuccess {
                node_name: node,
                param_name: name,
                new_value: value,
            },
            Some(error) => ParamMessage::ParamSetError {
                node_name: node,
                param_name: name,
                error: format!("Setting parameter failed: {}", error),
            },
        }],
        Event::Dumped { node, path, yaml } => vec![match tokio::fs::write(&path, yaml).await {
            Ok(()) => ParamMessage::DumpSuccess {
                node_name: node,
                file_path: path,
            },
            Err(e) => ParamMessage::Error(format!("Cannot write {}: {}", path, e)),
        }],
        Event::Loaded {
            node,
            path,
            count,
            errors,
        } => vec![if errors.is_empty() {
            ParamMessage::LoadSuccess {
                node_name: node,
                file_path: path,
            }
        } else {
            ParamMessage::Error(format!(
                "Loading {} into {}: {} of {} parameters failed: {}",
                path,
                node,
                errors.len(),
                count,
                errors.join("; ")
            ))
        }],
        Event::Error { message } => vec![ParamMessage::Error(message)],
    }
}

/// Owns the helper: forwards the app's requests, turns the helper's events
/// into `ParamMessage`s, refreshes periodically and restarts the helper if
/// it exits.
pub struct ParamBridgeHandle {
    commands: mpsc::UnboundedSender<ParamWatchMessage>,
    done: crossbeam::channel::Receiver<()>,
}

impl ParamBridgeHandle {
    pub fn new(
        sender: Sender<ParamMessage>,
        refresh_interval: Duration,
        initial_fetch: bool,
    ) -> Self {
        let (commands, receiver) = mpsc::unbounded_channel();
        let (done_sender, done) = crossbeam::channel::bounded(1);
        RUNTIME.spawn(async move {
            supervise(sender, receiver, refresh_interval, initial_fetch).await;
            let _ = done_sender.send(());
        });
        Self { commands, done }
    }

    /// Queue a request; false if the bridge is shut down.
    pub fn send(&self, msg: ParamWatchMessage) -> bool {
        self.commands.send(msg).is_ok()
    }

    /// Stop the helper and wait until it has been killed.
    pub fn shutdown(&self) {
        if self.send(ParamWatchMessage::Shutdown) {
            let _ = self.done.recv_timeout(SHUTDOWN_TIMEOUT);
        }
    }
}

async fn supervise(
    sender: Sender<ParamMessage>,
    mut commands: mpsc::UnboundedReceiver<ParamWatchMessage>,
    refresh_interval: Duration,
    initial_fetch: bool,
) {
    // Requests made while the helper was down, sent once it is back.
    let mut pending = Vec::new();
    let mut list_first = initial_fetch;
    loop {
        match Helper::spawn(PARAM_BRIDGE_SCRIPT) {
            Ok(mut helper) => {
                if list_first {
                    pending.insert(0, ParamWatchMessage::RefreshList);
                }
                list_first = true;
                let mut alive = true;
                for msg in pending.drain(..) {
                    alive = alive && forward(&mut helper, &sender, msg).await;
                }
                if alive && run(&mut helper, &sender, &mut commands, refresh_interval).await {
                    return;
                }
                let reason = helper.exit_reason().await;
                crate::debug_log(&format!("Params helper exited: {}", reason));
                let _ = sender.send(ParamMessage::Error(format!(
                    "ROS helper exited ({}), restarting",
                    reason
                )));
            }
            Err(e) => {
                let _ = sender.send(ParamMessage::Error(format!(
                    "Cannot start python3 for the ROS helper: {}",
                    e
                )));
            }
        }
        let restart = tokio::time::sleep(RESTART_DELAY);
        tokio::pin!(restart);
        loop {
            tokio::select! {
                _ = &mut restart => break,
                msg = commands.recv() => match msg {
                    None | Some(ParamWatchMessage::Shutdown) => return,
                    Some(msg) => pending.push(msg),
                },
            }
        }
    }
}

/// Send a request to the helper; false if it has exited.
async fn forward(
    helper: &mut Helper,
    sender: &Sender<ParamMessage>,
    msg: ParamWatchMessage,
) -> bool {
    match command(msg) {
        Ok(command) => helper.send(&command).await.is_ok(),
        Err(message) => {
            let _ = sender.send(message);
            true
        }
    }
}

/// Serve one helper until it exits (false) or shutdown is requested (true).
async fn run(
    helper: &mut Helper,
    sender: &Sender<ParamMessage>,
    commands: &mut mpsc::UnboundedReceiver<ParamWatchMessage>,
    refresh_interval: Duration,
) -> bool {
    let mut refresh = tokio::time::interval_at(
        tokio::time::Instant::now() + refresh_interval,
        refresh_interval,
    );
    refresh.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            msg = commands.recv() => match msg {
                None | Some(ParamWatchMessage::Shutdown) => return true,
                Some(msg) => {
                    if !forward(helper, sender, msg).await {
                        return false;
                    }
                }
            },
            frame = helper.frames.recv() => {
                let Some((kind, payload)) = frame else { return false };
                if kind != EVENT {
                    continue;
                }
                let messages = match serde_json::from_slice::<Event>(&payload) {
                    Ok(event) => event_messages(event).await,
                    Err(e) => {
                        crate::debug_log(&format!("Unreadable helper event: {}", e));
                        vec![]
                    }
                };
                for msg in messages {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn event(json: &str) -> Event {
        serde_json::from_str(json).unwrap()
    }

    #[test]
    fn turns_a_param_list_into_param_infos() {
        let list = event(
            r#"{"type": "param_list", "nodes": [{"node": "/cam", "params": [
                {"name": "fps", "type": 3, "value": 30.0},
                {"name": "topics", "type": 9, "value": ["a", "b"]}]}]}"#,
        );
        let messages = RUNTIME.block_on(event_messages(list));
        let [ParamMessage::ParamList(params)] = messages.as_slice() else {
            panic!("expected a parameter list");
        };
        assert_eq!(params[0].node_name, "/cam");
        assert_eq!(params[0].value.as_deref(), Some("30.0"));
        assert_eq!(params[0].param_type, "Double");
        assert_eq!(params[1].value.as_deref(), Some("[\"a\", \"b\"]"));
        assert_eq!(params[1].param_type, "String[]");
    }

    #[test]
    fn reports_set_failures_with_the_reason() {
        let result = event(
            r#"{"type": "set_result", "node": "/n", "name": "p", "value": "x", "error": "p: Wrong parameter type"}"#,
        );
        let messages = RUNTIME.block_on(event_messages(result));
        let [ParamMessage::ParamSetError { error, .. }] = messages.as_slice() else {
            panic!("expected a set error");
        };
        assert_eq!(error, "Setting parameter failed: p: Wrong parameter type");
    }

    #[test]
    fn load_reads_the_file_before_asking_the_helper() {
        let missing = command(ParamWatchMessage::LoadParam {
            node_name: "/n".into(),
            file_path: "/nonexistent/params.yaml".into(),
        });
        assert!(matches!(missing, Err(ParamMessage::Error(m)) if m.contains("/nonexistent")));
    }
}
