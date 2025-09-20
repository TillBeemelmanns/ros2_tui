use super::ros::{self, ParamInfo};
use crossbeam::channel::Sender;
use once_cell::sync::Lazy;
use std::thread;
use std::time::Duration;
use tokio::runtime::Runtime;

static RUNTIME: Lazy<Runtime> =
    Lazy::new(|| Runtime::new().expect("Failed to create Tokio runtime"));

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
    RefreshNodeDelayed {
        node_name: String,
        delay_ms: u64,
    },
    GetParam {
        node_name: String,
        param_name: String,
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

pub struct ParamWatcherHandle {
    _handle: thread::JoinHandle<()>,
}

impl ParamWatcherHandle {
    pub fn new(sender: Sender<ParamMessage>, refresh_interval: Duration) -> Self {
        let sender_clone = sender.clone();
        let handle = thread::spawn(move || {
            let mut last_refresh = std::time::Instant::now();

            loop {
                // Check if it's time for periodic refresh
                if last_refresh.elapsed() >= refresh_interval {
                    match RUNTIME.block_on(ros::get_param_list_with_values()) {
                        Ok(params) => {
                            if sender_clone.send(ParamMessage::ParamList(params)).is_err() {
                                break;
                            }
                        }
                        Err(e) => {
                            if sender_clone
                                .send(ParamMessage::Error(format!(
                                    "Failed to get parameter list: {}",
                                    e
                                )))
                                .is_err()
                            {
                                break;
                            }
                        }
                    }

                    last_refresh = std::time::Instant::now();
                }

                thread::sleep(Duration::from_millis(1000));
            }
        });

        // Initial refresh
        RUNTIME.spawn(async move {
            crate::debug_log("Starting initial parameter fetch with improved method...");
            match ros::get_param_list_with_values().await {
                Ok(params) => {
                    crate::debug_log(&format!("Fetched {} parameters successfully", params.len()));
                    let _ = sender.send(ParamMessage::ParamList(params));
                }
                Err(e) => {
                    crate::debug_log(&format!("Failed to get initial parameter list: {}", e));
                    let _ = sender.send(ParamMessage::Error(format!(
                        "Failed to get initial parameter list: {}",
                        e
                    )));
                }
            }
        });

        Self { _handle: handle }
    }
}

pub struct ParamValueWatcherHandle {
    _handle: thread::JoinHandle<()>,
}

impl ParamValueWatcherHandle {
    pub fn new(message_sender: Sender<ParamMessage>) -> (Self, Sender<ParamWatchMessage>) {
        let (watch_sender, watch_receiver) = crossbeam::channel::unbounded();

        let msg_sender = message_sender.clone();
        let handle = thread::spawn(move || {
            loop {
                match watch_receiver.recv() {
                    Ok(ParamWatchMessage::RefreshList) => {
                        match RUNTIME.block_on(ros::get_param_list_with_values()) {
                            Ok(params) => {
                                if msg_sender.send(ParamMessage::ParamList(params)).is_err() {
                                    break;
                                }
                            }
                            Err(e) => {
                                if msg_sender
                                    .send(ParamMessage::Error(format!(
                                        "Failed to refresh parameter list: {}",
                                        e
                                    )))
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                    Ok(ParamWatchMessage::RefreshNode { node_name }) => {
                        match RUNTIME.block_on(ros::get_node_params_with_values(&node_name)) {
                            Ok(params) => {
                                // Send parameter values for all parameters in this node
                                for param in params {
                                    if let Some(value) = param.value {
                                        if msg_sender
                                            .send(ParamMessage::ParamValue {
                                                node_name: param.node_name,
                                                param_name: param.param_name,
                                                value,
                                                param_type: param.param_type,
                                            })
                                            .is_err()
                                        {
                                            break;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                if msg_sender
                                    .send(ParamMessage::Error(format!(
                                        "Failed to refresh parameters for node {}: {}",
                                        node_name, e
                                    )))
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                    Ok(ParamWatchMessage::RefreshNodeDelayed {
                        node_name,
                        delay_ms,
                    }) => {
                        // Add delay before refreshing to allow parameter set to take effect
                        thread::sleep(std::time::Duration::from_millis(delay_ms));
                        match RUNTIME.block_on(ros::get_node_params_with_values(&node_name)) {
                            Ok(params) => {
                                // Send parameter values for all parameters in this node
                                for param in params {
                                    if let Some(value) = param.value {
                                        if msg_sender
                                            .send(ParamMessage::ParamValue {
                                                node_name: param.node_name,
                                                param_name: param.param_name,
                                                value,
                                                param_type: param.param_type,
                                            })
                                            .is_err()
                                        {
                                            break;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                if msg_sender
                                    .send(ParamMessage::Error(format!(
                                        "Failed to refresh node {} (delayed): {}",
                                        node_name, e
                                    )))
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                    Ok(ParamWatchMessage::GetParam {
                        node_name,
                        param_name,
                    }) => {
                        match RUNTIME.block_on(ros::get_single_param_value(&node_name, &param_name))
                        {
                            Ok((value, param_type)) => {
                                if msg_sender
                                    .send(ParamMessage::ParamValue {
                                        node_name,
                                        param_name,
                                        value,
                                        param_type,
                                    })
                                    .is_err()
                                {
                                    break;
                                }
                            }
                            Err(e) => {
                                if msg_sender
                                    .send(ParamMessage::Error(format!(
                                        "Failed to get parameter {}/{}: {}",
                                        node_name, param_name, e
                                    )))
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                    Ok(ParamWatchMessage::SetParam {
                        node_name,
                        param_name,
                        value,
                    }) => {
                        match RUNTIME.block_on(ros::set_param_value(
                            &node_name,
                            &param_name,
                            &value,
                        )) {
                            Ok(success_message) => {
                                if msg_sender
                                    .send(ParamMessage::ParamSetSuccess {
                                        node_name,
                                        param_name,
                                        new_value: format!("{} ({})", value, success_message),
                                    })
                                    .is_err()
                                {
                                    break;
                                }
                            }
                            Err(e) => {
                                if msg_sender
                                    .send(ParamMessage::ParamSetError {
                                        node_name,
                                        param_name,
                                        error: e.to_string(),
                                    })
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                    Ok(ParamWatchMessage::DumpParam {
                        node_name,
                        file_path,
                    }) => match RUNTIME.block_on(ros::dump_params(&node_name, &file_path)) {
                        Ok(()) => {
                            if msg_sender
                                .send(ParamMessage::DumpSuccess {
                                    node_name,
                                    file_path,
                                })
                                .is_err()
                            {
                                break;
                            }
                        }
                        Err(e) => {
                            if msg_sender
                                .send(ParamMessage::Error(format!(
                                    "Failed to dump parameters from {}: {}",
                                    node_name, e
                                )))
                                .is_err()
                            {
                                break;
                            }
                        }
                    },
                    Ok(ParamWatchMessage::LoadParam {
                        node_name,
                        file_path,
                    }) => match RUNTIME.block_on(ros::load_params(&node_name, &file_path)) {
                        Ok(()) => {
                            if msg_sender
                                .send(ParamMessage::LoadSuccess {
                                    node_name,
                                    file_path,
                                })
                                .is_err()
                            {
                                break;
                            }
                        }
                        Err(e) => {
                            if msg_sender
                                .send(ParamMessage::Error(format!(
                                    "Failed to load parameters to {}: {}",
                                    node_name, e
                                )))
                                .is_err()
                            {
                                break;
                            }
                        }
                    },
                    Ok(ParamWatchMessage::Shutdown) => {
                        break;
                    }
                    Err(_) => {
                        break;
                    }
                }
            }
        });

        let watcher = Self { _handle: handle };

        (watcher, watch_sender)
    }
}
