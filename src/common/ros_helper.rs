//! A long-lived `rclpy` helper process, through which the apps do all their
//! ROS access instead of running `ros2` CLI commands.
//!
//! The helper script takes JSON command lines on stdin and writes frames to
//! stdout: a little-endian u32 length, a u8 kind and the payload. Each app
//! embeds its own script and defines the kinds it uses.

use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::process::{Child, ChildStdin, Command};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

/// Delay before restarting a helper that exited.
pub const RESTART_DELAY: Duration = Duration::from_secs(2);

/// Frame kind of a JSON event, the same for every helper.
pub const EVENT: u8 = 0;

/// Check that `python3` can import `rclpy`, i.e. that ROS2 is sourced.
pub fn check_ros_environment() -> Result<(), String> {
    let output = std::process::Command::new("python3")
        .args(["-c", "import rclpy"])
        .output()
        .map_err(|e| format!("python3 not found ({})", e))?;
    if output.status.success() {
        Ok(())
    } else {
        Err(String::from_utf8_lossy(&output.stderr)
            .lines()
            .last()
            .unwrap_or("cannot import rclpy")
            .to_string())
    }
}

/// A running helper: its stdin for commands, and frames read from stdout.
pub struct Helper {
    child: Child,
    stdin: ChildStdin,
    pub frames: mpsc::Receiver<(u8, Vec<u8>)>,
    stderr: JoinHandle<String>,
}

impl Helper {
    pub fn spawn(script: &str) -> std::io::Result<Self> {
        crate::debug_log("Starting the ROS helper");
        let mut child = Command::new("python3")
            .arg("-c")
            .arg(script)
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .kill_on_drop(true)
            .spawn()?;
        let (Some(stdin), Some(mut stdout), Some(mut stderr)) =
            (child.stdin.take(), child.stdout.take(), child.stderr.take())
        else {
            return Err(std::io::Error::other("helper pipes missing"));
        };

        // Reading frames in their own task keeps the supervisor's select
        // cancel-safe; the channel closes when the helper exits.
        let (frame_sender, frames) = mpsc::channel(64);
        tokio::spawn(async move {
            loop {
                let mut header = [0u8; 5];
                if stdout.read_exact(&mut header).await.is_err() {
                    break;
                }
                let len = u32::from_le_bytes(header[..4].try_into().unwrap()) as usize;
                let mut payload = vec![0u8; len];
                if stdout.read_exact(&mut payload).await.is_err()
                    || frame_sender.send((header[4], payload)).await.is_err()
                {
                    break;
                }
            }
        });
        let stderr = tokio::spawn(async move {
            let mut text = String::new();
            let _ = stderr.read_to_string(&mut text).await;
            text
        });
        Ok(Self {
            child,
            stdin,
            frames,
            stderr,
        })
    }

    pub async fn send(&mut self, command: &serde_json::Value) -> std::io::Result<()> {
        let mut line = command.to_string();
        line.push('\n');
        self.stdin.write_all(line.as_bytes()).await?;
        self.stdin.flush().await
    }

    /// The last line the helper wrote to stderr, for error messages.
    pub async fn exit_reason(self) -> String {
        drop(self.child);
        let stderr = self.stderr.await.unwrap_or_default();
        stderr
            .lines()
            .rev()
            .find(|line| !line.trim().is_empty())
            .unwrap_or("no output")
            .trim()
            .to_string()
    }
}
