use std::process::Output;
use std::time::Duration;
use tokio::process::Command;

/// Default timeout for one-shot `ros2` CLI invocations.
///
/// Without a deadline, a single unresponsive node (one whose parameter or
/// topic services never answer) stalls an entire refresh cycle and freezes the
/// UI. Bounding each call lets the app skip a misbehaving node and keep going.
pub const ROS2_COMMAND_TIMEOUT: Duration = Duration::from_secs(10);

/// Run `program` with `args`, capturing its output, aborting if it runs longer
/// than `timeout`.
///
/// Arguments are passed directly to the process (no shell), so values such as
/// node names, parameter values, or file paths cannot be word-split or
/// interpreted by a shell. The child is spawned with `kill_on_drop`, so a
/// timeout terminates the process rather than leaking it. A timeout is reported
/// as an [`std::io::Error`] of kind [`std::io::ErrorKind::TimedOut`].
pub async fn run_with_timeout(
    program: &str,
    args: &[&str],
    timeout: Duration,
) -> std::io::Result<Output> {
    let child = Command::new(program)
        .args(args)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .kill_on_drop(true)
        .spawn()?;

    match tokio::time::timeout(timeout, child.wait_with_output()).await {
        Ok(result) => result,
        Err(_) => Err(std::io::Error::new(
            std::io::ErrorKind::TimedOut,
            format!(
                "`{} {}` timed out after {:?}",
                program,
                args.join(" "),
                timeout
            ),
        )),
    }
}
