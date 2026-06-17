use ros2_tui::common::run_with_timeout;
use std::io::ErrorKind;
use std::time::{Duration, Instant};

/// A command that finishes within the timeout returns its captured output.
#[tokio::test]
async fn returns_output_for_fast_command() {
    let output = run_with_timeout("echo", &["hello world"], Duration::from_secs(10))
        .await
        .expect("echo should succeed");

    assert!(output.status.success());
    assert_eq!(
        String::from_utf8_lossy(&output.stdout).trim(),
        "hello world"
    );
}

/// A command that outlives the timeout is aborted promptly and reported as a
/// `TimedOut` error rather than hanging — the core guarantee that keeps a single
/// unresponsive `ros2` call from freezing the UI.
#[tokio::test]
async fn aborts_slow_command_with_timeout() {
    let start = Instant::now();
    let result = run_with_timeout("sleep", &["30"], Duration::from_millis(300)).await;
    let elapsed = start.elapsed();

    let err = result.expect_err("sleep 30 should time out");
    assert_eq!(err.kind(), ErrorKind::TimedOut);
    // Should return near the timeout, nowhere near the 30s the command requested.
    assert!(
        elapsed < Duration::from_secs(5),
        "timed-out command returned too slowly: {:?}",
        elapsed
    );
}

/// Arguments are passed directly to the process, so shell metacharacters in a
/// value are treated as literal text and never interpreted by a shell.
#[tokio::test]
async fn arguments_are_not_shell_interpreted() {
    let payload = "$(touch /tmp/ros2_tui_should_not_exist); a b | c";
    let output = run_with_timeout("echo", &[payload], Duration::from_secs(10))
        .await
        .expect("echo should succeed");

    assert_eq!(String::from_utf8_lossy(&output.stdout).trim(), payload);
    assert!(
        !std::path::Path::new("/tmp/ros2_tui_should_not_exist").exists(),
        "command substitution must not have been executed"
    );
}
