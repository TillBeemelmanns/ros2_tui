use std::fs::OpenOptions;
use std::io::{self, Write};
use std::sync::Mutex;

lazy_static::lazy_static! {
    static ref DEBUG_FILE: Mutex<Option<std::fs::File>> = Mutex::new(None);
    static ref DEBUG_ENABLED: Mutex<bool> = Mutex::new(false);
}

pub fn debug_log(msg: &str) {
    // Check if debug logging is enabled
    if let Ok(enabled) = DEBUG_ENABLED.lock() {
        if !*enabled {
            return;
        }
    } else {
        return; // If mutex is poisoned, don't log
    }

    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis();

    if let Ok(mut file_opt) = DEBUG_FILE.lock() {
        if let Some(file) = file_opt.as_mut() {
            let _ = writeln!(file, "[{}] {}", timestamp, msg);
            let _ = file.flush();
        }
    }
}

pub fn enable_debug_logging(log_filename: &str) -> io::Result<()> {
    // Get the directory where the executable is located
    let exe_path = std::env::current_exe()?;
    let exe_dir = exe_path.parent().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::NotFound,
            "Could not find executable directory",
        )
    })?;
    let log_path = exe_dir.join(log_filename);

    // Initialize the debug file in the same directory as the executable
    let debug_file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&log_path)?;

    // Set the debug file
    if let Ok(mut file_opt) = DEBUG_FILE.lock() {
        *file_opt = Some(debug_file);
    }

    // Enable debug logging
    if let Ok(mut enabled) = DEBUG_ENABLED.lock() {
        *enabled = true;
    }

    Ok(())
}
