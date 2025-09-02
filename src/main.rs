use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use crossterm::ExecutableCommand;
use ratatui::backend::{CrosstermBackend, Backend};
use ratatui::Terminal;
use std::io::{self, stdout, Write};
use std::time::Duration;
use clap::{Command, Arg};
use std::fs::OpenOptions;
use std::sync::Mutex;

mod app;
mod ros;
mod ui;
mod topic_watcher;

use crate::app::App;

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

pub fn enable_debug_logging() -> io::Result<()> {
    // Get the directory where the executable is located
    let exe_path = std::env::current_exe()?;
    let exe_dir = exe_path.parent().ok_or_else(|| {
        io::Error::new(io::ErrorKind::NotFound, "Could not find executable directory")
    })?;
    let log_path = exe_dir.join("toptop_debug.log");
    
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

fn main() -> io::Result<()> {
    let matches = Command::new("toptop")
        .version("0.1.0")
        .author("Till Beemelmanns")
        .about("A TUI for monitoring ROS2 topics")
        .arg(
            Arg::new("refresh")
                .long("refresh")
                .value_name("SECONDS")
                .help("Refresh rate for topic list in seconds")
                .default_value("5")
        )
        .arg(
            Arg::new("detail-refresh")
                .long("detail-refresh")
                .value_name("SECONDS")
                .help("Refresh rate for topic details in seconds")
                .default_value("30")
        )
        .arg(
            Arg::new("no-initial-fetch")
                .long("no-initial-fetch")
                .help("Skip the initial topic list fetch (for debugging)")
                .action(clap::ArgAction::SetTrue)
        )
        .arg(
            Arg::new("verbose")
                .short('v')
                .long("verbose")
                .help("Enable debug logging to toptop_debug.log (written in executable directory)")
                .action(clap::ArgAction::SetTrue)
        )
        .get_matches();

    let refresh_rate: u64 = matches.get_one::<String>("refresh")
        .unwrap()
        .parse()
        .unwrap_or(5);  // Default to 5 seconds
    
    let detail_refresh_rate: u64 = matches.get_one::<String>("detail-refresh")
        .unwrap()
        .parse()
        .unwrap_or(30);
    
    let _skip_initial_fetch = matches.get_flag("no-initial-fetch");
    let verbose = matches.get_flag("verbose");
    
    // Enable debug logging if verbose flag is set
    if verbose {
        enable_debug_logging()?;
        debug_log("=== TOPTOP STARTING ===");
    }

    debug_log(&format!("Configuration: refresh_rate={}s, detail_refresh_rate={}s", refresh_rate, detail_refresh_rate));

    // Check if ros2 is available
    debug_log("Checking if ros2 command is available...");
    if let Err(e) = std::process::Command::new("ros2")
        .arg("--help")
        .output()
    {
        debug_log(&format!("Error: ros2 command not found: {:?}", e));
        eprintln!("Error: ros2 command not found. Please ensure ROS2 is installed and sourced.");
        std::process::exit(1);
    }
    debug_log("ros2 command is available");

    // setup terminal
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout()))?;

    debug_log("Creating App instance...");
    let mut app = App::new(
        Duration::from_secs(refresh_rate),
        Duration::from_secs(detail_refresh_rate)
    );
    debug_log("App created successfully");
    
    let result = run_app(&mut terminal, &mut app);

    // restore terminal
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;
    
    result
}

fn run_app<B: Backend>(terminal: &mut Terminal<B>, app: &mut App) -> io::Result<()> {
    loop {
        // Process any pending messages from worker threads
        app.try_receive_messages();
        
        // Update loading animation every frame (about 200ms at 200ms polling) for very fast blinking
        app.update_loading_animation();
        
        // Draw the UI
        terminal.draw(|f| ui::ui(f, app))?;

        // Handle events with timeout
        if event::poll(Duration::from_millis(200))? { // ~5 FPS - much more efficient
            match event::read()? {
                Event::Key(key) => {
                    match handle_key_event(key) {
                        Some(AppAction::Quit) => {
                            return Ok(()); // Exit immediately
                        }
                        Some(AppAction::MoveUp) => {
                            app.select_previous();
                        }
                        Some(AppAction::MoveDown) => {
                            app.select_next();
                        }
                        Some(AppAction::Refresh) => {
                            app.on_key('r');
                        }
                        Some(AppAction::ToggleWatch) => {
                            debug_log("User pressed toggle watch");
                            app.toggle_watch_current_topic();
                        }
                        None => {}
                    }
                }
                Event::Resize(_, _) => {
                    // Handle terminal resize
                }
                _ => {}
            }
        }

        if app.should_quit {
            debug_log("Application quitting - shutting down background tasks");
            app.shutdown();
            return Ok(());
        }
    }
}

enum AppAction {
    Quit,
    MoveUp,
    MoveDown,
    Refresh,
    ToggleWatch,
}

fn handle_key_event(key: KeyEvent) -> Option<AppAction> {
    match key.code {
        KeyCode::Char('q') => Some(AppAction::Quit),
        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => Some(AppAction::Quit),
        KeyCode::Up | KeyCode::Char('k') => Some(AppAction::MoveUp),
        KeyCode::Down | KeyCode::Char('j') => Some(AppAction::MoveDown),
        KeyCode::Char('r') | KeyCode::F(5) => Some(AppAction::Refresh),
        KeyCode::Enter | KeyCode::Char(' ') => Some(AppAction::ToggleWatch),
        KeyCode::Esc => Some(AppAction::Quit),
        _ => None,
    }
}