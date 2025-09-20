use clap::{Arg, Command};
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use crossterm::ExecutableCommand;
use ratatui::backend::{Backend, CrosstermBackend};
use ratatui::Terminal;
use std::io::{self, stdout};
use std::time::Duration;

use ros2_tui::common::*;
use ros2_tui::topics::*;

fn main() -> io::Result<()> {
    let matches = Command::new("topics")
        .version("0.1.0")
        .author("Till Beemelmanns")
        .about("A TUI for monitoring ROS2 topics")
        .arg(
            Arg::new("refresh")
                .long("refresh")
                .value_name("SECONDS")
                .help("Refresh rate for topic list in seconds")
                .default_value("5"),
        )
        .arg(
            Arg::new("detail-refresh")
                .long("detail-refresh")
                .value_name("SECONDS")
                .help("Refresh rate for topic details in seconds")
                .default_value("30"),
        )
        .arg(
            Arg::new("no-initial-fetch")
                .long("no-initial-fetch")
                .help("Skip the initial topic list fetch (for debugging)")
                .action(clap::ArgAction::SetTrue),
        )
        .arg(
            Arg::new("verbose")
                .short('v')
                .long("verbose")
                .help("Enable debug logging to topics_debug.log (written in executable directory)")
                .action(clap::ArgAction::SetTrue),
        )
        .get_matches();

    let refresh_rate: u64 = matches
        .get_one::<String>("refresh")
        .unwrap()
        .parse()
        .unwrap_or(5); // Default to 5 seconds

    let detail_refresh_rate: u64 = matches
        .get_one::<String>("detail-refresh")
        .unwrap()
        .parse()
        .unwrap_or(30);

    let _skip_initial_fetch = matches.get_flag("no-initial-fetch");
    let verbose = matches.get_flag("verbose");

    // Enable debug logging if verbose flag is set
    if verbose {
        enable_debug_logging("topics_debug.log")?;
        debug_log("=== TOPICS STARTING ===");
    }

    debug_log(&format!(
        "Configuration: refresh_rate={}s, detail_refresh_rate={}s",
        refresh_rate, detail_refresh_rate
    ));

    // Check if ros2 is available
    debug_log("Checking if ros2 command is available...");
    if let Err(e) = std::process::Command::new("ros2").arg("--help").output() {
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
        Duration::from_secs(detail_refresh_rate),
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
        terminal.draw(|f| ui(f, app))?;

        // Handle events with timeout
        if event::poll(Duration::from_millis(50))? {
            // ~20 FPS - very fast for animation
            match event::read()? {
                Event::Key(key) => match key.code {
                    KeyCode::Char('q') => app.on_key('q'),
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        app.should_quit = true
                    }
                    KeyCode::Char('c') if !app.is_in_search_mode() => app.toggle_collapse_all(),
                    KeyCode::Up => app.on_up(),
                    KeyCode::Down => app.on_down(),
                    KeyCode::Left => app.on_left(),
                    KeyCode::Right => app.on_right(),
                    KeyCode::Char('k') if !app.is_in_search_mode() => app.on_up(),
                    KeyCode::Char('j') if !app.is_in_search_mode() => app.on_down(),
                    KeyCode::Char('h') if !app.is_in_search_mode() => app.on_left(),
                    KeyCode::Char('l') if !app.is_in_search_mode() => app.on_right(),
                    KeyCode::Enter => app.on_enter(),
                    KeyCode::Tab => app.on_tab(),
                    KeyCode::Char(' ') => app.on_space(),
                    KeyCode::Char('r') | KeyCode::F(5) => app.on_key('r'),
                    KeyCode::Char('e') if !app.is_in_search_mode() => app.toggle_echo(),
                    KeyCode::PageUp => app.echo_page_up(),
                    KeyCode::PageDown => app.echo_page_down(),
                    KeyCode::Home => app.echo_home(),
                    KeyCode::End => app.echo_end(),
                    KeyCode::F(4) => app.on_f4(),
                    KeyCode::Esc => app.on_escape(),
                    KeyCode::Backspace => app.on_backspace(),
                    KeyCode::Char(c) => app.on_key(c),
                    _ => {}
                },
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
