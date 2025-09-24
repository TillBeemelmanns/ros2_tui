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
use ros2_tui::params::*;

fn main() -> io::Result<()> {
    let matches = Command::new("params")
        .version("0.1.3")
        .author("Till Beemelmanns")
        .about("A TUI for managing ROS2 parameters")
        .arg(
            Arg::new("refresh")
                .long("refresh")
                .value_name("SECONDS")
                .help("Refresh rate for parameter list in seconds")
                .default_value("5"),
        )
        .arg(
            Arg::new("no-initial-fetch")
                .long("no-initial-fetch")
                .help("Skip the initial parameter list fetch (for debugging)")
                .action(clap::ArgAction::SetTrue),
        )
        .arg(
            Arg::new("verbose")
                .short('v')
                .long("verbose")
                .help("Enable debug logging to params_debug.log (written in executable directory)")
                .action(clap::ArgAction::SetTrue),
        )
        .get_matches();

    let refresh_rate: u64 = matches
        .get_one::<String>("refresh")
        .unwrap()
        .parse()
        .unwrap_or(5); // Default to 5 seconds

    let _skip_initial_fetch = matches.get_flag("no-initial-fetch");
    let verbose = matches.get_flag("verbose");

    // Enable debug logging if verbose flag is set
    if verbose {
        enable_debug_logging("params_debug.log")?;
        debug_log("=== PARAMS STARTING ===");
    }

    debug_log(&format!("Configuration: refresh_rate={}s", refresh_rate));

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

    debug_log("Creating ParamsApp instance...");
    let mut app = ParamsApp::new(Duration::from_secs(refresh_rate));
    debug_log("ParamsApp created successfully");

    let result = run_app(&mut terminal, &mut app);

    // restore terminal
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}

fn run_app<B: Backend>(terminal: &mut Terminal<B>, app: &mut ParamsApp) -> io::Result<()> {
    loop {
        // Process any pending messages from worker threads
        app.try_receive_messages();

        // Parameters are loaded automatically via YAML dump - no animation needed

        // Draw the UI
        terminal.draw(|f| ui(f, app))?;

        // Handle events with timeout
        if event::poll(Duration::from_millis(50))? {
            // ~20 FPS - very fast for animation
            match event::read()? {
                Event::Key(key) => {
                    match key.code {
                        KeyCode::Char('q') => app.on_key('q'),
                        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                            app.should_quit = true
                        }
                        KeyCode::Char('c')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.toggle_collapse_all()
                        }
                        KeyCode::Up
                            if app.mode == AppMode::ParamList || app.mode == AppMode::Search =>
                        {
                            app.on_up()
                        }
                        KeyCode::Down
                            if app.mode == AppMode::ParamList || app.mode == AppMode::Search =>
                        {
                            app.on_down()
                        }
                        KeyCode::Left if app.mode == AppMode::ParamList => app.on_left(),
                        KeyCode::Right if app.mode == AppMode::ParamList => app.on_right(),
                        KeyCode::Left if app.mode == AppMode::SetParameter => {
                            app.move_edit_cursor_left()
                        }
                        KeyCode::Right if app.mode == AppMode::SetParameter => {
                            app.move_edit_cursor_right()
                        }
                        KeyCode::Left
                            if app.mode == AppMode::DumpParameters
                                || app.mode == AppMode::LoadParameters =>
                        {
                            app.move_file_cursor_left()
                        }
                        KeyCode::Right
                            if app.mode == AppMode::DumpParameters
                                || app.mode == AppMode::LoadParameters =>
                        {
                            app.move_file_cursor_right()
                        }
                        KeyCode::Char('k')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_up()
                        }
                        KeyCode::Char('j')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_down()
                        }
                        KeyCode::Char('h')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_left()
                        }
                        KeyCode::Char('l')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_right()
                        }
                        KeyCode::Enter => match app.mode {
                            AppMode::SetParameter => app.confirm_set_parameter(),
                            AppMode::DumpParameters => app.confirm_dump_parameters(),
                            AppMode::LoadParameters => app.confirm_load_parameters(),
                            _ => app.on_enter(),
                        },
                        KeyCode::Tab if app.mode == AppMode::ParamList => app.on_tab(),
                        KeyCode::Char(' ') if app.mode == AppMode::ParamList => app.on_space(),
                        KeyCode::Char('r') | KeyCode::F(5) if app.mode == AppMode::ParamList => {
                            app.on_key('r')
                        }
                        KeyCode::Char('e')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.toggle_expand_all()
                        }
                        KeyCode::F(4) => app.on_f4(),
                        KeyCode::Esc => app.on_escape(),
                        KeyCode::Backspace => app.on_backspace(),
                        KeyCode::Char('s')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_set_parameter()
                        }
                        KeyCode::Char('d')
                            if !app.is_in_search_mode() && app.mode == AppMode::ParamList =>
                        {
                            app.on_dump_parameters()
                        }
                        KeyCode::Char('l')
                            if key.modifiers.contains(KeyModifiers::CONTROL)
                                && app.mode == AppMode::ParamList =>
                        {
                            app.on_load_parameters()
                        }
                        _ if app.mode == AppMode::Warning => app.on_escape(), // Any key dismisses warning
                        KeyCode::Char(c) => app.on_key(c),
                        _ => {}
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
