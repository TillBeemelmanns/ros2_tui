use crossterm::terminal::{disable_raw_mode, LeaveAlternateScreen};
use crossterm::ExecutableCommand;

/// Restore the terminal before the default panic message is printed.
///
/// Without this, a panic leaves the terminal in raw mode on the alternate
/// screen, hiding the panic message and leaving the shell unusable.
pub fn install_panic_hook() {
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = std::io::stdout().execute(LeaveAlternateScreen);
        default_hook(info);
    }));
}
