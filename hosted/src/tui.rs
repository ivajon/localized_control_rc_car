//! Defines a few TUI wrappers that can be useful for things like plotting.

use std::io::{self, Stdout};

use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, CompletedFrame, Frame, Terminal};

pub mod graph;
pub mod input_box;

pub struct TerminalWrapper {
    term: Option<Terminal<CrosstermBackend<Stdout>>>,
}

impl TerminalWrapper {
    fn new(term: Terminal<CrosstermBackend<Stdout>>) -> Self {
        Self { term: Some(term) }
    }

    pub fn draw<F>(&mut self, frame: F) -> io::Result<CompletedFrame>
    where
        F: FnOnce(&mut Frame),
    {
        unsafe { self.term.as_mut().unwrap_unchecked().draw(frame) }
    }
}

impl Drop for TerminalWrapper {
    fn drop(&mut self) {
        // Should be checked by the type-system.
        let mut term = unsafe { self.term.take().unwrap_unchecked() };
        // restore terminal
        let _ = disable_raw_mode();
        let _ = execute!(
            term.backend_mut(),
            LeaveAlternateScreen,
            DisableMouseCapture
        );
        let _ = term.show_cursor();
    }
}

/// Initiates the terminal.
#[must_use]
pub fn initiate_terminal() -> io::Result<TerminalWrapper> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    Ok(TerminalWrapper::new(Terminal::new(backend)?))
}
