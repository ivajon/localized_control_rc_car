//! # [Ratatui] Chart example
//!
//! The latest version of this example is available in the [examples] folder in the repository.
//!
//! Please note that the examples are designed to be run against the `main` branch of the Github
//! repository. This means that you may not be able to compile with the latest release version on
//! crates.io, or the one that you have installed locally.
//!
//! See the [examples readme] for more information on finding examples that match the version of the
//! library you are using.
//!
//! [Ratatui]: https://github.com/ratatui-org/ratatui
//! [examples]: https://github.com/ratatui-org/ratatui/blob/main/examples
//! [examples readme]: https://github.com/ratatui-org/ratatui/blob/main/examples/README.md

use std::{
    error::Error,
    io,
    time::{Duration, Instant},
};

use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    prelude::*,
    widgets::{block::Title, Axis, Block, Borders, Chart, Dataset, GraphType, LegendPosition},
};

impl Iterator for SinSignal {
    type Item = (f64, f64);
    fn next(&mut self) -> Option<Self::Item> {
        let point = (self.x, (self.x * 1.0 / self.period).sin() * self.scale);
        self.x += self.interval;
        Some(point)
    }
}

enum InputMode {
    Normal,
    Editing,
}

struct App {}

impl App {
    fn new() -> Self {
        todo!()
    }
    fn on_tick(&mut self) {}

    fn move_cursor_left(&mut self) {}

    fn move_cursor_right(&mut self) {}

    fn enter_char(&mut self, new_char: char) {}

    fn delete_char(&mut self) {}

    fn clamp_cursor(&self, new_cursor_pos: usize) -> usize {
        todo!()
    }

    fn reset_cursor(&mut self) {}

    fn submit_message(&mut self) {}
}

fn main() -> Result<(), Box<dyn Error>> {
    // setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // create app and run it
    let tick_rate = Duration::from_millis(250);
    let app = App::new();
    let res = run_app(&mut terminal, app, tick_rate);

    // restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    if let Err(err) = res {
        println!("{err:?}");
    }

    Ok(())
}

fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    mut app: App,
    tick_rate: Duration,
) -> io::Result<()> {
    let mut last_tick = Instant::now();
    loop {
        terminal.draw(|f| ui(f, &app))?;

        let timeout = tick_rate.saturating_sub(last_tick.elapsed());
        if crossterm::event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char('q') {
                    return Ok(());
                }
            }
        }
        if last_tick.elapsed() >= tick_rate {
            app.on_tick();
            last_tick = Instant::now();
        }
    }
}

// Look in to https://github.com/ratatui-org/ratatui/blob/main/examples/user_input.rs
fn ui(frame: &mut Frame, app: &App) {
    let area = frame.size();

    let vertical = Layout::vertical([Constraint::Percentage(70), Constraint::Percentage(30)]);
    let [chart1, bottom] = vertical.areas(area);
    // let [line_chart, scatter] = horizontal.areas(bottom);

    render_chart1(frame, chart1, app);
}

fn render_input(f: &mut Frame, area: Rect, app: &App) {}

fn render_chart1(f: &mut Frame, area: Rect, app: &App) {
    let x_labels = vec![
        Span::styled(
            format!("{}", app.window[0]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{}", (app.window[0] + app.window[1]) / 2.0)),
        Span::styled(
            format!("{}", app.window[1]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];
    let datasets = vec![
        Dataset::default()
            .name("Measured")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Cyan))
            .data(&app.data1),
        Dataset::default()
            .name("Refference")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Yellow))
            .data(&app.data2),
    ];

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title("Chart 1".cyan().bold())
                .borders(Borders::ALL),
        )
        .x_axis(
            Axis::default()
                .title("[s]")
                .style(Style::default().fg(Color::Gray))
                .labels(x_labels)
                .bounds(app.window),
        )
        .y_axis(
            Axis::default()
                .title("[cm/s]")
                .style(Style::default().fg(Color::Gray))
                .labels(vec!["-20".bold(), "0".into(), "20".bold()])
                .bounds([-20.0, 20.0]),
        );

    f.render_widget(chart, area);
}
