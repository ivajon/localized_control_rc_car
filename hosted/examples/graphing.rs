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
#![feature(ascii_char)]

use rand::Rng;
use std::{
    collections::HashMap,
    error::Error,
    io::{self, Stdout},
    pin::pin,
    sync::{
        mpsc::{channel, Sender},
        Arc, RwLock,
    },
};

use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{
        self, disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
    },
};
use ratatui::{
    prelude::*,
    widgets::{block::Title, Axis, Block, Borders, Chart, Dataset, GraphType, Paragraph},
};
use tokio::{
    sync::{mpsc, Mutex},
    task::JoinHandle,
    time::{self, Duration, Instant},
};

#[derive(Clone)]
pub struct Graph {
    /// Time to leak some memory boys.
    values: HashMap<String, (Vec<(f64, f64)>, Style, f64, f64)>,
    // If this was not just for a lab we really should replace these with something better.
    datasets: Vec<Dataset<'static>>,
    widget: Chart<'static>,
}

#[derive(Clone)]
pub struct InputBox {
    input: String,
    cursor: usize,
    mode: bool,
    log: Vec<String>,
    widget: Paragraph<'static>,
}

pub type MeasurementReader = mpsc::Receiver<(String, (f64, f64))>;
pub type MeasurementWriter = mpsc::Sender<(String, (f64, f64))>;

impl Graph {
    fn init() -> (
        Arc<Mutex<Box<Self>>>,
        MeasurementWriter,
        Vec<tokio::task::JoinHandle<()>>,
    ) {
        let ret = Arc::new(Mutex::new(Box::new(Self {
            values: HashMap::new(),
            datasets: Vec::new(),
            widget: Chart::new(Vec::new()),
        })));

        let (sender_external, rec_external) = mpsc::channel(10);
        let (sender_internal, rec_internal) = mpsc::channel(10);

        (
            ret.clone(),
            sender_external,
            vec![
                tokio::spawn(Self::plot(ret.clone(), rec_internal)),
                tokio::spawn(Self::measurement_register(
                    ret.clone(),
                    rec_external,
                    sender_internal,
                )),
            ],
        )
    }

    fn draw<'a>(&self, frame: &mut Frame<'a>, area: Rect) {
        frame.render_widget(self.widget.clone(), area);
    }

    async fn plot(graph: Arc<Mutex<Box<Self>>>, mut reader: mpsc::Receiver<()>) {
        // Only re-draw the ui when ever a new measurement is registered.
        while let Some(_) = reader.recv().await {
            {
                let graph_inner = &mut graph.lock().await;

                let mut datasets = Vec::new();
                let mut max = -f64::INFINITY;
                let mut min = f64::INFINITY;
                graph_inner.values.clone().into_iter().for_each(
                    |(id, (data, style, inner_max, inner_min))| {
                        if max < inner_max {
                            max = inner_max;
                        }
                        if min > inner_min {
                            min = inner_min;
                        }
                        datasets.push(
                            Dataset::default()
                                .name(id.clone())
                                // This scares me.
                                // Not as much when we re-draw only on measurements
                                .data(data.leak())
                                .style(style.clone())
                                .marker(Marker::Dot)
                                .clone(),
                        )
                    },
                );
                // This is probably horrendously slow.
                let new_chart = Chart::new(datasets.clone())
                    .block(
                        Block::default()
                            .title(
                                Title::default()
                                    .content("Line chart".cyan().bold())
                                    .alignment(Alignment::Center),
                            )
                            .borders(Borders::ALL),
                    )
                    .x_axis(Axis::default().bounds([0., 1000.]).title("Time [uS]"))
                    .y_axis(
                        Axis::default()
                            .bounds([min, max])
                            .labels(vec![
                                format!("min : {min:.1}").into(),
                                format!("max : {max:.1}").into(),
                            ])
                            .title("Velocity [cm/s]"),
                    );
                graph_inner.widget = new_chart;
            }
        }
    }

    async fn measurement_register(
        graph: Arc<Mutex<Box<Self>>>,
        mut reader: MeasurementReader,
        sender: mpsc::Sender<()>,
    ) {
        let styles = vec![
            Style::default().green(),
            Style::default().yellow(),
            Style::default().blue(),
            Style::default().white(),
            Style::default().cyan(),
        ];
        let mut ptr = 0;

        while let Some((name, (x, y))) = reader.recv().await {
            let mut graph_inner = graph.lock().await;
            match graph_inner.values.get_mut(&name) {
                Some(data) => {
                    if data.2 < y {
                        data.2 = y;
                    }
                    if data.3 > y {
                        data.3 = y;
                    }
                    data.0.push((x, y));
                }
                None => {
                    let style = match styles.get(ptr) {
                        Some(style) => *style,
                        None => {
                            ptr = 0;
                            styles[0]
                        }
                    };
                    ptr += 1;
                    graph_inner.values.insert(name, (vec![(x, y)], style, y, y));
                }
            }

            match sender.send(()).await {
                Ok(_) => {}
                _ => break,
            }
        }
        println!("Message queue closed.")
    }
}

pub type ModeSwitchReader = mpsc::Receiver<bool>;
pub type ModeSwitchWriter = mpsc::Sender<bool>;
pub type NumReader = mpsc::Receiver<u8>;
pub type NumWriter = mpsc::Sender<u8>;
pub type BackSpaceReader = mpsc::Receiver<bool>;
pub type BackSpaceWriter = mpsc::Sender<bool>;
pub type EnterReader = mpsc::Receiver<()>;
pub type EnterWriter = mpsc::Sender<()>;
pub type QReader = mpsc::Receiver<()>;
pub type QWriter = mpsc::Sender<()>;

pub type CommitReader = mpsc::Receiver<f64>;
pub type CommitWriter = mpsc::Sender<f64>;
pub type KillReader = mpsc::Receiver<()>;
pub type KillWriter = mpsc::Sender<()>;

impl InputBox {
    fn init() -> (
        Arc<Mutex<Box<Self>>>,
        CommitReader,
        KillReader,
        Vec<tokio::task::JoinHandle<()>>,
    ) {
        let ret = Arc::new(Mutex::new(Box::new(Self {
            input: String::new(),
            cursor: 0,
            log: Vec::new(),
            mode: false,
            widget: Paragraph::new(Text::default()),
        })));
        let (mode_writer, mode_reader) = mpsc::channel(10);
        let (num_writer, num_reader) = mpsc::channel(10);
        let (backspace_writer, backspace_reader) = mpsc::channel(10);
        let (enter_writer, enter_reader) = mpsc::channel(10);
        let (q_writer, q_reader) = mpsc::channel(10);

        let (commit_writer, commit_reader) = mpsc::channel(10);
        let (kill_writer, kill_reader) = mpsc::channel(10);

        (
            ret.clone(),
            commit_reader,
            kill_reader,
            vec![
                tokio::spawn(Self::mode_switcher(ret.clone(), mode_reader)),
                tokio::spawn(Self::input(ret.clone(), num_reader)),
                tokio::spawn(Self::delete(ret.clone(), backspace_reader)),
                tokio::spawn(Self::commit(ret.clone(), enter_reader, commit_writer)),
                tokio::spawn(Self::killer(ret.clone(), q_reader, kill_writer)),
                tokio::spawn(Self::event_trampoline(
                    num_writer,
                    mode_writer,
                    enter_writer,
                    q_writer,
                    backspace_writer,
                )),
            ],
        )
    }

    fn draw<'a>(&self, frame: &mut Frame<'a>, area: Rect) {
        frame.render_widget(self.widget.clone(), area);
    }

    fn redraw(&mut self) {
        self.widget = Paragraph::new(self.input.clone()).yellow();
    }

    async fn mode_switcher(text_box: Arc<Mutex<Box<Self>>>, mut reader: ModeSwitchReader) {
        while let Some(mode) = reader.recv().await {
            let mut input_box = text_box.lock().await;
            input_box.mode = mode;
        }
    }

    async fn input(text_box: Arc<Mutex<Box<Self>>>, mut reader: NumReader) {
        while let Some(inp) = reader.recv().await {
            let mut locked_box = text_box.lock().await;
            if !locked_box.mode {
                continue;
            }

            let c = match char::from_digit(inp as u32, 10) {
                Some(c) => c,
                None => continue,
            };
            locked_box.input.push(c);
            locked_box.redraw();
        }
    }

    async fn delete(text_box: Arc<Mutex<Box<Self>>>, mut reader: BackSpaceReader) {
        while let Some(inp) = reader.recv().await {
            let mut locked_box = text_box.lock().await;
            if !locked_box.mode {
                continue;
            }
            match inp {
                true => {
                    while locked_box
                        .input
                        .chars()
                        .last()
                        .is_some_and(|char| char.is_whitespace())
                    {
                        locked_box.input.pop();
                    }
                }
                false => {
                    locked_box.input.pop();
                }
            };
            locked_box.redraw();
        }
    }

    async fn commit(
        text_box: Arc<Mutex<Box<Self>>>,
        mut reader: EnterReader,
        commit_channel: CommitWriter,
    ) {
        while let Some(_) = reader.recv().await {
            let mut locked_box = text_box.lock().await;

            let number: u32 = match locked_box.input.parse() {
                Ok(val) => val,
                Err(_) => continue,
            };

            locked_box.input = String::new();

            commit_channel.send(number as f64).await.unwrap();
            locked_box.redraw();
        }
    }

    async fn killer(
        text_box: Arc<Mutex<Box<Self>>>,
        mut reader: QReader,
        kill_channel: KillWriter,
    ) {
        while let Some(_) = reader.recv().await {
            let locked_box = text_box.lock().await;
            if !locked_box.mode {
                kill_channel.send(()).await.unwrap();
                return;
            }
        }
    }

    async fn event_trampoline(
        char_writer: NumWriter,
        mode_writer: ModeSwitchWriter,
        commit_channel: EnterWriter,
        q_channel: QWriter,
        backspace_channel: BackSpaceWriter,
    ) {
        let mut reader = pin!(crossterm::event::EventStream::new());
        while let Some(event) = reader.as_mut().next().await {
            if event.is_err() {
                continue;
            }

            let event: Event = event.unwrap();

            match event {
                Event::Key(key) => {
                    match key.code {
                        KeyCode::Char('q') => {
                            q_channel.send(()).await.unwrap();
                        }
                        KeyCode::Char('i') => {
                            mode_writer.send(true).await.unwrap();
                        }
                        KeyCode::Char(char) => {
                            let ascii = match char.as_ascii() {
                                Some(v) => v,
                                None => continue,
                            }
                            .to_u8();
                            if ascii >= 48 && ascii < 58 {
                                char_writer.send(ascii - 48).await.unwrap();
                            }
                        }
                        KeyCode::Enter => {
                            // Commit.
                            commit_channel.send(()).await.unwrap();
                        }

                        KeyCode::Backspace => {
                            // Allow word deletion.
                            backspace_channel
                                .send(key.modifiers.contains(KeyModifiers::CONTROL))
                                .await
                                .unwrap();
                        }

                        KeyCode::Esc => {
                            // Revert to normal mode
                            mode_writer.send(false).await.unwrap();
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}

pub struct MockSpi {
    target_value: f64,
    last_measured: f64,
    time_step: f64,
}

impl MockSpi {
    fn init(
        measurement_writer: MeasurementWriter,
        reference_reader: CommitReader,
    ) -> Vec<JoinHandle<()>> {
        let ret = Arc::new(Mutex::new(MockSpi {
            target_value: 0.,
            last_measured: 0.,
            time_step: 0.,
        }));
        vec![
            tokio::spawn(Self::bogus_measurement(ret.clone(), measurement_writer)),
            tokio::spawn(Self::set_reference(ret.clone(), reference_reader)),
        ]
    }

    async fn bogus_measurement(spi: Arc<Mutex<Self>>, measurement_writer: MeasurementWriter) {
        loop {
            let (target, mut measured, time_step) = {
                let mut spi = spi.lock().await;
                spi.time_step += 1.;
                let (target, measured, time_step) =
                    (spi.target_value, spi.last_measured, spi.time_step + 1.);
                (target, measured, time_step)
            };
            if target > measured {
                measured += 2.;
            } else if target < measured {
                measured -= 2.;
            } else {
                let rng = rand::thread_rng().gen_range((-1.)..(1.));
                measured += rng;
            }
            {
                let mut spi = spi.lock().await;
                spi.last_measured = measured;
            }

            measurement_writer
                .send(("measured".to_string(), (time_step, measured)))
                .await
                .unwrap();
            measurement_writer
                .send(("target".to_string(), (time_step, target)))
                .await
                .unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    async fn set_reference(spi: Arc<Mutex<Self>>, mut refference_reader: CommitReader) {
        while let Some(value) = refference_reader.recv().await {
            let mut spi = spi.lock().await;
            spi.target_value = value;
        }
    }
}

async fn thread_manager(
    threads: Vec<JoinHandle<()>>,
    mut kill_command: KillReader,
    frontend_killer: mpsc::Sender<()>,
    mut killer_sync: mpsc::Receiver<()>,
) {
    while let Some(_) = kill_command.recv().await {
        frontend_killer.send(()).await.unwrap();
        killer_sync.recv().await.unwrap();
        time::sleep(Duration::from_millis(500)).await;
        threads.into_iter().for_each(|handle| {
            let _ = handle.abort();
        });
        return;
    }
}
#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup terminal

    let (terminal, chart_area, input_area) = initiate_terminal();

    let terminal = Arc::new(Mutex::new(terminal));
    // create app and run it
    let tick_rate = Duration::from_millis(250);

    // Spawn all of the tasks
    let (graph, register_channel, graph_handles) = Graph::init();
    let (input, commit, kill, input_handles) = InputBox::init();
    let mock_spi_channels: Vec<JoinHandle<()>> = MockSpi::init(register_channel, commit);

    let (frontend_killer, reader) = mpsc::channel(1);
    let (killer_ack, killer_sync) = mpsc::channel(1);
    // Spawn a task that manages killing all tasks
    let mut handles = vec![tokio::spawn(run_frontend(
        terminal.clone(),
        chart_area,
        input_area,
        graph,
        input,
        tick_rate,
        reader,
        killer_ack,
    ))];
    graph_handles.into_iter().for_each(|el| handles.push(el));
    mock_spi_channels
        .into_iter()
        .for_each(|el| handles.push(el));
    input_handles.into_iter().for_each(|el| handles.push(el));
    let kill_handle = tokio::spawn(thread_manager(handles, kill, frontend_killer, killer_sync));

    // Block until all threads exit

    while !kill_handle.is_finished() {}

    let mut terminal = terminal.lock().await;
    restore_terminal(&mut terminal);

    Ok(())
}

fn initiate_terminal() -> (Terminal<CrosstermBackend<Stdout>>, Rect, Rect) {
    enable_raw_mode().unwrap();
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture).unwrap();
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend).unwrap();
    let frame = terminal.get_frame();
    let area = frame.size();

    let vertical = Layout::vertical([Constraint::Percentage(70), Constraint::Percentage(30)]);

    let [chart, text_area] = vertical.areas(area);
    (terminal, chart, text_area)
}

fn restore_terminal(terminal: &mut Terminal<CrosstermBackend<Stdout>>) {
    // restore terminal
    let _ = disable_raw_mode();
    let _ = execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    );
    let _ = terminal.show_cursor();
}

async fn run_frontend<'a, B: Backend>(
    terminal: Arc<Mutex<Terminal<B>>>,
    _chart_area: Rect,
    _input_area: Rect,
    graph: Arc<Mutex<Box<Graph>>>,
    input: Arc<Mutex<Box<InputBox>>>,
    tick_rate: Duration,
    mut reader: mpsc::Receiver<()>,
    killer_ack: mpsc::Sender<()>,
) {
    let mut last_tick = Instant::now();
    while let Err(_) = reader.try_recv() {
        {
            let mut terminal = terminal.lock().await;
            let graph = graph.lock().await;
            let input = input.lock().await;

            match terminal.draw(|frame| {
                let vertical =
                    Layout::vertical([Constraint::Percentage(70), Constraint::Percentage(30)]);

                let size = frame.size();
                if size.height != 0 {
                    let [chart, text_area] = vertical.areas(frame.size());
                    graph.draw(frame, chart);
                    input.draw(frame, text_area);
                }
            }) {
                Err(_) => {
                    killer_ack.send(()).await.unwrap();
                    return;
                }
                Ok(_) => {}
            };
        }
        tokio::time::sleep_until(last_tick + tick_rate).await;
        last_tick = Instant::now();
    }
    killer_ack.send(()).await.unwrap();
}
