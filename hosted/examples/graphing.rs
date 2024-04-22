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

use linux_embedded_hal::{spidev::{SpiModeFlags, Spidev}, SpidevDevice};
use rand::Rng;
use shared::protocol::{
    v0_0_1::{Payload, V0_0_1},
    Message, Parse,
};
use std::{
    collections::HashMap,
    error::Error,
    io::{self, Read, Stdout, Write},
    pin::pin,
    sync::Arc,
};

use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    prelude::*,
    widgets::{block::Title, Axis, Block, Borders, Chart, Dataset, Paragraph},
};
use tokio::{
    sync::{mpsc, Mutex},
    task::JoinHandle,
    time::Duration,
};

macro_rules! unwrap_or_break {
    ($tokens:expr) => {
        match $tokens {
            Ok(_) => {}
            _ => break,
        }
    };
}

impl MockSpi {
    fn init(
        measurement_writer: MeasurementWriter,
        reference_reader: CommitReader,
    ) -> Option<Vec<JoinHandle<()>>> {
        let spi = Spi::init("/dev/spidev1.0")?;

        let ret = Arc::new(Mutex::new(MockSpi {
            target_value: 0.,
            spi,
        }));
        Some(vec![
            tokio::spawn(Self::measurement(ret.clone(), measurement_writer)),
            tokio::spawn(Self::set_reference(ret.clone(), reference_reader)),
        ])
    }

    async fn measurement(spi: Arc<Mutex<Self>>, measurement_writer: MeasurementWriter) {
        loop {
            let mut spi = spi.lock().await;
            // Read a command, if it does not work, discard it and re-try.
            let read = match spi.spi.read() {
                Ok(read) => read,
                Err(_) => {
                    // Wait for a longer time before polling as there is no new data to be read..
                    tokio::time::sleep(Duration::from_millis(500)).await;
                    continue;
                }
            };
            for el in read.into_iter() {
                if let Payload::CurrentVelocity { velocity, time_us } = el {
                    unwrap_or_break!(
                        measurement_writer
                            .send(("measured".to_string(), (time_us as f64, velocity as f64)))
                            .await
                    );
                    unwrap_or_break!(
                        measurement_writer
                            .send(("target".to_string(), (time_us as f64, spi.target_value)))
                            .await
                    );
                }
            }

            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    async fn set_reference(spi: Arc<Mutex<Self>>, mut reference_reader: CommitReader) {
        while let Some(value) = reference_reader.recv().await {
            let mut spi = spi.lock().await;
            spi.target_value = value;
            match spi.spi.write(Payload::SetSpeed {
                velocity: value as u32,
                hold_for_us: 0,
            }) {
                Ok(_) => {}
                _ => break,
            }
        }
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

pub type InputConstructor = (
    Arc<Mutex<Box<InputBox>>>,
    CommitReader,
    KillReader,
    Vec<tokio::task::JoinHandle<()>>,
);

impl InputBox {
    fn init(redraw_writer: RedrawWriter) -> InputConstructor {
        let ret = Arc::new(Mutex::new(Box::new(Self {
            input: String::new(),
            log: Vec::new(),
            mode: false,
            widget: Paragraph::new("> ".to_string()).yellow().block(
                Block::default()
                    .title(
                        Title::default()
                            .content("Input box".cyan().bold())
                            .alignment(Alignment::Center),
                    )
                    .borders(Borders::ALL)
                    .gray(),
            ),
            redraw_writer,
        })));
        let (mode_writer, mode_reader) = mpsc::channel(1024);
        let (num_writer, num_reader) = mpsc::channel(1024);
        let (backspace_writer, backspace_reader) = mpsc::channel(1024);
        let (enter_writer, enter_reader) = mpsc::channel(1024);
        let (q_writer, q_reader) = mpsc::channel(1024);

        let (commit_writer, commit_reader) = mpsc::channel(1024);
        let (kill_writer, kill_reader) = mpsc::channel(1024);

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

    fn widget(&self) -> impl Widget {
        self.widget.clone()
    }

    fn redraw(&mut self) {
        self.widget = Paragraph::new(format!(
            "> {}\n{} [cm/s]",
            self.input,
            self.log.join(" [cm/s]\n")
        ))
        .gray()
        .block(
            Block::default()
                .title(
                    Title::default()
                        .content("Input box".cyan().bold())
                        .alignment(Alignment::Center),
                )
                .borders(Borders::ALL)
                .gray(),
        );

        if self.mode {
            self.widget = self
                .widget
                .clone()
                .block(
                    Block::default()
                        .title(
                            Title::default()
                                .content("Input box".cyan().bold())
                                .alignment(Alignment::Center),
                        )
                        .borders(Borders::ALL)
                        .yellow()
                        .slow_blink(),
                )
                .yellow();
        }
    }

    async fn mode_switcher(text_box: Arc<Mutex<Box<Self>>>, mut reader: ModeSwitchReader) {
        while let Some(mode) = reader.recv().await {
            let mut input_box = text_box.lock().await;
            input_box.mode = mode;
            input_box.redraw();
            input_box.redraw_writer.send(()).await.unwrap();
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
            locked_box.redraw_writer.send(()).await.unwrap();
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
            locked_box.redraw_writer.send(()).await.unwrap();
        }
    }

    async fn commit(
        text_box: Arc<Mutex<Box<Self>>>,
        mut reader: EnterReader,
        commit_channel: CommitWriter,
    ) {
        while reader.recv().await.is_some() {
            let mut locked_box = text_box.lock().await;

            let number: u32 = match locked_box.input.parse() {
                Ok(val) => val,
                Err(_) => continue,
            };

            let to_append = locked_box.input.clone();

            locked_box.log.insert(0, to_append);
            locked_box.input = String::new();

            commit_channel.send(number as f64).await.unwrap();
            locked_box.redraw();
            locked_box.redraw_writer.send(()).await.unwrap();
        }
    }

    async fn killer(
        text_box: Arc<Mutex<Box<Self>>>,
        mut reader: QReader,
        kill_channel: KillWriter,
    ) {
        while reader.recv().await.is_some() {
            let locked_box = text_box.lock().await;
            if !locked_box.mode {
                kill_channel.send(()).await.unwrap();

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

            if let Event::Key(key) = event {
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
                        if (48..58).contains(&ascii) {
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
        }
    }
}

pub struct MockSpi {
    target_value: f64,
    last_measured: f64,
    time_step: f64,
    spi: Spidev,
}

impl MockSpi {
    fn init(
        measurement_writer: MeasurementWriter,
        reference_reader: CommitReader,
    ) -> Vec<JoinHandle<()>> {
        let spi = linux_embedded_hal::spidev::Spidev::open("/dev/spidev0.0").unwrap();

        let ret = Arc::new(Mutex::new(MockSpi {
            target_value: 0.,
            last_measured: 0.,
            time_step: 0.,
            spi,
        }));
        vec![
            tokio::spawn(Self::bogus_measurement(ret.clone(), measurement_writer)),
            tokio::spawn(Self::set_reference(ret.clone(), reference_reader)),
        ]
    }

    async fn bogus_measurement(spi: Arc<Mutex<Self>>, measurement_writer: MeasurementWriter) {
        loop {
            // let (target, mut measured, time_step) = {
            //     let mut spi = spi.lock().await;
            //     spi.time_step += 1.;
            //     let (target, measured, time_step) =
            //         (spi.target_value, spi.last_measured, spi.time_step + 1.);
            //     (target, measured, time_step)
            // };
            // if target > measured {
            //     measured += 2.;
            // } else if target < measured {
            //     measured -= 2.;
            // } else {
            //     let rng = rand::thread_rng().gen_range((-1.)..1.);
            //     measured += rng;
            // }
            // {
            //     let mut spi = spi.lock().await;
            //     spi.last_measured = measured;
            // }
            let (target, measured, time_step) = {
                let mut spi = spi.lock().await;
                let mut buf = Vec::new();
                let read = spi.spi.read(buf.as_mut_slice());
                match shared::protocol::Message::<V0_0_1>::try_parse(&mut buf.into_iter()) {
                    Some(message) => {
                        match message.payload() {
                            Payload::CurrentVelocity { velocity, time_us } => {
                        (spi.target_value, velocity as f64, time_us as f64)
                    }
                            _ => {
                        tokio::time::sleep(Duration::from_millis(500)).await;
                        continue;

                            }
                        }
                    }
                    _ => {
                        tokio::time::sleep(Duration::from_millis(500)).await;
                        continue;
                    }
                }
            };

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
            let mut target: Vec<u8> = Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: value as u32,
                hold_for_us: 0,
            })
            .collect();
            spi.spi.write(target.as_slice());
        }
    }
}

async fn thread_manager(
    threads: Vec<JoinHandle<()>>,
    mut kill_command: KillReader,
    frontend_killer: mpsc::Sender<()>,
) {
    let _ = kill_command.recv().await;

    frontend_killer.send(()).await.unwrap();
    threads.into_iter().for_each(|handle| {
        handle.abort();
    });
}
#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // console_subscriber::init();

    start_app().await;
    Ok(())
}

async fn start_app() {
    let (redraw_writer, redraw_reader) = mpsc::channel(1024);

    // setup terminal
    let (terminal, chart_area, input_area) = initiate_terminal();

    let terminal = Arc::new(Mutex::new(terminal));

    // Spawn everything for the graph.
    let (graph, register_channel, graph_handles) = Graph::init(redraw_writer.clone());
    // Spawn everything for the input box.
    let (input, commit, kill, input_handles) = InputBox::init(redraw_writer);

    // TODO! Replace this with the real SPI manager.
    let mock_spi_channels: Vec<JoinHandle<()>> = MockSpi::init(register_channel, commit);

    /*
       Manages all of the tasks, if the frontend_killer gets a message the thread_manager exists the program by aborting
       all of the tasks.
    */
    let (frontend_killer, reader) = mpsc::channel(1);

    // Spawn the frontend renderer.
    let mut handles = vec![tokio::spawn(run_frontend(
        terminal.clone(),
        chart_area,
        input_area,
        graph,
        input,
        reader,
        redraw_reader,
    ))];

    // Combine all handles.
    graph_handles.into_iter().for_each(|el| handles.push(el));
    mock_spi_channels
        .into_iter()
        .for_each(|el| handles.push(el));
    input_handles.into_iter().for_each(|el| handles.push(el));
    // Spawn the thread manager.
    let kill_handle = tokio::spawn(thread_manager(handles, kill, frontend_killer));

    // Block until all threads exit
    let _ = kill_handle.await;

    let mut terminal = terminal.lock().await;
    restore_terminal(&mut terminal);
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
    mut kill_reader: mpsc::Receiver<()>,
    mut redraw_reader: mpsc::Receiver<()>,
) {
    while redraw_reader.recv().await.is_some() {
        if kill_reader.try_recv().is_ok() {
            break;
        }
        {
            let mut terminal = terminal.lock().await;

            let input = {
                let input = input.lock().await;
                input.widget()
            };

            let graph = {
                let graph = graph.lock().await;

                graph.widget()
            };

            let vertical =
                Layout::vertical([Constraint::Percentage(70), Constraint::Percentage(30)]);

            let res = terminal.draw(|frame| {
                let [chart, text_area] = vertical.areas(frame.size());
                frame.render_widget(graph, chart);
                frame.render_widget(input, text_area);
            });

            if res.is_err() {
                return;
            }
        }
    }
}
