//! Defines a short example that uses the [`Spi`] to read data using our
//! [`protocol`](shared::protocol). It then plots the data using our [`tui`](hosted::tui)
//! abstractions.

#![feature(ascii_char)]

use hosted::{
    cv::camera_monitor::{StateChange, Vision},
    init_logging,
    spi::Spi,
    tui::{
        graph::{Graph, MeasurementWriter},
        initiate_terminal,
        input_box::{CommitReader, CommitWriter, InputBox, KillReader},
        TerminalWrapper,
    },
};
use rand::Rng;
use ratatui::layout::{Constraint, Layout};
use shared::protocol::v0_0_1::{Payload, V0_0_1};
use std::{error::Error, sync::Arc};

use log::{info, LevelFilter};
use log4rs::append::{console::ConsoleAppender, file::FileAppender};
use log4rs::config::{Appender, Config, Root};
use log4rs::encode::pattern::PatternEncoder;

use tokio::{
    sync::{
        mpsc::{self, Receiver},
        Mutex,
    },
    task::JoinHandle,
    time::Duration,
};

pub struct RealSpi {
    target_value: f64,
    spi: Spi<V0_0_1>,
}

macro_rules! unwrap_or_break {
    ($tokens:expr) => {
        match $tokens {
            Ok(_) => {}
            _ => break,
        }
    };
}

impl RealSpi {
    fn init(
        measurement_writer: MeasurementWriter,
        reference_reader: CommitReader,
    ) -> Option<Vec<JoinHandle<()>>> {
        let spi = Spi::init("/dev/spidev0.0")?;

        let ret = Arc::new(Mutex::new(RealSpi {
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
) {
    let _ = kill_command.recv().await;

    println!("Killing the program");
    // Do not unwrap would be strange if the thread managed died due to a process having died.
    let _ = frontend_killer.send(()).await;
    threads.into_iter().for_each(|handle| {
        handle.abort();
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // console_subscriber::init();
    let file_path = "/tmp/foo.log";

    // Build a stderr logger.

    // Logging to log file.
    let logfile = FileAppender::builder()
        // Pattern: https://docs.rs/log4rs/*/log4rs/encode/pattern/index.html
        .encoder(Box::new(PatternEncoder::new("{l} - {m}\n")))
        .build(file_path)
        .unwrap();

    // Log Trace level output to file where trace is the default level
    // and the programmatically specified level to stderr.
    let config = Config::builder()
        .appender(Appender::builder().build("logfile", Box::new(logfile)))
        .build(
            Root::builder()
                .appender("logfile")
                .build(LevelFilter::Trace),
        )
        .unwrap();

    // Use this to change log levels at runtime.
    // This means you can change the default log level to trace
    // if you are trying to debug an issue and need more logs on then turn it off
    // once you are done.
    let _handle = log4rs::init_config(config)?;


    info!("Application started");

    let _ = start_app().await;
    println!("Cleanup completed, cya :)");
    info!("Application closed");

    Ok(())
}

/// Spawns all of the threads that are needed for the app to work
/// and does not return until all threads have exited.
async fn start_app() -> Result<(), ()> {
    let (redraw_writer, redraw_reader) = mpsc::channel(1024);

    // setup terminal
    let terminal = initiate_terminal();

    let terminal = Arc::new(Mutex::new(terminal.map_err(|_| ())?));

    // Spawn everything for the graph.
    let (graph, register_channel, graph_handles) = Graph::init(redraw_writer.clone());
    // Spawn everything for the input box.
    let (input, commit, commit_writer, kill, input_handles) = InputBox::init(redraw_writer);

    let (state_change_sender, state_change_receiver) = mpsc::channel(1024);

    let (vision, vision_handles) = match Vision::init(state_change_sender) {
        Ok(value) => value,
        Err(_e) => {
            // Kill all threads if we encountered an error.
            for el in graph_handles {
                el.abort();
            }
            for el in input_handles {
                el.abort();
            }
            return Err(());
        }
    };

    // TODO! Replace this with the real SPI manager.
    let mock_spi_channels: Vec<JoinHandle<()>> = /* match */ MockSpi::init(register_channel, commit); /* { */
    // Some(value) => value,
    // None => {
    //     for el in vision_handles {
    //         el.abort();
    //     }
    //     // Kill all threads if we encountered an error.
    //     for el in graph_handles {
    //         el.abort();
    //     }
    //     for el in input_handles {
    //         el.abort();
    //     }
    //     return Err(());
    // }
    //};

    /*
       Manages all of the tasks, if the frontend_killer gets a message the thread_manager exists the program by aborting
       all of the tasks.
    */
    let (frontend_killer, reader) = mpsc::channel(1);

    // Spawn the frontend renderer.
    let mut handles = vec![
        tokio::spawn(run_frontend(
            terminal.clone(),
            graph,
            input,
            vision,
            reader,
            redraw_reader,
        )),
        tokio::spawn(start_stop(state_change_receiver, commit_writer)),
    ];

    // Combine all handles.
    graph_handles.into_iter().for_each(|el| handles.push(el));
    mock_spi_channels
        .into_iter()
        .for_each(|el| handles.push(el));
    input_handles.into_iter().for_each(|el| handles.push(el));
    vision_handles.into_iter().for_each(|el| handles.push(el));
    // Spawn the thread manager.
    let kill_handle = tokio::spawn(thread_manager(handles, kill, frontend_killer));

    // Block until all threads exit
    let _ = kill_handle.await;

    Ok(())
}

async fn start_stop(mut state_change: Receiver<StateChange>, spi_writer: CommitWriter) {
    let mut state = StateChange::Stop;
    while let Some(change) = state_change.recv().await {
        if change != state {
            match change {
                StateChange::Start => spi_writer.send(100.),
                StateChange::Stop => spi_writer.send(0.),
            }
            .await
            .unwrap();
            state = change;
        }
    }
}

/// Draws the frontend of the application.
async fn run_frontend(
    terminal: Arc<Mutex<TerminalWrapper>>,
    graph: Arc<Mutex<Box<Graph>>>,
    input: Arc<Mutex<Box<InputBox>>>,
    vision: Arc<Mutex<Vision>>,
    mut kill_reader: mpsc::Receiver<()>,
    mut redraw_reader: mpsc::Receiver<()>,
) {
    loop {
        if kill_reader.try_recv().is_ok() {
            break;
        }
        {
            // Acquire terminal first for the shortest possible critical sections.
            let mut terminal = terminal.lock().await;

            // Grab the two widgets.

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
            let horizontal =
                Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]);

            let image_source = vision.lock().await;
            let image = image_source.widget();
            let res = terminal.draw(|frame| {
                let [chart, text_area] = vertical.areas(frame.size());
                let [chart, image_area] = horizontal.areas(chart);
                frame.render_widget(graph, chart);
                frame.render_widget(input, text_area);
                if let Some(image) = image {
                    frame.render_widget(image, image_area);
                }
            });

            if res.is_err() {
                return;
            }
        }
        if redraw_reader.recv().await.is_none() {
            break;
        }
    }
}
