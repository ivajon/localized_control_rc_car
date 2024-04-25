//! Defines a short example that uses the [`Spi`] to read data using our
//! [`protocol`](shared::protocol). It then plots the data using our [`tui`](hosted::tui)
//! abstractions.

#![feature(ascii_char)]

use hosted::{
    spi::{Error as SpiError, Spi},
    tui::{
        graph::{Graph, MeasurementWriter},
        initiate_terminal,
        input_box::{CommitReader, InputBox, KillReader},
        TerminalWrapper,
    },
};
use ratatui::layout::{Constraint, Layout};
use shared::protocol::v0_0_1::{Payload, V0_0_1};
use std::{error::Error, sync::Arc};

use tokio::{
    sync::{mpsc, Mutex},
    task::JoinHandle,
    time::Duration,
};

pub struct MockSpi {
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

impl MockSpi {
    fn init(
        measurement_writer: MeasurementWriter,
        reference_reader: CommitReader,
    ) -> Result<Vec<JoinHandle<()>>, SpiError> {
        let spi = Spi::init("/dev/spidev0.0")?;

        let ret = Arc::new(Mutex::new(MockSpi {
            target_value: 0.,
            spi,
        }));
        Ok(vec![
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

async fn thread_manager(
    threads: Vec<JoinHandle<()>>,
    mut kill_command: KillReader,
    frontend_killer: mpsc::Sender<()>,
) {
    let _ = kill_command.recv().await;

    // Do not unwrap would be strange if the thread managed died due to a process having died.
    let _ = frontend_killer.send(()).await;
    threads.into_iter().for_each(|handle| {
        handle.abort();
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // console_subscriber::init();

    let res = start_app().await;
    eprintln!("Some error occured during SPI configuration {:?}", res);
    Ok(())
}

/// Spawns all of the threads that are needed for the app to work
/// and does not return until all threads have exited.
async fn start_app() -> Result<(), SpiError> {
    let (redraw_writer, redraw_reader) = mpsc::channel(1024);

    // setup terminal
    let terminal = initiate_terminal();

    // This is not correct, but let's say that it is.
    let terminal = Arc::new(Mutex::new(terminal.map_err(|_| SpiError::NothingToRead)?));

    // Spawn everything for the graph.
    let (graph, register_channel, graph_handles) = Graph::init(redraw_writer.clone());
    // Spawn everything for the input box.
    let (input, commit, kill, input_handles) = InputBox::init(redraw_writer);

    // TODO! Replace this with the real SPI manager.
    let mock_spi_channels: Vec<JoinHandle<()>> = match MockSpi::init(register_channel, commit) {
        Ok(value) => value,
        Err(e) => {
            // Kill all threads if we encountered an error.
            for el in graph_handles {
                el.abort();
            }
            for el in input_handles {
                el.abort();
            }

            eprintln!("Some error occured during SPI configuration {:?}", e);
            return Err(e);
        }
    };

    /*
       Manages all of the tasks, if the frontend_killer gets a message the thread_manager exists the program by aborting
       all of the tasks.
    */
    let (frontend_killer, reader) = mpsc::channel(1);

    // Spawn the frontend renderer.
    let mut handles = vec![tokio::spawn(run_frontend(
        terminal.clone(),
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
    Ok(())
}

/// Draws the frontend of the application.
async fn run_frontend(
    terminal: Arc<Mutex<TerminalWrapper>>,
    graph: Arc<Mutex<Box<Graph>>>,
    input: Arc<Mutex<Box<InputBox>>>,
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

            let res = terminal.draw(|frame| {
                let [chart, text_area] = vertical.areas(frame.size());
                frame.render_widget(graph, chart);
                frame.render_widget(input, text_area);
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
