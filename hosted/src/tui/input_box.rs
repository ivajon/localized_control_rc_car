//! Defines an input box that works in two modes, input and visual. In input mode one cannot exit
//! the program since the input is directed in to the input box. In visual mode the program can be
//! exited using the button 'q'.

use crossterm::event::{Event, KeyCode, KeyModifiers};
use ratatui::{
    prelude::*,
    widgets::{block::Title, Block, Borders, Paragraph},
};
use std::{pin::pin, sync::Arc};
use tokio::sync::{mpsc, Mutex};

macro_rules! unwrap_or_break {
    ($tokens:expr) => {
        match $tokens {
            Ok(value) => value,
            _ => break,
        }
    };
}

#[derive(Clone)]
/// A simple input box that spawns a few threads upon creation.
///
/// This should be the only input managing task spawned in the tui.
/// It does not, however, make any efforts to enforce this.
pub struct InputBox {
    input: String,
    mode: bool,
    log: Vec<String>,
    widget: Paragraph<'static>,
    redraw_writer: RedrawWriter,
}

/// This channel gets an f64 whenever there is a new value
/// to send to some target, spi or similar.
pub type CommitReader = mpsc::Receiver<f64>;
/// Writes to the commit channel.
pub type CommitWriter = mpsc::Sender<f64>;
/// This channel gets () whenever the program
/// should be shut down i.e. q is pressed in normal mode.
pub type KillReader = mpsc::Receiver<()>;
/// This channel gets a () whenever the input
/// box wants to trigger a full redraw.
pub type RedrawWriter = mpsc::Sender<()>;

/// A composite return type that contains the needed channels and
/// all join handles needed to kill the box.
pub type InputConstructor = (
    Arc<Mutex<Box<InputBox>>>,
    CommitReader,
    CommitWriter,
    KillReader,
    Vec<tokio::task::JoinHandle<()>>,
);

// Internals

/// Switches between input and visual mode.
type ModeSwitchReader = mpsc::Receiver<bool>;
/// Switches between input and visual mode.
type ModeSwitchWriter = mpsc::Sender<bool>;
/// A channel that manages transmissions of numbers.
type NumReader = mpsc::Receiver<u8>;
/// A channel that manages transmissions of numbers.
type NumWriter = mpsc::Sender<u8>;
/// Sends true if <ctrl> + backspace false if backspace.
type BackSpaceReader = mpsc::Receiver<bool>;
/// Sends true if <ctrl> + backspace false if backspace.
type BackSpaceWriter = mpsc::Sender<bool>;
/// Sends () when enter is pressed.
type EnterReader = mpsc::Receiver<()>;
/// Sends () when enter is pressed.
type EnterWriter = mpsc::Sender<()>;
/// Sends () when q is pressed.
type QReader = mpsc::Receiver<()>;
/// Sends () when q is pressed.
type QWriter = mpsc::Sender<()>;
/// Sends a request to the thread pool to exit the program when q is pressed in normal mode.
type KillWriter = mpsc::Sender<()>;

impl InputBox {
    /// Initiates the new box and spawns all of the needed
    /// tasks.
    pub fn init(redraw_writer: RedrawWriter) -> InputConstructor {
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

        // Create all channels.
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
            commit_writer.clone(),
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

    /// Returns the contained widget in the latest form.
    pub fn widget(&self) -> impl Widget {
        self.widget.clone()
    }

    /// re-renders the widget.
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
            unwrap_or_break!(input_box.redraw_writer.send(()).await);
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
            unwrap_or_break!(locked_box.redraw_writer.send(()).await);
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
            unwrap_or_break!(locked_box.redraw_writer.send(()).await);
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

            unwrap_or_break!(commit_channel.send(number as f64).await);
            locked_box.redraw();
            unwrap_or_break!(locked_box.redraw_writer.send(()).await);
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
                unwrap_or_break!(kill_channel.send(()).await);
                return;
            }
        }
    }

    /// Receives all of the needed events.
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

            let event: Event = unwrap_or_break!(event);

            if let Event::Key(key) = event {
                match key.code {
                    KeyCode::Char('q') => {
                        unwrap_or_break!(q_channel.send(()).await);
                    }
                    KeyCode::Char('i') => {
                        unwrap_or_break!(mode_writer.send(true).await);
                    }
                    KeyCode::Char(char) => {
                        let ascii = match char.as_ascii() {
                            Some(v) => v,
                            None => continue,
                        }
                        .to_u8();
                        if (48..58).contains(&ascii) {
                            unwrap_or_break!(char_writer.send(ascii - 48).await);
                        }
                    }
                    KeyCode::Enter => {
                        // Commit.
                        unwrap_or_break!(commit_channel.send(()).await);
                    }

                    KeyCode::Backspace => {
                        // Allow word deletion.
                        unwrap_or_break!(
                            backspace_channel
                                .send(key.modifiers.contains(KeyModifiers::CONTROL))
                                .await
                        );
                    }

                    KeyCode::Esc => {
                        // Revert to normal mode
                        unwrap_or_break!(mode_writer.send(false).await);
                    }
                    _ => {}
                }
            }
        }
    }
}
