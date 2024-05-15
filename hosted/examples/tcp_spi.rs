//! Example for TCP over SPI where we use tcp_monitor and spi.

use hosted::{spi::Spi, tcp_monitor::TcpMonitor, tui::input_box::CommitReader};
use shared::protocol::v0_0_1::{Payload, V0_0_1};
use std::sync::Arc;
use tokio::{
    io::AsyncWriteExt,
    net::TcpStream,
    sync::{mpsc, Mutex},
    task::JoinHandle,
};

#[tokio::main]
async fn main() -> std::io::Result<()> {
    println!("RUSTY STARTED :)");
    let (commit_writer, commit_reader) = mpsc::channel(1024);
    let _spi_handle = MockSpi::init(commit_reader).unwrap();
    let (_sessions, _tcp_handle) = TcpMonitor::init(commit_writer).unwrap();

    tokio::spawn(protocol());
    loop {}
}

pub async fn protocol() -> std::io::Result<()> {
    // Connect to peer
    let mut stream = TcpStream::connect("127.0.0.1:8080").await.unwrap();

    stream.write_all(&[1, 175]).await?; // Commit
    Ok(())
}

/// The MockSpi is the reason why we can talk over TCP using SPI
pub struct MockSpi {
    target_value: f64,
    spi: Spi<V0_0_1>, // change to v2
}

impl MockSpi {
    fn init(reference_reader: CommitReader) -> Option<Vec<JoinHandle<()>>> {
        let spi = Spi::init("/dev/spidev1.0")?;

        let ret = Arc::new(Mutex::new(MockSpi {
            target_value: 0.,
            spi,
        }));
        Some(vec![tokio::spawn(Self::set_reference(
            ret.clone(),
            reference_reader,
        ))])
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
