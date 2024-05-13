use hosted::tcp_monitor::{self, TcpMonitor};
use tokio::{
    io::{AsyncWriteExt},
    net::{TcpStream},
};

#[tokio::main]
async fn main() -> std::io::Result<()> {
    TcpMonitor::init();
    tokio::spawn(protocol());
    loop{}
}

pub async fn protocol() -> std::io::Result<()> {
    // Connect to peer
    let mut stream = TcpStream::connect("127.0.0.1:8080").await.unwrap();

    stream.write_all(&[1,175]).await?; // Commit
    Ok(())
}
