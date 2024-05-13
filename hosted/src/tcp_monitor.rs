use tokio::{
    io::{AsyncReadExt},
    net::{TcpListener, TcpSocket, TcpStream},
};
use crate::tui::input_box::CommitWriter;

pub struct TcpMonitor {
    socket: TcpSocket,
}

impl TcpMonitor {
    pub fn init(commit_writer: CommitWriter) {
        let addr = "127.0.0.1:8080".parse().unwrap();
        let socket = TcpSocket::new_v4().unwrap();
        socket.bind(addr);

        let listener = socket.listen(1024).unwrap();
        tokio::spawn(Self::new_connection_manager(listener, commit_writer));
    }

    pub async fn new_connection_manager(listener: TcpListener, commit_writer: CommitWriter) {
        loop {
            let (socket, _) = listener.accept().await.unwrap();
            tokio::spawn(Self::cv_protocol(socket, commit_writer.clone()));
        }
    }

    pub async fn cv_protocol(mut socket: TcpStream, commit_writer: CommitWriter) {
        loop {
            let mut buf = vec![0; 8];
            let n = socket.read(&mut buf).await.expect("Failed to read data from Camera");

            if n == 0 {
                break;
            }
            if n <= 2 {
                if buf[0] == 0 {
                    //println!("kill");
                    if let Err(_) = commit_writer.send(0.).await {
                        break;
                    }
                    continue;
                } else if buf[0] == 1 {
                    //println!("start");
                    if let Err(_) = commit_writer.send(50.).await {
                        break;
                    }
                    continue;
                }
            }
             if n == 2 {
                if buf[0] == 2 {
                    //println!("Speed is now: {:?}", buf[1]);
                    if let Err(_) = commit_writer.send(buf[1] as f64).await {
                        break;
                    }
                    continue;
                }
            }

        }
    }
}

/*
pub async fn eko(mut socket: TcpStream) {
    let mut buf = vec![0; 8];
    let n = socket
        .read(&mut buf)
        .await
        .expect("failed to read data from socket");

    if n == 0 {
        return;
    }
    socket
        .write_all(&buf[0..n])
        .await
        .expect("failed to write data to socket");
}

pub async fn peepo() -> std::io::Result<()> {
    // Connect to peer
    let mut stream = TcpStream::connect("127.0.0.1:8080").await.unwrap();

    stream.write_all([0b00, 0b01, 010]).await?; // Commit
    println!("peepo here");
    Ok(())
} */
