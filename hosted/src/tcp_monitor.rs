//!

use std::{error::Error, sync::Arc};
use crate::tui::input_box::CommitWriter;
use tokio::{
    io::AsyncReadExt,
    net::{TcpListener, TcpSocket, TcpStream},
    sync::Mutex,
    task::JoinHandle,
};

pub struct TcpMonitor {}

impl TcpMonitor {
    pub fn init(
        commit_writer: CommitWriter,
    ) -> Result<(Arc<Mutex<Vec<JoinHandle<()>>>>, JoinHandle<()>), Box<dyn Error>> {
        let addr = "127.0.0.1:8080".parse()?;
        let socket = TcpSocket::new_v4()?;
        socket.bind(addr)?;
        let handles = Arc::new(Mutex::new(Vec::with_capacity(2)));
        let listener: TcpListener = socket.listen(1024).unwrap();
        Ok((
            handles.clone(),
            tokio::spawn(Self::new_connection_manager(
                listener,
                commit_writer,
                handles,
            )),
        ))
    }

    pub async fn new_connection_manager(
        listener: TcpListener,
        commit_writer: CommitWriter,
        handles: Arc<Mutex<Vec<JoinHandle<()>>>>,
    ) -> () {
        loop {
            let (socket, _) = listener.accept().await.unwrap();
            let mut handles = handles.lock().await;
            let handle = tokio::spawn(Self::cv_protocol(socket, commit_writer.clone()));
            handles.push(handle);
        }
    }

    pub async fn cv_protocol(mut socket: TcpStream, commit_writer: CommitWriter) {
        loop {
            let mut buf = vec![0; 8];
            let n = match socket.read(&mut buf).await {
                Ok(bytes) => bytes,
                _ => return,
            };
            println!("got buf: {:?}", buf);
            if n == 0 {
                break;
            }
            if n <= 2 {
                if buf[0] == 0 {
                    if let Err(_) = commit_writer.send(0.).await {
                        break;
                    }
                    continue;
                } else if buf[0] == 1 {
                    if let Err(_) = commit_writer.send(50.).await {
                        break;
                    }
                    continue;
                }
            }
            if n == 2 {
                if buf[0] == 2 {
                    if let Err(_) = commit_writer.send(buf[1] as f64).await {
                        break;
                    }
                    continue;
                }
            }
        }
    }
}
