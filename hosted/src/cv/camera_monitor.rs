use std::{io, sync::Arc};

use log::info;
use ratatui::widgets::{
    canvas::{Canvas, Points},
    Widget,
};
use tokio::{sync::mpsc::Sender, sync::Mutex, task::JoinHandle};

use crate::cv::color_code::Red;

use super::{
    buffer::Buffer,
    color_code::{ColorCode, GrayScale, Green, RGB},
    draw_on_canvas,
    graphical::{Circle, Line},
    kernel::{averaging, GAUSSIAN, LAPLACIAN},
    rgb_stream::VideoStream,
    transform::{HoughCircles, HoughLines, Transform},
    HighLight,
};

pub struct Vision {
    latest_images: Option<(Buffer<GrayScale>, Buffer<Green>, Buffer<Red>)>,
    latest_process_result: Option<(
        Buffer<GrayScale>,
        Option<Circle<Green>>,
        Option<Circle<Red>>,
    )>,

    latest_edges: Vec<Line<GrayScale>>,
}

#[derive(Clone, PartialEq)]
pub enum StateChange {
    Start,
    Stop,
}

// THIS IS UGLY, REMOVE LATER
const SCALING: usize = 10;

macro_rules! unwrap_or_return {
    ($e:expr) => {
        match $e {
            Ok(val) => val,
            Err(_e) => {
                return;
            }
        }
    };
}

type VisionReturn = (Arc<Mutex<Vision>>, Vec<JoinHandle<()>>);

type FeatureSender = Sender<StateChange>;

impl Vision {
    pub fn init(sender: FeatureSender) -> Result<VisionReturn, eye_hal::Error> {
        let stream_handle = VideoStream::<RGB>::new()?;

        let vision = Arc::new(Mutex::new(Vision {
            latest_images: None,
            latest_process_result: None,
            latest_edges: Vec::new(),
        }));

        let join_handles = vec![
            tokio::spawn(Self::stream_consumer(stream_handle, vision.clone())),
            tokio::spawn(Self::feature_extraction(vision.clone(), sender)),
            tokio::spawn(Self::line_extraction(vision.clone())),
        ];
        Ok((vision, join_handles))
    }

    /// Returns a drawable widget with the latest information.
    pub fn widget(&self) -> Option<impl Widget + '_> {
        let (img, gc, rc) = self.latest_process_result.clone()?;
        let points: Vec<HighLight> = (&img).into();
        Some(
            Canvas::default()
                .x_bounds([0., img.width as f64])
                .y_bounds([0., img.height as f64])
                .paint(move |ctx| {
                    // points.iter().for_each(|HighLight { x, y }| {
                    //     ctx.draw(&Points {
                    //         coords: &[(*x as f64, (img.height - *y) as f64)],
                    //         color: GrayScale::get_color(&GrayScale::highlight()),
                    //     })
                    // });

                    if let Some(mut gc) = gc.clone() {
                        gc.center.1 = img.height as isize - gc.center.1;
                        ctx.draw(&gc);
                    }
                    if let Some(mut rc) = rc.clone() {
                        rc.center.1 = img.height as isize - rc.center.1;
                        ctx.draw(&rc);
                    }

                    for mut line in self.latest_edges.clone() {
                        line.start.1 = img.height as usize - line.start.1;
                        line.stop.1 = img.height as usize - line.stop.1;
                        ctx.draw(&line.clone());
                    }
                })
                .marker(ratatui::symbols::Marker::Dot),
        )
    }

    fn process<Color: ColorCode<Marker = u8>>(
        mut buffer: Buffer<Color>,
    ) -> (Buffer<Color>, Vec<Circle<Color>>) {
        buffer.limit_upper(200);
        //buffer.limit_lower(100);
        buffer.conv(&GAUSSIAN);
        buffer.conv(&averaging::<5>());

        let mut smaller_buffer: Buffer<Color> = buffer.down_sample::<SCALING>();

        smaller_buffer.conv(&GAUSSIAN);
        smaller_buffer.conv(&LAPLACIAN);
        smaller_buffer.threshold_percentile::<5>();
        let highlights: Vec<HighLight> = (&smaller_buffer).into();

        let circle_transform = HoughCircles::new(
            0..(smaller_buffer.width as isize),
            0..(smaller_buffer.height as isize),
            10..(smaller_buffer.height / 2),
            50,
        );
        (smaller_buffer, circle_transform.apply(&highlights))
    }

    fn detect<Color: ColorCode<Marker = u8>>(
        circles: &Vec<Circle<GrayScale>>,
        buffer: &mut Buffer<Color>,
    ) -> Option<Circle<Color>> {
        // Simply check the center of the circles and average the value, larger than
        // some threshold -> that color circle detected.

        if circles.len() == 0 {
            return None;
        }

        let kernel = averaging::<5>();

        let mut target_buff = buffer.buffer.clone();
        for circle in circles.iter() {
            let (x, y) = (
                circle.center.0 as usize * SCALING,
                circle.center.1 as usize * SCALING,
            );
            kernel.apply(buffer, x, y, &mut target_buff);
            // This is super duper slow, we should re-do this.
            let to_check = Buffer::<Color>::new(target_buff.clone(), buffer.width, buffer.height);
            if to_check[(x, y)] > 150 {
                return Some(circle.clone().convert());
            }
        }
        None
    }

    async fn line_extraction(vision: Arc<Mutex<Vision>>) {
        loop {
            let (mut gray_frame, _green_frame, _red_frame) = {
                let stream = vision.lock().await;
                if stream.latest_images.is_none() {
                    // This will likely not happen.
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                    continue;
                }

                // This was just checked.
                let frames = unsafe { stream.latest_images.clone().unwrap_unchecked() };

                frames
            };
            gray_frame.limit_upper(200);
            //buffer.limit_lower(100);
            gray_frame.conv(&GAUSSIAN);
            gray_frame.conv(&averaging::<5>());

            let mut smaller_buffer: Buffer<GrayScale> = gray_frame.down_sample::<SCALING>();

            smaller_buffer.conv(&GAUSSIAN);
            smaller_buffer.conv(&LAPLACIAN);
            smaller_buffer.threshold_percentile::<5>();
            let highlights: Vec<HighLight> = (&smaller_buffer).into();

            let max_len = (smaller_buffer.width.pow(2) as f32 + smaller_buffer.height.pow(2) as f32)
                .sqrt() as isize;

            let line_transform: HoughLines<std::ops::Range<isize>, std::ops::Range<isize>> =
                HoughLines::new(0..180, 0..(max_len), 70, (5, 40));

            let lines: Vec<Line<GrayScale>> = line_transform.apply(&highlights);
            info!("Found lines : {:?}", lines);

            {
                let mut stream = vision.lock().await;
                stream.latest_edges = lines;
            }
        }
    }

    async fn feature_extraction(vision: Arc<Mutex<Vision>>, sender: FeatureSender) {
        loop {
            let (gray_frame, mut green_frame, mut red_frame) = {
                let stream = vision.lock().await;
                if stream.latest_images.is_none() {
                    // This will likely not happen.
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                    continue;
                }

                // This was just checked.
                let frames = unsafe { stream.latest_images.clone().unwrap_unchecked() };

                frames
            };

            let (gray, circles) = Self::process(gray_frame);

            let green_circle = Self::detect(&circles, &mut green_frame);
            let red_circle = Self::detect(&circles, &mut red_frame);

            if let Some(_circle) = green_circle.clone() {
                unwrap_or_return!(sender.send(StateChange::Start).await);
            }
            if let Some(_circle) = red_circle.clone() {
                unwrap_or_return!(sender.send(StateChange::Stop).await);
            }
            {
                let mut stream = vision.lock().await;
                stream.latest_process_result = Some((gray, green_circle, red_circle));
            }
        }
    }

    async fn stream_consumer<'device>(
        mut stream_handle: VideoStream<'device, RGB>,
        vision: Arc<Mutex<Vision>>,
    ) -> () {
        let refresh = tokio::time::Duration::from_millis(unwrap_or_return!(stream_handle
            .interval()
            .as_millis()
            .try_into()));
        let stream = stream_handle.into_iter();
        while let Some((frame_index, rgb_frame)) = stream.enumerate().next() {
            let time = tokio::time::Instant::now();

            if frame_index % 20 != 0 {
                tokio::time::sleep_until(time + refresh.into()).await;
                continue;
            }
            let grey_frame = rgb_frame.convert::<GrayScale>();
            let green_frame = rgb_frame.convert::<Green>();
            let red_frame = rgb_frame.convert::<Red>();
            {
                let mut vision_inner = vision.lock().await;
                vision_inner.latest_images = Some((grey_frame, green_frame, red_frame));
            }
            tokio::time::sleep_until(time + refresh.into()).await;
        }
        todo!()
    }
}
