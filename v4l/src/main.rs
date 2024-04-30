use std::error::Error;

use rcv::{
    buffer::Buffer,
    color_code::{GrayScale, Green, Red, RGB},
    display_buffer,
    graphical::Draw,
    kernel::{averaging, AVERAGING, GAUSSIAN, LAPLACIAN, PREWIT, SOBEL},
    rgb_stream::VideoStream,
    transform::{HoughCircles, HoughLines, Transform},
};

extern crate jpeg_decoder as jpeg;

const SCALING: usize = 10;

fn main() -> Result<(), Box<dyn Error>> {
    let mut stream_handle = VideoStream::<RGB>::new()?;
    let mut stream = stream_handle.into_iter();

    let (width, height) = stream.dimensions();

    for _i in 0..500 {
        let buffer_color: Buffer<RGB> = stream.next().unwrap();

        let mut buffer = buffer_color.convert::<Green>();

        display_buffer(&buffer, "old_image.png").unwrap();

        buffer.limit_upper(200);
        //buffer.limit_lower(100);
        buffer.conv(&GAUSSIAN);
        buffer.conv(&averaging::<5>());

        let mut target = Vec::new();
        let mut smaller_buffer: Buffer<Green> = buffer.down_sample::<SCALING>(target);

        smaller_buffer.conv(&GAUSSIAN);
        //smaller_buffer.conv(&LAPLACIAN);
        //smaller_buffer.threshold_percentile::<5>();
        //smaller_buffer.conv(&averaging::<5>());
        smaller_buffer.conv(&LAPLACIAN);
        //smaller_buffer.conv(&GAUSSIAN);
        //smaller_buffer.threshold_percentile::<20>();
        //smaller_buffer.limit_lower(20);

        //smaller_buffer.conv(&SOBEL.clone());
        //smaller_buffer += &horizontal;
        smaller_buffer.threshold_percentile::<5>();
        let highlights: Vec<rcv::HighLight> = (&smaller_buffer).into();

        let circle_transform = HoughCircles::new(
            0..(smaller_buffer.width as isize),
            0..(smaller_buffer.height as isize),
            10..(smaller_buffer.height / 2),
            55,
        );
        let circles = circle_transform.apply(&highlights);
        println!("Found {} circles", circles.len());

        circles
            .iter()
            .for_each(|circle| circle.draw(&mut smaller_buffer, 255));

        display_buffer(&smaller_buffer, "result_image_no_lines.png").unwrap();

        let max_len = ((smaller_buffer.width.clone().pow(2) + smaller_buffer.height.clone().pow(2))
            as f32)
            .sqrt() as isize;

        /*
        let line_transform = HoughLines::new(
            ((-max_len)..max_len).step_by(1),
            //(-90)..90,
            (-90)..90,
            5 as u32,
            (1, 200),
        );

        let lines = line_transform.apply(&highlights);

        println!("Found {} lines", lines.len());

        let mut data: Vec<u8> = (0..(smaller_buffer.width * smaller_buffer.height))
            .map(|_| 0)
            .collect();

        let mut line_buffer = Buffer::new(&mut data, smaller_buffer.width, smaller_buffer.height);

        lines
            .iter()
            .for_each(|line| line.draw(&mut line_buffer, 255));

        display_buffer(&line_buffer, "result_image_lines.png").unwrap();
        */
        println!("Drew the lines");
    }
    Ok(())
}
