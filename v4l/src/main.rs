use std::error::Error;

use rcv::{
    buffer::{Buffer, GrayScale},
    display_buffer,
    graphical::Draw,
    kernel::{averaging, AVERAGING, GAUSSIAN, LAPLACIAN, PREWIT, SOBEL},
    rgb_stream::VideoStream,
    transform::{HoughCircles, HoughLines, Transform},
};

extern crate jpeg_decoder as jpeg;

const SCALING: usize = 8;

fn main() -> Result<(), Box<dyn Error>> {
    let mut stream = VideoStream::<GrayScale>::new()?;

    let (width, height) = stream.dimensions();

    for _i in 0..500 {
        let mut pixels = stream.next().unwrap();

        let mut buffer = crate::Buffer::new(pixels.as_mut_slice(), width, height);

        buffer.limit_upper(200);
        buffer.limit_lower(100);

        display_buffer(&buffer, "old_image.png").unwrap();

        let mut target = Vec::new();
        let mut smaller_buffer = buffer.down_sample::<SCALING>(&mut target);

        smaller_buffer.conv(&GAUSSIAN);
        smaller_buffer.conv(&LAPLACIAN);
        smaller_buffer.threshold_percentile::<10>();
        smaller_buffer.conv(&averaging::<3>());
        smaller_buffer.threshold_percentile::<10>();
        smaller_buffer.conv(&SOBEL);
        //smaller_buffer.threshold_percentile::<20>();
        //smaller_buffer.limit_lower(20);

        //smaller_buffer.conv(&SOBEL.clone());
        //smaller_buffer += &horizontal;
        smaller_buffer.threshold_percentile::<5>();
        let highlights: Vec<rcv::HighLight> = (&smaller_buffer).into();

        let circle_transform = HoughCircles::new(
            0..smaller_buffer.width,
            0..smaller_buffer.height,
            1..100,
            100,
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
        let line_transform = HoughLines::new(
            ((-max_len)..max_len).step_by(10),
            //(-90)..90,
            (-90)..90,
            30 as u32,
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
        println!("Drew the lines");
    }
    Ok(())
}
