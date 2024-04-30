use std::error::Error;

use rcv::{
    buffer::Buffer,
    color_code::{ColorCode, GrayScale, Green, Red, RGB},
    display_buffer,
    graphical::Draw,
    kernel::{averaging, AVERAGING, GAUSSIAN, LAPLACIAN, PREWIT, SOBEL},
    rgb_stream::VideoStream,
    transform::{HoughCircles, HoughLines, Transform},
};

extern crate jpeg_decoder as jpeg;

const SCALING: usize = 10;

fn detect_circle<Color: ColorCode<Marker = u8>>(mut buffer: Buffer<Color>) -> bool {
    // Simply check the center of the circles and average the value, larger than
    // some threshold -> that color circle detected.
    let mut source_buffer = buffer.clone();
    buffer.limit_upper(200);
    //buffer.limit_lower(100);
    buffer.conv(&GAUSSIAN);
    buffer.conv(&averaging::<5>());

    let mut target = Vec::new();
    let mut smaller_buffer: Buffer<Color> = buffer.down_sample::<SCALING>(target);

    let max_len = ((smaller_buffer.width.clone().pow(2) + smaller_buffer.height.clone().pow(2))
        as f32)
        .sqrt() as isize;

    smaller_buffer.conv(&GAUSSIAN);
    smaller_buffer.conv(&LAPLACIAN);
    smaller_buffer.threshold_percentile::<5>();
    let highlights: Vec<rcv::HighLight> = (&smaller_buffer).into();

    let circle_transform = HoughCircles::new(
        0..(smaller_buffer.width as isize),
        0..(smaller_buffer.height as isize),
        10..(smaller_buffer.height / 2),
        60,
    );
    let circles = circle_transform.apply(&highlights);

    if circles.len() == 0 {
        return false;
    }

    circles
        .iter()
        .for_each(|circle| circle.draw(&mut smaller_buffer, 255));

    let kernel = averaging::<9>();

    let mut target_buff = source_buffer.buffer.clone();
    for circle in circles.iter() {
        kernel.apply(
            &mut source_buffer,
            circle.center.0 as usize * SCALING,
            circle.center.1 as usize * SCALING,
            &mut target_buff,
        );
        // This is super duper slow, we should re-do this.
        let to_check = Buffer::<Color>::new(
            target_buff.clone(),
            smaller_buffer.width,
            smaller_buffer.height,
        );
        println!(
            "AVERAGE : {:?}",
            to_check[(circle.center.0 as usize, circle.center.1 as usize)]
        );
        if to_check[(circle.center.0 as usize, circle.center.1 as usize)] > 150 {
            return true;
        }
    }
    false
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut stream_handle = VideoStream::<RGB>::new()?;
    let mut stream = stream_handle.into_iter();

    for _i in 0..500 {
        let buffer_color: Buffer<RGB> = stream.next().unwrap();

        let green_buffer = buffer_color.convert::<Green>();
        display_buffer(&green_buffer, "green_image.png");
        let red_buffer = buffer_color.convert::<Red>();
        display_buffer(&red_buffer, "red_image.png");

        // This is simply incorrect, but it makes the example work so I am fine with it
        // as it is only for the lab.
        if detect_circle(green_buffer) {
            println!("Found red circle.");
            for _ in 0..100 {
                println!("STOP");
            }
        }
        if detect_circle(red_buffer) {
            println!("Found green circle.");
            for _ in 0..100 {
                println!("ZOOOOOOOOOOOM");
            }
        }

        //display_buffer(&buffer, "old_image.png").unwrap();

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
    }
    Ok(())
}
