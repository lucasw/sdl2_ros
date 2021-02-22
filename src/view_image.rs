use rosrust_msg::sensor_msgs::Image;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels;
use sdl2::surface::Surface;
use std::time::Duration;
 
pub fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
 
    let window = video_subsystem.window("rust-sdl2 demo", 2000, 1000)
        .position_centered()
        .build()
        .unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();

    {
        let mut win_surface = window.surface(&event_pump).unwrap();

        // temp test image
        let mut msg = Image::default();
        msg.width = 2000;
        msg.step = msg.width * 4;  // * 3;
        msg.height = 1000;
        msg.encoding = "bgra8".to_string();  // "bgr8";
        msg.data.resize((msg.height * msg.step) as usize, 128);
        /*
        for (count, pix) in msg.data.iter_mut().enumerate() {
            *pix = 128;  // ((count / 256) % 256) as u8;
        }
        */
        for i in 0..msg.data.len() {
            msg.data[i] = (i / 4 % 256) as u8;
        }
        let surface = Surface::from_data_pixelmasks(
            msg.data.as_mut_slice(),
            msg.width,
            msg.height,
            msg.step,
            pixels::PixelMasks {
                bpp: 32,
                rmask: 0x00ff0000,
                gmask: 0x0000ff00,
                bmask: 0x000000ff,
                amask: 0xff000000,
            },
        ).unwrap();

        surface.blit(None, &mut win_surface, None).unwrap();
        // surface.blit(
        //     sdl2::rect::Rect::new(0, 0, 300, 500),
        //     &mut win_surface,
        //     sdl2::rect::Rect::new(0, 0, 300, 500)).unwrap();
        win_surface.update_window().unwrap();
    }

    'running: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                _ => {
                    println!("unhandled '{:?}'", event);
                }
            }
        }
        // The rest of the game loop goes here...
        {
            let mut win_surface = window.surface(&event_pump).unwrap();
            win_surface.fill_rect(
                sdl2::rect::Rect::new(100, 100, 200, 300),
                sdl2::pixels::Color::GRAY,
            ).unwrap();
            win_surface.update_window().unwrap();
        }
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}