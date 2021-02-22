use rosrust_msg::sensor_msgs::Image;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels;
use sdl2::surface::Surface;
use std::time::Duration;

fn make_test_image(width: usize, height: usize) -> Image {
    // temp test image
    let mut msg = Image::default();
    msg.width = width as u32;
    let chan: usize = 3;  // 4;
    msg.step = msg.width * chan as u32;
    msg.height = height as u32;
    // msg.encoding = "bgra8".to_string();
    msg.encoding = "bgr8".to_string();
    msg.data.resize((msg.height * msg.step) as usize, 128);
    // generate test image
    for i in (0..msg.data.len()).step_by(chan) {
        let x = (i / chan) % msg.width as usize;
        let y = i / msg.step as usize;
        msg.data[i] = (x % 256) as u8;
        msg.data[i + 1] = 40;  // ((x / 4 * y / 4) % 256) as u8;
        msg.data[i + 2] = (255 * y / msg.height as usize) as u8;
        if chan == 4 {
            msg.data[i + 3] = 255;
        }
    }
    msg
}
struct ViewImage {
    _sdl_context: sdl2::Sdl,
    window: sdl2::video::Window,
    event_pump: sdl2::EventPump,
}

impl ViewImage {
    fn default(width: usize, height: usize) -> Self {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem.window("sdl2_ros", width as u32, height as u32)
            .position_centered()
            .build()
            .unwrap();
        let event_pump = sdl_context.event_pump().unwrap();

        Self {
            _sdl_context: sdl_context,
            window,
            event_pump,
        }
    }

    fn msg_to_surface(&mut self, msg: &mut Image)
    {
        let surface = Surface::from_data_pixelmasks(
            msg.data.as_mut_slice(),
            msg.width,
            msg.height,
            msg.step,
            pixels::PixelMasks {
                bpp: 24,  // 32,
                rmask: 0x00ff0000,
                gmask: 0x0000ff00,
                bmask: 0x000000ff,
                amask: 0xff000000,
            },
        ).unwrap();

        let mut win_surface = self.window.surface(&self.event_pump).unwrap();
        // TODO(lucas) define a destination sdl2::rect::Rect that preserves aspect ratio
        //     sdl2::rect::Rect::new(0, 0, 300, 500),
        surface.blit(None, &mut win_surface, None).unwrap();
        win_surface.update_window().unwrap();
    }

    fn update(&mut self) -> bool{
        for event in self.event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    return false;
                },
                _ => {
                    println!("unhandled '{:?}'", event);
                }
            }
        }

        {
            let mut win_surface = self.window.surface(&self.event_pump).unwrap();
            win_surface.fill_rect(
                sdl2::rect::Rect::new(100, 100, 200, 300),
                sdl2::pixels::Color::BLUE,
            ).unwrap();
            win_surface.update_window().unwrap();
        }
        true
    }
}

pub fn main() {
    let mut view_image = ViewImage::default(2000, 1000);
    let mut msg = make_test_image(2000, 1000);
    view_image.msg_to_surface(&mut msg);

    'running: loop {
        if !view_image.update() {
            break 'running;
        }
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}