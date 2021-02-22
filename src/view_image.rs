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
struct ViewImageBase {
    _sdl_context: sdl2::Sdl,
    window: sdl2::video::Window,
    event_pump: sdl2::EventPump,
    // txrx: (crossbeam_channel::Sender<Image>, crossbeam_channel::Receiver<Image>),
    rx: crossbeam_channel::Receiver<Image>,
}
struct ViewImage {
    base: ViewImageBase,
    _image_sub: rosrust::Subscriber,
}

/*
impl ViewImageBase {
    fn image_callback(&mut self, msg: Image) {
        let (tx, _) = self.txrx;
        if let Err(err) = tx.send(msg) {
            if rosrust::is_ok() {
                eprintln!("{:?}", err);
            }
        }
    }
}
*/

impl ViewImage {
    fn default(width: usize, height: usize) -> Self {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();
        let window = video_subsystem.window("sdl2_ros", width as u32, height as u32)
            .position_centered()
            .build()
            .unwrap();
        let event_pump = sdl_context.event_pump().unwrap();

        let (tx, rx) = crossbeam_channel::unbounded::<Image>();

        let base = ViewImageBase {
            _sdl_context: sdl_context,
            window,
            event_pump,
            rx,
        };

        // TODO(lucasw) want this to the callback in ViewImageBase
        let image_callback = {
            move |msg: Image| {
                if let Err(err) = tx.send(msg) {
                    // an error on shutdown isn't interesting (though preventing those
                    // in the future would be nice
                    if rosrust::is_ok() {
                        eprintln!("{:?}", err);
                    }
                }
            }
        };

        let _image_sub = rosrust::subscribe("image_in", 4, image_callback).unwrap();
        Self {
            base,
            _image_sub,
        }
    }

    fn msg_to_surface(&mut self, msg: &mut Image) {
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

        let mut win_surface = self.base.window.surface(&self.base.event_pump).unwrap();
        // TODO(lucas) define a destination sdl2::rect::Rect that
        // centers and preserves aspect ratio
        //     sdl2::rect::Rect::new(0, 0, 300, 500),
        surface.blit(None, &mut win_surface, None).unwrap();
        win_surface.update_window().unwrap();
    }

    fn update(&mut self) -> bool {
        for event in self.base.event_pump.poll_iter() {
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
            let mut some_msg = None;
            // empty every message from the channel, keep the latest
            loop {
                let possible_msg = self.base.rx.try_recv();
                if let Ok(new_msg) = possible_msg {
                    some_msg = Some(new_msg);
                } else {
                    break;
                }
            }
            if let Some(mut msg) = some_msg {
                self.msg_to_surface(&mut msg);
            }

            /*
            win_surface.fill_rect(
                sdl2::rect::Rect::new(100, 100, 200, 300),
                sdl2::pixels::Color::BLUE,
            ).unwrap();
            win_surface.update_window().unwrap();
            */
        }
        true
    }
}

pub fn main() {
    rosrust::init("view_image");
    let mut view_image = ViewImage::default(2000, 1000);
    let mut msg = make_test_image(1000, 1000);
    view_image.msg_to_surface(&mut msg);

    'running: loop {
        if !view_image.update() {
            break 'running;
        }
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}