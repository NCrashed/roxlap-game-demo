use legion::{Resources, Schedule, World};
use sdl2::{
    event::Event,
    keyboard::Scancode,
    mixer::InitFlag,
    render::{Canvas, TextureCreator, WindowCanvas},
    video::{Window, WindowContext},
    EventPump,
};

const INITIAL_WINDOW_WIDTH: u32 = 1280;
const INITIAL_WINDOW_HEIGHT: u32 = 720;

pub struct CanvasResources {
    pub canvas: Canvas<Window>,
    pub texture_creator: TextureCreator<WindowContext>,
}

fn initialize() -> Result<(WindowCanvas, EventPump), String> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;
    let _audio = sdl_context.audio()?;

    let _mixer_context =
        sdl2::mixer::init(InitFlag::MP3 | InitFlag::FLAC | InitFlag::MOD | InitFlag::OGG)?;
    sdl2::mixer::allocate_channels(20);

    let window = video_subsystem
        .window("ROXLAP GAME DEMO", INITIAL_WINDOW_WIDTH, INITIAL_WINDOW_HEIGHT)
        .resizable()
        .position_centered()
        .build()
        .expect("could not initialize video subsystem");

    let mut canvas = window
        .into_canvas()
        .accelerated()
        .present_vsync()
        .build()
        .expect("could not make a canvas");

    canvas.present();

    let event_pump = sdl_context.event_pump().unwrap();

    Ok((canvas, event_pump))
}

fn initial_resources(canvas: Canvas<Window>, world: &World) -> Resources {
    let mut resources = Resources::default();
    let texture_creator = canvas.texture_creator();

    let canvas_resources = CanvasResources {
        canvas,
        texture_creator,
    };
    resources.insert(canvas_resources);

    resources
}

fn main() {
    std::env::set_var("RUST_LOG", "info");
    std::env::set_var("RUST_BACKTRACE", "1");
    env_logger::init();
    let (canvas, mut event_pump) = initialize().unwrap();

    let mut schedule = Schedule::builder().build();
    let mut world = World::default();
    let mut resources = initial_resources(canvas, &mut world);

    loop {
        schedule.execute(&mut world, &mut resources);
    }
}
