use glam::DVec3;
use legion::{system, world::SubWorld, IntoQuery};
use roxlap_core::{
    opticast, rasterizer::ScratchPool, scalar_rasterizer::ScalarRasterizer, update_lighting,
    Camera, Engine, GridView, OpticastSettings,
};
use roxlap_formats::{edit::MAXZDIM, vxl::Vxl};
use sdl2::pixels::{Color, PixelFormatEnum};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody},
    fonts::FontRenderer,
    systems::performance_info::PerformanceInfo,
    CanvasResources,
};

const WIDTH: u32 = 800;
const HEIGHT: u32 = 600;

const VSID: u32 = 32;
const GROUND_Z: i32 = 200;
const CUBE_EDGE: i32 = 10;

#[system]
#[read_component(Miner)]
#[read_component(NewtonBody)]
pub fn render(
    #[resource] canvas_resources: &mut CanvasResources,
    #[resource] world_map: &mut Vxl,
    #[resource] engine: &mut Engine,
    #[resource] font_renderer: &FontRenderer,
    #[resource] perf: &PerformanceInfo,
    world: &SubWorld,
) {
    update_lighting(
        &mut world_map.data,
        &world_map.column_offset,
        world_map.vsid,
        0,
        0,
        0,
        world_map.vsid as i32,
        world_map.vsid as i32,
        MAXZDIM,
        engine.lightmode(),
        engine.lights(),
    );

    // Translate the miner's rigid-body state to a voxlap Camera.
    //
    // Body-local axis conventions (matching miner_input.rs):
    //   -Z = nose (forward), +X = right wing, +Y = top of body
    //
    // Rotating each body-local axis by `orientation` gives its world-space
    // direction.  Voxlap's Camera wants `down` (not `up`), so we negate Y.
    let camera = {
        let mut query = <(&Miner, &NewtonBody)>::query();
        if let Some((_, body)) = query.iter(world).next() {
            let fwd = body.orientation * DVec3::NEG_Z;
            let right = body.orientation * DVec3::X;
            let up = body.orientation * DVec3::Y;
            Camera {
                pos: body.pos.to_array(),
                forward: fwd.to_array(),
                right: right.to_array(),
                down: (-up).to_array(),
            }
        } else {
            // Fallback: fixed view matching the original hardcoded demo camera
            // (yaw = 0, pitch = 0.15 rad downward, hovering in front of the cube).
            let cx = f64::from(VSID) * 0.5;
            let cy = f64::from(VSID) * 0.5;
            let cz = f64::from(GROUND_Z) - f64::from(CUBE_EDGE) - 6.0;
            let (sp, cp) = (0.15_f64.sin(), 0.15_f64.cos());
            Camera {
                pos: [cx - 16.0, cy, cz],
                forward: [cp, 0.0, sp],
                right: [0.0, 1.0, 0.0],
                down: [-sp, 0.0, cp],
            }
        }
    };

    let mut pool = ScratchPool::new(WIDTH, HEIGHT, world_map.vsid);
    let mut framebuffer: Vec<u32> = vec![0u32; (WIDTH * HEIGHT) as usize];
    let mut zbuffer: Vec<f32> = vec![0.0f32; (WIDTH * HEIGHT) as usize];
    let settings = OpticastSettings::for_oracle_framebuffer(WIDTH, HEIGHT);

    {
        let grid = GridView::from_single_vxl(&world_map);
        let mut rasterizer =
            ScalarRasterizer::new(&mut framebuffer, &mut zbuffer, WIDTH as usize, grid);
        let _ = opticast(&mut rasterizer, &mut pool, &camera, &settings, grid);
    }

    let mut texture = canvas_resources
        .texture_creator
        .create_texture_streaming(PixelFormatEnum::ARGB8888, WIDTH, HEIGHT)
        .map_err(|e| e.to_string())
        .unwrap();

    let row_bytes = (WIDTH * 4) as usize;
    texture
        .update(None, bytemuck::cast_slice(&framebuffer), row_bytes)
        .expect("Failed to update texture");

    canvas_resources.canvas.clear();
    canvas_resources.canvas.copy(&texture, None, None).unwrap();

    font_renderer.draw_text(
        &mut canvas_resources.canvas,
        &canvas_resources.texture_creator,
        &format!("FPS {}\nF.TIME {} uS", perf.fps, perf.frame_time_us),
        4,
        4,
        16.0,
        Color::YELLOW,
    );

    canvas_resources.canvas.present();
}
