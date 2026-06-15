use bytemuck::Zeroable;
use glam::{DQuat, DVec3};
use legion::{system, systems::CommandBuffer, world::SubWorld, IntoQuery};
use rand::RngExt;
use roxlap_gpu::{GpuRenderer, SpriteInstance, SpriteInstanceTransform};

use crate::{
    components::{asteroid::AsteroidMarker, miner::Miner, newton_body::NewtonBody},
    generation::chunks::{missing_chunks, CHUNK_SIZE, LOAD_RADIUS},
    world::build_asteroid_sprite_model,
    GeneratedChunks, SpriteData,
};

/// Asteroids spawned per chunk (placed at chunk centre).
const ASTEROIDS_PER_CHUNK: u32 = 1;
/// Maximum chunks populated per tick to avoid first-frame hitching.
const CHUNKS_PER_TICK: usize = 32;

#[system]
#[read_component(Miner)]
#[read_component(NewtonBody)]
pub fn chunk_population(
    #[resource] generated: &mut GeneratedChunks,
    #[resource] gpu: &mut GpuRenderer,
    #[resource] sprite_data: &mut SpriteData,
    world: &SubWorld,
    commands: &mut CommandBuffer,
) {
    let ship_pos = {
        let mut q = <(&Miner, &NewtonBody)>::query();
        match q.iter(world).next() {
            Some((_, body)) => body.pos,
            None => return,
        }
    };

    let to_generate: Vec<_> = missing_chunks(ship_pos, LOAD_RADIUS, &generated.0)
        .into_iter()
        .take(CHUNKS_PER_TICK)
        .collect();

    if to_generate.is_empty() {
        return;
    }

    let mut rng = rand::rng();
    let mut next_id = sprite_data.instance_count;

    for &chunk in &to_generate {
        let chunk_centre = (chunk.as_dvec3() + DVec3::splat(0.5)) * CHUNK_SIZE as f64;
        for _ in 0..ASTEROIDS_PER_CHUNK {
            // Each asteroid gets its own model so individual voxels can be
            // edited independently when the asteroid is damaged or destroyed.
            sprite_data.registry.add(build_asteroid_sprite_model());
            let angular_vel = DVec3::new(
                (rng.random::<f64>() - 0.5) * 2.0,
                (rng.random::<f64>() - 0.5) * 2.0,
                (rng.random::<f64>() - 0.5) * 2.0,
            );
            commands.push((
                AsteroidMarker { model_id: next_id },
                NewtonBody {
                    mass: 1.0,
                    pos: chunk_centre,
                    vel: DVec3::ZERO,
                    orientation: DQuat::IDENTITY,
                    angular_vel,
                },
            ));
            next_id += 1;
        }
        generated.0.insert(chunk);
    }

    // Each slot i uses model i (1:1 mapping).
    let placeholder = SpriteInstanceTransform::zeroed();
    let instances: Vec<SpriteInstance> = (0..next_id)
        .map(|id| SpriteInstance {
            model_id: id,
            transform: placeholder,
        })
        .collect();
    gpu.set_sprite_instances(&sprite_data.registry, &instances);
    sprite_data.instance_count = next_id;
}
