use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody},
    Dt, PlayerInput,
};

// How fast angular velocity grows per second while a rotation key is held.
// Units: radians / s². Multiply by dt each frame → radians/s added to angular_vel.
const ANGULAR_ACCEL: f64 = 1.2;

// How fast linear velocity grows per second while a thrust key is held.
// Units: voxels / s². Multiply by dt each frame → voxels/s added to vel.
const LINEAR_ACCEL: f64 = 20.0;

#[system]
#[read_component(Miner)]
#[write_component(NewtonBody)]
pub fn miner_input(
    world: &mut SubWorld,
    #[resource] inputs: &HashSet<PlayerInput>,
    #[resource] dt: &Dt,
) {
    let mut query = <(&Miner, &mut NewtonBody)>::query();
    for (_, body) in query.iter_mut(world) {
        // Derive the three local axes from the body's current orientation quaternion.
        // Rotating a world-space basis vector by the quaternion gives the equivalent
        // axis in world space as the body sees it right now.
        let forward = body.orientation * DVec3::NEG_Z; // nose direction
        let right = body.orientation * DVec3::X; // right wing direction
        let up = body.orientation * DVec3::Y; // top of body direction

        for input in inputs {
            match input {
                PlayerInput::PitchCW => body.angular_vel += right * ANGULAR_ACCEL * dt.0,
                PlayerInput::PitchCCW => body.angular_vel -= right * ANGULAR_ACCEL * dt.0,
                PlayerInput::YawCW => body.angular_vel += up * ANGULAR_ACCEL * dt.0,
                PlayerInput::YawCCW => body.angular_vel -= up * ANGULAR_ACCEL * dt.0,
                PlayerInput::RollCW => body.angular_vel += forward * ANGULAR_ACCEL * dt.0,
                PlayerInput::RollCCW => body.angular_vel -= forward * ANGULAR_ACCEL * dt.0,
                PlayerInput::IncTrust => body.vel += forward * LINEAR_ACCEL * dt.0,
                PlayerInput::DecTrust => body.vel -= forward * LINEAR_ACCEL * dt.0,
            }
        }
    }
}
