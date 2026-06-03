use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody},
    Dt, PlayerInput,
};

const ANGULAR_ACCEL: f64 = 1.2;
const LINEAR_ACCEL: f64 = 20.0;

/// Damp `vel`'s component along `axis` toward zero by at most `amount`,
/// without overshooting.
fn damp_axis(vel: &mut DVec3, axis: DVec3, amount: f64) {
    let v = vel.dot(axis);
    *vel -= axis * v.signum() * amount.min(v.abs());
}

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
        let forward = body.orientation * DVec3::NEG_Z;
        let right = body.orientation * DVec3::X;
        let up = body.orientation * DVec3::Y;

        let damp = ANGULAR_ACCEL * dt.0;

        // Acceleration for held keys.
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

        // Deceleration for unpressed rotation axes — symmetric with acceleration
        // so stopping takes the same time as spinning up.
        if !inputs.contains(&PlayerInput::PitchCW) && !inputs.contains(&PlayerInput::PitchCCW) {
            damp_axis(&mut body.angular_vel, right, damp);
        }
        if !inputs.contains(&PlayerInput::YawCW) && !inputs.contains(&PlayerInput::YawCCW) {
            damp_axis(&mut body.angular_vel, up, damp);
        }
        if !inputs.contains(&PlayerInput::RollCW) && !inputs.contains(&PlayerInput::RollCCW) {
            damp_axis(&mut body.angular_vel, forward, damp);
        }
        if !inputs.contains(&PlayerInput::IncTrust) && !inputs.contains(&PlayerInput::DecTrust) {
            damp_axis(&mut body.vel, forward, LINEAR_ACCEL * dt.0);
        }
    }
}
