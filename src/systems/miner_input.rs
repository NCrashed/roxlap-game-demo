use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody},
    input::PlayerInput,
    Dt,
};

const ANGULAR_ACCEL: f64 = 1.2;
const LINEAR_ACCEL: f64 = 20.0;

pub fn apply_miner_input(inputs: &HashSet<PlayerInput>, dt: f64, body: &mut NewtonBody) {
    let forward = body.orientation * DVec3::NEG_Z;
    let angular_step = ANGULAR_ACCEL * dt;
    let linear_step = LINEAR_ACCEL * dt;

    let mut net_roll: f64 = 0.0;
    let mut damp = false;

    for input in inputs {
        match input {
            PlayerInput::RollCW => net_roll += 1.0,
            PlayerInput::RollCCW => net_roll -= 1.0,
            PlayerInput::Damp => damp = true,
        }
    }

    // Roll around forward axis
    let free = 1.0 - net_roll.abs();
    let brake = damp as u8 as f64 * free;
    let roll_v = body.angular_vel.dot(forward);
    let roll_delta =
        angular_step * net_roll - roll_v.signum() * angular_step.min(roll_v.abs()) * brake;
    body.angular_vel += forward * roll_delta;

    if damp {
        // Kill non-roll angular velocity (autopilot steering axes)
        let roll_component = body.angular_vel.dot(forward) * forward;
        let lateral = body.angular_vel - roll_component;
        let lat_len = lateral.length();
        if lat_len > angular_step {
            body.angular_vel -= lateral.normalize() * angular_step;
        } else {
            body.angular_vel -= lateral;
        }
        // Kill linear velocity
        let speed = body.vel.length();
        if speed > linear_step {
            body.vel -= body.vel.normalize() * linear_step;
        } else {
            body.vel = DVec3::ZERO;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    use proptest::prelude::*;

    fn arb_player_input() -> impl Strategy<Value = PlayerInput> {
        prop_oneof![
            Just(PlayerInput::RollCW),
            Just(PlayerInput::RollCCW),
            Just(PlayerInput::Damp),
        ]
    }

    fn arb_inputs() -> impl Strategy<Value = HashSet<PlayerInput>> {
        prop::collection::hash_set(arb_player_input(), 0..=3)
    }

    fn arb_vec3() -> impl Strategy<Value = DVec3> {
        (-50.0f64..50.0, -50.0f64..50.0, -50.0f64..50.0).prop_map(|(x, y, z)| DVec3::new(x, y, z))
    }

    fn arb_body() -> impl Strategy<Value = NewtonBody> {
        (arb_vec3(), arb_vec3()).prop_map(|(vel, angular_vel)| NewtonBody {
            mass: 1.0,
            pos: DVec3::ZERO,
            vel,
            angular_vel,
            orientation: DQuat::IDENTITY,
        })
    }

    fn arb_dt() -> impl Strategy<Value = f64> {
        0.001f64..0.1
    }

    // No input: body must be unchanged.
    proptest! {
        #[test]
        fn no_input_body_unchanged(mut body in arb_body(), dt in arb_dt()) {
            let before = body;
            apply_miner_input(&HashSet::new(), dt, &mut body);
            prop_assert_eq!(body.vel, before.vel);
            prop_assert_eq!(body.angular_vel, before.angular_vel);
        }
    }

    // Tab held: angular and linear velocity magnitudes must not increase.
    proptest! {
        #[test]
        fn tab_does_not_grow(mut body in arb_body(), dt in arb_dt()) {
            let inputs: HashSet<PlayerInput> = [PlayerInput::Damp].into_iter().collect();
            let before_ang = body.angular_vel.length();
            let before_lin = body.vel.length();
            apply_miner_input(&inputs, dt, &mut body);
            prop_assert!(body.angular_vel.length() <= before_ang + 1e-12);
            prop_assert!(body.vel.length() <= before_lin + 1e-12);
        }
    }

    // Opposite roll keys cancel each other.
    proptest! {
        #[test]
        fn opposite_roll_keys_cancel(mut body in arb_body(), dt in arb_dt()) {
            let mut body_no_input = body;
            let mut both = HashSet::new();
            both.insert(PlayerInput::RollCW);
            both.insert(PlayerInput::RollCCW);
            apply_miner_input(&both, dt, &mut body);
            apply_miner_input(&HashSet::new(), dt, &mut body_no_input);
            let fwd = body.orientation * DVec3::NEG_Z;
            let diff = (body.angular_vel.dot(fwd) - body_no_input.angular_vel.dot(fwd)).abs();
            prop_assert!(diff < 1e-12);
        }
    }

    // Any input combination must not produce NaN or infinite velocities.
    proptest! {
        #[test]
        fn no_nan_or_inf(mut body in arb_body(), inputs in arb_inputs(), dt in arb_dt()) {
            apply_miner_input(&inputs, dt, &mut body);
            prop_assert!(body.vel.is_finite());
            prop_assert!(body.angular_vel.is_finite());
        }
    }
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
        apply_miner_input(inputs, dt.0, body);
    }
}
