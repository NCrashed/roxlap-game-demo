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

/// Damp `vel`'s component along `axis` toward zero by at most `amount`,
/// without overshooting.
#[cfg(test)]
fn damp_axis(vel: &mut DVec3, axis: DVec3, amount: f64) {
    let v = vel.dot(axis);
    *vel -= axis * v.signum() * amount.min(v.abs());
}

pub fn apply_miner_input(inputs: &HashSet<PlayerInput>, dt: f64, body: &mut NewtonBody) {
    let forward = body.orientation * DVec3::NEG_Z;
    let right = body.orientation * DVec3::X;
    let up = body.orientation * DVec3::Y;

    let angular_step = ANGULAR_ACCEL * dt;
    let linear_step = LINEAR_ACCEL * dt;

    let mut net_pitch: f64 = 0.0;
    let mut net_yaw: f64 = 0.0;
    let mut net_roll: f64 = 0.0;
    let mut net_thrust: f64 = 0.0;
    let mut damp = false;

    for input in inputs {
        match input {
            PlayerInput::PitchCW => net_pitch += 1.0,
            PlayerInput::PitchCCW => net_pitch -= 1.0,
            PlayerInput::YawCW => net_yaw += 1.0,
            PlayerInput::YawCCW => net_yaw -= 1.0,
            PlayerInput::RollCW => net_roll += 1.0,
            PlayerInput::RollCCW => net_roll -= 1.0,
            PlayerInput::IncTrust => net_thrust += 1.0,
            PlayerInput::DecTrust => net_thrust -= 1.0,
            PlayerInput::Damp => damp = true,
        }
    }

    // Each update is a single expression: accel term + damp term, one always zero.
    // free: 1.0 when axis undriven, 0.0 when key held (kills damp term).
    let axis_delta = |net: f64, v: f64, step: f64| -> f64 {
        let free = 1.0 - net.abs();
        let brake = damp as u8 as f64 * free;
        step * net - v.signum() * step.min(v.abs()) * brake
    };

    body.angular_vel += right * axis_delta(net_pitch, body.angular_vel.dot(right), angular_step);
    body.angular_vel += up * axis_delta(net_yaw, body.angular_vel.dot(up), angular_step);
    body.angular_vel += forward * axis_delta(net_roll, body.angular_vel.dot(forward), angular_step);
    body.vel += forward * axis_delta(net_thrust, body.vel.dot(forward), linear_step);
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    use proptest::prelude::*;

    // ── Strategies ──────────────────────────────────────────────────────────

    fn arb_player_input() -> impl Strategy<Value = PlayerInput> {
        prop_oneof![
            Just(PlayerInput::PitchCW),
            Just(PlayerInput::PitchCCW),
            Just(PlayerInput::YawCW),
            Just(PlayerInput::YawCCW),
            Just(PlayerInput::RollCW),
            Just(PlayerInput::RollCCW),
            Just(PlayerInput::IncTrust),
            Just(PlayerInput::DecTrust),
            Just(PlayerInput::Damp),
        ]
    }

    fn arb_inputs() -> impl Strategy<Value = HashSet<PlayerInput>> {
        prop::collection::hash_set(arb_player_input(), 0..=9)
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

    // ── Properties ──────────────────────────────────────────────────────────

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

    // damp_axis must never overshoot: the velocity component along an axis
    // must not change sign after damping.
    proptest! {
        #[test]
        fn damp_axis_never_overshoots(
            vx in -50.0f64..50.0,
            vy in -50.0f64..50.0,
            vz in -50.0f64..50.0,
            amount in 0.0f64..100.0,
        ) {
            let axis = DVec3::X;
            let mut vel = DVec3::new(vx, vy, vz);
            let before_proj = vel.dot(axis);
            damp_axis(&mut vel, axis, amount);
            let after_proj = vel.dot(axis);
            prop_assert!(before_proj * after_proj >= 0.0);
        }
    }

    // Tab held, axis free: velocity magnitude must not increase.
    proptest! {
        #[test]
        fn tab_free_axis_does_not_grow(mut body in arb_body(), dt in arb_dt()) {
            let inputs: HashSet<PlayerInput> = [PlayerInput::Damp].into_iter().collect();
            let right = DVec3::X;
            let up    = DVec3::Y;
            let fwd   = DVec3::NEG_Z;
            let before_pitch  = body.angular_vel.dot(right).abs();
            let before_yaw    = body.angular_vel.dot(up).abs();
            let before_roll   = body.angular_vel.dot(fwd).abs();
            let before_thrust = body.vel.dot(fwd).abs();
            apply_miner_input(&inputs, dt, &mut body);
            prop_assert!(body.angular_vel.dot(right).abs() <= before_pitch  + 1e-12);
            prop_assert!(body.angular_vel.dot(up).abs()    <= before_yaw    + 1e-12);
            prop_assert!(body.angular_vel.dot(fwd).abs()   <= before_roll   + 1e-12);
            prop_assert!(body.vel.dot(fwd).abs()           <= before_thrust + 1e-12);
        }
    }

    // Opposite keys cancel on pitch axis.
    proptest! {
        #[test]
        fn opposite_keys_cancel(mut body in arb_body(), dt in arb_dt()) {
            let mut body_cancel = body;
            let mut both = HashSet::new();
            both.insert(PlayerInput::PitchCW);
            both.insert(PlayerInput::PitchCCW);
            apply_miner_input(&both, dt, &mut body);
            apply_miner_input(&HashSet::new(), dt, &mut body_cancel);
            let diff = (body.angular_vel.dot(DVec3::X) - body_cancel.angular_vel.dot(DVec3::X)).abs();
            prop_assert!(diff < 1e-12);
        }
    }

    // Any combination of inputs must not produce NaN or infinite velocities.
    proptest! {
        #[test]
        fn no_nan_or_inf(
            mut body in arb_body(),
            inputs in arb_inputs(),
            dt in arb_dt(),
        ) {
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
