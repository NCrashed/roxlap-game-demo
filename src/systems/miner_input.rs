use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody, thruster::ThrusterBank},
    input::PlayerInput,
    Dt,
};

/// Linear acceleration applied per frame when a thrust key is held (m/s²).
const THRUST_ACCEL: f64 = 5.0;

pub fn apply_miner_input(inputs: &HashSet<PlayerInput>, bank: &mut ThrusterBank, mass: f64) {
    let cw = inputs.contains(&PlayerInput::RollCW);
    let ccw = inputs.contains(&PlayerInput::RollCCW);
    if cw == ccw {
        return;
    }
    let sign = if cw { 1.0_f64 } else { -1.0 };
    bank.command += DVec3::NEG_Z * (bank.max_accel(mass) * sign);
}

/// Translate the miner tangential to its nose: W/S = body ±Y (up/down),
/// A/D = body ±X (left/right).  No thrust along the nose axis.
pub fn apply_miner_translation(inputs: &HashSet<PlayerInput>, body: &mut NewtonBody, dt: f64) {
    let up = inputs.contains(&PlayerInput::ThrustUp);
    let down = inputs.contains(&PlayerInput::ThrustDown);
    let left = inputs.contains(&PlayerInput::ThrustLeft);
    let right = inputs.contains(&PlayerInput::ThrustRight);

    let mut local = DVec3::ZERO;
    if up != down {
        local.y = if up { 1.0 } else { -1.0 };
    }
    if left != right {
        local.x = if right { 1.0 } else { -1.0 };
    }

    if local.length_squared() > 1e-15 {
        body.vel += (body.orientation * local.normalize()) * (THRUST_ACCEL * dt);
    }
}

#[system]
#[read_component(Miner)]
#[write_component(NewtonBody)]
#[write_component(ThrusterBank)]
pub fn miner_input(
    world: &mut SubWorld,
    #[resource] inputs: &HashSet<PlayerInput>,
    #[resource] dt: &Dt,
) {
    let mut query = <(&Miner, &mut NewtonBody, &mut ThrusterBank)>::query();
    for (_, body, bank) in query.iter_mut(world) {
        apply_miner_input(inputs, bank, body.mass);
        apply_miner_translation(inputs, body, dt.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::{newton_body::NewtonBody, thruster::ThrusterBank};
    use glam::{DQuat, DVec3};
    use proptest::prelude::*;

    fn make_body() -> NewtonBody {
        NewtonBody {
            mass: 1.0,
            pos: DVec3::ZERO,
            vel: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_vel: DVec3::ZERO,
        }
    }

    fn make_bank() -> ThrusterBank {
        ThrusterBank::new(1.0, 0.3)
    }

    fn arb_player_input() -> impl Strategy<Value = PlayerInput> {
        prop_oneof![
            Just(PlayerInput::RollCW),
            Just(PlayerInput::RollCCW),
            Just(PlayerInput::ThrustUp),
            Just(PlayerInput::ThrustDown),
            Just(PlayerInput::ThrustLeft),
            Just(PlayerInput::ThrustRight),
        ]
    }

    fn arb_inputs() -> impl Strategy<Value = HashSet<PlayerInput>> {
        prop::collection::hash_set(arb_player_input(), 0..=2)
    }

    // No input: bank.command unchanged.
    #[test]
    fn no_input_bank_unchanged() {
        let mut bank = make_bank();
        apply_miner_input(&HashSet::new(), &mut bank, 1.0);
        assert_eq!(bank.command, DVec3::ZERO);
    }

    // Opposite keys cancel.
    #[test]
    fn opposite_roll_keys_cancel() {
        let mut both = HashSet::new();
        both.insert(PlayerInput::RollCW);
        both.insert(PlayerInput::RollCCW);
        let mut bank = make_bank();
        apply_miner_input(&both, &mut bank, 1.0);
        assert_eq!(bank.command, DVec3::ZERO);
    }

    // Any input must not produce NaN or infinite command.
    proptest! {
        #[test]
        fn no_nan_or_inf(inputs in arb_inputs()) {
            let mut bank = make_bank();
            apply_miner_input(&inputs, &mut bank, 1.0);
            prop_assert!(bank.command.is_finite());
        }
    }

    // Roll command is along body -Z only.
    #[test]
    fn roll_command_along_neg_z() {
        let mut bank = make_bank();
        let inputs: HashSet<PlayerInput> = [PlayerInput::RollCW].into_iter().collect();
        apply_miner_input(&inputs, &mut bank, 1.0);
        let perp = bank.command - DVec3::NEG_Z * bank.command.dot(DVec3::NEG_Z);
        assert!(
            perp.length() < 1e-12,
            "command has off-axis component: {perp:?}"
        );
    }

    // ── apply_miner_translation ───────────────────────────────────────────

    #[test]
    fn no_thrust_input_vel_unchanged() {
        let mut body = make_body();
        apply_miner_translation(&HashSet::new(), &mut body, 1.0 / 60.0);
        assert_eq!(body.vel, DVec3::ZERO);
    }

    #[test]
    fn opposite_thrust_keys_cancel() {
        for (a, b) in [
            (PlayerInput::ThrustUp, PlayerInput::ThrustDown),
            (PlayerInput::ThrustLeft, PlayerInput::ThrustRight),
        ] {
            let inputs: HashSet<PlayerInput> = [a, b].into_iter().collect();
            let mut body = make_body();
            apply_miner_translation(&inputs, &mut body, 1.0 / 60.0);
            assert_eq!(body.vel, DVec3::ZERO);
        }
    }

    #[test]
    fn up_thrust_along_body_y() {
        let inputs: HashSet<PlayerInput> = [PlayerInput::ThrustUp].into_iter().collect();
        let mut body = make_body(); // identity orientation: body Y = world Y
        apply_miner_translation(&inputs, &mut body, 1.0);
        assert!(body.vel.dot(DVec3::Y) > 0.0, "up thrust not along body +Y");
        let perp = body.vel - DVec3::Y * body.vel.dot(DVec3::Y);
        assert!(perp.length() < 1e-12, "spurious off-axis component");
    }

    #[test]
    fn right_thrust_along_body_x() {
        let inputs: HashSet<PlayerInput> = [PlayerInput::ThrustRight].into_iter().collect();
        let mut body = make_body();
        apply_miner_translation(&inputs, &mut body, 1.0);
        assert!(body.vel.dot(DVec3::X) > 0.0, "right thrust not along +X");
        let perp = body.vel - DVec3::X * body.vel.dot(DVec3::X);
        assert!(perp.length() < 1e-12, "spurious off-axis component");
    }

    #[test]
    fn thrust_respects_orientation() {
        // Ship pitched 90° around X: nose now points +Y.
        use std::f64::consts::FRAC_PI_2;
        let inputs: HashSet<PlayerInput> = [PlayerInput::ThrustUp].into_iter().collect();
        let mut body = make_body();
        body.orientation = DQuat::from_rotation_x(FRAC_PI_2);
        apply_miner_translation(&inputs, &mut body, 1.0);
        // rotation_x(π/2): body +Y maps to world +Z. Vel must be non-zero and pos unchanged.
        assert!(
            body.vel.length() > 1e-9,
            "no vel after thrust with non-identity orientation"
        );
        assert_eq!(body.pos, DVec3::ZERO);
    }
}
