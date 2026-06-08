use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, thruster::ThrusterBank},
    input::PlayerInput,
    Dt,
};

/// Roll angular acceleration (rad/s²) when a roll key is held.
const ANGULAR_ACCEL: f64 = 1.2;

pub fn apply_miner_input(inputs: &HashSet<PlayerInput>, dt: f64, bank: &mut ThrusterBank) {
    let mut net_roll: f64 = 0.0;
    for input in inputs {
        match input {
            PlayerInput::RollCW => net_roll += 1.0,
            PlayerInput::RollCCW => net_roll -= 1.0,
        }
    }
    if net_roll != 0.0 {
        // Body-space: −Z is the forward/roll axis.
        bank.command += DVec3::NEG_Z * (ANGULAR_ACCEL * dt * net_roll);
    }
}

#[system]
#[read_component(Miner)]
#[write_component(ThrusterBank)]
pub fn miner_input(
    world: &mut SubWorld,
    #[resource] inputs: &HashSet<PlayerInput>,
    #[resource] dt: &Dt,
) {
    let mut query = <(&Miner, &mut ThrusterBank)>::query();
    for (_, bank) in query.iter_mut(world) {
        apply_miner_input(inputs, dt.0, bank);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::thruster::ThrusterBank;
    use proptest::prelude::*;

    fn make_bank() -> ThrusterBank {
        ThrusterBank::new(1.0, 0.75)
    }

    fn arb_player_input() -> impl Strategy<Value = PlayerInput> {
        prop_oneof![Just(PlayerInput::RollCW), Just(PlayerInput::RollCCW),]
    }

    fn arb_inputs() -> impl Strategy<Value = HashSet<PlayerInput>> {
        prop::collection::hash_set(arb_player_input(), 0..=2)
    }

    fn arb_dt() -> impl Strategy<Value = f64> {
        0.001f64..0.1
    }

    // No input: bank.command unchanged.
    proptest! {
        #[test]
        fn no_input_bank_unchanged(dt in arb_dt()) {
            let mut bank = make_bank();
            apply_miner_input(&HashSet::new(), dt, &mut bank);
            prop_assert_eq!(bank.command, DVec3::ZERO);
        }
    }

    // Opposite keys cancel.
    proptest! {
        #[test]
        fn opposite_roll_keys_cancel(dt in arb_dt()) {
            let mut both = HashSet::new();
            both.insert(PlayerInput::RollCW);
            both.insert(PlayerInput::RollCCW);
            let mut bank = make_bank();
            apply_miner_input(&both, dt, &mut bank);
            prop_assert_eq!(bank.command, DVec3::ZERO);
        }
    }

    // Any input must not produce NaN or infinite command.
    proptest! {
        #[test]
        fn no_nan_or_inf(inputs in arb_inputs(), dt in arb_dt()) {
            let mut bank = make_bank();
            apply_miner_input(&inputs, dt, &mut bank);
            prop_assert!(bank.command.is_finite());
        }
    }

    // Roll command is along body -Z only.
    proptest! {
        #[test]
        fn roll_command_along_neg_z(dt in arb_dt()) {
            let mut bank = make_bank();
            let inputs: HashSet<PlayerInput> = [PlayerInput::RollCW].into_iter().collect();
            apply_miner_input(&inputs, dt, &mut bank);
            // Command should be along NEG_Z (or zero for no input)
            let perp = bank.command - DVec3::NEG_Z * bank.command.dot(DVec3::NEG_Z);
            prop_assert!(perp.length() < 1e-12, "command has off-axis component: {perp:?}");
        }
    }
}
