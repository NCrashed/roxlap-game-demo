use std::collections::HashSet;

use glam::DVec3;
use legion::{world::SubWorld, *};

use crate::{
    components::{miner::Miner, newton_body::NewtonBody, thruster::ThrusterBank},
    input::PlayerInput,
};

pub fn apply_miner_input(inputs: &HashSet<PlayerInput>, bank: &mut ThrusterBank, mass: f64) {
    let cw = inputs.contains(&PlayerInput::RollCW);
    let ccw = inputs.contains(&PlayerInput::RollCCW);
    if cw == ccw {
        return;
    }
    let sign = if cw { 1.0_f64 } else { -1.0 };
    bank.command += DVec3::NEG_Z * (bank.max_accel(mass) * sign);
}

#[system]
#[read_component(Miner)]
#[read_component(NewtonBody)]
#[write_component(ThrusterBank)]
pub fn miner_input(world: &mut SubWorld, #[resource] inputs: &HashSet<PlayerInput>) {
    let mut query = <(&Miner, &NewtonBody, &mut ThrusterBank)>::query();
    for (_, body, bank) in query.iter_mut(world) {
        apply_miner_input(inputs, bank, body.mass);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::thruster::ThrusterBank;
    use proptest::prelude::*;

    fn make_bank() -> ThrusterBank {
        ThrusterBank::new(1.0, 0.3)
    }

    fn arb_player_input() -> impl Strategy<Value = PlayerInput> {
        prop_oneof![Just(PlayerInput::RollCW), Just(PlayerInput::RollCCW),]
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
}
