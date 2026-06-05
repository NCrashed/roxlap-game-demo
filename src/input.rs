use sdl2::keyboard::Scancode;

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum PlayerInput {
    PitchCW,
    PitchCCW,
    YawCW,
    YawCCW,
    RollCW,
    RollCCW,
    IncTrust,
    DecTrust,
    Damp,
}

impl PlayerInput {
    pub fn from_scancode(scancode: Scancode) -> Option<Self> {
        match scancode {
            Scancode::A => Some(PlayerInput::YawCW),
            Scancode::D => Some(PlayerInput::YawCCW),
            Scancode::W => Some(PlayerInput::PitchCW),
            Scancode::S => Some(PlayerInput::PitchCCW),
            Scancode::Q => Some(PlayerInput::RollCCW),
            Scancode::E => Some(PlayerInput::RollCW),
            Scancode::LShift => Some(PlayerInput::IncTrust),
            Scancode::LCtrl => Some(PlayerInput::DecTrust),
            Scancode::Tab => Some(PlayerInput::Damp),
            _ => None,
        }
    }
}
