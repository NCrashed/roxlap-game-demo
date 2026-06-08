use sdl2::keyboard::Scancode;

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum PlayerInput {
    RollCW,
    RollCCW,
}

impl PlayerInput {
    pub fn from_scancode(scancode: Scancode) -> Option<Self> {
        match scancode {
            Scancode::Q => Some(PlayerInput::RollCCW),
            Scancode::E => Some(PlayerInput::RollCW),
            _ => None,
        }
    }
}
