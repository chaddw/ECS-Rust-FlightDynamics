//KeyboardState Component

//SPECS
use specs::prelude::*;


//Component state machine for keyboard presses
#[derive(Debug)]
pub struct KeyboardState
{
    pub thrust_up: bool,
    pub thrust_down: bool,
    pub left_rudder: bool,
    pub right_rudder: bool,
    pub roll_left: bool,
    pub roll_right: bool,
    pub pitch_up: bool,
    pub pitch_down: bool,
    pub flaps_down: bool,
    pub zero_flaps: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}

