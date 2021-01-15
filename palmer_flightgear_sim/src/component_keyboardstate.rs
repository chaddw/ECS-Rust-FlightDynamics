//KeyboardState Component

//SPECS
use specs::prelude::*;


//Component tracking whether a key is pressed or not
#[derive(Debug)]
pub struct KeyboardState
{
    pub throttle_up: bool,
    pub throttle_down: bool,
    pub aoa_up: bool,
    pub aoa_down: bool,
    pub bank_right: bool,
    pub bank_left: bool,
    pub flaps_down: bool,
    pub zero_flaps: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}