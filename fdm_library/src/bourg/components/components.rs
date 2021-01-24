//Components
//((((conflicting implementation for `flightgear::FGNetFDM))))!!!

//SPECS
use specs::prelude::*;

//Import structures to make into Components
use crate::bourg::fdm::structures::KeyboardState;
use crate::bourg::fdm::structures::DataFDM;
//use crate::flightgear::FGNetFDM;

//Component holding the state of the airplane
impl Component for DataFDM
{
    type Storage = VecStorage<Self>;
}

//Component for tracking the state of the keyboard
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}

// //Component containing the FGNetFDM structure to be sent to FlightGear
// impl Component for FGNetFDM
// {
//     type Storage = VecStorage<Self>;
// }