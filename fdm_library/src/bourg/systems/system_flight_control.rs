//This file contains the FlightControl System

//SPECS
use specs::prelude::*;

//Get data needed for the System to work
use crate::bourg::fdm::structures::KeyboardState;

//Get function to call
use crate::bourg::fdm::flight_control::flt_ctrl;

//System to handle user input
pub struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = WriteStorage<'a, KeyboardState>;

    fn run(&mut self, mut keyboardstate: Self::SystemData) 
    {
        for mut keystate in (&mut keyboardstate).join() 
        {
            //Call the flight control function
            flt_ctrl(&mut keystate);
        }
    }
}