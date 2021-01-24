//This file contains the FlightControl System

//SPECS
use specs::prelude::*;

//Get data needed
use crate::palmer::fdm::structures::KeyboardState;

//Get function to call
use crate::palmer::fdm::flight_control::flt_ctrl;

//System to handle user input
pub struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = WriteStorage<'a, KeyboardState>;

    fn run(&mut self, mut keyboardstate: Self::SystemData) 
    {
        for mut keystate in (&mut keyboardstate).join() 
        {
            //Call flight control function to handle key presses
            flt_ctrl(&mut keystate);
        }
    }
}