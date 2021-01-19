//This file contains the FlightControl System

//SPECS
use specs::prelude::*;

//Flight control
use device_query::{DeviceQuery, DeviceState, Keycode};

//Exit program
use std::process;

//Get data needed for the System to work
use crate::bourg_fdm::components::component_keyboardstate::KeyboardState;

//System to handle user input
pub struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = WriteStorage<'a, KeyboardState>;

    fn run(&mut self, mut keyboardstate: Self::SystemData) 
    {
        for mut keystate in (&mut keyboardstate).join() 
        {
            //Set all states false before we know if they are being activated
            //rudder, ailerons, and elevators will be zerod in EOM system
            keystate.thrust_up = false; 
            keystate.thrust_down = false;

            keystate.left_rudder = false;
            keystate.right_rudder = false;

            keystate.roll_left = false;
            keystate.roll_right = false;
 
            keystate.pitch_up = false;
            keystate.pitch_down = false;

            //Flaps are toggled on and off, so they dont need to be set to false each time

            //Setup device query states
            let device_state = DeviceState::new();
            let keys: Vec<Keycode> = device_state.get_keys();

            //Thrust
            if keys.contains(&Keycode::A)
            {
                keystate.thrust_up = true;
            }
            else if keys.contains(&Keycode::Z)
            {
                keystate.thrust_down = true;
            }

            //Rudders for yaw
            if keys.contains(&Keycode::N)
            {
                keystate.left_rudder = true;
            }
            else if keys.contains(&Keycode::M)
            {
                keystate.right_rudder = true;
            }

            //Ailerons for roll
            if keys.contains(&Keycode::Left)
            {
                keystate.roll_left = true;
            }
            else if keys.contains(&Keycode::Right)
            {
                keystate.roll_right = true;
            }

            //Elevators for Pitch
            if keys.contains(&Keycode::Down)
            {
                keystate.pitch_up = true;
            }
            else if keys.contains(&Keycode::Up)
            {
                keystate.pitch_down = true;
            }

            //Flaps for lift
            if keys.contains(&Keycode::F)
            {
                keystate.flaps_down = true;
                keystate.zero_flaps = false;
            }
            else if keys.contains(&Keycode::G)
            {
                keystate.zero_flaps = true;
                keystate.flaps_down = false;
            }

            //Quit program
            if keys.contains(&Keycode::Q)
            {
                process::exit(1);
            }

        }
    }
}