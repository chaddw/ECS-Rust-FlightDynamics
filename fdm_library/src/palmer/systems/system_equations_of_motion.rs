//This file contains EquationsOfMotion System

//SPECS
use specs::prelude::*;

//Get data needed to perform the System operations
use crate::palmer::fdm::structures::KeyboardState;
use crate::palmer::fdm::structures::DataFDM;
use crate::palmer::resources::delta_time::DeltaTime;

//Get function to call
use crate::palmer::fdm::equations_of_motion::eom;

//System to perform physics calculations using a 4th-order Runge-Kutta solver
pub struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        Read<'a, DeltaTime>,
        WriteStorage<'a, DataFDM>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (dt, mut datafdm, keyboardstate): Self::SystemData) 
    {
        //Get DeltaTime resource
        let dt = dt.0;
        for (mut fdm, keystate) in (&mut datafdm, &keyboardstate).join() 
        {
            //Call eom function, which calls plane_right_hand_side function
            eom(&mut fdm, &keystate, dt);
        }
    }
}

