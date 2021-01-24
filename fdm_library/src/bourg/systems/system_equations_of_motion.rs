//This file contains EquationsOfMotion System

//SPECS
use specs::prelude::*;

//Get Component data needed to perform the System operations
use crate::bourg::fdm::structures::DataFDM;
use crate::bourg::fdm::structures::KeyboardState;

//Get Resources
use crate::bourg::resources::delta_time::DeltaTime;
use crate::bourg::resources::max_thrust::MaxThrust;
use crate::bourg::resources::delta_thrust::DeltaThrust;

//Function to call
use crate::bourg::fdm::equations_of_motion::eom;

//System to perform equations of motion physics calculations based on forces
pub struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        Read<'a, DeltaTime>,
        Read<'a, MaxThrust>,
        Read<'a, DeltaThrust>,
        WriteStorage<'a, DataFDM>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (dt, max_thrust, d_thrust, mut datafdm, keyboardstate): Self::SystemData) 
    {
        //Get resources
        let dt = dt.0;
        let d_thrust = d_thrust.0;
        let max_thrust = max_thrust.0;

        for (mut fdm, keystate) in (&mut datafdm, &keyboardstate).join() 
        {
            //Call eom function, which also calls calc_loads
            eom(&mut fdm, &keystate, dt, d_thrust, max_thrust);
        }
    }
}





