//This file contains EquationsOfMotion System

//Get data needed to perform the System operations
use crate::data::KeyboardState;
use crate::data::DataFDM;
use crate::data::DeltaTime;

//SPECS
use specs::prelude::*;

//System to perform physics calculations using a 4th-order Runge-Kutta solver
pub struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        Read<'a, DeltaTime>,
        WriteStorage<'a, DataFDM>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (delta, mut datafdm, keyboardstate): Self::SystemData) 
    {
        //Get DeltaTime resource
        let delta = delta.0;
        for (mut fdm, keystate) in (&mut datafdm, &keyboardstate).join() 
        {
            
            let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


            //Handle the input states
            //Thrust states
            if fdm.throttle < 1.0 && keystate.thrust_up == true
            {
                fdm.throttle = fdm.throttle + 0.05;
                
            }   
            else if fdm.throttle > 0.0 && keystate.thrust_down == true
            {
                fdm.throttle = fdm.throttle - 0.05;
                if fdm.throttle < 0.001 
                {
                    fdm.throttle = 0.0;
                }
            }  

            //Angle of attack states
            if fdm.alpha < 20.0 && keystate.aoa_up == true
            {
                fdm.alpha = fdm.alpha + 1.0;
            
            }  
            else if fdm.alpha > -16.0 && keystate.aoa_down == true
            {
                fdm.alpha = fdm.alpha - 1.0
            }  

            //Bank states
            if fdm.bank < 20.0 && keystate.bank_right == true
            {
                fdm.bank = fdm.bank + 1.0;
            
            }  
            else if fdm.bank > -16.0 && keystate.bank_left == true
            {
                fdm.bank = fdm.bank - 1.0;
            }  

            //Flap states
            if fdm.flap == 0.0 && keystate.flaps_down == true
            {
                fdm.flap = 20.0;
            }  
            else if fdm.flap == 20.0 && keystate.flaps_down == true
            {
                fdm.flap = 40.0;
            }  
            else if (fdm.flap == 20.0 || fdm.flap == 40.0) && keystate.zero_flaps == true
            {
                fdm.flap = 0.0;
            }  
        

            //Will be used to calculate delta_traveled
            //Capture total distance traveled before frame
            let priorx = fdm.q[1]; 

            //Retrieve value of dependent variable
            let mut q = fdm.q.clone();

            //Get the static time variable DT
            let ds = delta;

            // Compute the four Runge-Kutta steps, The return 
            // value of plane_right_hand_side method is an array
            // of delta-q values for each of the four steps
            plane_right_hand_side(&mut fdm, &mut q, &mut qcopy,  &ds, 0.0, &mut dq1);
            plane_right_hand_side(&mut fdm, &mut q, &mut dq1,    &ds, 0.5, &mut dq2);
            plane_right_hand_side(&mut fdm, &mut q, &mut dq2,    &ds, 0.5, &mut dq3);
            plane_right_hand_side(&mut fdm, &mut q, &mut dq3,    &ds, 1.0, &mut dq4);

            //  Update the dependent and independent variable values
            //  at the new dependent variable location and store the
            //  values in the ODE object arrays.
        
            for i in 0..6
            {
                q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
                fdm.q[i] = q[i];
            }
    
            //Get distance traveled from last frame to this frame in meters
            fdm.delta_traveled = (fdm.q[1] - priorx).abs(); 
            //Add this distance traveled to the x axis position, ecef uses meters
            fdm.ecef_vec.x = fdm.ecef_vec.x + fdm.delta_traveled;

            //Calculate airspeed
            fdm.airspeed = (fdm.q[0] * fdm.q[0] + fdm.q[2] * fdm.q[2] + fdm.q[4] * fdm.q[4]).sqrt();


            //Print some relevant datas, set precision to match that of Palmer's C model
            println!("Total distance x (m) =    {:.6}", fdm.q[1]);
            println!("Altitude (m) =            {:.6}", fdm.q[5]);
            println!("Airspeed (m/s) =          {:.6}", fdm.airspeed);
            println!("Throttle =                {}", fdm.throttle);
            println!("Angle of attack (deg) =   {}", fdm.alpha);
            println!("Bank angle (deg) =        {}", fdm.bank);
            println!("Flap deflection (deg) =   {}", fdm.flap);

        }
    }
}


//This function is called by the EquationsOfMotion System and loads the right-hand sides for the plane ODEs
fn plane_right_hand_side(fdm: &mut DataFDM, q: &mut Vec<f64>, delta_q: &mut Vec<f64>, &ds: & f64, q_scale: f64, dq: &mut Vec<f64>)
{
    let mut new_q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 
 
    let negativeone = -1.0_f64;
    let pi = negativeone.acos();
    let g: f64 = -9.81;
    let mut cl: f64;
    let cos_p: f64;   //  climb angle
    let sin_p: f64;   //  climb angle
    let cos_t: f64;   //  heading angle
    let sin_t: f64;   //  heading angle

    //  Convert bank angle from degrees to radians
    //  Angle of attack is not converted because the
    //  Cl-alpha curve is defined in terms of degrees.
    let bank = fdm.bank.to_radians();

    //  Compute the intermediate values of the 
    //  dependent variables.
    for i in 0..6
    {
        new_q[i] = q[i] + q_scale * delta_q[i]; 
    }

    //  Assign convenenience variables to the intermediate 
    //  values of the locations and velocities.
    let vx: f64 = new_q[0];
    let vy: f64 = new_q[2];
    let vz: f64 = new_q[4];
    let _x: f64 = new_q[1];
    let _y: f64 = new_q[3];
    let z: f64 = new_q[5];
    let vh: f64 = (vx * vx + vy * vy).sqrt();
    let vtotal: f64 = (vx * vx + vy * vy + vz * vz).sqrt();

    //  Compute the air density
    let temperature: f64 = 288.15 - 0.0065 * z;
    let grp: f64 = 1.0 - 0.0065 * z / 288.15;
    let pressure: f64 = 101325.0 * (grp.powf(5.25));
    let density: f64 = 0.00348 * pressure / temperature;

    //  Compute power drop-off factor
    let omega: f64 = density / 1.225;
    let factor: f64 = (omega - 0.12)/  0.88;

    //  Compute thrust 
    let advance_ratio: f64 = vtotal / (fdm.mass_properties.engine_rps * fdm.mass_properties.prop_diameter);
    let thrust: f64 = fdm.throttle * factor * fdm.mass_properties.engine_power * (fdm.mass_properties.a + fdm.mass_properties.b * advance_ratio * advance_ratio) / (fdm.mass_properties.engine_rps * fdm.mass_properties.prop_diameter);

    //  Compute lift coefficient. The Cl curve is 
    //  modeled using two straight lines.
    if  fdm.alpha < fdm.mass_properties.alpha_cl_max
    {
        cl = fdm.mass_properties.cl_slope0 * fdm.alpha + fdm.mass_properties.cl0;
    }
    else 
    {
        cl = fdm.mass_properties.cl_slope1 * fdm.alpha + fdm.mass_properties.cl1;
    }

    //  Include effects of flaps and ground effects.
    //  Ground effects are present if the plane is
    //  within 5 meters of the ground.
    if fdm.flap == 20.0
    {
        cl += 0.25;
    }
    if fdm.flap == 40.0
    {
        cl += 0.5;
    }
    if z < 5.0
    {
        cl += 0.25;
    }

    //  Compute lift
    let lift: f64 = 0.5 * cl * density * vtotal * vtotal * fdm.mass_properties.wing_area;

    // Compute drag coefficient
    let aspect_ratio: f64 = fdm.mass_properties.wing_span * fdm.mass_properties.wing_span / fdm.mass_properties.wing_area;
    let cd = fdm.mass_properties.cdp + cl * cl / (pi * aspect_ratio * fdm.mass_properties.eff);
    
    //  Compute drag force
    let drag: f64 = 0.5 * cd * density * vtotal * vtotal * fdm.mass_properties.wing_area;

    //  Define some shorthand convenience variables
    //  for use with the rotation matrix.
    //  Compute the sine and cosines of the climb angle,
    //  bank angle, and heading angle;
    let cos_w: f64 = bank.cos(); 
    let sin_w: f64 = bank.sin(); 

    if  vtotal == 0.0
    {
        cos_p = 1.0;
        sin_p = 0.0;
    }
    else
    {
        cos_p = vh / vtotal;  
        sin_p = vz / vtotal;  
    }
    
    if vh == 0.0
    {
        cos_t = 1.0;
        sin_t = 0.0;
    }
    else
    {
        cos_t = vx / vh;
        sin_t = vy / vh;
    }
    
    //  Convert the thrust, drag, and lift forces into
    //  x-, y-, and z-components using the rotation matrix.
    let fx: f64 = cos_t * cos_p * (thrust - drag) + (sin_t * sin_w - cos_t * sin_p * cos_w) * lift;
    let fy: f64 = sin_t * cos_p * (thrust - drag) + (-cos_t * sin_w - sin_t * sin_p * cos_w) * lift;
    let mut fz: f64 = sin_p * (thrust - drag) + cos_p * cos_w * lift;

    //  Add the gravity force to the z-direction force.
    fz = fz + fdm.mass_properties.mass * g;

    //  Since the plane can't sink into the ground, if the
    //  altitude is less than or equal to zero and the z-component
    //  of force is less than zero, set the z-force
    //  to be zero.
    if z <= 0.0 && fz <= 0.0  
    {
        fz = 0.0;
    }

    //  Load the right-hand sides of the ODE's
    dq[0] = ds * (fx / fdm.mass_properties.mass);
    dq[1] = ds * vx;
    dq[2] = ds * (fy / fdm.mass_properties.mass);
    dq[3] = ds * vy;
    dq[4] = ds * (fz / fdm.mass_properties.mass);
    dq[5] = ds * vz;
}

