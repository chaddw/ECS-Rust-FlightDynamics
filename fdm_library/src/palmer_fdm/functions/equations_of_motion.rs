//Function to coompute equations of motion

//Coordinate transforms/nalgebra vector
use coord_transforms::prelude::*;

//Get data needed to perform the System operations
use crate::palmer_fdm::components::component_keyboardstate::KeyboardState;
use crate::palmer_fdm::components::component_datafdm::DataFDM;

//get plane_right_hand_side function
use crate::palmer_fdm::functions::plane_right_hand_side::plane_rhs;

pub fn eom(mut fdm: &mut DataFDM, keystate: &KeyboardState, dt: f64)
{

    let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


    //Handle the input states
    //Thrust states
    if fdm.throttle < 1.0 && keystate.throttle_up == true
    {
        fdm.throttle = fdm.throttle + 0.05;
    }   
    else if fdm.throttle > 0.0 && keystate.throttle_down == true
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
    else if fdm.bank > -20.0 && keystate.bank_left == true
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


    //Retrieve value of dependent variable
    let mut q = fdm.q.clone();

    //Get the static time variable DT
    let ds = dt;

    // Compute the four Runge-Kutta steps, The return 
    // value of plane_right_hand_side method is an array
    // of delta-q values for each of the four steps
    plane_rhs(&mut fdm, &mut q, &mut qcopy,  &ds, 0.0, &mut dq1);
    plane_rhs(&mut fdm, &mut q, &mut dq1,    &ds, 0.5, &mut dq2);
    plane_rhs(&mut fdm, &mut q, &mut dq2,    &ds, 0.5, &mut dq3);
    plane_rhs(&mut fdm, &mut q, &mut dq3,    &ds, 1.0, &mut dq4);

    //  Update the dependent and independent variable values
    //  at the new dependent variable location and store the
    //  values in the ODE object arrays.

    for i in 0..6
    {
        q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
        fdm.q[i] = q[i];
    }

    //Calculate airspeed
    fdm.airspeed = (fdm.q[0] * fdm.q[0] + fdm.q[2] * fdm.q[2] + fdm.q[4] * fdm.q[4]).sqrt();

    //Calculate displacement based on velocities to add to the latitude and longitude

    //Create WGS84 ellipsoid
    let ellipsoid: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);

    //Take lat/lon origin and put into naglebra vector, and convert the lat/lon degrees to radians
    let origin = Vector3::new(fdm.lla_origin.x.to_radians(), fdm.lla_origin.y.to_radians(), fdm.lla_origin.z);

    //Load x/y/z velocities into a nalgebra vector representing the displacement
    let d = Vector3::new(fdm.q[0], fdm.q[2], fdm.q[4]);

    //Take East North Up cartesian coordinate displacement and calculate a new lat/lon/alt with respect to the origin
    let mut enu2lla = geo::enu2lla(&origin, &d, &ellipsoid);

    //Convert the lat/lon radians to degrees
    enu2lla.x = enu2lla.x.to_degrees();
    enu2lla.y = enu2lla.y.to_degrees();

    //Subtract the enu2lla results by the origin position get the displacement for the frame
    let displacement =  enu2lla - fdm.lla_origin;

    //Update position by adding old position and displacement with respect to time
    fdm.position = fdm.position + displacement * ds;

    
    //Print some relevant data
    println!("Latitude (deg) x-axis =   {}", fdm.position.x);
    println!("Longitude (deg) y-axis =  {}", fdm.position.y);
    println!("Altitude (m) =            {}", fdm.q[5]);
    println!("Airspeed (km/hr) =        {}", fdm.airspeed * 3.6); //convert from m/s to km/h
    println!("Heading angle (deg)       {}", fdm.heading_angle.to_degrees());
    println!("Climb angle (deg)         {}", fdm.climb_angle.to_degrees());
    println!("Climb rate (m/s)          {}", fdm.climb_rate);
    println!("Throttle % =              {}", fdm.throttle * 100.0);
    println!("Angle of attack (deg) =   {}", fdm.alpha);
    println!("Bank angle (deg) =        {}", fdm.bank);
    println!("Flap deflection (deg) =   {}", fdm.flap);

}
