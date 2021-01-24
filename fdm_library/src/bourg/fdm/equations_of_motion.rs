//This function computes the equations of motion

//Coordinate transforms
use coord_transforms::prelude::*;

//Get Component data needed to perform the System operations
use crate::bourg::fdm::structures::KeyboardState;
use crate::bourg::fdm::structures::DataFDM;

//Get keyboard flight control functions
use crate::bourg::fdm::keypresses::*;
use crate::bourg::fdm::calc_loads::calc_airplane_loads;

//Vector, Matrix, Quaternion, math utilities, constants
use crate::bourg::common::vector::Vector;
use crate::bourg::common::quaternion::Quaternion;
use crate::bourg::common::math_utils::deg_to_rad;
use crate::bourg::common::math_utils::rad_to_deg;

pub fn eom(mut fdm: &mut DataFDM, keystate: &KeyboardState, dt: f32, d_thrust: f32, max_thrust: f32)
{

    //Reset/zero the elevators, rudders, and ailerons every loop
    zero_rudder(&mut fdm);
    zero_ailerons(&mut fdm);
    zero_elevators(&mut fdm);
    //Flaps will be toggled on and off so flaps does not need to be zerod each time

    
    //Handle the input states
    //Thrust states
    if fdm.thrustforce < max_thrust && keystate.thrust_up == true
    {
        thrust_up(&mut fdm, d_thrust);
    }   
    else if fdm.thrustforce > 0.0 && keystate.thrust_down == true
    {
        thrust_down(&mut fdm, d_thrust);
    }

    //Rudder States
    if keystate.left_rudder == true
    { 
        yaw_left(&mut fdm);
    } 
    else if keystate.right_rudder == true
    { 
        yaw_right(&mut fdm);
    } 

    //Roll States
    if keystate.roll_left == true
    { 
        roll_left(&mut fdm);
    } 
    else if keystate.roll_right == true
    { 
        roll_right(&mut fdm);
    } 

    //Pitch States
    if keystate.pitch_up == true
    { 
        pitch_up(&mut fdm);
    } 
    else if keystate.pitch_down == true
    { 
        pitch_down(&mut fdm);
    } 

    //Flap States
    if keystate.flaps_down == true
    { 
        flaps_down(&mut fdm);
    }
    else if keystate.zero_flaps == true
    { 
        zero_flaps(&mut fdm);
    } 


    //Calculate all of the forces and moments on the airplane
    calc_airplane_loads(&mut fdm);

    //Calculate acceleration of airplane in earth space
    let ae = fdm.v_forces / fdm.mass;

    //Calculate velocity of airplane in earth space
    fdm.v_velocity = fdm.v_velocity + ae * dt; 


    //Calculate position of airplane in earth space

    //Create WGS84 ellipsoid
    let ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);

    //Take lat/lon origin and put into naglebra vector, and convert the lat/lon degrees to radians
    let origin = Vector3::new(deg_to_rad(fdm.lla_origin.x) as f64, deg_to_rad(fdm.lla_origin.y) as f64, fdm.lla_origin.z as f64);

    //Load x/y/z velocities into a nalgebra vector representing the displacement, convert from feet to meters
    let d = Vector3::new(fdm.v_velocity.x as f64 / 3.281, fdm.v_velocity.y as f64 / 3.281, fdm.v_velocity.z as f64 / 3.281);

    //Take East North Up cartesian coordinate displacement and calculate a new lat/lon/alt with respect to the origin
    let enu2lla = geo::enu2lla(&origin, &d, &ellipsoid);

    //Put the enu2lla results into the native custom vector type and convert lat/lon radians displaced to degrees 
    let enu2lla_converted = Vector::new(rad_to_deg(enu2lla.x as f32), rad_to_deg(enu2lla.y as f32), enu2lla.z as f32);

    //Subtract the enu2lla results by the origin position get the displacement for the frame
    let displacement = enu2lla_converted - fdm.lla_origin;

    //Update position by adding old position and displacement with respect to time
    fdm.v_position = fdm.v_position + displacement * dt; 


    //Calculate angular velocity of airplane in body space
    fdm.v_angular_velocity = fdm.v_angular_velocity + ((fdm.m_inertia_inverse * 
        (fdm.v_moments - Vector::crossproduct(&fdm.v_angular_velocity, &(fdm.m_inertia * fdm.v_angular_velocity)))) * dt);

    //Calculate the new rotation Quaternion
    fdm.q_orientation = fdm.q_orientation + (fdm.q_orientation * fdm.v_angular_velocity) * (0.5 * dt);

    //Now normalize the orientation Quaternion (make into unit Quaternion)
    let mag = fdm.q_orientation.magnitude();
    if mag != 0.0
    {
        fdm.q_orientation = fdm.q_orientation / mag;
    }

    //Calculate the velocity in body space
    fdm.v_velocity_body = Quaternion::qvrotate(&Quaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);

    //Calculate air speed
    fdm.f_speed = fdm.v_velocity.magnitude(); 

    //Get euler angles
    let euler = Quaternion::make_euler_from_q(&fdm.q_orientation);
    fdm.v_euler_angles.x = euler.x; 
    fdm.v_euler_angles.y = euler.y;
    fdm.v_euler_angles.z = euler.z;
    
    //Print some relevant data
    println!("Roll:             {}", fdm.v_euler_angles.x);
    println!("Pitch:            {}", -fdm.v_euler_angles.y);
    println!("Yaw:              {}", fdm.v_euler_angles.z);
    println!("Position x:       {}", fdm.v_position.x);
    println!("Position y:       {}", fdm.v_position.y);
    println!("Position z:       {}", fdm.v_position.z);
    println!("Airspeed (knots): {}", fdm.f_speed/1.688); //convert to knots
    println!("Thrust:           {}", fdm.thrustforce);


}