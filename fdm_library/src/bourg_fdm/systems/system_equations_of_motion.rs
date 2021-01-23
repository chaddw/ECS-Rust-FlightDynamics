//This file contains EquationsOfMotion System

//SPECS
use specs::prelude::*;

//Coordinate transforms
use coord_transforms::prelude::*;

//Get Component data needed to perform the System operations
use crate::bourg_fdm::components::component_keyboardstate::KeyboardState;
use crate::bourg_fdm::components::component_datafdm::DataFDM;

//Get Resources
use crate::bourg_fdm::resources::delta_time::DeltaTime;
use crate::bourg_fdm::resources::max_thrust::MaxThrust;
use crate::bourg_fdm::resources::delta_thrust::DeltaThrust;

//Vector, Matrix, Quaternion, math utilities, constants
use crate::bourg_fdm::common::vector::Vector;
use crate::bourg_fdm::common::matrix::Matrix;
use crate::bourg_fdm::common::quaternion::Quaternion;
use crate::bourg_fdm::common::math_utils::deg_to_rad;
use crate::bourg_fdm::common::math_utils::rad_to_deg;
use crate::bourg_fdm::common::constants::RHO;
use crate::bourg_fdm::common::constants::G;

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
            //Reset/zero the elevators, rudders, and ailerons every loo
            //Rudder
            fdm.element[6].f_incidence = 0.0;
            //Ailerons
            fdm.element[0].i_flap = 0;
            fdm.element[3].i_flap = 0;
            //Elevators
            fdm.element[4].i_flap = 0;
            fdm.element[5].i_flap = 0;
            //Flaps will be toggled on and off so flaps does not need to be zerod each time

            //Handle the input states
            //Thrust states
            if fdm.thrustforce < max_thrust && keystate.thrust_up == true
            {
                fdm.thrustforce = fdm.thrustforce + d_thrust;
            }   
            else if fdm.thrustforce > 0.0 && keystate.thrust_down == true
            {
                fdm.thrustforce = fdm.thrustforce - d_thrust;
            }

            //Rudder States
            if keystate.left_rudder == true
            { 
                fdm.element[6].f_incidence = 16.0;
            } 
            else if keystate.right_rudder == true
            { 
                fdm.element[6].f_incidence = -16.0;
            } 

            //Roll States
            if keystate.roll_left == true
            { 
                fdm.element[0].i_flap = 1;
                fdm.element[3].i_flap = -1;
            } 
           else if keystate.roll_right == true
            { 
                fdm.element[0].i_flap = -1;
                fdm.element[3].i_flap = 1;
            } 

            //Pitch States
            if keystate.pitch_up == true
            { 
                fdm.element[4].i_flap = 1;
                fdm.element[5].i_flap = 1;
            } 
            else if keystate.pitch_down == true
            { 
                fdm.element[4].i_flap = -1;
                fdm.element[5].i_flap = -1;
            } 

            //Flap States
            if keystate.flaps_down == true
            { 
                fdm.element[1].i_flap = -1;
                fdm.element[2].i_flap = -1;
                fdm.flaps = true;
            }
            else if keystate.zero_flaps == true
            { 
                fdm.element[1].i_flap = 0;
                fdm.element[2].i_flap = 0;
                fdm.flaps = false;
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
    }
}



//Calculates all of the forces and moments on the plane at any time (called inside eom system)
pub fn calc_airplane_loads(fdm: &mut DataFDM)
{
    let mut fb = Vector::new(0.0, 0.0, 0.0); //total force
    let mut mb = Vector::new(0.0, 0.0, 0.0); //total moment

    //Reset forces and moments
    fdm.v_forces = Vector::new(0.0, 0.0, 0.0);
    fdm.v_moments = Vector::new(0.0, 0.0, 0.0);

    //Define thrust vector, which acts through the plane's center of gravity
    let mut thrust = Vector::new(1.0, 0.0, 0.0);
    thrust = thrust * fdm.thrustforce;

    //Calculate forces and moments in body space
    let mut v_drag_vector = Vector::new(0.0, 0.0, 0.0);
    let mut _v_resultant= Vector::new(0.0, 0.0, 0.0);

    fdm.stalling = false;

    //Loop through the 7 lifting elements, skipping the fuselage
    for i in 0..8 
    {
        if i == 6 //Tail rudder. It is a special case because it can rotate, so the normal vector is recalculated
        {
            let inc: f32 = deg_to_rad(fdm.element[i].f_incidence);
            let di: f32 = deg_to_rad(fdm.element[i].f_dihedral);
            fdm.element[i].v_normal = Vector::new(inc.sin(),
                                                 inc.cos() * di.sin(), 
                                                 inc.cos() * di.cos());
            fdm.element[i].v_normal.normalize();
        }
       
        //Calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
        let mut vtmp = Vector::crossproduct(&fdm.v_angular_velocity, &fdm.element[i].v_cg_coords);
        let v_local_velocity = fdm.v_velocity_body + vtmp;

        //Calculate local air speed
        let f_local_speed: f32 = v_local_velocity.magnitude(); 

        //Find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
        if f_local_speed > 1.0
        {
            let v_local_vel_tmp = -v_local_velocity;
            v_drag_vector = v_local_vel_tmp / f_local_speed;
        }

        //Find direction that lift will act. lift is perpendicular to the drag vector
        let lift_tmp = Vector::crossproduct(&v_drag_vector, &fdm.element[i].v_normal);
        let mut v_lift_vector = Vector::crossproduct(&lift_tmp, &v_drag_vector);
        let mut _tmp = v_lift_vector.magnitude(); 
        v_lift_vector.normalize();
  
        //Find the angle of attack. its the angle between the lift vector and element normal vector 
        _tmp = v_drag_vector * fdm.element[i].v_normal;

        if _tmp > 1.0
        {
            _tmp = 1.0;
        }
        if _tmp < -1.0
        {
            _tmp = -1.0;
        }

        let f_attack_angle: f32 = rad_to_deg(_tmp.asin());

        //Determine lift and drag force on the element. Rho is defined as 0.0023769, which is density of air at sea level, slugs/ft^3
        _tmp = 0.5 * RHO * f_local_speed * f_local_speed * fdm.element[i].f_area;   

        if i == 6 //tail/rudder
        {
            _v_resultant = (v_lift_vector * rudder_lift_coefficient(f_attack_angle) + v_drag_vector * rudder_drag_coefficient(f_attack_angle)) * _tmp;

        }
        else if i == 7
        {
            _v_resultant = v_drag_vector * 0.5 * _tmp; //simulate fuselage drag

        }
        else
        {
            _v_resultant = (v_lift_vector * lift_coefficient(f_attack_angle, fdm.element[i].i_flap) + v_drag_vector * drag_coefficient(f_attack_angle, fdm.element[i].i_flap)) * _tmp;
        }

        //Check for stall. If the coefficient of lift is 0, stall is occuring.
        if i <= 3
        {
            if lift_coefficient(f_attack_angle, fdm.element[i].i_flap) == 0.0
            {
                fdm.stalling = true; 
            }
        }

        //Keep running total of resultant forces (total force)
        fb = fb + _v_resultant;

        //Calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
        vtmp = Vector::crossproduct(&fdm.element[i].v_cg_coords, &_v_resultant);
        mb = mb + vtmp;
     }

    //Add thrust
    fb = fb + thrust;

    //Convert forces from model space to earth space. rotates the vector by the unit Quaternion (QVRotate function)
     fdm.v_forces = Quaternion::qvrotate(&fdm.q_orientation, &fb);

    //Apply gravity (G is -32.174 ft/s^2), 
    fdm.v_forces.z = fdm.v_forces.z + (G) * fdm.mass;

    fdm.v_moments = fdm.v_moments + mb;
}


//Calculate mass properties based on the airplane's different body pieces
//This is called from inside main before the airplane Entity is created
pub fn calc_airplane_mass_properties(fdm: &mut DataFDM)
{
    let mut inc: f32;
    let mut di: f32;

    //Calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
    for  i in fdm.element.iter_mut()
    {
        inc = deg_to_rad(i.f_incidence);
        di = deg_to_rad(i.f_dihedral);
        i.v_normal = Vector::new(inc.sin(), inc.cos() * di.sin(), inc.cos() * di.cos());
        i.v_normal.normalize(); 
    }

    //Calculate total mass
    let mut total_mass: f32 = 0.0;
    for i in fdm.element.iter()
    {
        total_mass = total_mass + i.f_mass;
    }

    //Calculate combined center of gravity location
    let mut v_moment = Vector::new(0.0,0.0,0.0);
    for i in fdm.element.iter()
    {
        let _tmp = i.v_d_coords * i.f_mass;
        v_moment = v_moment + _tmp;
    }
    let cg = v_moment / total_mass; 

    //Calculate coordinates of each element with respect to the combined CG, relative position
    for i in fdm.element.iter_mut()
    {
        i.v_cg_coords = i.v_d_coords - cg;
    }

    //Calculate the moments and products of intertia for the combined elements
    let mut ixx: f32 = 0.0;
    let mut iyy: f32 = 0.0;
    let mut izz: f32 = 0.0;
    let mut ixy: f32 = 0.0;
    let mut ixz: f32 = 0.0;
    let mut iyz: f32 = 0.0;

    for i in fdm.element.iter()
    {
        ixx = ixx + i.v_local_inertia.x + i.f_mass *
            (i.v_cg_coords.y * i.v_cg_coords.y +
            i.v_cg_coords.z * i.v_cg_coords.z);

        iyy = iyy + i.v_local_inertia.y + i.f_mass *
            (i.v_cg_coords.z * i.v_cg_coords.z +
            i.v_cg_coords.x * i.v_cg_coords.x);

        izz = izz + i.v_local_inertia.z + i.f_mass *
            (i.v_cg_coords.x * i.v_cg_coords.x +
            i.v_cg_coords.y * i.v_cg_coords.y);

        ixy = ixy + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.y);

        ixz = ixz + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.z);
        
        iyz = iyz + i.f_mass * (i.v_cg_coords.y *
            i.v_cg_coords.z);
    }

    //Finally, set up airplanes mass and inertia Matrix
    fdm.mass = total_mass;
    fdm.m_inertia = Matrix::new(ixx, -ixy, -ixz,
                                  -ixy, iyy, -iyz,
                                  -ixz, -iyz, izz);

    //Get inverse of Matrix
    fdm.m_inertia_inverse = Matrix::inverse(&fdm.m_inertia);
}


//Functions to collect airfoil performance data:
//lift and drag coefficient data is given for a set of discrete attack angles, 
//so then linear interpolation is used to determine the coefficients for the 
//attack angle that falls between the discrete angles.

//Given the angle of attack and status of the flaps,
//return lift angle coefficient for camabred airfoil with 
//plain trailing-edge (+/- 15 degree inflation).
fn lift_coefficient(angle: f32, flaps: i32) -> f32
{
    let clf0 = vec![-0.54, -0.2, 0.2, 0.57, 0.92, 1.21, 1.43, 1.4, 1.0];
    let clfd = vec![0.0, 0.45, 0.85, 1.02, 1.39, 1.65, 1.75, 1.38, 1.17];
    let clfu = vec![-0.74, -0.4, 0.0, 0.27, 0.63, 0.92, 1.03, 1.1, 0.78];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cl: f32 = 0.0;

    for i in 0..8  
    {
        if a[i] <= angle && a[i + 1] > angle
        {
            if flaps == 0 //flaps not deflected
            {
                cl = clf0[i] - (a[i] - angle) * (clf0[i] - clf0[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
            else if flaps == -1 //flaps down
            {
                cl = clfd[i] - (a[i] - angle) * (clfd[i] - clfd[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
            else if flaps == 1 //flaps up
            {
                cl = clfu[i] - (a[i] - angle) * ( clfu[i] - clfu[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
        }

    }
    return cl;
}


//given angle of attack and flap status, 
//return drag coefficient for cambered airfoil with 
//plain trailing-edge flap (+/- 15 degree deflection).
fn drag_coefficient(angle: f32, flaps: i32) -> f32
{
    let cdf0 = vec![0.01, 0.0074, 0.004, 0.009, 0.013, 0.023, 0.05, 0.12, 0.21];
    let cdfd = vec![0.0065, 0.0043, 0.0055, 0.0153, 0.0221, 0.0391, 0.1, 0.195, 0.3];
    let cdfu = vec![0.005, 0.0043, 0.0055, 0.02601, 0.03757, 0.06647, 0.13, 0.1, 0.25];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f32 = 0.75; //0.5 in book but 0.75 in actual code

    for i in 0..8  
    {
        if a[i] <= angle && a[i + 1] > angle
        {
            if flaps == 0 //flaps not deflected
            {
                cd = cdf0[i] - (a[i] - angle) * (cdf0[i] - cdf0[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
            else if flaps == -1 //flaps down
            {
                cd = cdfd[i] - (a[i] - angle) * (cdfd[i] - cdfd[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
            else if flaps == 1 //flaps up
            {
                cd = cdfu[i] - (a[i] - angle) * ( cdfu[i] - cdfu[i + 1]) / (a[i] - a[i + 1]);
                break;
            }
        }

    }
    return cd;
}


//Rudder lift and drag coefficients are similar to that of the wing 
//but the coefficients themselves are different and the tail rudder 
//does not include flaps.
//Given attack angle, return lift coefficient for a symmetric (no camber) 
//airfoil without flaps.
fn rudder_lift_coefficient(angle: f32) -> f32
{
    let clf0 = vec![0.16, 0.456, 0.736, 0.968, 1.144, 1.12, 0.8];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cl: f32 = 0.0;
    let aa: f32 = angle.abs();

    for i in 0..6 
    {
        if a[i] <= aa && a[i + 1] > aa
        {
            cl = clf0[i] - (a[i] - aa) * (clf0[i] - clf0[i + 1]) / (a[i] - a[i + 1]);

            if angle < 0.0
            {
                cl = -cl;
            }
            break;
        }
    }
    return cl;
}

//Given attack angle, return drag coefficient for a symmetric (no camber) 
//airfoil without flaps.
fn rudder_drag_coefficient(angle: f32) -> f32
{
    let cdf0 = vec![0.0032, 0.0072, 0.0104, 0.0184, 0.04, 0.096, 0.168];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f32 = 0.75; //0.5 in book
    let aa: f32 = angle.abs();

    for i in 0..6  
    {
        if a[i] <= aa && a[i + 1] > aa
        {
            cd = cdf0[i] - (a[i] - aa) * (cdf0[i] - cdf0[i + 1]) / (a[i] - a[i + 1]);
            break;
        }
    }
    return cd;
}


