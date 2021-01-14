//This file contains EquationsOfMotion System

//SPECS
use specs::prelude::*;

//Coordinate transforms
use coord_transforms::prelude::*;

//Get data needed to perform the System operations
use crate::data::KeyboardState;
use crate::data::DataFDM;
use crate::data::DeltaTime;
use crate::data::MaxThrust;
use crate::data::DeltaThrust;

//Vector, Matrix, Quaternion module
use crate::common::Myvec;
use crate::common::Mymatrix;
use crate::common::Myquaternion;
use crate::common::deg_to_rad;
use crate::common::rad_to_deg;

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
            let ae: Myvec = Myvec::dividescalar(&fdm.v_forces, fdm.mass);

            //Calculate velocity of airplane in earth space
            fdm.v_velocity = Myvec::addvec(&fdm.v_velocity, &Myvec::multiplyscalar(&ae, dt)); 


            //Calculate position of airplane in earth space

            //Create WGS84 ellipsoid
            let ellipsoid: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);

            //Take lat/lon origin and put into naglebra vector, and convert the lat/lon degrees to radians
            let origin = Vector3::new(deg_to_rad(fdm.lla_origin.x) as f64, deg_to_rad(fdm.lla_origin.y) as f64, fdm.lla_origin.z as f64);

            //Load x/y/z velocities into a nalgebra vector representing the displacement, convert from feet to meters
            let d = Vector3::new(fdm.v_velocity.x as f64 / 3.281, fdm.v_velocity.y as f64 / 3.281, fdm.v_velocity.z as f64 / 3.281);

            //Take East North Up cartesian coordinate displacement and calculate a new lat/lon/alt with respect to the origin
            let enu2lla = geo::enu2lla(&origin, &d, &ellipsoid);

            //Put the enu2lla results into the native custom vector type and convert lat/lon radians displaced to degrees 
            let enu2lla_converted = Myvec::new(rad_to_deg(enu2lla.x as f32), rad_to_deg(enu2lla.y as f32), enu2lla.z as f32);

            //Subtract the enu2lla results by the origin position get the displacement for the frame
            let mut displacement = Myvec::subtractvec(&enu2lla_converted, &fdm.lla_origin);

            //Update position by adding old position and displacement with respect to time
            fdm.v_position = Myvec::addvec(&fdm.v_position, &Myvec::multiplyscalar(&displacement, dt)); 


            //Calculate angular velocity of airplane in body space
            let one = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia, &fdm.v_angular_velocity);
            let two = Myvec::crossproduct(&fdm.v_angular_velocity, &one);
            let three = Myvec::subtractvec(&fdm.v_moments, &two);
            let four = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia_inverse, &three);
            let five = Myvec::multiplyscalar(&four, dt);
            fdm.v_angular_velocity = Myvec::addvec(&fdm.v_angular_velocity, &five);

            //Calculate the new rotation quaternion
            let uno = Myquaternion::multiply_quat_by_vec(&fdm.q_orientation, &fdm.v_angular_velocity);
            let dos = Myquaternion::multiplyscalar(&uno, 0.5 * dt);
            fdm.q_orientation = Myquaternion::addquat(&fdm.q_orientation, &dos);

            //Now normalize the orientation quaternion (make into unit quaternion)
            let mag = fdm.q_orientation.magnitude();
            if mag != 0.0
            {
               fdm.q_orientation = Myquaternion::dividescalar(&fdm.q_orientation, mag);
            }

            //Calculate the velocity in body space
            fdm.v_velocity_body = Myquaternion::qvrotate(&Myquaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);
    
            //Calculate air speed
            fdm.f_speed = fdm.v_velocity.magnitude(); 
    
            //Get euler angles
            let euler = Myquaternion::make_euler_from_q(&fdm.q_orientation);
            fdm.v_euler_angles.x = euler.x; 
            fdm.v_euler_angles.y = euler.y;
            fdm.v_euler_angles.z = euler.z;
            
            //Print some relevant data
            println!("Roll:             {}", fdm.v_euler_angles.x);
            println!("Pitch:            {}", -fdm.v_euler_angles.y); //Pitch was negated when Euler angles were computed
            println!("Yaw:              {}", fdm.v_euler_angles.z);
            println!("Alt:              {}", fdm.v_position.z);
            println!("Thrust:           {}", fdm.thrustforce);
            println!("Airspeed (knots): {}", fdm.f_speed/1.688);
            println!("Position x:       {}", fdm.v_position.x);
            println!("Position y:       {}", fdm.v_position.y);
            println!("Position z:       {}", fdm.v_position.z);

        }
    }
}



//Calculates all of the forces and moments on the plane at any time (called inside eom system)
fn calc_airplane_loads(fdm: &mut DataFDM)
{
    let mut fb = Myvec::new(0.0, 0.0, 0.0); //total force
    let mut mb = Myvec::new(0.0, 0.0, 0.0); //total moment

    //Reset forces and moments
    fdm.v_forces = Myvec::new(0.0, 0.0, 0.0);
    fdm.v_moments = Myvec::new(0.0, 0.0, 0.0);

    //Define thrust vector, which acts through the plane's center of gravity
    let mut thrust = Myvec::new(1.0, 0.0, 0.0);
    thrust = Myvec::multiplyscalar(&thrust, fdm.thrustforce);

    //Calculate forces and moments in body space
    let mut v_drag_vector = Myvec::new(0.0, 0.0, 0.0);
    let mut v_resultant= Myvec::new(0.0, 0.0, 0.0);

    fdm.stalling = false;

    //Loop through the 7 lifting elements, skipping the fuselage
    for i in 0..8 
    {
        if i == 6 //Tail rudder. It is a special case because it can rotate, so the normal vector is recalculated
        {
            let inc: f32 = deg_to_rad(fdm.element[i].f_incidence);
            let di: f32 = deg_to_rad(fdm.element[i].f_dihedral);
            fdm.element[i].v_normal = Myvec::new(inc.sin(),
                                                 inc.cos() * di.sin(), 
                                                 inc.cos() * di.cos());
            fdm.element[i].v_normal.normalize();
        }
       
        //Calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
        let mut vtmp = Myvec::crossproduct(&fdm.v_angular_velocity, &fdm.element[i].v_cg_coords);
        let v_local_velocity = Myvec::addvec(&fdm.v_velocity_body, &vtmp);

        //Calculate local air speed
        let f_local_speed: f32 = fdm.v_velocity_body.magnitude(); 

        //Find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
        if f_local_speed > 1.0
        {
            let v_local_vel_tmp = Myvec::reverse(&v_local_velocity); //-vLocalVelocity
            v_drag_vector = Myvec::dividescalar(&v_local_vel_tmp, f_local_speed);
        }

        //Find direction that lift will act. lift is perpendicular to the drag vector
        let lift_tmp = Myvec::crossproduct(&v_drag_vector, &fdm.element[i].v_normal);
        let mut v_lift_vector = Myvec::crossproduct(&lift_tmp, &v_drag_vector);
        let mut tmp = v_lift_vector.magnitude(); 
        v_lift_vector.normalize();
  
        //Find the angle of attack. its the angle between the lift vector and element normal vector 
        tmp = Myvec::dotproduct(&v_drag_vector, &fdm.element[i].v_normal);

        if tmp > 1.0
        {
            tmp = 1.0;
        }
        if tmp < -1.0
        {
            tmp = -1.0;
        }

        let f_attack_angle: f32 = rad_to_deg(tmp.asin());

        //Determine lift and drag force on the element. Rho is defined as 0.0023769, which is density of air at sea level, slugs/ft^3
        tmp = 0.5 * 0.002376900055 * f_local_speed * f_local_speed * fdm.element[i].f_area;   

        if i == 6 //tail/rudder
        {
            let firstpart = Myvec::multiplyscalar(&v_lift_vector, rudder_lift_coefficient(f_attack_angle));
            let secondpart = Myvec::multiplyscalar(&v_drag_vector, rudder_drag_coefficient(f_attack_angle));
            let addtogether = Myvec::addvec(&firstpart, &secondpart);
            v_resultant = Myvec::multiplyscalar(&addtogether, tmp);
        }
        else if i == 7
        {
            v_resultant = Myvec::multiplyscalar(&v_drag_vector, 0.5 * tmp); //simulate fuselage drag

        }
        else
        {
            let firstpart = Myvec::multiplyscalar(&v_lift_vector, lift_coefficient(f_attack_angle, fdm.element[i].i_flap));
            let secondpart = Myvec::multiplyscalar(&v_drag_vector, drag_coefficient(f_attack_angle, fdm.element[i].i_flap));
            let addtogether = Myvec::addvec(&firstpart, &secondpart);
            v_resultant = Myvec::multiplyscalar(&addtogether, tmp);
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
        fb = Myvec::addvec(&fb, &v_resultant);

        //Calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
        vtmp = Myvec::crossproduct(&fdm.element[i].v_cg_coords, &v_resultant);
        mb = Myvec::addvec(&mb, &vtmp);
     }

    //Add thrust
    fb = Myvec::addvec(&fb, &thrust);

    //Convert forces from model space to earth space. rotates the vector by the unit quaternion (QVRotate function)
     fdm.v_forces = Myquaternion::qvrotate(&fdm.q_orientation, &fb);

    //Apply gravity (g is -32.174 ft/s^2), 
    fdm.v_forces.z = fdm.v_forces.z + (-32.17399979) * fdm.mass;

    fdm.v_moments = Myvec::addvec(&fdm.v_moments, &mb);
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
        i.v_normal = Myvec::new(inc.sin(), inc.cos() * di.sin(), inc.cos() * di.cos());
        i.v_normal.normalize(); 
    }

    //Calculate total mass
    let mut total_mass: f32 = 0.0;
    for i in fdm.element.iter()
    {
        total_mass = total_mass + i.f_mass;
    }

    //Calculate combined center of gravity location
    let mut v_moment = Myvec::new(0.0,0.0,0.0);
    for i in fdm.element.iter()
    {
        let tmp = Myvec::multiplyscalar(&i.v_d_coords, i.f_mass);
        v_moment = Myvec::addvec(&v_moment, &tmp);
    }
    let cg = Myvec::dividescalar(&v_moment, total_mass); 

    //Calculate coordinates of each element with respect to the combined CG, relative position
    for i in fdm.element.iter_mut()
    {
        i.v_cg_coords = Myvec::subtractvec(&i.v_d_coords, &cg);
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

    //Finally, set up airplanes mass and inertia matrix
    fdm.mass = total_mass;
    fdm.m_inertia = Mymatrix::new(ixx, -ixy, -ixz,
                                  -ixy, iyy, -iyz,
                                  -ixz, -iyz, izz);

    //Get inverse of matrix
    fdm.m_inertia_inverse = Mymatrix::inverse(&fdm.m_inertia);
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


#[cfg(test)]
mod tests
{

    use super::*;
    use crate::PointMass;
    
    //FDM tests
    #[test]
    fn calc_airplane_mass_properties_test() 
    {
        //create airplane to compute mass properties
        let mut fdm = DataFDM{ 
            element : vec![
                PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, 12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, 5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, -5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, -12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, 2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, -2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.93, v_d_coords: Myvec::new(2.25, 0.0, 5.0), v_local_inertia: Myvec::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 31.8, v_d_coords: Myvec::new(15.25, 0.0, 1.5), v_local_inertia: Myvec::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) }
            ], ..Default::default()};
    
        //Calculate mass properties on this airplane
        calc_airplane_mass_properties(&mut fdm);

        //C++ value to match
        let equal = Mymatrix::new (0.000393187161535024642944336, -0.0, -0.00001486624387325718998909,     
                                   -0.0, 0.000493949337396770715713501, -0.0,
                                   -0.00001486624387325718998909, -0.0, 0.000227076292503625154495239);

         assert_eq!(fdm.m_inertia_inverse, equal);
         
    }

    #[test]
    fn fdm_test() //Can test one flight data variable of the entire FDM at a time. Must change frame #, and get that value to compare from C++ output. Can also add flight controls artificially
    {
        let dt = 1.0 / 30.0;
        let d_thrust = 100.0;

        //create airplane to compute mass properties
        let mut fdm = DataFDM{ 
            element : vec![
                PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, 12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, 5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, -5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, -12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, 2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, -2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 2.93, v_d_coords: Myvec::new(2.25, 0.0, 5.0), v_local_inertia: Myvec::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
                PointMass{f_mass: 31.8, v_d_coords: Myvec::new(15.25, 0.0, 1.5), v_local_inertia: Myvec::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) }
            ], 
            
            
        //Define initial flight parameters
        v_position: Myvec{x: -5000.0, y: 0.0, z: 2000.0},
        v_velocity: Myvec{x: 60.0, y: 0.0, z: 0.0},
        f_speed: 60.0,
        v_forces: Myvec{x: 500.0, y: 0.0, z: 0.0},
        thrustforce: 500.0,
        q_orientation: Myquaternion::make_q_from_euler(0.0, 0.0, 0.0),

        //Everything else is zero to begin
        ..Default::default()
        };
    
        //Calculate mass properties on this airplane
        calc_airplane_mass_properties(&mut fdm);

        let mut current_frame: usize = 0;
        for _ in 0..5 //frame #
        {
            current_frame = current_frame + 1;

            //Can add flight control artificially to test
            //Roll test parameters
            if current_frame >= 1 && current_frame <= 5
            {
                //increase thrust
                fdm.thrustforce = fdm.thrustforce + d_thrust;
            }
            else if current_frame >= 6 && current_frame <= 246 //pitch up 8 seconds
            {
                //pitch up
                fdm.element[4].i_flap = 1;
                fdm.element[5].i_flap = 1;
            }
            else if current_frame >= 247 && current_frame <= 307 //roll right for 2 seconds
            {
                //roll right
                fdm.element[0].i_flap = -1;
                fdm.element[3].i_flap = 1;
            }

            //Calculate all of the forces and moments on the airplane
            calc_airplane_loads(&mut fdm);

            //Calculate acceleration of airplane in earth space
            let ae: Myvec = Myvec::dividescalar(&fdm.v_forces, fdm.mass);

            //Calculate velocity of airplane in earth space
            fdm.v_velocity = Myvec::addvec(&fdm.v_velocity, &Myvec::multiplyscalar(&ae, dt)); 

            //Calculate position of airplane in earth space
            fdm.v_position = Myvec::addvec(&fdm.v_position, &Myvec::multiplyscalar(&fdm.v_velocity, dt));

            //Calculate angular velocity of airplane in body space
            let one = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia, &fdm.v_angular_velocity);
            let two = Myvec::crossproduct(&fdm.v_angular_velocity, &one);
            let three = Myvec::subtractvec(&fdm.v_moments, &two);
            let four = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia_inverse, &three);
            let five = Myvec::multiplyscalar(&four, dt);
            fdm.v_angular_velocity = Myvec::addvec(&fdm.v_angular_velocity, &five);

            //Calculate the new rotation quaternion
            let uno = Myquaternion::multiply_quat_by_vec(&fdm.q_orientation, &fdm.v_angular_velocity);
            let dos = Myquaternion::multiplyscalar(&uno, 0.5 * dt);
            fdm.q_orientation = Myquaternion::addquat(&fdm.q_orientation, &dos);

            //Now normalize the orientation quaternion (make into unit quaternion)
            let mag = fdm.q_orientation.magnitude();
            if mag != 0.0
            {
                fdm.q_orientation = Myquaternion::dividescalar(&fdm.q_orientation, mag);
            }

            //Calculate the velocity in body space
            fdm.v_velocity_body = Myquaternion::qvrotate(&Myquaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);

            //Calculate air speed
            fdm.f_speed = fdm.v_velocity.magnitude();


            //Get euler angles
            let euler = Myquaternion::make_euler_from_q(&fdm.q_orientation);
            fdm.v_euler_angles.x = euler.x;
            fdm.v_euler_angles.y = euler.y;
            fdm.v_euler_angles.z = euler.z; 
    
        }

        //C++ value to match

        //with no roll
        //let equal = 1999.9642333984375; //altitude (first frame)
        //let equal = 1999.899169921875; //altitude (second frame)
        //let equal = 1999.8056640625; //altitude (third frame) 
        //let equal = 1973.194580078125; //altitude (frame 50) 

        //With roll
        //let equal = 1999.9642333984375; //altitude (frame 1) 
        //let equal = 1999.899169921875; //altitude (frame 2) 
        //let equal =  1999.8056640625; //altitude (frame 3) 
        //let equal =  1999.6846923828125; //altitude (frame 4) 
        let equal =  1999.537109375; //altitude (frame 5) 



        //change variable as needed for testing
         assert_eq!(fdm.v_position.z, equal);
         
    }
   

    //VECTOR TESTS
    #[test]
    fn v_magnitude_test1() 
    {
        let v = Myvec::new(1.0, 1.0, 1.0);
        assert_eq!(v.magnitude(), 1.73205077648162841796875);
    }

    #[test]
    fn v_magnitude_test2()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        assert_eq!(v.magnitude(), 3.7416574954986572265625);
    }

    #[test]
    fn v_magnitude_test3()
    {
        let v = Myvec::new(63.73516082763671875, -0.004545502364635467529296875, -8.11302661895751953125);
        assert_eq!(v.magnitude(), 64.24945068359375);
    }


    #[test]
    fn v_normalize_test1()
    {
        let mut v = Myvec::new(1.0, 1.0, 1.0);
        v.normalize();

        let equal = Myvec::new(0.57735025882720947265625, 0.57735025882720947265625, 0.57735025882720947265625);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test2()
    {
        let mut v = Myvec::new(3.0, 3.0, 3.0);
        v.normalize();

        let equal = Myvec::new(0.577350318431854248046875, 0.577350318431854248046875,0.577350318431854248046875);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test3()
    {
        let mut v = Myvec::new(1.0, 2.0, 3.0);
        v.normalize();

        let equal = Myvec::new(0.267261236906051635742188, 0.534522473812103271484375, 0.80178368091583251953125);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test4()
    {
        let mut v = Myvec::new(-0.061048544943332672119140625, 0.0, 0.998134791851043701171875);
        v.normalize();

        let equal = Myvec::new(-0.0610485486686229705810546875, 0.0, 0.9981348514556884765625);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_reverse_test1()
    {
        let mut v = Myvec::new(1.0, 2.0, 3.0);
        v = Myvec::reverse(&v);

        let equal = Myvec::new(-1.0, -2.0, -3.0);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_add_test1()
    {
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::addvec(&v, &u);

        let equal = Myvec::new(11.0, 22.0, 33.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_add_test2()
    {
        let v = Myvec::new(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.0007719345740042626857757568359375);

        let u = Myvec::new(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

        let uv = Myvec::addvec(&v, &u);

        let equal = Myvec::new(0.000301808759104460477828979, -0.00649327132850885391235352, 0.000772719329688698053359985);


        assert_eq!(uv, equal);
    }
        
    #[test]
    fn v_subtract_test1()
    {
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::subtractvec(&v, &u);

        let equal = Myvec::new(9.0, 18.0, 27.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_subtract_test2()
    {
        let v = Myvec::new(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.0007719345740042626857757568359375);

        let u = Myvec::new(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

        let uv = Myvec::subtractvec(&v, &u);

        let equal = Myvec::new(0.000281422457192093133926392, 0.00410096254199743270874023, 0.000771149818319827318191528);


        assert_eq!(uv, equal);
    }
    



    #[test]
    fn v_cross_product_test1()
    {
        
        let v = Myvec::new(1.0, 0.0, 0.0);
        let u = Myvec::new(0.0, 1.0, 1.0);
        let uv = Myvec::crossproduct(&v, &u);

        let equal = Myvec::new(0.0, -1.0, 1.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_cross_product_test2()
    {
        
        let v = Myvec::new( 1.01931509561836719512939453125e-05,
                            -0.005297116935253143310546875,
                            7.84755684435367584228515625e-07);

        let u = Myvec::new(1.81550502777099609375,
                           0.0,
                           -0.6772263050079345703125);

        let uv = Myvec::crossproduct(&v, &u);

        let equal = Myvec::new(0.0035873469896614551544189453125,
                               8.327797331730835139751434326171875e-06,
                               0.009616942144930362701416015625);
                               
        assert_eq!(uv, equal);

    }

    #[test]
    fn v_dot_product_test1()
    {
        
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::dotproduct(&v, &u);

        assert_eq!(uv, 140.0);
    }
        


    #[test]
    fn v_dot_product_test2()
    {
        
        let v = Myvec::new(-0.9932043552398681640625,
            -3.16645900966250337660312652587890625e-06,
            0.116383351385593414306640625);

        let u = Myvec::new(0.0, 0.0, 1.0);

        let uv = Myvec::dotproduct(&v, &u);

        assert_eq!(uv, 0.116383351385593414306640625);
    }

    #[test]
    fn v_multiply_by_scalar_test1()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        let u = Myvec::multiplyscalar(&v, 3.0);

        let equal = Myvec::new(3.0, 6.0, 9.0);
        assert_eq!(u, equal);
    }
        
    #[test]
    fn v_divide_by_scalar_test1()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        let u = Myvec::dividescalar(&v, 3.0);

        let equal = Myvec::new(0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(u, equal);
    }


    //MATRIX TESTS
    #[test]
    fn m_inverse_test1()
    {  
        let m = Mymatrix::new(1.0, 2.0, 3.0, 
                              4.0, 5.0, 6.0,
                              7.0, 8.0, 9.0).inverse();

        let equal = Mymatrix::new(-3.0, 6.0, -3.0,
                                  6.0, -12.0, 6.0,
                                  -3.0, 6.0, -3.0);
        assert_eq!(m, equal);
    }


    #[test]
    fn m_inverse_test2()
    { 
        let m = Mymatrix::new(2549.629150390625, -0.0, 166.91925048828125, 
                              -0.0, 2024.4990234375, -0.0, 
                              166.91925048828125, -0.0, 4414.73388671875).inverse();

        let equal = Mymatrix::new(0.0003931871615350246429443359375, -0.0, -1.48662438732571899890899658203125e-05,
                                  -0.0, 0.0004939493373967707157135009765625, -0.0,
                                  -1.48662438732571899890899658203125e-05, -0.0, 0.0002270762925036251544952392578125);
        assert_eq!(m, equal);
    }

        
    #[test]
    fn m_multiply_by_vec_test1()
    {  
        let m = Mymatrix::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let v = Mymatrix::multiply_matrix_by_vec(&m, &Myvec::new(3.0, 3.0, 3.0));

        let equal = Myvec::new(18.0, 45.0, 72.0);
        assert_eq!(v, equal);

    }

    #[test]
    fn m_multiply_by_vec_test2()
    {  
        let m = Mymatrix::new(2549.629150390625, -0.0, 166.91925048828125, 
                              -0.0, 2024.4990234375, -0.0, 
                              166.91925048828125, -0.0, 4414.73388671875);

        let v = Mymatrix::multiply_matrix_by_vec(&m, &Myvec::new(0.000029893242754042148590087890625, 0.063622482120990753173828125, -0.000000184518285095691680908203125));

        let equal = Myvec::new(0.076185882091522216796875, 128.80364990234375, 0.00417515868321061134338379);
        assert_eq!(v, equal);

    }




    //QUATERNION TESTS

    #[test]
    fn q_magnitude_test1()   
    { 
        let q = Myquaternion::new(2.0, 2.0, 2.0, 2.0);
        
        let equal = 4.0;
        assert_eq!(q.magnitude(), equal);
    }

    #[test]
    fn q_magnitude_test2()   
    { 
        let q = Myquaternion::new(8.1, 15.25, 0.1, 2.89);
        
        let equal = 17.5081310272216796875;
        assert_eq!(q.magnitude(), equal);
    }
        
    #[test]
    fn q_add_quaternion_test1()   
    {
        let mut q1 = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Myquaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = Myquaternion::addquat(&q1, &q2);

        let equal = Myquaternion::new(5.0, 2.0, 4.0, 6.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_add_quaternion_test2()   
    {
        let mut q1 = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = Myquaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        q1 = Myquaternion::addquat(&q1, &q2);

        let equal = Myquaternion::new(1.201631069183349609375, -0.375548690557479858398438, -0.0742167681455612182617188, -0.873655140399932861328125);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_scalar_test1()   
    {

        let mut q = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        q = Myquaternion::multiplyscalar(&q, 3.0);

        let equal = Myquaternion::new(6.0, 3.0, 6.0, 9.0);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_multiply_by_scalar_test2()   
    {

        let mut q = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = Myquaternion::multiplyscalar(&q, 0.016666668);

        let equal = Myquaternion::new(0.0166363250464200973510742, -0.000379226228687912225723267, -0.000666224921587854623794556, 0.00065028353128582239151001);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_divide_by_scalar_test1()   
    {
        let mut q = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        q = Myquaternion::dividescalar(&q, 3.0);

        let equal = Myquaternion::new(0.666666686534881591796875, 0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_divide_by_scalar_test2()   
    {
        let mut q = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = Myquaternion::dividescalar(&q, 0.016666668 );

        let equal = Myquaternion::new(59.8907623291015625,-1.36521422863006591796875, -2.3984096050262451171875, 2.3410205841064453125);
        assert_eq!(q, equal);
    }
        
    #[test]
    fn q_conjugate_test1()   
    {
        let q1 = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Myquaternion::conjugate(&q1);

        let equal = Myquaternion::new(2.0, -1.0, -2.0, -3.0);
        assert_eq!(q2, equal);
    }
        
    #[test]
    fn q_conjugate_test2()   
    {
        let q1 = Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let q2 = Myquaternion::conjugate(&q1);

        let equal = Myquaternion::new(0.319945394992828369140625, -0.0418616682291030883789062, 0.26201188564300537109375, 0.90952312946319580078125);
        assert_eq!(q2, equal);
    }

    #[test]
    fn q_multiply_by_quaternion_test1()
    {
        let mut q1 = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Myquaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = Myquaternion::multiplyquat(&q1, &q2);

        let equal = Myquaternion::new(-8.0, 5.0, 10.0 , 15.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_quaternion_test2()
    {
        let mut q1 = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        q1 = Myquaternion::multiplyquat(&q1, &q2);

        let equal = Myquaternion::new(0.345328778028488159179688, 0.0810852870345115661621094, -0.293385803699493408203125, -0.8877489566802978515625);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_vec_test1()
    {
        let mut q1 = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = Myvec::new(3.0, 1.0, 2.0);
        q1 = Myquaternion::multiply_quat_by_vec(&q1, &v);

        let equal = Myquaternion::new(-11.0, 7.0, 9.0, -1.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_vec_test2()
    {
        let mut q1 = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let v = Myvec::new(127.105736, -13.1427, -10.31398);
        q1 = Myquaternion::multiply_quat_by_vec(&q1, &v);

        let equal = Myquaternion::new(2.7691707611083984375, 127.79940032958984375, -8.39416790008544921875, -4.91529941558837890625);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_rotate_quaternion_by_vector_test1()
    {
        let q1 = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let mut v = Myvec::new(3.0, 1.0, 2.0);
        v = Myquaternion::qvrotate(&q1, &v);

        let equal = Myvec::new(-4.0, 62.0, 26.0);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_rotate_quaternion_by_vector_test2()
    {
        let q1 = Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let mut v = Myvec::new(127.105736, -13.1427, -10.31398);
        v = Myquaternion::qvrotate(&q1, &v);

        let equal = Myvec::new(128.1537475585937, -3.393682956695556640625, 0.285190761089324951171875);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_quaternion_from_euler_angles_test1()
    {
        let q = Myquaternion::make_q_from_euler(5.0, 7.0, 10.0);

        let equal = Myquaternion::new(0.993622303009033203125,  0.0380566865205764770507812, 0.0645529404282569885253906, 0.0842576175928115844726562);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_make_quaternion_from_euler_angles_test2()
    {
        let q = Myquaternion::make_q_from_euler(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);

        let equal = Myquaternion::new(0.998179376125335693359375, -0.0227535702288150787353516, -0.0399734899401664733886719, 0.0390170104801654815673828);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test1()
    {
        let q = Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = Myquaternion::make_euler_from_q(&q);

        let equal = Myvec::new(0.0, 90.0, 15.9453945159912109375);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test2()
    {
        let q = Myquaternion::new(0.99817944, -0.022753572, -0.039973494,0.03901701);
        let v = Myquaternion::make_euler_from_q(&q);

        let equal = Myvec::new(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test3()
    {
        let q = Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let v = Myquaternion::make_euler_from_q(&q);

        let equal = Myvec::new(30.3658618927001953125, -5.25052165985107421875, -142.664825439453125);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test4()
    {
        let q = Myquaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        let v = Myquaternion::make_euler_from_q(&q);

        let equal = Myvec::new(-6.178070545196533203125, -41.140392303466796875, -152.5458831787109375);
        assert_eq!(v, equal);
    }
}
