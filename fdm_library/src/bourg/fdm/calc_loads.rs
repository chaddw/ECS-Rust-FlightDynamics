//This function calculates all of the forces and moments on the plane at any time (called inside eom)

//Get DataFDM struct
use crate::bourg::fdm::structures::DataFDM;

//Math utils
use crate::bourg::common::math_utils::deg_to_rad;
use crate::bourg::common::math_utils::rad_to_deg;

//Vector, quaternion modules
use crate::bourg::common::vector::Vector;
use crate::bourg::common::quaternion::Quaternion;

//Constants
use crate::bourg::common::constants::RHO;
use crate::bourg::common::constants::G;

//Airfoil performance data calculations
use crate::bourg::fdm::airfoil_coef::rudder_drag_coefficient;
use crate::bourg::fdm::airfoil_coef::rudder_lift_coefficient;
use crate::bourg::fdm::airfoil_coef::drag_coefficient;
use crate::bourg::fdm::airfoil_coef::lift_coefficient;

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