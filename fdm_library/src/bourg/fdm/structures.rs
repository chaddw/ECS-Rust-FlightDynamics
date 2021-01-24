//This file contains all of the data structures used to compute equations of motion and handle key presses

//Vector, Matrix, Quaternion module
use crate::bourg::common::vector::Vector;
use crate::bourg::common::matrix::Matrix;
use crate::bourg::common::quaternion::Quaternion;

//Elements making up the bodystructure, this is part of the DataFDM structure
#[derive(Debug)]
pub struct PointMass
{
    pub f_mass: f32,
    pub v_d_coords: Vector, //"design position"
    pub v_local_inertia: Vector,
    pub f_incidence: f32,
    pub f_dihedral: f32,
    pub f_area: f32,
    pub i_flap: i32,
    pub v_normal: Vector,
    pub v_cg_coords: Vector //"corrected position"
}

//State of the airplane
#[derive(Debug, Default)]
pub struct DataFDM
{
    pub mass: f32, //total mass
    pub m_inertia: Matrix,
    pub m_inertia_inverse: Matrix,
    pub v_position: Vector, // position in earth coordinates
    pub lla_origin: Vector, // geodetic origin position
    pub v_velocity: Vector, // velocity in earth coordinates
    pub v_velocity_body: Vector, // velocity in body coordinates
    pub v_angular_velocity: Vector, // angular velocity in body coordinates
    pub v_euler_angles: Vector,   
    pub f_speed: f32, // speed (magnitude of the velocity)
    pub stalling: bool,
    pub flaps: bool,
    pub q_orientation: Quaternion, // orientation in earth coordinates 
    pub v_forces: Vector, // total force on body
    pub thrustforce: f32, // magnitude of thrust
    pub v_moments: Vector, // total moment (torque) on body
    pub element: Vec<PointMass>, // vector of point mass elements
}



//State machine for keyboard presses
#[derive(Debug)]
pub struct KeyboardState
{
    pub thrust_up: bool,
    pub thrust_down: bool,
    pub left_rudder: bool,
    pub right_rudder: bool,
    pub roll_left: bool,
    pub roll_right: bool,
    pub pitch_up: bool,
    pub pitch_down: bool,
    pub flaps_down: bool,
    pub zero_flaps: bool,
}