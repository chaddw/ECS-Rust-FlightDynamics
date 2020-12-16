//This file contains all of the data required by the System: EquationsOfMotion
//The Component is: DataFDM
//The structure that is a part of DataFDM is: PointMass
//The resources are: DeltaTime, MaxThrust, DeltaThrust

//SPECS
use specs::prelude::*;

//Vector, Matrix, Quaternion module
use crate::common::Myvec;
use crate::common::Mymatrix;
use crate::common::Myquaternion;

//Time step (delta time) shared resource
#[derive(Default)]
pub struct DeltaTime(pub f32);

//Max thrust potential resource
#[derive(Default)]
pub struct MaxThrust(pub f32);

//Delta thrust increment
#[derive(Default)]
pub struct DeltaThrust(pub f32);


//Elements making up the bodystructure, this is part of the DataFDM component
#[derive(Debug)]
pub struct PointMass
{
    pub f_mass: f32,
    pub v_d_coords: Myvec, //"design position"
    pub v_local_inertia: Myvec,
    pub f_incidence: f32,
    pub f_dihedral: f32,
    pub f_area: f32,
    pub i_flap: i32,
    pub v_normal: Myvec,
    pub v_cg_coords: Myvec //"corrected position"
}

//Component containing data on the airplane
#[derive(Debug, Default)]
pub struct DataFDM
{
    pub mass: f32, //total mass
    pub m_inertia: Mymatrix,
    pub m_inertia_inverse: Mymatrix,
    pub v_position: Myvec, // position in earth coordinates
    pub v_velocity: Myvec, // velocity in earth coordinates
    pub v_velocity_body: Myvec, // velocity in body coordinates
    pub v_angular_velocity: Myvec, // angular velocity in body coordinates
    pub v_euler_angles: Myvec,   
    pub f_speed: f32, // speed (magnitude of the velocity)
    pub stalling: bool,
    pub flaps: bool,
    pub q_orientation: Myquaternion, // orientation in earth coordinates 
    pub v_forces: Myvec, // total force on body
    pub thrustforce: f32, // magnitude of thrust
    pub v_moments: Myvec, // total moment (torque) on body
    pub element: Vec<PointMass>, // vector of point mass elements
    pub current_frame: usize,
}
impl Component for DataFDM
{
    type Storage = VecStorage<Self>;
}