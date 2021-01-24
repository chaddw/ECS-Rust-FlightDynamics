//This file contains the data structures for use to compute equations of motion and track keyboard presses

//Coordinate transforms/nalgebra vector
use coord_transforms::prelude::*;

//Performance data of the airplane, contains lifting surface data and mass properties. This structure is used in DataFDM Component
#[derive(Debug, Default)]
pub struct PerformanceData
{
    pub wing_area: f64,
    pub wing_span: f64,
    pub tail_area: f64,
    pub cl_slope0: f64,   // slope of Cl-alpha curve
    pub cl0: f64,         // intercept of Cl-alpha curve
    pub cl_slope1: f64,    // post-stall slope of Cl-alpha curve
    pub cl1: f64,        // post-stall intercept of Cl-alpha curve
    pub alpha_cl_max: f64,  // alpha when Cl=Clmax
    pub cdp: f64,         // parasite drag coefficient
    pub eff: f64,         // induced drag efficiency coefficient
    pub mass: f64,
    pub engine_power: f64,
    pub engine_rps: f64,   // revolutions per second
    pub prop_diameter: f64,
    pub a: f64,           //  propeller efficiency coefficient
    pub b: f64,           //  propeller efficiency coefficient
}


//Component containing data of the airplane
#[derive(Debug, Default)]
pub struct DataFDM
{
    pub q: Vec<f64>, //will store ODE results
    pub airspeed: f64, //speed m/s

    pub position: Vector3<f64>,
    pub lla_origin: Vector3<f64>,
    pub climb_angle: f64,
    pub heading_angle: f64,
    pub climb_rate: f64,

    pub bank: f64, //bank angle
    pub alpha: f64, //angle of attack
    pub throttle: f64, //throttle percentage
    pub flap: f64, //flap deflection amount

    pub mass_properties : PerformanceData,



}


//Component tracking whether a key is pressed or not
#[derive(Debug)]
pub struct KeyboardState
{
    pub throttle_up: bool,
    pub throttle_down: bool,
    pub aoa_up: bool,
    pub aoa_down: bool,
    pub bank_right: bool,
    pub bank_left: bool,
    pub flaps_down: bool,
    pub zero_flaps: bool,
}