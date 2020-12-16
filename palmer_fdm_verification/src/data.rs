//This file contains all of the data required by the System: EquationsOfMotion
//The Component is: DataFDM
//The structure that is a part of DataFDM is: PerformanceData
//The resource is:  DeltaThrust

//This file contains all of the Component data required by the System: DataFDM. Also, present here is the DeltaTime resource

//SPECS
use specs::prelude::*;

//Time step (delta time) shared resource
#[derive(Default)]
pub struct DeltaTime(pub f64);

//Performance data of the airplane, is used in DataFDM Component
#[derive(Debug, Default)]
pub struct PerformanceData
{
    pub wing_area: f64,
    pub wing_span: f64,
    pub tail_area: f64,
    pub cl_slope0: f64,   // slope of Cl-alpha curve
    pub cl0: f64,         // intercept of Cl-alpha curve
    pub cl_slope1: f64,   // post-stall slope of Cl-alpha curve
    pub cl1: f64,         // post-stall intercept of Cl-alpha curve
    pub alpha_cl_max: f64,// alpha when Cl=Clmax
    pub cdp: f64,         // parasite drag coefficient
    pub eff: f64,         // induced drag efficiency coefficient
    pub mass: f64,
    pub engine_power: f64,
    pub engine_rps: f64,  // revolutions per second
    pub prop_diameter: f64,
    pub a: f64,           //  propeller efficiency coefficient
    pub b: f64,           //  propeller efficiency coefficient
}

//Component containing data of the airplane
#[derive(Debug, Default)]
pub struct DataFDM
{
    pub current_frame: usize, //tracks current frame
    pub q: Vec<f64>, //will store ODE results
    pub airspeed: f64, //speed m/s
    pub bank: f64, //bank angle
    pub alpha: f64, //angle of attack
    pub throttle: f64, //throttle percentage
    pub flap: f64, //flap deflection amount

    pub mass_properties : PerformanceData, //lifting surface data

}
impl Component for DataFDM
{
    type Storage = VecStorage<Self>;
}
