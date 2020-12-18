//This file contains all of the data required by the Systems: EquationsOfMotion, FlightControl, MakePacket, SendPacket
//The Components are: KeyboardState, EquationsOfMotion, Packet 
//The structure part of DataFDM is: Performance Data. The structure part of Packet is: FGNetFDM
//The resource is: DeltaTime

//SPECS
use specs::prelude::*;

//Contains Vector3
//use coord_transforms::prelude::*;

//Converting FGNetFDM struct to bytes to be sent as a packet
use serde::{Deserialize, Serialize};

//Time step (delta time) shared resource
#[derive(Default)]
pub struct DeltaTime(pub f64);

//Component tracking whether a key is pressed or not
#[derive(Debug)]
pub struct KeyboardState
{
    pub thrust_up: bool,
    pub thrust_down: bool,
    pub aoa_up: bool,
    pub aoa_down: bool,
    pub bank_right: bool,
    pub bank_left: bool,
    pub flaps_down: bool,
    pub zero_flaps: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}



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

//Component containing data on the airplane
#[derive(Debug, Default)]
pub struct DataFDM
{
    pub q: Vec<f64>, //will store ODE results
    pub airspeed: f64, //speed m/s
    pub delta_traveled: f64, //tracks distance traveled per frame

    pub position: Vec<f64>,
    pub climb_angle: f64,
    pub heading_angle: f64,
    pub climb_rate: f64,

    pub bank: f64, //bank angle
    pub alpha: f64, //angle of attack
    pub throttle: f64, //throttle percentage
    pub flap: f64, //flap deflection amount

    pub mass_properties : PerformanceData,



}
impl Component for DataFDM
{
    type Storage = VecStorage<Self>;
}






//Component containg the the FGNetFDM structure, and its conversion into bytes
#[derive(Debug, Default)]
pub struct Packet
{
    pub fgnetfdm: FGNetFDM,
    pub bytes: Vec<u8>,
}
impl Component for Packet
{
    type Storage = VecStorage<Self>;
}


//FlightGear defined structure for UDP packets
#[derive(Debug, Default, Serialize, Deserialize)]
#[repr(C)]
pub struct FGNetFDM
{
    pub version: u32, // increment when data values change
    padding: f32, // padding

    // // Positions
    pub longitude: f64, // geodetic (radians)
    pub latitude: f64, // geodetic (radians)
    pub altitude: f64, // above sea level (meters)
    agl: f32, // above ground level (meters)
    pub phi: f32, // roll (radians)
    pub theta: f32, // pitch (radians)
    pub psi: f32, // yaw or true heading (radians)
    alpha: f32, // angle of attack (radians)
    beta: f32, // side slip angle (radians)

    // // Velocities
    phidot: f32, // roll rate (radians/sec)
    thetadot: f32, // pitch rate (radians/sec)
    psidot: f32, // yaw rate (radians/sec)
    vcas: f32, // calibrated airspeed
    climb_rate: f32, // feet per second
    v_north: f32, // north velocity in local/body frame, fps
    v_east: f32, // east velocity in local/body frame, fps
    v_down: f32, // down/vertical velocity in local/body frame, fps
    v_body_u: f32, // ECEF velocity in body frame
    v_body_v: f32, // ECEF velocity in body frame 
    v_body_w: f32, // ECEF velocity in body frame
    
    // // Accelerations
    a_x_pilot: f32, // X accel in body frame ft/sec^2
    a_y_pilot: f32, // Y accel in body frame ft/sec^2
    a_z_pilot: f32, // Z accel in body frame ft/sec^2

    // // Stall
    stall_warning: f32, // 0.0 - 1.0 indicating the amount of stall
    slip_deg: f32, // slip ball deflection
    
    // // Engine status
    num_engines: u32, // Number of valid engines
    eng_state: [f32; 4], // Engine state (off, cranking, running)
    rpm: [f32; 4], // // Engine RPM rev/min
    fuel_flow: [f32; 4], // Fuel flow gallons/hr
    fuel_px: [f32; 4], // Fuel pressure psi
    egt: [f32; 4], // Exhuast gas temp deg F
    cht: [f32; 4], // Cylinder head temp deg F
    mp_osi: [f32; 4], // Manifold pressure
    tit: [f32; 4], // Turbine Inlet Temperature
    oil_temp: [f32; 4], // Oil temp deg F
    oil_px: [f32; 4], // Oil pressure psi

    // // Consumables
    num_tanks: u32, // Max number of fuel tanks
    fuel_quantity: [f32; 4], 

    // // Gear status
    num_wheels: u32, 
    wow: [f32; 3], 
    gear_pos: [f32; 3],
    gear_steer: [f32; 3],
    gear_compression: [f32; 3],

    // // Environment
    cur_time: f32, // current unix time
    warp: f32, // offset in seconds to unix time
    visibility: f32, // visibility in meters (for env. effects)

    // // Control surface positions (normalized values)
    elevator: f32,
    elevator_trim_tab: f32, 
    left_flap: f32,
    right_flap: f32,
    left_aileron: f32, 
    right_aileron: f32, 
    rudder: f32, 
    nose_wheel: f32,
    speedbrake: f32,
    spoilers: f32,
}
