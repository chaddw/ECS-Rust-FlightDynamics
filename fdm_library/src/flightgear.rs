//FGNetFDM packet structure required by FlightGear

//Converting FGNetFDM struct to bytes to be sent as a packet
use serde::{Deserialize, Serialize};

//Structure for making a network packet to be sent to FlightGear
#[derive(Debug, Default, Serialize, Deserialize)]
#[repr(C)] 
pub struct FGNetFDM
{
    pub version: u32, // increment when data values change
    pub padding: f32, // padding

    // // Positions
    pub longitude: f64, // geodetic (radians)
    pub latitude: f64, // geodetic (radians)
    pub altitude: f64, // above sea level (meters)
    pub agl: f32, // above ground level (meters)
    pub phi: f32, // roll (radians)
    pub theta: f32, // pitch (radians)
    pub psi: f32, // yaw or true heading (radians)
    pub alpha: f32, // angle of attack (radians)
    pub beta: f32, // side slip angle (radians)

    // // Velocities
    pub phidot: f32, // roll rate (radians/sec)
    pub thetadot: f32, // pitch rate (radians/sec)
    pub psidot: f32, // yaw rate (radians/sec)
    pub vcas: f32, // calibrated airspeed
    pub climb_rate: f32, // feet per second
    pub v_north: f32, // north velocity in local/body frame, fps
    pub v_east: f32, // east velocity in local/body frame, fps
    pub v_down: f32, // down/vertical velocity in local/body frame, fps
    pub v_body_u: f32, // ECEF velocity in body frame
    pub v_body_v: f32, // ECEF velocity in body frame 
    pub v_body_w: f32, // ECEF velocity in body frame
    
    // // Accelerations
    pub a_x_pilot: f32, // X accel in body frame ft/sec^2
    pub a_y_pilot: f32, // Y accel in body frame ft/sec^2
    pub a_z_pilot: f32, // Z accel in body frame ft/sec^2

    // // Stall
    pub stall_warning: f32, // 0.0 - 1.0 indicating the amount of stall
    pub slip_deg: f32, // slip ball deflection
    
    // // Engine status
    pub num_engines: u32, // Number of valid engines
    pub eng_state: [f32; 4], // Engine state (off, cranking, running)
    pub rpm: [f32; 4], // // Engine RPM rev/min
    pub fuel_flow: [f32; 4], // Fuel flow gallons/hr
    pub fuel_px: [f32; 4], // Fuel pressure psi
    pub egt: [f32; 4], // Exhuast gas temp deg F
    pub cht: [f32; 4], // Cylinder head temp deg F
    pub mp_osi: [f32; 4], // Manifold pressure
    pub tit: [f32; 4], // Turbine Inlet Temperature
    pub oil_temp: [f32; 4], // Oil temp deg F
    pub oil_px: [f32; 4], // Oil pressure psi

    // // Consumables
    pub num_tanks: u32, // Max number of fuel tanks
    pub fuel_quantity: [f32; 4], 

    // // Gear status
    pub num_wheels: u32, 
    pub wow: [f32; 3], 
    pub gear_pos: [f32; 3],
    pub gear_steer: [f32; 3],
    pub gear_compression: [f32; 3],

    // // Environment
    pub cur_time: f32, // current unix time
    pub warp: f32, // offset in seconds to unix time
    pub visibility: f32, // visibility in meters (for env. effects)

    // // Control surface positions (normalized values)
    pub elevator: f32,
    pub elevator_trim_tab: f32, 
    pub left_flap: f32,
    pub right_flap: f32,
    pub left_aileron: f32, 
    pub right_aileron: f32, 
    pub rudder: f32, 
    pub nose_wheel: f32,
    pub speedbrake: f32,
    pub spoilers: f32,
}


