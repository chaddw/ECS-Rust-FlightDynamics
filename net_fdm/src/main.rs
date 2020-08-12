
use std::net::UdpSocket;
use std::{thread, time};
//use socket::{AF_INET, SO_REUSEADDR, SOCK_DGRAM, Socket, SOL_SOCKET};
//use std::time::{Duration, SystemTime};
//use socket::htonl;
//use serde::ser::{Serialize, Serializer, SerializeStruct};
//use serde::{Deserialize, Serialize};



//#[allow(unused_imports)]
//extern crate byteorder;
//use byteorder::{ByteOrder, NetworkEndian, BigEndian, LittleEndian, WriteBytesExt};
//use std::mem::transmute;
//use std::mem;



// enum stuff {
//     FG_MAX_ENGINES,
//     FG_MAX_WHEELS,
//     FG_MAX_TANKS,
// }//{FG_MAX_ENGINES:: 4, FG_MAX_WHEELS:: 3, FG_MAX_TANKS:: 4}


//#[derive(Serialize, Deserialize, Debug)]
#[derive(Default)]
#[repr(C)] //fixed padding issue
#[allow(non_snake_case)]
struct FGNetFDM
{ //recreate the structure

    //enumOfStuff: stuff(FG_MAX_ENGINES: 4, FG_MAX_WHEELS: 3, FG_MAX_TANKS: 4),


    version: u32, // uint32_t version;		// increment when data values change
    padding: f32, // uint32_t padding;		// padding

    // // Positions
    longitude: f64, // double longitude;		// geodetic (radians)
    latitude: f64, // double latitude;		// geodetic (radians)
    altitude: f64, // double altitude;		// above sea level (meters)
    agl: f32, // float agl;			// above ground level (meters)
    phi: f32, // float phi;			// roll (radians)
    theta: f32, // float theta;		// pitch (radians)
    psi: f32, // float psi;			// yaw or true heading (radians)
    alpha: f32, // float alpha;                // angle of attack (radians)
    beta: f32, // float beta;                 // side slip angle (radians)

    // // Velocities
    phidot: f32, // float phidot;		// roll rate (radians/sec)
    thetadot: f32, // float thetadot;		// pitch rate (radians/sec)
    psidot: f32, // float psidot;		// yaw rate (radians/sec)
    vcas: f32, // float vcas;		        // calibrated airspeed
    climb_rate: f32, // float climb_rate;		// feet per second
    v_north: f32, // float v_north;              // north velocity in local/body frame, fps
    v_east: f32, // float v_east;               // east velocity in local/body frame, fps
    v_down: f32, // float v_down;               // down/vertical velocity in local/body frame, fps
    v_body_u: f32, // float v_body_u;    // ECEF velocity in body frame
    v_body_v: f32, // float v_body_v;    // ECEF velocity in body frame 
    v_body_w: f32, // float v_body_w;    // ECEF velocity in body frame
    
    // // Accelerations
    A_X_pilot: f32, // float A_X_pilot;		// X accel in body frame ft/sec^2
    A_y_pilot: f32, // float A_Y_pilot;		// Y accel in body frame ft/sec^2
    A_Z_pilot: f32,// float A_Z_pilot;		// Z accel in body frame ft/sec^2

    // // Stall
    stall_warning: f32,// float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    slip_deg: f32, // float slip_deg;		// slip ball deflection

    // // Pressure
    
    // // Engine status
    num_engines: u32, // uint32_t num_engines;	     // Number of valid engines
    eng_state: [f32; 4], // uint32_t eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    rpm: [f32; 4], // float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
    fuel_flow: [f32; 4], // float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    fuel_px: [f32; 4], // float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    egt: [f32; 4], // float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    cht: [f32; 4], // float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    mp_osi: [f32; 4], // float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    tit: [f32; 4], // float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    oil_temp: [f32; 4], // float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    oil_px: [f32; 4], // float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // // Consumables
    num_tanks: u32, // uint32_t num_tanks;		// Max number of fuel tanks
    fuel_quantity: [f32; 4], // float fuel_quantity[FG_MAX_TANKS];

    // // Gear status
    num_wheels: u32, // uint32_t num_wheels;
    wow: [f32; 3], // uint32_t wow[FG_MAX_WHEELS];
    gear_pos: [f32; 3], // float gear_pos[FG_MAX_WHEELS];
    gear_steer: [f32; 3],// float gear_steer[FG_MAX_WHEELS];
    gear_compression: [f32; 3],// float gear_compression[FG_MAX_WHEELS];

    // // Environment
    cur_time: f32, // uint32_t cur_time;           // current unix time
    //                                 // FIXME: make this uint64_t before 2038
    warp: f32, // int32_t warp;                // offset in seconds to unix time
    visibility: f32, // float visibility;            // visibility in meters (for env. effects)

    // // Control surface positions (normalized values)
    elevator: f32, // float elevator;
    elevator_trim_tab: f32, // float elevator_trim_tab;
    left_flap: f32, // float left_flap;
    right_flap: f32, // float right_flap;
    left_aileron: f32, // float left_aileron;
    right_aileron: f32, // float right_aileron;
    rudder:f32, // float rudder;
    nose_wheel: f32,// float nose_wheel;
    speedbrake: f32,// float speedbrake;
    spoilers: f32// float spoilers;

}

    

#[allow(non_snake_case)]
fn main()
{

    let D2R = (3.14159 / 180.0) as f64; //should be float... htonl wants u32
    //println!("{}", D2R);
    let FG_NET_FDM_VERSION = 24_u32;
    let millis = time::Duration::from_millis(1000);

    let latitude: f64 = 45.59823;
    let longitude: f64= -120.6902;
    let altitude: f64 = 150.0;

    let mut roll: f32 = 0.0;
    let mut pitch: f32 = 0.0;
    let mut yaw: f32 = 0.0;

    let visibility: f32 = 5000.0;

    while(true)
    {

        thread::sleep(millis);

    //create fdm instance
    //let fdm = FGNetFDM{altitude: 150.0, ..Default::default() }; 
    let mut fdm: FGNetFDM = Default::default();


    //fdm.version = htonl(FG_NET_FDM_VERSION); //htonl is doing the same as below
    fdm.version = u32::from_be_bytes(FG_NET_FDM_VERSION.to_ne_bytes());
   
    fdm.latitude = f64::from_be_bytes((latitude * D2R).to_ne_bytes());
    fdm.longitude = f64::from_be_bytes((longitude * D2R).to_ne_bytes());
    fdm.altitude = f64::from_be_bytes((altitude).to_ne_bytes());


    //casting to make everything happy
    let _phi = roll*D2R as f32;
    let _theta = pitch*D2R as f32;
    let _psi = yaw*D2R as f32;
    fdm.phi = f32::from_be_bytes(_phi.to_ne_bytes());
    fdm.theta = f32::from_be_bytes(_theta.to_ne_bytes());
    fdm.psi = f32::from_be_bytes(_psi.to_ne_bytes());


    fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());// fdm.num_engines = htonl(1);

    fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());// fdm.num_tanks = htonl(1);
    //fdm.fuel_quantity[0] = htonl(100.0);

    fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());//fdm.num_wheels = htonl(1);

    //fdm.cur_time = htonl(SystemTime::now());
    fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());// fdm.warp = htonl(1);

    fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());//fdm.visibility = htonl(visibility); //this doesnt actually seem to do anything


    //create socket and connect to flightgear
    let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
    socket.connect("127.0.0.1:5500").expect("connect function failed");

    //convert struct array of u8
    let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };
    println!("{:?}", bytes);

    //finally send &[u8] of bytes to flight gear
     socket.send(bytes).expect("couldn't send message");

    roll = roll + 5.0; 
    if roll > 20.0
    {
        roll = 0.0;
    }
    println!("Roll: {}", roll);


    }
}


unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8]
{
    ::std::slice::from_raw_parts((p as *const T) as *const u8,::std::mem::size_of::<T>(),)
}



//Some references:

//converting to bytes
//https://stackoverflow.com/questions/29445026/converting-number-primitives-i32-f64-etc-to-byte-representations

//htonl function in rust
//https://docs.rs/socket/0.0.7/socket/fn.htonl.html

//struct padding in rust vs c++
//https://rust-lang.github.io/unsafe-code-guidelines/layout/structs-and-tuples.html

//sending struct as u8 slice
//https://stackoverflow.com/questions/29307474/how-can-i-convert-a-buffer-of-a-slice-of-bytes-u8-to-an-integer
//https://stackoverflow.com/questions/28127165/how-to-convert-struct-to-u8

