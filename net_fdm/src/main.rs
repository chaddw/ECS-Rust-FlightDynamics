
use std::net::UdpSocket;
use std::{thread, time};

//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --vc=0 --timeofday=noon --native-fdm=socket,in,30,,5500,udp


#[derive(Default)]
#[repr(C)] //fix padding issue
struct FGNetFDM
{ 
    version: u32, // increment when data values change
    padding: f32, // padding

    // // Positions
    longitude: f64, // geodetic (radians)
    latitude: f64, // geodetic (radians)
    altitude: f64, // above sea level (meters)
    agl: f32, // above ground level (meters)
    phi: f32, // roll (radians)
    theta: f32, // pitch (radians)
    psi: f32, // yaw or true heading (radians)
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
    spoilers: f32
}

fn main()
{

    let d2r = (3.14159 / 180.0) as f64; 

    let fg_net_fdm_version = 24_u32;
    let millis = time::Duration::from_millis(1000);

    let latitude: f64 = 45.59823;
    let longitude: f64= -120.6902;
    let altitude: f64 = 150.0;

    let mut roll: f32 = 0.0;
    let pitch: f32 = 0.0;
    let yaw: f32 = 0.0;

    let visibility: f32 = 5000.0;

    loop
    {
        thread::sleep(millis);

        //create fdm instance
        let mut fdm: FGNetFDM = Default::default();

        //convert to network byte order
        fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());
        fdm.latitude = f64::from_be_bytes((latitude * d2r).to_ne_bytes());
        fdm.longitude = f64::from_be_bytes((longitude * d2r).to_ne_bytes());
        fdm.altitude = f64::from_be_bytes((altitude).to_ne_bytes());

        //casting to make everything happy and then convert to network byte order
        let _phi = roll*d2r as f32;
        let _theta = pitch*d2r as f32;
        let _psi = yaw*d2r as f32;
        fdm.phi = f32::from_be_bytes(_phi.to_ne_bytes());
        fdm.theta = f32::from_be_bytes(_theta.to_ne_bytes());
        fdm.psi = f32::from_be_bytes(_psi.to_ne_bytes());

        //convert to network byte order
        fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());
        fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());

        //create socket and connect to flightgear
        let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
        socket.connect("127.0.0.1:5500").expect("connect function failed");

        //convert struct array of u8 of bytes
        let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };
        println!("{:?}", bytes);

        //finally send &[u8] of bytes to flight gear
        socket.send(bytes).expect("couldn't send message");

        //roll 5 degrees
        roll = roll + 5.0; 
        if roll > 20.0
        {
            roll = 0.0;
        }
        println!("Roll: {}", roll);
    }
}

//for converting to slice of u8
unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8]
{
    ::std::slice::from_raw_parts((p as *const T) as *const u8,::std::mem::size_of::<T>(),)
}

//Some references used:

    //converting to bytes
    //https://stackoverflow.com/questions/29445026/converting-number-primitives-i32-f64-etc-to-byte-representations

    //htonl function in rust
    //https://docs.rs/socket/0.0.7/socket/fn.htonl.html

    //struct padding in rust vs c++
    //https://rust-lang.github.io/unsafe-code-guidelines/layout/structs-and-tuples.html

    //sending struct as u8 slice
    //https://stackoverflow.com/questions/29307474/how-can-i-convert-a-buffer-of-a-slice-of-bytes-u8-to-an-integer
    //https://stackoverflow.com/questions/28127165/how-to-convert-struct-to-u8


