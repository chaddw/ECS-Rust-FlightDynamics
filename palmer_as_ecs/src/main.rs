//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --timeofday=noon --native-fdm=socket,in,30,,5500,udp


//Flight control
extern crate device_query;
use device_query::{DeviceQuery, DeviceState, Keycode};

//Specs
use specs::prelude::*;

//Networking
use std::net::UdpSocket;

//Ellipsoid global variable
#[macro_use]
extern crate lazy_static;

//Exit program
use std::process;

//Main loop
use std::{thread, time};

//Coordinate conversions
extern crate coord_transforms;
use coord_transforms::prelude::*;

//Converting FGNetFDM struct to bytes to be sent as a packet
use serde::{Deserialize, Serialize};


//////Component State Machine for keyboard
#[derive(Debug)]
struct KeyboardState
{
    thrust_up: bool,
    thrust_down: bool,
    aoa_up: bool,
    aoa_down: bool,
    bank_right: bool,
    bank_left: bool,
    flaps_down: bool,
    zero_flaps: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}


//////Component Performance data of the airplane
#[derive(Debug, Default)]
struct PerformanceData
{
    wing_area: f64,
    wing_span: f64,
    tail_area: f64,
    cl_slope0: f64,   // slope of Cl-alpha curve
    cl0: f64,         // intercept of Cl-alpha curve
    cl_slope1: f64,    // post-stall slope of Cl-alpha curve
    cl1: f64,        // post-stall intercept of Cl-alpha curve
    alpha_cl_max: f64,  // alpha when Cl=Clmax
    cdp: f64,         // parasite drag coefficient
    eff: f64,         // induced drag efficiency coefficient
    mass: f64,
    engine_power: f64,
    engine_rps: f64,   // revolutions per second
    prop_diameter: f64,
    a: f64,           //  propeller efficiency coefficient
    b: f64,           //  propeller efficiency coefficient
}

//Component containing data on the airplane
#[derive(Debug, Default)]
struct DataFDM
{
    current_frame: usize,  
    q: Vec<f64>, //will store ODE results
    airspeed: f64,
    delta_traveled: f64,

    bank: f64, //bank angle
    alpha: f64, //angle of attack
    throttle: f64, //throttle percentage
    flap: f64, //flap deflection amount

    mass_properties : PerformanceData,

    ecef_vec: Vector3<f64>

}
impl Component for DataFDM
{
    type Storage = VecStorage<Self>;
}

//////Component FGNetFDM for networking
#[derive(Debug, Default, Serialize, Deserialize)]
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
    spoilers: f32,
}


//Component containg the packet created
#[derive(Debug, Default)]
struct Packet
{
    bytes: Vec<u8>,
}
impl Component for Packet
{
    type Storage = VecStorage<Self>;
}

//System to handle user input
struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = ( 
        ReadStorage<'a, DataFDM>, 
        WriteStorage<'a, KeyboardState>,
    );
    //we dont need input data!!!! need a way to take this out and read in only keyboardstate
    fn run(&mut self, (fdmdata, mut keyboardstate): Self::SystemData) 
    {
        for (_fdm, keystate) in (&fdmdata, &mut keyboardstate).join() 
        {
            //Set all states false before we know if they are being activated
            keystate.thrust_up = false; 
            keystate.thrust_down = false;
            keystate.aoa_up = false;
            keystate.aoa_down = false;
            keystate.bank_left = false;
            keystate.bank_right = false;
            keystate.flaps_down = false;
            keystate.zero_flaps = false;

            //Setup device query states
            let device_state = DeviceState::new();
            let keys: Vec<Keycode> = device_state.get_keys();

            //Thrust
            if keys.contains(&Keycode::A)
            {
                keystate.thrust_up = true;
            }
            else if keys.contains(&Keycode::Z)
            {
                keystate.thrust_down = true;
            }

            //angle of attack
            if keys.contains(&Keycode::Down)
            {
                keystate.aoa_up = true;
            }
            else if keys.contains(&Keycode::Up)
            {
                keystate.aoa_down = true;
            }

            //bank 
            if keys.contains(&Keycode::Left)
            {
                keystate.bank_left = true;
            }
            else if keys.contains(&Keycode::Right)
            {
                keystate.bank_right = true;
            }

            //flaps
            if keys.contains(&Keycode::F)
            {
                keystate.flaps_down = true;
            }
            else if keys.contains(&Keycode::G)
            {
                keystate.zero_flaps = true;
            }

            //Quit program
            if keys.contains(&Keycode::Q)
            {
                process::exit(1);
            }

        }//end for
    }//end run
}//end system


//System to perform physics calculations
struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        WriteStorage<'a, DataFDM>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (mut fdmdata, keyboardstate): Self::SystemData) 
    {
        for (fdm, keystate) in (&mut fdmdata, &keyboardstate).join() 
        {
            
            let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


            //Handle the input states
            //Thrust states
            if fdm.throttle < 1.0 && keystate.thrust_up == true
            {
                fdm.throttle = fdm.throttle + 0.05;
                
            }   
            else if fdm.throttle > 0.0 && keystate.thrust_down == true
            {
                fdm.throttle = fdm.throttle - 0.05;
                if fdm.throttle < 0.001 
                {
                    fdm.throttle = 0.0;
                }
            }  
            //Angle of attack states
            if fdm.alpha < 20.0 && keystate.aoa_up == true
            {
                fdm.alpha = fdm.alpha + 1.0;
            
            }  
            else if fdm.alpha > -16.0 && keystate.aoa_down == true
            {
                fdm.alpha = fdm.alpha - 1.0
            }  
            //Bank states
            if fdm.bank < 20.0 && keystate.bank_right == true
            {
                fdm.bank = fdm.bank + 1.0;
            
            }  
            else if fdm.bank > -16.0 && keystate.bank_left == true
            {
                fdm.bank = fdm.bank - 1.0;
            }  

            //Flap states
            if fdm.flap == 0.0 && keystate.flaps_down == true
            {
                fdm.flap = 20.0;
            }  
            else if fdm.flap == 20.0 && keystate.flaps_down == true
            {
                fdm.flap = 40.0;
            }  
            else if (fdm.flap == 20.0 || fdm.flap == 40.0) && keystate.zero_flaps == true
            {
                fdm.flap = 0.0;
            }  
        
  
            //note:this closure is what was "planeRightHandSide"
            let a = |q: &mut Vec<f64>, delta_q: &mut Vec<f64>, &ds: & f64, q_scale: f64, dq: &mut Vec<f64>| 
            {
                let mut new_q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 
            
                let negativeone = -1.0_f64;
                let pi = negativeone.acos();
                let g: f64 = -9.81;
                let mut cl: f64;
                let cos_p: f64;   //  climb angle
                let sin_p: f64;   //  climb angle
                let cos_t: f64;   //  heading angle
                let sin_t: f64;   //  heading angle
            
                //  Convert bank angle from degrees to radians
                //  Angle of attack is not converted because the
                //  Cl-alpha curve is defined in terms of degrees.
                let bank = fdm.bank.to_radians();
            
                //  Compute the intermediate values of the 
                //  dependent variables.
                for i in 0..6
                {
                    new_q[i] = q[i] + q_scale * delta_q[i]; 
                }
            
                //  Assign convenenience variables to the intermediate 
                //  values of the locations and velocities.
                let vx: f64 = new_q[0];
                let vy: f64 = new_q[2];
                let vz: f64 = new_q[4];
                let _x: f64 = new_q[1];
                let _y: f64 = new_q[3];
                let z: f64 = new_q[5];
                let vh: f64 = (vx * vx + vy * vy).sqrt();
                let vtotal: f64 = (vx * vx + vy * vy + vz * vz).sqrt();
            
                //  Compute the air density
                let temperature: f64 = 288.15 - 0.0065 * z;
                let grp: f64 = 1.0 - 0.0065 * z / 288.15;
                let pressure: f64 = 101325.0 * (grp.powf(5.25));
                let density: f64 = 0.00348 * pressure / temperature;
            
                //  Compute power drop-off factor
                let omega: f64 = density / 1.225;
                let factor: f64 = (omega - 0.12)/  0.88;
            
                //  Compute thrust 
                let advance_ratio: f64 = vtotal / (fdm.mass_properties.engine_rps * fdm.mass_properties.prop_diameter);
                let thrust: f64 = fdm.throttle * factor * fdm.mass_properties.engine_power * (fdm.mass_properties.a + fdm.mass_properties.b * advance_ratio * advance_ratio) / (fdm.mass_properties.engine_rps * fdm.mass_properties.prop_diameter);
            
                //  Compute lift coefficient. The Cl curve is 
                //  modeled using two straight lines.
                if  fdm.alpha < fdm.mass_properties.alpha_cl_max
                {
                    cl = fdm.mass_properties.cl_slope0 * fdm.alpha + fdm.mass_properties.cl0;
                }
                else 
                {
                    cl = fdm.mass_properties.cl_slope1 * fdm.alpha + fdm.mass_properties.cl1;
                }
            
                //  Include effects of flaps and ground effects.
                //  Ground effects are present if the plane is
                //  within 5 meters of the ground.
                if fdm.flap == 20.0
                {
                    cl += 0.25;
                }
                if fdm.flap == 40.0
                {
                    cl += 0.5;
                }
                if z < 5.0
                {
                    cl += 0.25;
                }
            
                //  Compute lift
                let lift: f64 = 0.5 * cl * density * vtotal * vtotal * fdm.mass_properties.wing_area;
            
                // Compute drag coefficient
                let aspect_ratio: f64 = fdm.mass_properties.wing_span * fdm.mass_properties.wing_span / fdm.mass_properties.wing_area;
                let cd = fdm.mass_properties.cdp + cl * cl / (pi * aspect_ratio * fdm.mass_properties.eff);
                
                //  Compute drag force
                let drag: f64 = 0.5 * cd * density * vtotal * vtotal * fdm.mass_properties.wing_area;
            
                //  Define some shorthand convenience variables
                //  for use with the rotation matrix.
                //  Compute the sine and cosines of the climb angle,
                //  bank angle, and heading angle;
                let cos_w: f64 = bank.cos(); 
                let sin_w: f64 = bank.sin(); 
            
                if  vtotal == 0.0
                {
                    cos_p = 1.0;
                    sin_p = 0.0;
                }
                else
                {
                    cos_p = vh / vtotal;  
                    sin_p = vz / vtotal;  
                }
                
                if vh == 0.0
                {
                    cos_t = 1.0;
                    sin_t = 0.0;
                }
                else
                {
                    cos_t = vx / vh;
                    sin_t = vy / vh;
                }
            
                //  Convert the thrust, drag, and lift forces into
                //  x-, y-, and z-components using the rotation matrix.
                let fx: f64 = cos_t * cos_p * (thrust - drag) + (sin_t * sin_w - cos_t * sin_p * cos_w) * lift;
                let fy: f64 = sin_t * cos_p * (thrust - drag) + (-cos_t * sin_w - sin_t * sin_p * cos_w) * lift;
                let mut fz: f64 = sin_p * (thrust - drag) + cos_p * cos_w * lift;
            
                //  Add the gravity force to the z-direction force.
                fz = fz + fdm.mass_properties.mass * g;
            
                //  Since the plane can't sink into the ground, if the
                //  altitude is less than or equal to zero and the z-component
                //  of force is less than zero, set the z-force
                //  to be zero.
                if  z <= 0.0 && fz <= 0.0  
                {
                    fz = 0.0;
                }
            
                //  Load the right-hand sides of the ODE's
                dq[0] = ds * (fx / fdm.mass_properties.mass);
                dq[1] = ds * vx;
                dq[2] = ds * (fy / fdm.mass_properties.mass);
                dq[3] = ds * vy;
                dq[4] = ds * (fz / fdm.mass_properties.mass);
                dq[5] = ds * vz;

            }; //End closure


            //Begin what was "rangeKutta4" method
            let priorx = fdm.q[1]; //Will be used to calculate delta_traveled

            //Retrieve value of dependent variable
            let mut q = fdm.q.clone();

            //Get the static time variable DT
            let ds = DT;

            // Compute the four Runge-Kutta steps, The return 
            // value of planeRightHandSide method is an array
            // of delta-q values for each of the four steps

            //calls "planeRightHandSide 4 times"
            a(&mut q, &mut qcopy,  &ds, 0.0, &mut dq1);
            a(&mut q, &mut dq1,    &ds, 0.5, &mut dq2);
            a(&mut q, &mut dq2,    &ds, 0.5, &mut dq3);
            a(&mut q, &mut dq3,    &ds, 1.0, &mut dq4);

            //  Update the dependent and independent variable values
            //  at the new dependent variable location and store the
            //  values in the ODE object arrays.
        
            for i in 0..6
            {
                q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
                fdm.q[i] = q[i];
            }
    
            fdm.delta_traveled = (fdm.q[1]) - (priorx); //get the change in meters from last frame to this frame, will be used to calculate new latitude based on how far we've gone
            fdm.ecef_vec.x = fdm.ecef_vec.x + fdm.delta_traveled; //add latitude change to the ecef

            fdm.airspeed = (fdm.q[0] * fdm.q[0] + fdm.q[2] * fdm.q[2] + fdm.q[4] * fdm.q[4]).sqrt();


            //Print some relevant data
            println!("x traveled (m) = {}", fdm.q[1] / 3.6); //converted to meters
            println!("altitude (m) = {}", fdm.q[5]);
            println!("airspeed (km/h) = {}", fdm.airspeed);
            println!("throttle % = {}", fdm.throttle);
            println!("angle of attack (deg) = {}", fdm.alpha);
            println!("x travel change (m) since last frame = {}", fdm.delta_traveled);
            println!("bank angle (deg) = {}", fdm.bank);
            println!("flap deflection (deg) = {}", fdm.flap);
        }//end for
    }//end run
}//end system



//System to make packets
struct MakePacket;
impl<'a> System<'a> for MakePacket
{
    type SystemData = (
        ReadStorage<'a, DataFDM>,
        WriteStorage<'a, Packet>,
    );

    fn run(&mut self, (datafdm, mut packet): Self::SystemData) 
    {
        for (fdm, mut pckt) in (&datafdm, &mut packet).join() 
        {
            //All data passed into the FGNetFDM struct is converted to network byte order

            //Create fdm instance
            let mut fg_fdm: FGNetFDM = Default::default();

            //Set Roll, Pitch, Yaw
            let roll: f32 = fdm.bank as f32;
            let pitch: f32 = fdm.alpha as f32; //Will use angle of attack because its "easier"
            let yaw: f32 = 90.0; //Only need to face in one direction

            //Coordinate conversion: cartesian to geodetic
            let lla = geo::ecef2lla(&fdm.ecef_vec, &ELLIPSOID); 


           //try no ecef
        //    fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
        //    fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
        //    fdm.altitude = f64::from_be_bytes(fdm.q[5].to_ne_bytes()); //lla.z seems to increase altitude artificially...

            //Set lat, long, alt
            fg_fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            fg_fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
            fg_fdm.altitude = f64::from_be_bytes(fdm.q[5].to_ne_bytes()); //lla.z seems to increase altitude artificially...

            //Roll, Pitch, Yaw
            fg_fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
            fg_fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes()); //will use angle of attack because its "easier"
            fg_fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

            //Other airplane data
            let fg_net_fdm_version = 24_u32;
            fg_fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

            //Convert struct to array of u8 bytes
            pckt.bytes = bincode::serialize(&fg_fdm).unwrap();

        }//end for
    }//end run
}//end system



//System to send packets
struct SendPacket;
impl<'a> System<'a> for SendPacket
{
    type SystemData = (
        ReadStorage<'a, DataFDM>,
        ReadStorage<'a, Packet>,
    );
    //i do not need to read in datafdm...
    fn run(&mut self, (datafdm, packet): Self::SystemData) 
    {
        for (_fdm, pckt) in (&datafdm, &packet).join() 
        {
           
            //Finally send &[u8] of bytes over socket connected on FlightGear
            SOCKET.send(&pckt.bytes).expect("couldn't send packet");

        }//end for
    }//end run
}//end system


//Set some global variables:
lazy_static!
{
    //define earth ellipsoid
    static ref ELLIPSOID: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);
    //create socket
    static ref SOCKET: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
}

static FRAME_RATE: f64 = 30.0;
static DT: f64 = 1.0 / FRAME_RATE; //seconds

fn main()
{
    //Create variable to keep track of time elapsed
    let mut current_time: f64 = 0.0;
    let mut current_frame_main: usize = 0;

    //Create world
    let mut world = World::new();

    //Create dispatcher of the systems
    let mut dispatcher = DispatcherBuilder::new()
    .with(FlightControl, "flightcontrol", &[])
    .with(EquationsOfMotion, "EOM", &[])
    .with(MakePacket, "makepacket", &[])
    .with(SendPacket, "sendpacket", &[])
    .build();
    dispatcher.setup(&mut world);

    //Create plane entity with components
    let _plane = world.create_entity()
    .with(DataFDM{
        ecef_vec: Vector3::new(904799.960942606, -5528914.45139109, 3038233.40847236),

        current_frame: 0,
        q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0], //will store ODE results
        airspeed: 0.0,
        delta_traveled: 0.0,

        bank: 0.0, //bank angle
        alpha: 0.0,//  angle of attack
        throttle: 0.0, //throttle percentage
        flap: 0.0,  //  flap deflection amount

        mass_properties: PerformanceData{
            wing_area: 16.2,             //  wing wetted area, m^2
            wing_span: 10.9,             //  wing span, m
            tail_area: 2.0,              //  tail wetted area, m^2
            cl_slope0: 0.0889,           //  slope of Cl-alpha curve
            cl0: 0.178,                 //  Cl value when alpha = 0
            cl_slope1: -0.1,             //  slope of post-stall Cl-alpha curve
            cl1: 3.2,                   //  intercept of post-stall Cl-alpha curve
            alpha_cl_max: 16.0,           //  alpha at Cl(max)
            cdp: 0.034,                 //  parasitic drag coefficient
            eff: 0.77,                  //  induced drag efficiency coefficient
            mass: 1114.0,               //  airplane mass, kg
            engine_power: 119310.0,      //  peak engine power, W
            engine_rps: 40.0,            //  engine turnover rate, rev/s
            prop_diameter: 1.905,        //  propeller diameter, m
            a: 1.83,                    //  propeller efficiency curve fit coefficient
            b:-1.32,                    //  propeller efficiency curve fit coefficient
        }

    })
    .with(KeyboardState{
        thrust_up: false,
        thrust_down: false,

        aoa_up: false,
        aoa_down: false,

        bank_right: false,
        bank_left: false,

        flaps_down: false,
        zero_flaps: false,
    })
    .with(Packet{
        ..Default::default()
    })
    .build();

    //Connect to the socket on FlightGear
    SOCKET.connect("127.0.0.1:5500").expect("connect function failed");


    //create time type with the desired DT in milliseconds
    let timestep = time::Duration::from_millis((DT * 1000.0) as u64);

    //Loop simulation
    loop 
    {
        //get current time
        let start = time::Instant::now();

        //increment time count
        current_time = current_time + DT;
        current_frame_main = current_frame_main + 1;
        println!("{}", "====================================================");
        println!("time: {}, frames: {}", current_time, current_frame_main);

        //process this frame
        dispatcher.dispatch(&world);
        world.maintain();

        //find difference in time elapsed this loop versus the timestep
        let sleep_time = timestep.checked_sub(time::Instant::now().duration_since(start));

        //sleep for extra time if calculation took less time than the DT time step
        if sleep_time != None 
        {
            thread::sleep(sleep_time.unwrap());
        }
    }

}