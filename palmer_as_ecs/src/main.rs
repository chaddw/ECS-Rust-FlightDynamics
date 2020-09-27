#![recursion_limit="256"]
//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --vc=0 --timeofday=noon --native-fdm=socket,in,30,,5500,udp
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --vc=0 --timeofday=noon --native-fdm=socket,in,60,,5500,udp

//Imports for flight control function
extern crate crossterm;

//Async std crossterm
use std::{
    io::{stdout, Write},
    time::Duration,
};

use futures::{future::FutureExt, select, StreamExt};
use futures_timer::Delay;

use crossterm::{
    event::{Event, EventStream, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode},
};
//Crossterm output priniting
use crossterm::cursor;
use crossterm::terminal::{ Clear, ClearType};


//Specs
use specs::prelude::*;

//Coordinate conversions
extern crate coord_transforms;
use coord_transforms::prelude::*;

//Networking
use std::net::UdpSocket;

//Ellipsoid global variable
#[macro_use]
extern crate lazy_static;


//////Component Position
#[derive(Debug)]
struct Position
{
    ecef_vec: Vector3<f64>
}
impl Component for Position 
{
    type Storage = VecStorage<Self>;
}

//////Component State Machine for keyboard
#[derive(Debug)]
struct KeyboardState
{
    thrust_up: bool,
    thrust_down: bool,
    aoa_up: bool,
    aoa_down: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}


//////Component Performance data of the airplane
#[derive(Debug)]
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
impl Component for PerformanceData
{
    type Storage = VecStorage<Self>;
}

//////Component output results data
#[derive(Debug)]
struct OutputData
{
    s: f64, //time in seconds
    q: Vec<f64>, //will store ODE results
    airspeed: f64,
    delta_traveled: f64,
}
impl Component for OutputData
{
    type Storage = VecStorage<Self>;
}

//////Component user input data
#[derive(Debug)]
struct InputData
{
    bank: f64, //bank angle
    alpha: f64,//  angle of attack
    throttle: f64, //throttle percentage
    flap: String,  //  flap deflection amount (pointer in c)

}
impl Component for InputData
{
    type Storage = VecStorage<Self>;
}


//////Component FGNetFDM for networking
#[derive(Debug, Default)]
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
impl Component for FGNetFDM
{
    type Storage = VecStorage<Self>;
}
//for converting to slice of u8 
unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8]
{
    ::std::slice::from_raw_parts((p as *const T) as *const u8,::std::mem::size_of::<T>(),)
}


//System to perform physics calculations
struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        ReadStorage<'a, PerformanceData>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, OutputData>,
        WriteStorage<'a, InputData>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (performancedata, mut position, mut outputdata, mut inputdata, keyboardstate): Self::SystemData) 
    {
        for (perfdata, pos, outdata, inpdata, keystate) in (&performancedata, &mut position, &mut outputdata, &mut inputdata, &keyboardstate).join() 
        {
           // println!("{}", "inside eom");

            let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


            //Handle the input states
            //Thrust states
            if inpdata.throttle < 1.0 && keystate.thrust_up == true
            {
                inpdata.throttle = inpdata.throttle + 0.05;
                
            }   
            else if inpdata.throttle > 0.0 && keystate.thrust_down == true
            {
                inpdata.throttle = inpdata.throttle - 0.05;
                if inpdata.throttle < 0.001 
                {
                    inpdata.throttle = 0.0;
                }
            }  
            //Angle of attack states
            if inpdata.alpha < 20.0 && keystate.aoa_up == true
            {
                inpdata.alpha = inpdata.alpha + 1.0;
            
            }  
            else if inpdata.alpha > -16.0 && keystate.aoa_down == true
            {
                inpdata.alpha = inpdata.alpha - 1.0
            }  
        
  
            //note:this closure is what was "planeRightHandSide"... might should make this a function outside the system which is called in the system. could pass in the component structs
            let a = |q: &mut Vec<f64>, delta_q: &mut Vec<f64>, &ds: & f64, q_scale: f64, dq: &mut Vec<f64>| 
            {
                let mut new_q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 
            
                let yo = -1.0_f64;
                let pi = yo.acos();
                let g: f64 = -9.81;
                let mut cl: f64;
                let cos_p: f64;   //  climb angle
                let sin_p: f64;   //  climb angle
                let cos_t: f64;   //  heading angle
                let sin_t: f64;   //  heading angle
            
                //  Convert bank angle from degrees to radians
                //  Angle of attack is not converted because the
                //  Cl-alpha curve is defined in terms of degrees.
                let bank = inpdata.bank.to_radians();
            
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
                let advance_ratio: f64 = vtotal / (perfdata.engine_rps * perfdata.prop_diameter);
                let thrust: f64 = inpdata.throttle * factor * perfdata.engine_power * (perfdata.a + perfdata.b * advance_ratio * advance_ratio) / (perfdata.engine_rps * perfdata.prop_diameter);
            
                //  Compute lift coefficient. The Cl curve is 
                //  modeled using two straight lines.
                if  inpdata.alpha < perfdata.alpha_cl_max
                {
                    cl = perfdata.cl_slope0 * inpdata.alpha + perfdata.cl0;
                }
                else 
                {
                    cl = perfdata.cl_slope1 * inpdata.alpha + perfdata.cl1;
                }
            
                //  Include effects of flaps and ground effects.
                //  Ground effects are present if the plane is
                //  within 5 meters of the ground.
                if inpdata.flap == "20"
                {
                    cl += 0.25;
                }
                if inpdata.flap == "40"
                {
                    cl += 0.5;
                }
                if z < 5.0
                {
                    cl += 0.25;
                }
            
                //  Compute lift
                let lift: f64 = 0.5 * cl * density * vtotal * vtotal * perfdata.wing_area;
            
                // //  Compute drag coefficient
                let aspect_ratio: f64 = perfdata.wing_span * perfdata.wing_span / perfdata.wing_area;
                let cd = perfdata.cdp + cl * cl / (pi * aspect_ratio * perfdata.eff);
                
                // //  Compute drag force
                let drag: f64 = 0.5 * cd * density * vtotal * vtotal * perfdata.wing_area;
            
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
                fz = fz + perfdata.mass * g;
            
                //  Since the plane can't sink into the ground, if the
                //  altitude is less than or equal to zero and the z-component
                //  of force is less than zero, set the z-force
                //  to be zero.
                if  z <= 0.0 && fz <= 0.0  
                {
                    fz = 0.0;
                }
            
                //  Load the right-hand sides of the ODE's
                dq[0] = ds * (fx / perfdata.mass);
                dq[1] = ds * vx;
                dq[2] = ds * (fy / perfdata.mass);
                dq[3] = ds * vy;
                dq[4] = ds * (fz / perfdata.mass);
                dq[5] = ds * vz;

            }; //End closure


            //Begin what was "rangeKutta4" method
            let priorx = outdata.q[1]; //Will be used to calculate delta_traveled

            //Retrieve value of dependent variable
            let mut q = outdata.q.clone();

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
            outdata.s = outdata.s + ds;
        
            for i in 0..6
            {
                q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
                outdata.q[i] = q[i];
            }
    
            outdata.delta_traveled = (outdata.q[1] / 3.6) - (priorx / 3.6); //get the change in meters from last frame to this frame, will be used to calculate new latitude based on how far we've gone
            pos.ecef_vec.x = pos.ecef_vec.x + outdata.delta_traveled; //add latitude change to the ecef longitude
            
            outdata.airspeed = (outdata.q[0] * outdata.q[0] + outdata.q[2] * outdata.q[2] + outdata.q[4] * outdata.q[4]).sqrt();

        }//end for
    }//end run
}//end system



//System to send packets
struct SendPacket;
impl<'a> System<'a> for SendPacket
{
    type SystemData = (
        ReadStorage<'a, Position>,
        ReadStorage<'a, OutputData>,
        ReadStorage<'a, FGNetFDM>,
        ReadStorage<'a, InputData>,
    );

    fn run(&mut self, (position, outdata, fgnetfdm, inputdata): Self::SystemData) 
    {
        for (pos, outdata, _netfdm, inpdata) in (&position, &outdata, &fgnetfdm, &inputdata).join() 
        {
            let visibility: f32 = 5000.0;
            let fg_net_fdm_version = 24_u32;

            let roll: f32 = 0.0; //No roll in 2D
            let pitch: f32 = inpdata.alpha as f32; //Will use angle of attack because its "easier"
            let yaw: f32 = 90.0; //Only need to face in one direction

            //create fdm instance
            let mut fdm: FGNetFDM = Default::default();

            //convert to network byte order
            fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

            //coordinate conversion
            let lla = geo::ecef2lla(&pos.ecef_vec, &ELLIPSOID); //make new geo coords

            fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
            fdm.altitude = f64::from_be_bytes(outdata.q[5].to_ne_bytes()); // we can just use the value the model operates on (try lla.z tho)

            //convert to network byte order
            fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
            fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes()); //will use angle of attack because its "easier"
            fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

            //convert to network byte order
            fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());
            fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());

            //convert struct array of u8 of bytes
            let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };

            //finally send &[u8] of bytes to flight gear
            //Connect first (would be nice to only do this once...)
            SOCKET.connect("127.0.0.1:5500").expect("connect function failed");
            //Send!
            SOCKET.send(bytes).expect("couldn't send message");


            //Print some relevant data
            disable_raw_mode().unwrap(); //Get out of raw mode to print clearly
            println!("time = {}", outdata.s);
            println!("x traveled (m) = {}", outdata.q[1] / 3.6); //converted to meters
            println!("altitude (m) = {}", outdata.q[5]);
            println!("airspeed (km/h) = {}", outdata.airspeed);
            println!("throttle % = {}", inpdata.throttle);
            println!("angle of attack (deg) = {}", inpdata.alpha);
            println!("x travel change (m) since last frame = {}", outdata.delta_traveled);
            //println!("bank angle (deg) = {}", inpdata.bank);
            //println!("y = {}", outdata.q[3]);
            enable_raw_mode().unwrap(); //Return to raw
  

        }//end for
    }//end run
}//end system


async fn handle_input(thrust_up: &mut bool, thrust_down: &mut bool, aoa_up: &mut bool, aoa_down: &mut bool) 
{
    let mut reader = EventStream::new();
    let mut delay = Delay::new(Duration::from_millis(50)).fuse(); //Delay time (this greatly affects the speed of the simulation...)
    let mut event = reader.next().fuse();

    //Either the time delay or keyboard event happens and then this function will be called again in the system
    select!
    {
        _ = delay => 
        { 
            return; //Exit if time expires
        }, 

        maybe_event = event =>
        {
            match maybe_event 
            {
                Some(Ok(event)) => 
                {
                    println!("Event::{:?}\r", event);

                    //Thrust
                    if event == Event::Key(KeyCode::Char('t').into()) 
                    {
                        *thrust_up = true;
                    }
                    else  if event == Event::Key(KeyCode::Char('g').into()) 
                    {
                        *thrust_down = true;
                    }

                    //Angle of attack
                    else  if event == Event::Key(KeyCode::Char('y').into()) 
                    {
                        *aoa_up = true;
                    }
                    else  if event == Event::Key(KeyCode::Char('h').into()) 
                    {
                        *aoa_down = true;
                    }
                }
                Some(Err(e)) => println!("Error: {:?}\r", e),

                None => return,
            }
        },
    };
}

//System to handle user input
struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = ( 
        ReadStorage<'a, InputData>, 
        WriteStorage<'a, KeyboardState>,
    );

    fn run(&mut self, (inputdata, mut keyboardstate): Self::SystemData) 
    {
        for (_inpdata, keystate) in (&inputdata, &mut keyboardstate).join() 
        {
            //Set all states false before we know if they are being activated
            keystate.thrust_up = false; 
            keystate.thrust_down = false;
            keystate.aoa_up = false;
            keystate.aoa_down = false;

            enable_raw_mode().unwrap();

            let mut stdout = stdout();

            //This will make output not as crazy
            execute!(stdout, Clear(ClearType::All), cursor::MoveTo(0, 0)) .unwrap();
           
            //Handle flight control
            async_std::task::block_on(handle_input(&mut keystate.thrust_up, &mut keystate.thrust_down,&mut keystate.aoa_up, &mut keystate.aoa_down ));
        
            disable_raw_mode().unwrap();

        }//end for
    }//end run
}//end system



//Time in between each eom calculation
static DT: f64 = 0.5; //0.0167 

//Macro to define other globals
lazy_static!
{
    //define earth ellipsoid
    static ref ELLIPSOID: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);
    //create socket
    static ref SOCKET: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
}


fn main()
{
    //Create world
    let mut world = World::new();
    world.register::<Position>();
    world.register::<PerformanceData>();
    world.register::<OutputData>();
    world.register::<InputData>();
    world.register::<FGNetFDM>();
    world.register::<KeyboardState>();

    //Create dispatcher of the systems
    let mut dispatcher = DispatcherBuilder::new()
    .with(FlightControl, "flightcontrol", &[])
    .with(EquationsOfMotion, "EOM", &[])
    .with(SendPacket, "sendpacket", &[])
    .build();

    dispatcher.setup(&mut world);

    //Create plane entity with components
    let _plane = world.create_entity()
    .with(Position{
        ecef_vec: Vector3::new(904799.960942606, -5528914.45139109, 3038233.40847236)}) //location of runway at 0 height
    .with(PerformanceData{
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
        })
    .with(OutputData{
        s: 0.0, //time in seconds
        q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0], //will store ODE results
        airspeed: 0.0,
        delta_traveled: 0.0,
        })
    .with(InputData{
        bank: 0.0, //bank angle
        alpha: 0.0,//  angle of attack
        throttle: 0.0, //throttle percentage
        flap: String::from("0"),  //  flap deflection amount (pointer in c)
        })
    .with(FGNetFDM{
        ..Default::default()
        })
    .with(KeyboardState{
        thrust_up: false,
        thrust_down: false,
        aoa_up: false,
        aoa_down: false,
    })
    .build();

    //Loop simulation
    loop 
    {
        dispatcher.dispatch(&world);
        world.maintain();
    }

}