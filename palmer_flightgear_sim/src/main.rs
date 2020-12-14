//This main file sets up the ECS architecture, creates the entity, and runs the main simulation loop

//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --timeofday=noon --native-fdm=socket,in,30,,5500,udp

//Cessna Skyhawk visual
//fgfs.exe --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --timeofday=noon --native-fdm=socket,in,30,,5500,udp

//Coordinate conversions
use coord_transforms::prelude::*;

//Specs
use specs::prelude::*;

//Networking
use std::net::UdpSocket;

//Ellipsoid global variable
#[macro_use]
extern crate lazy_static;

//Main loop
use std::{thread, time};

//Import Components and resources
mod data;
use crate::data::KeyboardState;
use crate::data::Packet;
use crate::data::DataFDM;
use crate::data::PerformanceData;
use crate::data::DeltaTime;

//Import Systems
mod equations_of_motion;
mod flight_control;
mod make_packet;
mod send_packet;


//Set some global variables:
//failed in trying to get these to be resources they do not implement default
//these could be reinitialized each time the systems runs to get rid of the global aspect
lazy_static!
{
    //define earth ellipsoid
    static ref ELLIPSOID: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);
    //create socket
    static ref SOCKET: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
}

fn main()
{
    //Create variable to keep track of time elapsed
    let mut current_time = 0.0;
    let mut current_frame_main: usize = 0;

    //Create world
    let mut world = World::new();
    
    //Register Components in the world
    world.register::<DataFDM>();
    world.register::<KeyboardState>();
    world.register::<Packet>();

    //Choose frame rate, which will calculate delta time
    let frame_rate: f64 = 30.0;
    let dt: f64 = 1.0 / frame_rate; //seconds

    //add dt as a specs resource
    world.insert(DeltaTime(dt)); 

   
    //Create dispatcher of the Systems
    let mut dispatcher = DispatcherBuilder::new()
    .with(flight_control::FlightControl, "flightcontrol", &[])
    .with(equations_of_motion::EquationsOfMotion, "EOM", &[])
    .with(make_packet::MakePacket, "makepacket", &[])
    .with(send_packet::SendPacket, "sendpacket", &[])
    .build();
    dispatcher.setup(&mut world);

    //Create airplane Entity with Components
    let _plane = world.create_entity()
    .with(DataFDM{
        ecef_vec: Vector3::new(904799.960942606, -5528914.45139109, 3038233.40847236),
        current_frame: 0,
        q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0], //will store ODE results
        airspeed: 0.0,
        delta_traveled: 0.0,
        bank: 0.0, //bank angle
        alpha: 0.0,//angle of attack
        throttle: 0.0, //throttle percentage
        flap: 0.0,  //flap deflection amount
        mass_properties: PerformanceData{
            wing_area: 16.2,            //  wing wetted area, m^2
            wing_span: 10.9,            //  wing span, m
            tail_area: 2.0,             //  tail wetted area, m^2
            cl_slope0: 0.0889,          //  slope of Cl-alpha curve
            cl0: 0.178,                 //  Cl value when alpha = 0
            cl_slope1: -0.1,            //  slope of post-stall Cl-alpha curve
            cl1: 3.2,                   //  intercept of post-stall Cl-alpha curve
            alpha_cl_max: 16.0,         //  alpha at Cl(max)
            cdp: 0.034,                 //  parasitic drag coefficient
            eff: 0.77,                  //  induced drag efficiency coefficient
            mass: 1114.0,               //  airplane mass, kg
            engine_power: 119310.0,     //  peak engine power, W
            engine_rps: 40.0,           //  engine turnover rate, rev/s
            prop_diameter: 1.905,       //  propeller diameter, m
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

    //Create time type with the dt in milliseconds
    let timestep = time::Duration::from_millis((dt * 1000.0) as u64);

    //Loop simulation
    loop 
    {
        //get current time
        let start = time::Instant::now();

        //increment time count
        current_time = current_time + dt;
        current_frame_main = current_frame_main + 1;
        println!("{}", "====================================================");
        println!("time (seconds): {}, frames: {}", current_time, current_frame_main);

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