
//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --vc=0 --timeofday=noon --native-fdm=socket,in,30,,5500,udp
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --vc=0 --timeofday=noon --native-fdm=socket,in,60,,5500,udp

//imports for flight control function
#[macro_use]
extern crate crossterm;
use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, Clear, ClearType};
use std::io::{stdout, Write};

//Specs
use specs::prelude::*;
use specs::Entities;

//coordinate conversions
extern crate coord_transforms;
use coord_transforms::prelude::*;

//networking
use std::net::UdpSocket;
use std::{thread, time};


//used for ellipsoid
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

//////Component Performance data of the airplane
#[derive(Debug)]
struct PerformanceData
{
    wingArea: f64,
    wingSpan: f64,
    tailArea: f64,
    clSlope0: f64,   // slope of Cl-alpha curve
    cl0: f64,         // intercept of Cl-alpha curve
    clSlope1: f64,    // post-stall slope of Cl-alpha curve
    cl1: f64,        // post-stall intercept of Cl-alpha curve
    alphaClMax: f64,  // alpha when Cl=Clmax
    cdp: f64,         // parasite drag coefficient
    eff: f64,         // induced drag efficiency coefficient
    mass: f64,
    enginePower: f64,
    engineRps: f64,   // revolutions per second
    propDiameter: f64,
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
    );


    fn run(&mut self, (performancedata, mut position, mut outputdata, mut inputdata): Self::SystemData) 
    {


        for (perfdata, pos, outdata, inpdata) in (&performancedata, &mut position, &mut outputdata, &mut inputdata).join() 
        {
            println!("{}", "inside EOM for loop");

            let mut q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

           
            //perfdata: PerformanceData, pos: Position, outdata: OutputData, inpdata: InputData
            //this closure is what was "planeRightHandSide"
            let mut a = |q: &mut Vec<f64>, deltaQ: &mut Vec<f64>, &ds: & f64, qScale: f64, mut dq: &mut Vec<f64>| 
            {

 
                     let mut newQ = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; //[f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 
                 
                     let yo = -1.0_f64;
                     let pi = yo.acos();
                     let G: f64 = -9.81;
                     let mut cl: f64 = 0.0;
                     let mut cosP: f64 = 0.0;   //  climb angle
                     let mut sinP: f64= 0.0;   //  climb angle
                     let mut cosT: f64 = 0.0;   //  heading angle
                     let mut sinT: f64 = 0.0;   //  heading angle
                     let mut bank: f64 = 0.0;
                 
                     //  Convert bank angle from degrees to radians
                     //  Angle of attack is not converted because the
                     //  Cl-alpha curve is defined in terms of degrees.
                     bank = inpdata.bank.to_radians();
                 
                     //  Compute the intermediate values of the 
                     //  dependent variables.
                     for i in 0..6
                     {
                         newQ[i] = q[i] + qScale * deltaQ[i]; 
                     }
                 
                     //  Assign convenenience variables to the intermediate 
                     //  values of the locations and velocities.
                     let vx: f64 = newQ[0];
                     let vy: f64 = newQ[2];
                     let vz: f64 = newQ[4];
                     let x: f64 = newQ[1];
                     let y: f64 = newQ[3];
                     let z: f64 = newQ[5];
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
                     let advanceRatio: f64 = vtotal / (perfdata.engineRps * perfdata.propDiameter);
                     let thrust: f64 = inpdata.throttle * factor * perfdata.enginePower * (perfdata.a + perfdata.b * advanceRatio * advanceRatio) / (perfdata.engineRps * perfdata.propDiameter);
                 
                     //  Compute lift coefficient. The Cl curve is 
                     //  modeled using two straight lines.
                     if  inpdata.alpha < perfdata.alphaClMax
                     {
                         cl = perfdata.clSlope0 * inpdata.alpha + perfdata.cl0;
                     }
                     else 
                     {
                         cl = perfdata.clSlope1 * inpdata.alpha + perfdata.cl1;
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
                     let lift: f64 = 0.5 * cl * density * vtotal * vtotal * perfdata.wingArea;
                 
                     // //  Compute drag coefficient
                     let aspectRatio: f64 = perfdata.wingSpan * perfdata.wingSpan / perfdata.wingArea;
                     let cd = perfdata.cdp + cl * cl / (pi * aspectRatio * perfdata.eff);
                     
                     // //  Compute drag force
                     let drag: f64 = 0.5 * cd * density * vtotal * vtotal * perfdata.wingArea;
                 
                     //  Define some shorthand convenience variables
                     //  for use with the rotation matrix.
                     //  Compute the sine and cosines of the climb angle,
                     //  bank angle, and heading angle;
                     let cosW: f64 = bank.cos(); 
                     let sinW: f64 = bank.sin(); 
                 
                     if  vtotal == 0.0
                     {
                         cosP = 1.0;
                         sinP = 0.0;
                     }
                     else
                     {
                         cosP = vh / vtotal;  
                         sinP = vz / vtotal;  
                     }
                     
                     if vh == 0.0
                     {
                         cosT = 1.0;
                         sinT = 0.0;
                     }
                     else
                     {
                         cosT = vx / vh;
                         sinT = vy / vh;
                     }
                 
                     //  Convert the thrust, drag, and lift forces into
                     //  x-, y-, and z-components using the rotation matrix.
                     let Fx: f64 = cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift;
                     let Fy: f64 = sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift;
                     let mut Fz: f64 = sinP * (thrust - drag) + cosP * cosW * lift;
                 
                     //  Add the gravity force to the z-direction force.
                     Fz = Fz + perfdata.mass * G;
                 
                     //  Since the plane can't sink into the ground, if the
                     //  altitude is less than or equal to zero and the z-component
                     //  of force is less than zero, set the z-force
                     //  to be zero.
                     if  z <= 0.0 && Fz <= 0.0  
                     {
                         Fz = 0.0;
                     }
                 
                     //  Load the right-hand sides of the ODE's
                     dq[0] = ds * (Fx / perfdata.mass);
                     dq[1] = ds * vx;
                     dq[2] = ds * (Fy / perfdata.mass);
                     dq[3] = ds * vy;
                     dq[4] = ds * (Fz / perfdata.mass);
                     dq[5] = ds * vz;
                
 
 
            };



            //begin what was "rangeKutta4" method
            let priorx = outdata.q[1]; //will be used to calculate delta_traveled

            // let mut q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            // let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            // let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

            //retrieve value of dependent variable
            q = outdata.q.clone();

            //get the static time variable dt
            let ds = dt;

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
    
            outdata.delta_traveled = ((outdata.q[1] / 3.6) - (priorx / 3.6)); //get the change in meters from last frame to this frame, will be used to calculate new latitude based on how far we've gone
            pos.ecef_vec.x = pos.ecef_vec.x + outdata.delta_traveled; //add latitude change to the ecef longitude


            outdata.airspeed = (outdata.q[0] * outdata.q[0] + outdata.q[2] * outdata.q[2] + outdata.q[4] * outdata.q[4]).sqrt();


        }//end for
    }//end run
}//end system













// //System to send packets
// struct SendPacket;
// impl<'a> System<'a> for SendPacket
// {
//     type SystemData = (
//         ReadStorage<'a, Position>,
//         ReadStorage<'a, OutputData>,
//         WriteStorage<'a, FGNetFDM>,
//     );

//     fn run(&mut self, (position, outdata, mut fgnetfdm): Self::SystemData) 
//     {
//         for (pos, outdata, netfdm) in (&pposition, &outdata, &mut fgnetfdm).join() 
//         {

//             //create and send the net fdm packet 
//            // fn get_fdm_data_and_send_packet(socket: &UdpSocket)
//             {
//                 //ktts (shuttle landing facility) geo coordinates 28.6327 -80.706, 0.0
//                 let visibility: f32 = 5000.0;
//                 let fg_net_fdm_version = 24_u32;

//                 let roll: f32 = 0.0; //no roll in 2D
//                 let mut pitch: f32 = 0.0; //will use angle of attack because its "easier"
//                 let yaw: f32 = 90.0; //we only need to face in one direction

//                 //create fdm instance
//                 let mut fdm: FGNetFDM = Default::default();

//                 //convert to network byte order
//                 fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

//                 //coordinate conversion
//                 let lla = geo::ecef2lla(&self.ecef_vec, &ELLIPSOID); //make new geo coords

//                 fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
//                 fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
//                 fdm.altitude = f64::from_be_bytes(self.q[5].to_ne_bytes()); // we can just use the value the model operates on

//                 //convert to network byte order
//                 pitch = self.alpha as f32;
//                 fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
//                 fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes()); //will use angle of attack because its "easier"
//                 fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

//                 //convert to network byte order
//                 fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());
//                 fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());
//                 fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());
//                 fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());
//                 fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());

//                 //convert struct array of u8 of bytes
//                 let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };
//                 //println!("{:?}", bytes);

//                 //finally send &[u8] of bytes to flight gear
//                 socket.send(bytes).expect("couldn't send message");
//             }

//         }//end for
//     }//end run
// }//end system


//System to handle user input
struct FlightControl;
impl<'a> System<'a> for FlightControl
{
    type SystemData = (
        WriteStorage<'a, InputData>, 
        ReadStorage<'a, OutputData>,
    );

    fn run(&mut self, (mut inputdata, outputdata): Self::SystemData) 
    {
        for (inpdata, outdata) in (&mut inputdata, &outputdata).join()
        {
            println!("{}", "inside flight control");
            //input flight control closures to define the airplane behavior
            // let mut inc_thrust = || 
            // {
            //     inpdata.throttle = inpdata.throttle + 0.05; //increased throttle by 5%
            //     if inpdata.throttle > 1.0
            //     {
            //         inpdata.throttle = 1.0;
            //     }
            // };

            // let mut dec_thrust = || 
            // {
            //     inpdata.throttle = inpdata.throttle - 0.05; //dec throttle by 5%
            //     if inpdata.throttle < 0.0
            //     {
            //         inpdata.throttle = 0.0;
            //     }
            // };
        
            // let mut inc_aoa = || 
            // {
            //     inpdata.alpha = inpdata.alpha + 1.0; //increased angle of attack by 1 degree
            //     if inpdata.alpha > 20.0
            //     {
            //         inpdata.alpha = 20.0;
            //     }
            // };
        
            // let mut dec_aoa = || 
            // {
            //     inpdata.alpha = inpdata.alpha - 1.0; //decreased angle of attack by 1 degree
            //     if inpdata.alpha < -16.0
            //     {
            //         inpdata.alpha = -16.0
            //     }
            // };
           

               //going into raw mode
               enable_raw_mode().unwrap();
       
               let no_modifiers = KeyModifiers::empty();
           
               //key detection, this needs to happen asynchronously because more than 1 loop can be pressed at a time, also the simulation needs to continue synchronously
               //loop 
               //{
                   //matching the key pressed
                   match read().unwrap() //like a switch statement
                   {
                       //increase thrust
                           Event::Key(KeyEvent {
                           code: KeyCode::Char('t'),
                           modifiers: no_modifiers,
                       }) => (if inpdata.throttle < 1.0 
                                {
                                    inpdata.throttle = inpdata.throttle + 0.05
                                }),     //inc throttle by 5%         
       
                       //decrease thrust
                       Event::Key(KeyEvent {
                           code: KeyCode::Char('g'),
                           modifiers: no_modifiers,
                       }) => (if inpdata.throttle > 0.0 
                                {
                                    inpdata.throttle = inpdata.throttle - 0.05
                                }),     //dec throttle by 5%
       
                       //increase angle of attack
                       Event::Key(KeyEvent {
                           code: KeyCode::Char('y'),
                           modifiers: no_modifiers,
                       }) =>  (if inpdata.alpha < 20.0
                                {
                                    inpdata.alpha = inpdata.alpha + 1.0            //increased angle of attack by 1 degree
                                }),

                       //increase angle of attack
                       Event::Key(KeyEvent {
                           code: KeyCode::Char('h'),
                           modifiers: no_modifiers,
                       }) => (if inpdata.alpha > -16.0
                                {
                                    inpdata.alpha = inpdata.alpha - 1.0             //decreased angle of attack by 1 degree
                                }),

                       //quit
                       Event::Key(KeyEvent {
                           code: KeyCode::Char('q'),
                           modifiers: KeyModifiers::CONTROL,
                       }) => println!("{}", "you cant quit now!"), //need a way to quit gracefully
       
                       _ => (),
       
                   }
               //}

       
               //disabling raw mode
               disable_raw_mode().unwrap();
       
           //https://stackoverflow.com/questions/60130532/detect-keydown-in-rust
           

        }//end for
    }//end run
}//end system





static dt: f64 = 0.5; //time in between each frame
lazy_static!
{
    static ref ELLIPSOID: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);
}
//initialize plane and solves for the plane motion with range-kutta
fn main()
{

    let mut world = World::new();
    let mut dispatcher = DispatcherBuilder::new()
    .with(EquationsOfMotion, "Equations_Of_Motion", &[])
    .with(FlightControl, "Flight_control",&[])
    .build();

    dispatcher.setup(&mut world);

    //create plane entity with components
    let plane = world.create_entity()
    .with(Position{
        ecef_vec: Vector3::new(904799.960942606, -5528914.45139109, 3038233.40847236)}) //location of runway at 0 height
    .with(PerformanceData{
        wingArea: 16.2,             //  wing wetted area, m^2
        wingSpan: 10.9,             //  wing span, m
        tailArea: 2.0,              //  tail wetted area, m^2
        clSlope0: 0.0889,           //  slope of Cl-alpha curve
        cl0: 0.178,                 //  Cl value when alpha = 0
        clSlope1: -0.1,             //  slope of post-stall Cl-alpha curve
        cl1: 3.2,                   //  intercept of post-stall Cl-alpha curve
        alphaClMax: 16.0,           //  alpha at Cl(max)
        cdp: 0.034,                 //  parasitic drag coefficient
        eff: 0.77,                  //  induced drag efficiency coefficient
        mass: 1114.0,               //  airplane mass, kg
        enginePower: 119310.0,      //  peak engine power, W
        engineRps: 40.0,            //  engine turnover rate, rev/s
        propDiameter: 1.905,        //  propeller diameter, m
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

    .build();


    let runtime = time::Duration::from_secs(1);
    loop 
    {
        let start = time::Instant::now();
        dispatcher.dispatch(&world);
        world.maintain();

        // Create frame_rate loop
        let sleep_time = runtime.checked_sub(time::Instant::now().duration_since(start));
        if sleep_time != None 
        {
            thread::sleep(sleep_time.unwrap());
        }
        //println!("{:#?}", plane)
    }

    // //set some variables for conveinience 
    // let mut x: f64 = 0.0;
    // let mut y: f64 = 0.0;
    // let mut z: f64 = 0.0;
    // let mut v: f64 = 0.0;
    // let mut time: f64 = 0.0;

  
    // //create socket and connect to flightgear
    // let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
    // socket.connect("127.0.0.1:5500").expect("connect function failed");
  
    // //acclerate for time in seconds
    // while plane.s < 60.0 
    // {
    //     plane.planeRungeKutta4( dt); //step simulation

    //     //get and send packet data
    //     plane.get_fdm_data_and_send_packet(&socket);

    //     //set some variables for conveinience
    //     time = plane.s;
    //     x = plane.q[1];
    //     y = plane.q[3];
    //     z = plane.q[5];
    //     v = (plane.q[0] * plane.q[0] + plane.q[2] * plane.q[2] + plane.q[4] * plane.q[4]).sqrt();
    //     plane.airspeed = v;

    //     //print out some relevant info
    //     println!("time = {}", time);
    //     println!("x traveled (m) = {}", x / 3.6); //convert to meters
    //     println!("x travel change (m) since last frame = {}", plane.delta_traveled);
    //     println!("y = {}", y);
    //     println!("altitude (m) = {}", z);
    //     println!("airspeed (km/h) = {}", v);
    //     println!("throttle % = {}", plane.throttle);
    //     println!("angle of attack (deg) = {}", plane.alpha);
    //     println!("bank angle (deg) = {}", plane.bank);
    //     println!("{}", "--------------------------------------");
    //     //println!("{:#?}", plane);

    //     //get user input and call flight control functions
    //     //plane.flight_control();
    // }

}