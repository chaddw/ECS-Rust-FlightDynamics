//imports for flight control function
#[macro_use]
extern crate crossterm;
use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, Clear, ClearType};
use std::io::{stdout, Write};


//********************************************
//  This structure defines the data required
//  to model a plane.
//********************************************
#[derive(Debug)]
struct Plane 
{

    numEqns: usize, //int
    s: f64,

  //  q: [f64; 6],
    q: Vec<f64>,
 
    bank: f64,
    alpha: f64,//  angle of attack
    throttle: f64,
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
    flap: String, //&'static str,        //  flap deflection amount (pointer in c)

    longitude: f64,
    airspeed: f64

}


//NET FDM CODE, which is now in get_fdm_and_send_packet method
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
//for converting to slice of u8
unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8]
{
    ::std::slice::from_raw_parts((p as *const T) as *const u8,::std::mem::size_of::<T>(),)
}



impl Plane
{

    //************************************************************
    //  This method solves for the plane motion using a
    //  4th-order Runge-Kutta solver
    //************************************************************
    fn planeRungeKutta4(&mut self, mut ds: f64)
    {

        // let mut q: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq1: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq2: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq3: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq4: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let mut q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        //retrieve value of dependent variable
        q = self.q.clone();

        // Compute the four Runge-Kutta steps, The return 
        // value of planeRightHandSide method is an array
        // of delta-q values for each of the four steps.

        self.planeRightHandSide( &mut q, &mut qcopy, &mut ds, 0.0, &mut dq1);
        self.planeRightHandSide( &mut q, &mut dq1,   &mut ds, 0.5, &mut dq2);
        self.planeRightHandSide( &mut q, &mut dq2,   &mut ds, 0.5, &mut dq3);
        self.planeRightHandSide( &mut q, &mut dq3,   &mut ds, 1.0, &mut dq4);

        //  Update the dependent and independent variable values
        //  at the new dependent variable location and store the
        //  values in the ODE object arrays.
        self.s = self.s + ds;

        for i in 0..self.numEqns
        {
            q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
            self.q[i] = q[i];
        }
    }


    //*************************************************************
    //  This method loads the right-hand sides for the plane ODEs
    //*************************************************************
    fn planeRightHandSide(&mut self, q: &mut Vec<f64>, deltaQ: &mut Vec<f64>, &mut ds: &mut f64, qScale:  f64, mut dq: &mut Vec<f64>)
    {

    //  q[0] = vx = dxdt
    //  q[1] = x
    //  q[2] = vy = dydt
    //  q[3] = y
    //  q[4] = vz = dzdt
    //  q[5] = z
        let mut newQ = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; //[f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 

        let yo = -1.0_f64;
        let pi = yo.acos();

        let G: f64 = -9.81;
    
        let mut cl: f64 = 0.0;

        let mut cosP: f64 = 0.0;   //  climb angle
        let mut sinP: f64= 0.0;   //  climb angle
        let mut cosT: f64 = 0.0;   //  heading angle
        let mut sinT: f64 = 0.0;   //  heading angle
       
        //  Convert bank angle from degrees to radians
        //  Angle of attack is not converted because the
        //  Cl-alpha curve is defined in terms of degrees.
        self.bank = self.bank.to_radians();

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
        let advanceRatio: f64 = vtotal / (self.engineRps * self.propDiameter);
        let thrust: f64 = self.throttle * factor * self.enginePower * (self.a + self.b * advanceRatio * advanceRatio) / (self.engineRps * self.propDiameter);

        //  Compute lift coefficient. The Cl curve is 
        //  modeled using two straight lines.
        if  self.alpha < self.alphaClMax
        {
            cl = self.clSlope0 * self.alpha + self.cl0;
        }
        else 
        {
            cl = self.clSlope1 * self.alpha + self.cl1;
        }

        //  Include effects of flaps and ground effects.
        //  Ground effects are present if the plane is
        //  within 5 meters of the ground.
        if self.flap == "20"
        {
            cl += 0.25;
        }
        if self.flap == "40"
        {
            cl += 0.5;
        }
        if z < 5.0
        {
            cl += 0.25;
        }

        //  Compute lift
        let lift: f64 = 0.5 * cl * density * vtotal * vtotal * self.wingArea;

        // //  Compute drag coefficient
        let aspectRatio: f64 = self.wingSpan * self.wingSpan / self.wingArea;
        let cd = self.cdp + cl * cl / (pi * aspectRatio * self.eff);
        
        // //  Compute drag force
        let drag: f64 = 0.5 * cd * density * vtotal * vtotal * self.wingArea;


        //  Define some shorthand convenience variables
        //  for use with the rotation matrix.
        //  Compute the sine and cosines of the climb angle,
        //  bank angle, and heading angle;
        let cosW: f64 = self.bank.cos(); 
        let sinW: f64 = self.bank.sin(); 

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
        Fz = Fz + self.mass * G;

        //  Since the plane can't sink into the ground, if the
        //  altitude is less than or equal to zero and the z-component
        //  of force is less than zero, set the z-force
        //  to be zero.
        if  z <= 0.0 && Fz <= 0.0  
        {
            Fz = 0.0;
        }

        //  Load the right-hand sides of the ODE's
        dq[0] = ds * (Fx / self.mass);
        dq[1] = ds * vx;
        dq[2] = ds * (Fy / self.mass);
        dq[3] = ds * vy;
        dq[4] = ds * (Fz / self.mass);
        dq[5] = ds * vz;
        // if dq[5] < 0.0
        // {
        //     dq[5] == 0;
        // }
    }




    //create and send the net fdm packet 
    fn get_fdm_data_and_send_packet(&mut self)
    {
        ///net fdm main
        let fg_net_fdm_version = 24_u32;
       // let millis = time::Duration::from_millis(1000); //time in between packet sends

        //airplane automatically points north...
        let latitude: f64 = 21.3252; //45.59823; //21.3188 put it slightly left of the runway, so increased latitude goes left of runway (latitude inc goes north), 
        //let longitude: f64 = -157.943;   //-120.69202; //increased longitude goes east, decreased longitude goes backwards on runway.


       // let altitude: f64 = self.altitude;            

        // let latitude: f64 = 21.3252;
        // let longitude: f64= -157.943;
        // let altitude: f64 = 10.0;


        let roll: f32 = 0.0;
        let pitch: f32 = 0.0;
        let yaw: f32 = 52.0;

        let visibility: f32 = 5000.0;

 
       // thread::sleep(millis); 


        //create fdm instance
        let mut fdm: FGNetFDM = Default::default();


        //convert to network byte order
        fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());
        fdm.latitude = f64::from_be_bytes((latitude.to_radians()).to_ne_bytes());

        //calculate the degrees of longitude traveled in time based on dt
       let longitude_to_add: f64 = self.airspeed * (dt/3600.0);
       self.longitude = self.longitude - longitude_to_add;
       fdm.longitude = f64::from_be_bytes((self.longitude.to_radians()).to_ne_bytes());
        // fdm.longitude = f64::from_be_bytes(longitude.to_radians().to_ne_bytes());

        fdm.altitude = f64::from_be_bytes(self.q[5].to_ne_bytes());

        let alpha_tmp = self.alpha as f32;
        fdm.alpha = f32::from_be_bytes(alpha_tmp.to_ne_bytes());

        //convert to network byte order
        fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
        fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes());
        fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

        //convert to network byte order
        fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());
        fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());
        fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());

        //should only do this once...
        //create socket and connect to flightgear
        let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
        socket.connect("127.0.0.1:5500").expect("connect function failed");

        //convert struct array of u8 of bytes
        let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };
        //println!("{:?}", bytes);

        //finally send &[u8] of bytes to flight gear
        socket.send(bytes).expect("couldn't send message");
    }




} //end impl







static dt: f64 = 0.002;
//initialize plane and solves for the plane motion with range-kutta
fn main()
{
    let mut x: f64 = 0.0;
    let mut z: f64 = 0.0;
    let mut v: f64 = 0.0;
    let mut time: f64 = 0.0;
   // let dt = 0.0001; //determines how many steps through time

    //create plane, set airplane data
    let mut plane = Plane {
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
    bank: 0.0,
    alpha: 4.0, 
    throttle: 1.0, 
    flap: String::from("0"),    //  Flap setting
    numEqns: 6, 
    s: 0.0,                     //  time
   // q: [0.0;6]
     q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],               //  vx, x, vy, y, vz, z
     longitude: -157.943,
     airspeed: 0.0

    };



  
    //accelerate the plane for 1 hour seconds
    while plane.s < 40.0
    {
        plane.planeRungeKutta4( dt);

        time = plane.s;
        x = plane.q[1];
        z = plane.q[5];
        v = (plane.q[0] * plane.q[0] + plane.q[2] * plane.q[2] + plane.q[4] * plane.q[4]).sqrt();
        plane.airspeed = v;



        //if im going x speed how many degrees of longitude did i pass in dt seconds. example for dt = 0.1. airspeed * .1/3600 hours = longitude degrees passed
        plane.get_fdm_data_and_send_packet();
        //println!("time = {}, x = {}, altitude = {}, airspeed = {} ", time, x, z, v);

        println!("time = {}", time);
        println!("x = {}", x);
        println!("altitude = {}", z);
        println!("airspeed = {}", v);
        println!("longitude = {}", plane.longitude);
        println!("{}", "--------------------------------------");
        //println!("{:#?}", plane);
    }

}




//Some references used for networking

    //converting to bytes
    //https://stackoverflow.com/questions/29445026/converting-number-primitives-i32-f64-etc-to-byte-representations

    //htonl function in rust
    //https://docs.rs/socket/0.0.7/socket/fn.htonl.html

    //struct padding in rust vs c++
    //https://rust-lang.github.io/unsafe-code-guidelines/layout/structs-and-tuples.html

    //sending struct as u8 slice
    //https://stackoverflow.com/questions/29307474/how-can-i-convert-a-buffer-of-a-slice-of-bytes-u8-to-an-integer
    //https://stackoverflow.com/questions/28127165/how-to-convert-struct-to-u8

    //how jsbsim fills in the data socket FGOutputFG.cpp
    //https://github.com/JSBSim-Team/jsbsim/blob/4d87ce79b0ee4b0542885ae78e51c5fe7d637dea/src/input_output/FGOutputFG.cpp






    //CONTROLS TO GO INTO IMPL

    

    //  //flight controls
    //  fn inc_thrust(&mut self)
    //  {
    //     self.thrustforce = self.thrustforce + d_thrust;
    //     if self.thrustforce > maxthrust
    //     {
    //         self.thrustforce = maxthrust;
    //     }
    //     //println!("{}","increase thrust");
    //  }
    //  fn dec_thrust(&mut self)
    //  {
    //     self.thrustforce = self.thrustforce - d_thrust;
    //     if self.thrustforce < 0.0
    //     {
    //         self.thrustforce = 0.0;
    //     }
    //    // println!("{}","decrease thrust");
    //  }

    //  fn left_rudder(&mut self)
    //  {
    //      self.element[6].f_incidence = 16.0;
    //     // println!("{}","left rudder");
    //  }
    //  fn right_rudder(&mut self)
    //  {
    //      self.element[6].f_incidence = -16.0;
    //     // println!("{}","right rudder");
    //  }
    //  fn zero_rudder(&mut self)
    //  {
    //      self.element[6].f_incidence = 0.0;
    //    //  println!("{}","zero rudder");
    //  }



    //  fn roll_left(&mut self)
    //  {
    //      self.element[0].i_flap = 1;
    //      self.element[3].i_flap = -1;
    //      //println!("{}","roll left");
    //  }
    //  fn roll_right(&mut self)
    //  {
    //      self.element[0].i_flap = -1;
    //      self.element[3].i_flap = 1;
    //    //  println!("{}","roll right");
    //  }
    //  fn zero_ailerons(&mut self)
    //  {
    //      self.element[0].i_flap = 0;
    //      self.element[3].i_flap = 0;
    //   //   println!("{}","zero ailerons");
    //  }



    //  fn pitch_up(&mut self)
    //  {
    //      self.element[4].i_flap = 1;
    //      self.element[5].i_flap = 1;
    //    //  println!("{}","pitch up");
    //  }
    //  fn pitch_down(&mut self)
    //  {
    //      self.element[4].i_flap = -1;
    //      self.element[5].i_flap = -1;
    //    //  println!("{}","pitch down");
    //  }
    //  fn zero_elevators(&mut self)
    //  {
    //      self.element[4].i_flap = 0;
    //      self.element[5].i_flap = 0;
    //     // println!("{}","zero elevators");
    //  }

     

    //  fn flaps_down(&mut self)
    //  {
    //      self.element[1].i_flap = -1;
    //      self.element[2].i_flap = -1;
    //      self.flaps = true;
    //    //  println!("{}","flaps down");
    //  }
    //  fn zero_flaps(&mut self)
    //  {
    //      self.element[1].i_flap = 0;
    //      self.element[2].i_flap = 0;
    //      self.flaps = false;
    //    //  println!("{}","zero flaps");
    //  }

    //  //use flight control input 
    //  fn flight_control(&mut self)
    //  {

    //     let mut stdout = stdout();
    //     //going into raw mode
    //     enable_raw_mode().unwrap();
    
    //     //clearing the screen, going to top left corner and printing welcoming message
    //     execute!(stdout, Clear(ClearType::All), cursor::MoveTo(0, 0), Print("Fly me! cntrl + q to quit")) .unwrap();
    //     //ctrl + q to exit, w = pitch down, s = pitch up, a = roll left, d = roll right, t = increase thrust, y = decrease thrust, z = yaw left, x = yaw right, landing flaps up = f, landing flaps down = g 
    
    
    //     let no_modifiers = KeyModifiers::empty();
    
    //     //key detection, this needs to happen asynchronously because more than 1 loop can be pressed at a time, also the simulation needs to continue synchronously
    //     loop 
    //     {
    //         self.zero_rudder();
    //         self.zero_ailerons();
    //         self.zero_elevators();
    //         //going to top left corner
    //         execute!(stdout, cursor::MoveTo(0, 0)).unwrap();

    //         //matching the key
    //         match read().unwrap() //like a switch statement
    //         {
                
    //             //pitch down
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('w'),
    //                 modifiers: no_modifiers,
    //             }) => self.pitch_down(),

    //             //pitch up
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('s'),
    //                 modifiers: no_modifiers,
    //             }) => self.pitch_up(),

    //             //roll left
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('a'),
    //                 modifiers: no_modifiers,
    //             }) => self.roll_left(),

    //             //roll right
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('d'),
    //                 modifiers: no_modifiers,
    //             }) => self.roll_right(),

    //             //increase thrust
    //                 Event::Key(KeyEvent {
    //                 code: KeyCode::Char('t'),
    //                 modifiers: no_modifiers,
    //             }) => self.inc_thrust(),              

    //             //decrease thrust
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('y'),
    //                 modifiers: no_modifiers,
    //             }) => self.dec_thrust(),

    //              //yaw left
    //              Event::Key(KeyEvent {
    //                 code: KeyCode::Char('z'),
    //                 modifiers: no_modifiers,
    //             }) => self.left_rudder(),    

    //             //yaw right
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('x'),
    //                 modifiers: no_modifiers,
    //             }) => self.right_rudder(),

    //              //landing flaps down
    //              Event::Key(KeyEvent {
    //                 code: KeyCode::Char('g'),
    //                 modifiers: no_modifiers,
    //             }) => self.flaps_down(), 
                
                
    //              //landing flaps up
    //              Event::Key(KeyEvent {
    //                 code: KeyCode::Char('f'),
    //                 modifiers: no_modifiers,
    //             }) => self.zero_flaps(),               


    //             //quit
    //             Event::Key(KeyEvent {
    //                 code: KeyCode::Char('q'),
    //                 modifiers: KeyModifiers::CONTROL,
    //             }) => break,

    //             _ => (),

               
    //         }

    //        // self.step_simulation(1.0);//this needs to be constantly called in real time, while key press is being listened to as well...

    //     }

    //     //disabling raw mode
    //     disable_raw_mode().unwrap();

    // }//https://stackoverflow.com/questions/60130532/detect-keydown-in-rust