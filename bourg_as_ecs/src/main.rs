#![recursion_limit="512"]
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
//Crossterm output printing
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

//Exit program
use std::process;

//quaternion and vector matrix operations
extern crate nalgebra as na;
use na::{Matrix3, Vector3, UnitQuaternion, Unit, Quaternion}; //took off unit


// //////Component Position
// #[derive(Debug)]
// struct Position
// {
//     ecef_vec: Vector3<f64>
// }
// impl Component for Position 
// {
//     type Storage = VecStorage<Self>;
// }

//////Component State Machine for keyboard
#[derive(Debug)]
struct KeyboardState
{
    thrust_up: bool,
    thrust_down: bool,

    left_rudder: bool,
    right_rudder: bool,
    zero_rudder: bool,

    roll_left: bool,
    roll_right: bool,
    zero_ailerons: bool,

    pitch_up: bool,
    pitch_down: bool,
    zero_elevators: bool,

    flaps_down: bool,
    zero_flaps: bool,

}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}



//the point masses are elements making up the bodystructure 
#[derive(Debug)]
struct PointMass
{
    f_mass: f32,
    v_d_coords: Vector3<f32>, //"design position"
    v_local_inertia: Vector3<f32>,
    f_incidence: f32,
    f_dihedral: f32,
    f_area: f32,
    i_flap: i32,
    v_normal: Vector3<f32>,
    v_cg_coords: Vector3<f32> //"corrected position"

}

//////Component 
#[derive(Debug, Default)]
struct RigidBody
{
    mass: f32, //total mass
    m_inertia: Matrix3<f32>,
    m_inertia_inverse: Matrix3<f32>,
    v_position: Vector3<f32>,          // position in earth coordinates
    v_velocity: Vector3<f32>,          // velocity in earth coordinates
    v_velocity_body: Vector3<f32>,      // velocity in body coordinates
    v_angular_velocity: Vector3<f32>,   // angular velocity in body coordinates
    v_euler_angles: Vector3<f32>,       // Euler angles in body coordinates
    f_speed: f32,                // speed (magnitude of the velocity)
    stalling: bool,
    flaps: bool,
    q_orientation: Quaternion<f32>,
    q_orientation_unit: UnitQuaternion<f32>,   // orientation in earth coordinates
    v_forces: Vector3<f32>,            // total force on body
    thrustforce: f32,           //magnitude of thrust
    v_moments: Vector3<f32>,          // total moment (torque) on body

    element: Vec<PointMass> //vector of point mass elements
}
impl Component for RigidBody
{
    type Storage = VecStorage<Self>;
}

// //////Component output results data
// #[derive(Debug)]
// struct OutputData
// {
//     s: f64, //time in seconds
//     q: Vec<f64>, //will store ODE results
//     airspeed: f64,
//     delta_traveled: f64,
// }
// impl Component for OutputData
// {
//     type Storage = VecStorage<Self>;
// }

// //////Component user input data
// #[derive(Debug)]
// struct InputData
// {
//     bank: f64, //bank angle
//     alpha: f64,//  angle of attack
//     throttle: f64, //throttle percentage
//     flap: String,  //  flap deflection amount (pointer in c)

// }
// impl Component for InputData
// {
//     type Storage = VecStorage<Self>;
// }


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





//MASS PROP ONLY CALLED ONCE AT BEGGINING
fn calc_airplane_mass_properties(rigidbod: &mut RigidBody)
{
    println!("{}", "calculating mass properties...");
    let mut inn: f32;
    let mut di: f32;

    //calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
    for  i in rigidbod.element.iter_mut()
    {
        inn = (i.f_incidence).to_radians();
        di = (i.f_dihedral).to_radians();
        i.v_normal = Vector3::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
        i.v_normal = i.v_normal.normalize();
    }

    //calculate total mass
    let mut total_mass: f32 = 0.0;
    for i in rigidbod.element.iter()
    {
        total_mass = total_mass + i.f_mass;
    }
    //println!("Total mass: {}", total_mass);

    //calculate combined center of gravity location
    let mut first_moment_x: f32 = 0.0;
    let mut first_moment_y: f32 = 0.0;
    let mut first_moment_z: f32 = 0.0;
    for i in rigidbod.element.iter()
    {
        //X coord
        first_moment_x = first_moment_x + i.f_mass * i.v_d_coords.x;
        //y coord
        first_moment_y = first_moment_y + i.f_mass * i.v_d_coords.y;
        //z coord
        first_moment_z = first_moment_z + i.f_mass * i.v_d_coords.z;
        // vMoment = vMoment * i.f_mass * i.v_d_coords; //vector multiplcation not set up properly... oh well. even with nalgebra it panics
    }
    let v_moment = Vector3::new(first_moment_x, first_moment_y, first_moment_z); //remember there is a v_moments in rigid body.. we'll see how this plays out
    let cg = v_moment / total_mass; //operator overload works! Vector / scalar
    //println!("Combined center of gravity {:?}", cg);

    //calculate coordinates of each element with respect to the combined CG, relative position
    for i in rigidbod.element.iter_mut()
    {
        i.v_cg_coords.x = i.v_d_coords.x - cg.x;
        i.v_cg_coords.y = i.v_d_coords.y - cg.y;
        i.v_cg_coords.z = i.v_d_coords.z - cg.z;
    }

    //calculate the moments and products of intertia for the combined elements
    let mut ixx: f32 = 0.0;
    let mut iyy: f32 = 0.0;
    let mut izz: f32 = 0.0;
    let mut ixy: f32 = 0.0;
    let mut ixz: f32 = 0.0;
    let mut iyz: f32 = 0.0;

    for i in rigidbod.element.iter()
    {
        ixx = ixx + i.v_local_inertia.x + i.f_mass *
            (i.v_cg_coords.y * i.v_cg_coords.y +
            i.v_cg_coords.z * i.v_cg_coords.z);

        iyy = iyy + i.v_local_inertia.y + i.f_mass *
            (i.v_cg_coords.z * i.v_cg_coords.z +
            i.v_cg_coords.x * i.v_cg_coords.x);

        izz = izz + i.v_local_inertia.z + i.f_mass *
            (i.v_cg_coords.x * i.v_cg_coords.x +
            i.v_cg_coords.y * i.v_cg_coords.y);

        ixy = ixy + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.y);

        ixz = ixz + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.z);
        
        iyz = iyz + i.f_mass * (i.v_cg_coords.y *
            i.v_cg_coords.z);
    }

    //finally, set up airplanes mass and inertia matrix
    rigidbod.mass = total_mass;

    //using nalgebra matrix
    rigidbod.m_inertia = Matrix3::new(ixx, -ixy, -ixz,
                                 -ixy, iyy, -iyz,
                                 -ixz, -iyz, izz);

    //get inverse of matrix
    rigidbod.m_inertia_inverse = rigidbod.m_inertia.try_inverse().unwrap();

    
}









//FORCES
    //calculates all of the forces and moments on the plane at any time
    fn calc_airplane_loads(rigidbod: &mut RigidBody)
    {
        //println!("{}", "calculating forces...");

        let mut fb = Vector3::new(0.0, 0.0, 0.0);
        let mut mb = Vector3::new(0.0, 0.0, 0.0);

        //reset forces and moments
        rigidbod.v_forces = Vector3::new(0.0, 0.0, 0.0);
        rigidbod.v_moments = Vector3::new(0.0, 0.0, 0.0);

        //define thrust vector, which acts throguh the plane's center of gravity
        let mut thrust = Vector3::new(1.0, 0.0, 0.0);
        thrust = thrust * rigidbod.thrustforce; 
  
        //calculate forces and moments in body space
        let mut v_local_velocity = Vector3::new(0.0, 0.0, 0.0);
        let mut f_local_speed: f32 = 0.0;
        let mut v_drag_vector = Vector3::new(0.0, 0.0, 0.0);
        let mut v_lift_vector = Vector3::new(0.0, 0.0, 0.0);
        let mut f_attack_angle: f32 = 0.0;
        let mut tmp: f32 = 0.0;
        let mut v_resultant= Vector3::new(0.0, 0.0, 0.0);
        let mut vtmp = Vector3::new(50.0, 0.0, 0.0);
        //let mut stalling: bool = false;

        //loop through the 7 lifting elements, skipping the fuselage (the last element)
        for i in 0..6 
        {
            if i == 6 //tail rudder. its a special case because it can rotate, so the normal vector is recalculated
            {
                let inn: f32 = (rigidbod.element[i].f_incidence).to_radians();
                let di: f32 = (rigidbod.element[i].f_dihedral).to_radians();
                rigidbod.element[i].v_normal = Vector3::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
                rigidbod.element[i].v_normal = rigidbod.element[i].v_normal.normalize();
            }

            //calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
            //rotation part

            vtmp = rigidbod.v_angular_velocity.cross(&rigidbod.element[i].v_cg_coords); //crossproduct

            v_local_velocity = rigidbod.v_velocity_body + vtmp;

            //calculate local air speed
            f_local_speed = rigidbod.v_velocity_body.magnitude();

            //find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
            if f_local_speed > 1.0
            {
                v_drag_vector = -v_local_velocity / f_local_speed;
            }


            //find direction that lift will act. lift is perpendicular to the drag vector
            //(i think there is a problem with normalizing)
            v_lift_vector = (v_drag_vector.cross(&rigidbod.element[i].v_normal)).cross(&v_drag_vector);
            tmp = v_lift_vector.magnitude();
            v_lift_vector = v_lift_vector.normalize();
   
            //find the angle of attack. its the angle between the lfit vector and eelement normal vector. 
            tmp = v_drag_vector.dot(&rigidbod.element[i].v_normal);
            if tmp > 1.0
            {
                tmp = 1.0;
            }
            if tmp < -1.0
            {
                tmp = -1.0;
            }

            f_attack_angle = (tmp.sin()).to_radians();

            //determine lift and drag force on the element (rho is 1.225 in the book)
            tmp = 0.5 * 1.225 * f_local_speed * f_local_speed * rigidbod.element[i].f_area;
            if i == 6 //tail rudder
            {
                v_resultant = (v_lift_vector * rudder_lift_coefficient(f_attack_angle) + v_drag_vector * rudder_drag_coefficient(f_attack_angle)) * tmp;
            }
            else
            {
                v_resultant = (v_lift_vector * lift_coefficient(f_attack_angle, rigidbod.element[i].i_flap) + v_drag_vector * drag_coefficient(f_attack_angle, rigidbod.element[i].i_flap)) * tmp;
            }

            //check for stall. if the coefficient of lift is 0, stall is occuring.
            if lift_coefficient(f_attack_angle, rigidbod.element[i].i_flap) == 0.0
            {
                rigidbod.stalling = true; //probably will need to add stalling to rigid body variables
            }

            //keep running total of resultant forces (total force)
            fb = fb + v_resultant;

            //calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
            vtmp = rigidbod.element[i].v_cg_coords.cross(&v_resultant);
            mb = mb + vtmp;
        }


        //add thrust
        fb = fb + thrust;

        //convert forces from model space to earth space. rotates the vector by the unit quaternion (QVRotate function)
        rigidbod.v_forces = rigidbod.q_orientation_unit.transform_vector(&fb);

        //apply gravity (g is -32.174 ft/s^2)
        rigidbod.v_forces.z = rigidbod.v_forces.z + (-32.174) * rigidbod.mass;

        rigidbod.v_moments = rigidbod.v_moments + mb;
    }




//System to perform physics calculations based on forces "stepsimulation"
struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        WriteStorage<'a, RigidBody>,
        ReadStorage<'a, KeyboardState>
    );

    fn run(&mut self, (mut rigidbody, keyboardstate): Self::SystemData) 
    {
        for (mut rigidbod, keystate) in (&mut rigidbody, &keyboardstate).join() 
        {
            
            println!("{}", "inside eom");
            let mut ae = Vector3::new(0.0, 0.0, 0.0);
    
            //calculate all of the forces and moments on the airplane
            calc_airplane_loads(&mut rigidbod);
    
            //calculate acceleration of airplane in earth coordinates
            ae = rigidbod.v_forces / rigidbod.mass;
    
            //calculate velocity of airplane in earth coordinates
            rigidbod.v_velocity = rigidbod.v_velocity + ae * DT;
    
            //calculate position of airplane in earth coordinates
            rigidbod.v_position = rigidbod.v_position + rigidbod.v_velocity * DT;
    
            //handle rotations
            let mut mag: f32 = 0.0;
    
            //calculate angular velocity of airplane in body coordinates
            rigidbod.v_angular_velocity = rigidbod.v_angular_velocity + rigidbod. m_inertia_inverse * ( rigidbod.v_moments - ( rigidbod.v_angular_velocity.cross(&(rigidbod.m_inertia * rigidbod.v_angular_velocity)))) * DT;
    
    
            //calculate the new rotation quaternion
    
            //we need angular velocity to be in a quaternion form for the multiplication.... ( i think this gives an accurate result...) because 
            //nalgebra wont let me multiply the vector3 of angular velocity with the unit quaternion ( or a regular quaternion)
    
            //so create Quaternion based on the angular velocity ( i hope this math works out properly given the work around with nalgbra...) if not ill have to do it by hand
            let qtmp =  Quaternion::new(0.0, rigidbod.v_angular_velocity.x, rigidbod.v_angular_velocity.y, rigidbod.v_angular_velocity.z);                    
            rigidbod.q_orientation = rigidbod.q_orientation + (rigidbod.q_orientation * qtmp) * (0.5 * DT); 
    
            //now normalize the orientation quaternion (make into unit quaternion)
            rigidbod.q_orientation_unit = UnitQuaternion::new_normalize(rigidbod.q_orientation);
    
            //calculate the velocity in body coordinates
            rigidbod.v_velocity_body = rigidbod.q_orientation_unit.transform_vector(&rigidbod.v_velocity);
    
    
            //calculate air speed
            rigidbod.f_speed = rigidbod.v_velocity.magnitude();
    
            //get euler angles for our info
            let euler = rigidbod.q_orientation_unit.euler_angles();
            rigidbod.v_euler_angles.x = euler.0; //roll
            rigidbod.v_euler_angles.y = euler.1; //pitch
            rigidbod.v_euler_angles.z = euler.2; //yaw
    

        }//end for
    }//end run
}//end system



//System to send packets
struct SendPacket;
impl<'a> System<'a> for SendPacket
{
    type SystemData = (
            ReadStorage<'a, RigidBody>,
            ReadStorage<'a, FGNetFDM>,
    );

    fn run(&mut self, (rigidbody, fgnetfdm): Self::SystemData) 
    {
        for (rigidbod, _netfdm,) in (&rigidbody, &fgnetfdm).join() 
        {

            println!("{}", "inside send packet");


            //All data passed into the FGNetFDM struct is converted to network byte order

            //Create fdm instance
            let mut fdm: FGNetFDM = Default::default();

            //Set Roll, Pitch, Yaw
            let roll: f32 = rigidbod.v_euler_angles.x;
            let pitch: f32 =  rigidbod.v_euler_angles.y; 
            let yaw: f32 = rigidbod.v_euler_angles.z;  

            //Coordinate conversion: cartesian to geodetic
            //Need to first make the position vector into f64 to make coordinate transform happy
            let temp_ecef_vec: Vector3<f64> = Vector3::new(rigidbod.v_position.x as f64, rigidbod.v_position.y as f64, rigidbod.v_position.z as f64);
            let lla = geo::ecef2lla(&temp_ecef_vec, &ELLIPSOID); 

            //Set lat, long, alt
            fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
            fdm.altitude = f64::from_be_bytes(lla.z.to_ne_bytes()); //lla.z seems to increase altitude artificially...

            //Roll, Pitch, Yaw
            fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
            fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes()); //will use angle of attack because its "easier"
            fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

            //Other airplane data
            let fg_net_fdm_version = 24_u32;
            let visibility: f32 = 5000.0;
            fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());
            fdm.num_engines = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.num_tanks = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.num_wheels = u32::from_be_bytes(1_u32.to_ne_bytes());
            fdm.warp = f32::from_be_bytes(1_f32.to_ne_bytes());
            fdm.visibility = f32::from_be_bytes(visibility.to_ne_bytes());


            //Convert struct array of u8 of bytes
            let bytes: &[u8] = unsafe { any_as_u8_slice(&fdm) };

            //Finally send &[u8] of bytes to flight gear
            //Connect first (would be nice to only do this once...)
            SOCKET.connect("127.0.0.1:5500").expect("connect function failed");
            //Send!
            SOCKET.send(bytes).expect("couldn't send message");


            //Print some relevant data
            disable_raw_mode().unwrap(); //Get out of raw mode to print clearly
            //println!("time = {}", outdata.s);
            //println!("x traveled (m) = {}", outdata.q[1] / 3.6); //converted to meters
            println!("{}", rigidbod.v_position);
           // println!("altitude (m) = {}", lla.z);
            //println!("airspeed (km/h) = {}", outdata.airspeed);
            //println!("throttle % = {}", inpdata.throttle);
           // println!("angle of attack (deg) = {}", inpdata.alpha);
            //println!("x travel change (m) since last frame = {}", outdata.delta_traveled);
            //println!("bank angle (deg) = {}", inpdata.bank);
            //println!("y = {}", outdata.q[3]);
            enable_raw_mode().unwrap(); //Return to raw
  

        }//end for
    }//end run
}//end system


async fn handle_input(thrust_up: &mut bool, thrust_down: &mut bool, left_rudder: &mut bool, right_rudder: &mut bool, roll_left: &mut bool, roll_right: &mut bool, pitch_up: &mut bool, pitch_down: &mut bool, flaps_down: &mut bool, zero_flaps: &mut bool) 
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
                    else if event == Event::Key(KeyCode::Char('g').into()) 
                    {
                        *thrust_down = true;
                    }

                    //Rudders for yaw
                    else if event == Event::Key(KeyCode::Char('e').into()) 
                    {
                        *left_rudder = true;
                    }
                    else if event == Event::Key(KeyCode::Char('r').into()) 
                    {
                        *right_rudder = true;
                    }
                    // else if event == Event::Key(KeyCode::Char('d').into()) 
                    // {
                    //     *zero_rudder = true;
                    // }

                    //Ailerons for roll
                    else if event == Event::Key(KeyCode::Char('o').into()) 
                    {
                        *roll_left = true;
                    }
                    else if event == Event::Key(KeyCode::Char('p').into()) 
                    {
                        *roll_right = true;
                    }
                    // else if event == Event::Key(KeyCode::Char('g').into()) 
                    // {
                    //     *zero_ailerons = true;
                    // }


                    //Elevators for Pitch
                    else if event == Event::Key(KeyCode::Char('y').into()) 
                    {
                        *pitch_up = true;
                    }
                    else if event == Event::Key(KeyCode::Char('h').into()) 
                    {
                        *pitch_down = true;
                    }
                    // else if event == Event::Key(KeyCode::Char('h').into()) 
                    // {
                    //     *zero_elevators = true;
                    // }

                    //Flaps for lift
                    else if event == Event::Key(KeyCode::Char('f').into()) 
                    {
                        *flaps_down = true;
                    }
                    else if event == Event::Key(KeyCode::Char('z').into()) 
                    {
                        *zero_flaps = true;
                    }

                    //Quit program
                    else if event == Event::Key(KeyCode::Char('q').into()) 
                    {
                        //Exit program... maybe a better way to do this?
                        disable_raw_mode().unwrap();
                        process::exit(1);
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
        ReadStorage<'a, RigidBody>, 
        WriteStorage<'a, KeyboardState>,
    );

    fn run(&mut self, (rigidbody, mut keyboardstate): Self::SystemData) 
    {
        for (rigidbod, keystate) in (&rigidbody, &mut keyboardstate).join() 
        {
            println!("{}", "inside flight control");
            //Set all states false before we know if they are being activated
            keystate.thrust_up = false; 
            keystate.thrust_down = false;

            keystate.left_rudder = false;
            keystate.right_rudder = false;
            //keystate.zero_rudder = true,
        
            keystate.roll_left = false;
            keystate.roll_right = false;
            //keystate.zero_ailerons = true,
        
            keystate.pitch_up = false;
            keystate.pitch_down = false;
            //keystate.zero_elevators = true,
        
            keystate.flaps_down = false;
            keystate.zero_flaps = false;

            //Enter raw mode for terminal input
            enable_raw_mode().unwrap();

            let mut stdout = stdout();

            //This will make output not as crazy
            execute!(stdout, Clear(ClearType::All), cursor::MoveTo(0, 0)) .unwrap();
           
            //Handle flight control
            async_std::task::block_on(handle_input(&mut keystate.thrust_up, &mut keystate.thrust_down, &mut keystate.left_rudder, &mut keystate.right_rudder, &mut keystate.roll_left, &mut keystate.roll_right, &mut keystate.pitch_up,&mut keystate.pitch_down, &mut keystate.flaps_down, &mut keystate.zero_flaps));
        
            disable_raw_mode().unwrap();

        }//end for
    }//end run
}//end system


//create dummy plane to calculate mass properties






//Set some global variables:

//Time in between each eom calculation
//static DT: f64 = 0.5; //0.0167 

//Macro to define other globals
lazy_static!
{
    //define earth ellipsoid
    static ref ELLIPSOID: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);
    //create socket
    static ref SOCKET: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
}

//Time in between each eom calculation
static DT: f32 = 0.5; //0.0167 
fn main()
{
    //Create world
    let mut world = World::new();
    world.register::<RigidBody>();
    world.register::<KeyboardState>();

    //Create dispatcher of the systems
    let mut dispatcher = DispatcherBuilder::new()
    .with(FlightControl, "flightcontrol", &[])
    .with(EquationsOfMotion, "EOM", &[])
    .with(SendPacket, "sendpacket", &[])
    .build();
    dispatcher.setup(&mut world);




    //need its mass, inertia, and intertia inverse
    let mut myairplane = RigidBody{
        mass: 0.0,
        m_inertia: Matrix3::new(0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0),
        m_inertia_inverse: Matrix3::new(0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0),
        v_position: Vector3::new(904799.960942606, -5528914.45139109, 3038233.40847236),        //set initial position
        v_velocity: Vector3::new(60.0, 0.0, 0.0),        //set initial velocity
        v_euler_angles: Vector3::new(0.0, 0.0,0.0), //not defined in book here
        f_speed: 60.0,
        v_angular_velocity: Vector3::new(0.0, 0.0, 0.0),        //set angular velocity
        v_forces: Vector3::new(500.0, 0.0, 0.0),        //set initial thrust, forces, and moments
        thrustforce: 500.0,   //this isnt written in the rigid body intiialization for some reason...
        v_moments: Vector3::new(0.0, 0.0, 0.0),
        v_velocity_body: Vector3::new(0.0, 0.0, 0.0),        //zero the velocity in body space coordinates
        //set these to false at first, will control later with keyboard... these are not defined in the structure
        stalling: false,
        flaps: false,
        q_orientation: Quaternion::new(0.0, 0.0, 0.0, 0.0), //from_euler_angles(0.0, 0.0, 0.0),
        q_orientation_unit: UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        element: vec![
            PointMass{f_mass: 6.56, v_d_coords: Vector3::new(14.5, 12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector3::new(14.5, 5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector3::new(14.5, -5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 6.56, v_d_coords: Vector3::new(14.5, -12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector3::new(3.03, 2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector3::new(3.03, -2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.93, v_d_coords: Vector3::new(2.25, 0.0, 5.0), v_local_inertia: Vector3::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 31.8, v_d_coords: Vector3::new(15.25, 0.0, 1.5), v_local_inertia: Vector3::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) }
            ]
    };
    //work on dummyairplane
    calc_airplane_mass_properties(&mut myairplane);



    //Create plane entity with components (using dummy airplane to take on the calculated mass properties)
    let _plane = world.create_entity()
    .with(RigidBody{
        mass: myairplane.mass,
        m_inertia: myairplane.m_inertia,
        m_inertia_inverse: myairplane.m_inertia_inverse,
        v_position: myairplane.v_position,        //set initial position
        v_velocity: myairplane.v_velocity,        //set initial velocity
        v_euler_angles: myairplane.v_euler_angles, //not defined in book here
        f_speed: myairplane.f_speed,
        v_angular_velocity: myairplane.v_angular_velocity,        //set angular velocity
        v_forces: myairplane.v_forces,        //set initial thrust, forces, and moments
        thrustforce: myairplane.thrustforce,   //this isnt written in the rigid body intiialization for some reason...
        v_moments: myairplane.v_moments,
        v_velocity_body: myairplane.v_velocity_body,        //zero the velocity in body space coordinates
        //set these to false at first, will control later with keyboard... these are not defined in the structure
        stalling: false,
        flaps: false,
        q_orientation: myairplane.q_orientation, //from_euler_angles(0.0, 0.0, 0.0),
        q_orientation_unit: myairplane.q_orientation_unit,
        element: myairplane.element
    })
    .with(KeyboardState{
        thrust_up: false,
        thrust_down: false,
    
        left_rudder: false,
        right_rudder: false,
        zero_rudder: false,
    
        roll_left: false,
        roll_right: false,
        zero_ailerons: false,
    
        pitch_up: false,
        pitch_down: false,
        zero_elevators: false,
    
        flaps_down: false,
        zero_flaps: false,
    })
    .with(FGNetFDM{
        ..Default::default()
        })
    .build();


    //Loop simulation
    loop 
    {
        dispatcher.dispatch(&world);
        world.maintain();
    }

}





    //given the angle of attack and status of the flaps, return lfit angle coefficient for camabred airfoil with plain trailing-edge (+/- 15 degree inflation)
    fn lift_coefficient(angle: f32, flaps: i32) -> f32
    {

 
        let clf0 = vec![(0.54 * -1.0), (0.2 * -1.0), 0.2, 0.57, 0.92, 1.21, 1.43, 1.4, 1.0]; //why cant i just make a number negative with '-'?...weird
        let clfd = vec![0.0, 0.45, 0.85, 1.02, 1.39, 1.65, 1.75, 1.38, 1.17];
        let clfu = vec![(0.74 * -1.0), (0.4 * -1.0), 0.0, 0.27, 0.63, 0.92, 1.03, 1.1, 0.78];
        let a = vec![(8.0 * -1.0), (4.0 * -1.0), 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

        let mut cl: f32 = 0.0;

        for i in 0..8  
        {
            if a[i] <= angle && a[i + 1] > angle
            {
                if flaps == 0 //flaps not deflected
                {
                    cl = clf0[i] - (a[i] - angle) * (clf0[i] - clf0[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
                else if flaps == -1 //flaps down
                {
                    cl = clfd[i] - (a[i] - angle) * (clfd[i] - clfd[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
                else if flaps == 1 //flaps up
                {
                    cl = clfu[i] - (a[i] - angle) * ( clfu[i] - clfu[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
            }

        }

        return cl;
    }


    //functions to collect airfoil performance data
    //lift and drag coefficient data is given for a set of discrete attack angles, so then linear interpolation is used to determine the coefficients for the attack angle that falls between the discrete angles
    //given angle of attack and flap status, return drag coefficient for cambered airfoil with plain trailing-edge flap (+/- 15 degree deflection)
    fn drag_coefficient(angle: f32, flaps: i32) -> f32
    {
        let cdf0 = vec![0.01, 0.0074, 0.004, 0.009, 0.013, 0.023, 0.05, 0.12, 0.21];
        let cdfd = vec![0.0065, 0.0043, 0.0055, 0.0153, 0.0221, 0.0391, 0.1, 0.195, 0.3];
        let cdfu = vec![0.005, 0.0043, 0.0055, 0.02601, 0.03757, 0.06647, 0.13, 0.1, 0.25];
        let a = vec![(8.0 * -1.0), (4.0 * -1.0), 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

        let mut cd: f32 = 0.5;

        for i in 0..8  
        {
            if a[i] <= angle && a[i + 1] > angle
            {
                if flaps == 0 //flaps not deflected
                {
                    cd = cdf0[i] - (a[i] - angle) * (cdf0[i] - cdf0[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
                else if flaps == -1 //flaps down
                {
                    cd = cdfd[i] - (a[i] - angle) * (cdfd[i] - cdfd[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
                else if flaps == 1 //flaps up
                {
                    cd = cdfu[i] - (a[i] - angle) * ( cdfu[i] - cdfu[i + 1]) / (a[i] - a[i + 1]);
                    break;
                }
            }

        }

        return cd;
    }


    //Rudder lift and drag coefficients are similar to that of the wing but the coefficients themselves are different and the tail rudder does not include flaps
    //given attack angle, return lift coefficient for a symmetric (no camber) airfoil without flaps
    fn rudder_lift_coefficient(angle: f32) -> f32
    {
        let clf0 = vec![0.16, 0.456, 0.736, 0.968, 1.144, 1.12, 0.8];
        let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

        let mut cl: f32 = 0.0;
        let aa: f32 = angle.abs();

        for i in 0..8  
        {
            if a[i] <= aa && a[i + 1] > aa
            {
                cl = clf0[i] - (a[i] - aa) * (clf0[i] - clf0[i + 1]) / (a[i] - a[i + 1]);

                if angle < 0.0
                {
                    cl = -cl;
                }
                break;
            }
        }
        return cl;
    }
  
     //given attack angle, return drag coefficient for a symmetric (no camber) airfoil without flaps
     fn rudder_drag_coefficient(angle: f32) -> f32
     {
         let cdf0 = vec![0.0032, 0.0072, 0.0104, 0.0184, 0.04, 0.096, 0.168];
         let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];
 
         let mut cd: f32 = 0.5;
         let aa: f32 = angle.abs();
 
         for i in 0..8  
         {
             if a[i] <= aa && a[i + 1] > aa
             {
                 cd = cdf0[i] - (a[i] - aa) * (cdf0[i] - cdf0[i + 1]) / (a[i] - a[i + 1]);
 
                 break;
             }
         }
         return cd;
     }


