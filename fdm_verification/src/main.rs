//ECS(SPECS)
use specs::prelude::*;

//Vector, Matrix, Quaternion module
mod common;

//Component State Machine for keyboard
#[derive(Debug)]
struct KeyboardState
{
    thrust_up: bool,
    thrust_down: bool,
    left_rudder: bool,
    right_rudder: bool,
    roll_left: bool,
    roll_right: bool,
    pitch_up: bool,
    pitch_down: bool,
    flaps_down: bool,
    zero_flaps: bool,
}
impl Component for KeyboardState
{
    type Storage = VecStorage<Self>;
}

//elements making up the bodystructure. this is part of the RigidBody component
#[derive(Debug)]
struct PointMass
{
    f_mass: f32,
    v_d_coords: common::Myvec, //"design position"
    v_local_inertia: common::Myvec,
    f_incidence: f32,
    f_dihedral: f32,
    f_area: f32,
    i_flap: i32,
    v_normal: common::Myvec,
    v_cg_coords: common::Myvec //"corrected position"
}

//Component containing data on the airplane
#[derive(Debug, Default)]
struct RigidBody
{
    mass: f32,                                  //total mass
    m_inertia: common::Mymatrix,
    m_inertia_inverse: common::Mymatrix,
    v_position: common::Myvec, // position in earth coordinates
    v_velocity: common::Myvec, // velocity in earth coordinates
    v_velocity_body: common::Myvec, // velocity in body coordinates
    v_angular_velocity: common::Myvec, // angular velocity in body coordinates
    v_euler_angles: common::Myvec,   
    f_speed: f32, // speed (magnitude of the velocity)
    stalling: bool,
    flaps: bool,
    q_orientation: common::Myquaternion, // orientation in earth coordinates 
    v_forces: common::Myvec, // total force on body
    thrustforce: f32, // magnitude of thrust
    v_moments: common::Myvec, // total moment (torque) on body
    element: Vec<PointMass>, // vector of point mass elements
    current_frame: usize,
}
impl Component for RigidBody
{
    type Storage = VecStorage<Self>;
}


//Calculate mass properties based on the airplane's different body pieces
fn calc_airplane_mass_properties(rigidbod: &mut RigidBody)
{
    let mut inn: f32;
    let mut di: f32;

    //Calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
    for  i in rigidbod.element.iter_mut()
    {
        inn = (i.f_incidence).to_radians();
        di = (i.f_dihedral).to_radians();
        i.v_normal = common::Myvec::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
        i.v_normal.normalize(); 
    }

    //Calculate total mass
    let mut total_mass: f32 = 0.0;
    for i in rigidbod.element.iter()
    {
        total_mass = total_mass + i.f_mass;
    }

    //Calculate combined center of gravity location
    let mut v_moment = common::Myvec::new(0.0,0.0,0.0);
    for i in rigidbod.element.iter()
    {
        let tmp = common::Myvec::multiplyscalar(&i.v_d_coords, i.f_mass);
        v_moment = common::Myvec::addvec(&v_moment, &tmp);
    }
    let cg = common::Myvec::dividescalar(&v_moment, total_mass); 

    //Calculate coordinates of each element with respect to the combined CG, relative position
    for i in rigidbod.element.iter_mut()
    {
        i.v_cg_coords = common::Myvec::subtractvec(&i.v_d_coords, &cg);
    }

    //Calculate the moments and products of intertia for the combined elements
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

    //Finally, set up airplanes mass and inertia matrix
    rigidbod.mass = total_mass;
    rigidbod.m_inertia = common::Mymatrix::new(ixx, -ixy, -ixz,
                                               -ixy, iyy, -iyz,
                                               -ixz, -iyz, izz);

    //Get inverse of matrix
    rigidbod.m_inertia_inverse = common::Mymatrix::inverse(&rigidbod.m_inertia);
}


//FORCES
//calculates all of the forces and moments on the plane at any time (called inside eom system)
fn calc_airplane_loads(rigidbod: &mut RigidBody)
{
    let mut fb = common::Myvec::new(0.0, 0.0, 0.0); //total force
    let mut mb = common::Myvec::new(0.0, 0.0, 0.0); //total moment

    //Reset forces and moments
    rigidbod.v_forces = common::Myvec::new(0.0, 0.0, 0.0);
    rigidbod.v_moments = common::Myvec::new(0.0, 0.0, 0.0);

    //Define thrust vector, which acts through the plane's center of gravity
    let mut thrust = common::Myvec::new(1.0, 0.0, 0.0);
    thrust = common::Myvec::multiplyscalar(&thrust, rigidbod.thrustforce);

    //Calculate forces and moments in body space
    let mut v_drag_vector = common::Myvec::new(0.0, 0.0, 0.0); //VERY IMPORTANT THAT THESE WERE SET TO 1.0... 11/15 not anymore..
    let mut v_resultant= common::Myvec::new(0.0, 0.0, 0.0);

    rigidbod.stalling = false;

    //Loop through the 7 lifting elements, skipping the fuselage
    for i in 0..8 
    {
        if i == 6 //Tail rudder. its a special case because it can rotate, so the normal vector is recalculated
        {
            let inn: f32 = (rigidbod.element[i].f_incidence).to_radians();
            let di: f32 = (rigidbod.element[i].f_dihedral).to_radians();
            rigidbod.element[i].v_normal = common::Myvec::new(inn.sin(),
                                                              inn.cos() * di.sin(), 
                                                              inn.cos() * di.cos());
            rigidbod.element[i].v_normal.normalize();
        }
       
        //Calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
        let mut vtmp = common::Myvec::crossproduct(&rigidbod.v_angular_velocity, &rigidbod.element[i].v_cg_coords);
        let v_local_velocity = common::Myvec::addvec(&rigidbod.v_velocity_body, &vtmp);

        //Calculate local air speed
        let f_local_speed: f32 = rigidbod.v_velocity_body.magnitude(); 

        //Find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
        if f_local_speed > 1.0
        {
            let v_local_vel_tmp = common::Myvec::reverse_aka_conjugate(&v_local_velocity); //-vLocalVelocity
            v_drag_vector = common::Myvec::dividescalar(&v_local_vel_tmp, f_local_speed);
        }

        //Find direction that lift will act. lift is perpendicular to the drag vector
        let lift_tmp = common::Myvec::crossproduct(&v_drag_vector, &rigidbod.element[i].v_normal);
        let mut v_lift_vector = common::Myvec::crossproduct(&lift_tmp, &v_drag_vector);
        let mut tmp = v_lift_vector.magnitude(); 
        v_lift_vector.normalize();
  
        //Find the angle of attack. its the angle between the lift vector and element normal vector 
        tmp = common::Myvec::dotproduct(&v_drag_vector, &rigidbod.element[i].v_normal);

        if tmp > 1.0
        {
            tmp = 1.0;
        }
        if tmp < -1.0
        {
            tmp = -1.0;
        }

        let f_attack_angle: f32 = tmp.asin().to_degrees();

        //Determine lift and drag force on the element rho is defined as 0.0023769
        tmp = 0.5 * 0.002376900055 * f_local_speed * f_local_speed * rigidbod.element[i].f_area;   

        if i == 6 //tail/rudder
        {
            let firstpart = common::Myvec::multiplyscalar(&v_lift_vector, rudder_lift_coefficient(f_attack_angle));
            let secondpart = common::Myvec::multiplyscalar(&v_drag_vector, rudder_drag_coefficient(f_attack_angle));
            let addtogether = common::Myvec::addvec(&firstpart, &secondpart);
            v_resultant = common::Myvec::multiplyscalar(&addtogether, tmp);
        }
        else if i == 7
        {
            v_resultant = common::Myvec::multiplyscalar(&v_drag_vector, 0.5 * tmp); //simulate fuseulage

        }
        else
        {
            let firstpart = common::Myvec::multiplyscalar(&v_lift_vector, lift_coefficient(f_attack_angle, rigidbod.element[i].i_flap));
            let secondpart = common::Myvec::multiplyscalar(&v_drag_vector, drag_coefficient(f_attack_angle, rigidbod.element[i].i_flap));
            let addtogether = common::Myvec::addvec(&firstpart, &secondpart);
            v_resultant = common::Myvec::multiplyscalar(&addtogether, tmp);
        }

        //Check for stall. if the coefficient of lift is 0, stall is occuring.
        if i <= 3
        {
            if lift_coefficient(f_attack_angle, rigidbod.element[i].i_flap) == 0.0
            {
                rigidbod.stalling = true; 
            }
        }

        //Keep running total of resultant forces (total force)
        fb = common::Myvec::addvec(&fb, &v_resultant);

        //Calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
        vtmp = common::Myvec::crossproduct(&rigidbod.element[i].v_cg_coords, &v_resultant);
        mb = common::Myvec::addvec(&mb, &vtmp);
     }

    //Add thrust
    fb = common::Myvec::addvec(&fb, &thrust);

    //Convert forces from model space to earth space. rotates the vector by the unit quaternion (QVRotate function)
     rigidbod.v_forces = common::Myquaternion::qvrotate(&rigidbod.q_orientation, &fb);

    //Apply gravity (g is -32.174 ft/s^2)
    rigidbod.v_forces.z = rigidbod.v_forces.z + (-32.17399979) * rigidbod.mass;

    rigidbod.v_moments = common::Myvec::addvec(&rigidbod.v_moments, &mb);
}


//System to perform equations of motion physics calculations based on forces
struct EquationsOfMotion;
impl<'a> System<'a> for EquationsOfMotion
{
    type SystemData = (
        WriteStorage<'a, RigidBody>,
        WriteStorage<'a, KeyboardState> //Write needed for equivalency test because we will use flight controls artificially
    );

    fn run(&mut self, (mut rigidbody, mut keyboardstate): Self::SystemData) 
    {
        for (mut rigidbod, mut keystate) in (&mut rigidbody, &mut keyboardstate).join() 
        {
            //FOR EQUIVALENCY TESTS: increment current frame tracker
            rigidbod.current_frame = rigidbod.current_frame + 1;

            //reset/zero the elevators, rudders, and ailerons every loo
            //rudder
            rigidbod.element[6].f_incidence = 0.0;
            //ailerons
            rigidbod.element[0].i_flap = 0;
            rigidbod.element[3].i_flap = 0;
            //elevators
            rigidbod.element[4].i_flap = 0;
            rigidbod.element[5].i_flap = 0;
            //flaps will be toggled on and off so flaps does not need to be zerod each time

            //Set all states false before we know if they are being activated in the test
            keystate.thrust_up = false; 
            keystate.thrust_down = false;

            keystate.left_rudder = false;
            keystate.right_rudder = false;
        
            keystate.roll_left = false;
            keystate.roll_right = false;
 
            keystate.pitch_up = false;
            keystate.pitch_down = false;
            //flaps are toggled on and off, so they dont need to be set to false each time


            //FOR EQUIVALENCY TESTS: set flight controls artificially based on current frame
            //all that is needed is to just set that keystate to true to activate that functionality

            //TEST 1 (its just the default with no functionality )

            //TEST 2 THRUST
            // if rigidbod.current_frame >= 1 && rigidbod.current_frame <= 5
            // {
            //     keystate.thrust_down = true;
            // }

            //TEST 3 Roll 
            // if rigidbod.current_frame >= 1 && rigidbod.current_frame <= 5
            // {
            //     keystate.thrust_up = true;
            // }
            // else if rigidbod.current_frame >= 6 && rigidbod.current_frame <= 246 //pitch up 8 seconds
            // {
            //     keystate.pitch_up = true;
            // }
            // else if rigidbod.current_frame >= 247 && rigidbod.current_frame <= 307 //roll right for a 2 second
            // {
            //     keystate.roll_right = true;
            // }


            //TEST 4 Pitch
            //increase thrust by 500
            // if rigidbod.current_frame >= 1 && rigidbod.current_frame <= 5
            // {
            //     keystate.thrust_up = true;
            // }
  
            // keystate.pitch_up = true;


            //TEST 5 Yaw
            // if rigidbod.current_frame >= 1 && rigidbod.current_frame <= 5
            // {
            //     keystate.thrust_up = true;
            // }
            // else if rigidbod.current_frame >= 6 && rigidbod.current_frame <= 246 //pitch up 8 seconds
            // {
            //     keystate.pitch_up = true;
            // }
            // else if rigidbod.current_frame % 9 == 0
            // {
            //     keystate.right_rudder = true;
            // }

            //TEST 6 Flaps
            //keystate.flaps_down = true;


            //Handle the input states
            //Thrust states
            if rigidbod.thrustforce < MAX_THRUST && keystate.thrust_up == true
            {
                rigidbod.thrustforce = rigidbod.thrustforce + D_THRUST;
            }   
            else if rigidbod.thrustforce > 0.0 && keystate.thrust_down == true
            {
                rigidbod.thrustforce = rigidbod.thrustforce - D_THRUST;
            }


            if keystate.left_rudder == true
            { 
                rigidbod.element[6].f_incidence = 16.0;
            } 
            else if keystate.right_rudder == true
            { 
                rigidbod.element[6].f_incidence = -16.0;
            } 

            if keystate.roll_left == true
            { 
                rigidbod.element[0].i_flap = 1;
                rigidbod.element[3].i_flap = -1;
            } 
           else if keystate.roll_right == true
            { 
                rigidbod.element[0].i_flap = -1;
                rigidbod.element[3].i_flap = 1;
            } 

            //Pitch States
            if keystate.pitch_up == true
            { 
                rigidbod.element[4].i_flap = 1;
                rigidbod.element[5].i_flap = 1;
            } 
            else if keystate.pitch_down == true
            { 
                rigidbod.element[4].i_flap = -1;
                rigidbod.element[5].i_flap = -1;

            } 

            //Flap States
            if keystate.flaps_down == true
            { 
                rigidbod.element[1].i_flap = -1;
                rigidbod.element[2].i_flap = -1;
                rigidbod.flaps = true;
            }
            else if keystate.zero_flaps == true
            { 
                rigidbod.element[1].i_flap = 0;
                rigidbod.element[2].i_flap = 0;
                rigidbod.flaps = false;
            } 

    
            //Calculate all of the forces and moments on the airplane
            calc_airplane_loads(&mut rigidbod);

            //Calculate acceleration of airplane in earth space
            let ae: common::Myvec = common::Myvec::dividescalar(&rigidbod.v_forces, rigidbod.mass);

            //Calculate velocity of airplane in earth space
            let ae_mult_dt_tmp = common::Myvec::multiplyscalar(&ae, DT);
            rigidbod.v_velocity = common::Myvec::addvec(&rigidbod.v_velocity, &ae_mult_dt_tmp); 

            //Calculate position of airplane in earth space
            let vel_mult_dt_tmp = common::Myvec::multiplyscalar(&rigidbod.v_velocity, DT);
            rigidbod.v_position = common::Myvec::addvec(&rigidbod.v_position, &vel_mult_dt_tmp); 

            //Calculate angular velocity of airplane in body space
            let one = common::Mymatrix::multiply_matrix_by_vec(&rigidbod.m_inertia, &rigidbod.v_angular_velocity);
            let two = common::Myvec::crossproduct(&rigidbod.v_angular_velocity, &one);
            let three = common::Myvec::subtractvec(&rigidbod.v_moments, &two);
            let four = common::Mymatrix::multiply_matrix_by_vec(&rigidbod.m_inertia_inverse, &three);
            let five = common::Myvec::multiplyscalar(&four, DT);
            rigidbod.v_angular_velocity = common::Myvec::addvec(&rigidbod.v_angular_velocity, &five);

            //Calculate the new rotation quaternion
            let uno = common::Myquaternion::multiply_quat_by_vec(&rigidbod.q_orientation, &rigidbod.v_angular_velocity);
            let dos = common::Myquaternion::multiplyscalar(&uno, 0.5 * DT);
            rigidbod.q_orientation = common::Myquaternion::addquat(&rigidbod.q_orientation, &dos);

            //Now normalize the orientation quaternion (make into unit quaternion)
            let mag = rigidbod.q_orientation.magnitude();
            if mag != 0.0
            {
               rigidbod.q_orientation = common::Myquaternion::dividescalar(&rigidbod.q_orientation, mag);
            }

            //Calculate the velocity in body space:
            rigidbod.v_velocity_body = common::Myquaternion::qvrotate(&common::Myquaternion::conjugate(&rigidbod.q_orientation), &rigidbod.v_velocity);
    
            //Calculate air speed
            rigidbod.f_speed = rigidbod.v_velocity.magnitude(); 
    
            //Get euler angles for our info
            let euler = common::Myquaternion::make_euler_from_q(&rigidbod.q_orientation);
            //FOR FLIGHTGEAR SIMULATION: yaw OR roll is negated odepending on position starting, pitch is always negated
            //FOR FDM EQUIVALENCE TESTING: we stay true to the bourg model and do not negate the euler angles
            rigidbod.v_euler_angles.x = euler.x; 
            rigidbod.v_euler_angles.y = euler.y;
            rigidbod.v_euler_angles.z = euler.z; 
            
            //Print some relevant data
            println!("roll:             {}", rigidbod.v_euler_angles.x);
            println!("pitch:            {}", -rigidbod.v_euler_angles.y); //pitch is negated only on the print statement in bourgs fdm
            println!("yaw:              {}", rigidbod.v_euler_angles.z);
            println!("alt:              {}", rigidbod.v_position.z);
            println!("thrust:           {}", rigidbod.thrustforce);
            println!("speed:            {}", rigidbod.f_speed/1.688);
            println!("pos x:            {}", rigidbod.v_position.x);
            println!("pos y:            {}", rigidbod.v_position.y);
            println!("pos z:            {}", rigidbod.v_position.z);

        }//end for
    }//end run
}//end system


//Thrust
static MAX_THRUST: f32 = 3000.0; //max thrustforce value
static D_THRUST: f32 = 100.0;   //change in thrust per keypress

//Time in between each update
static FRAME_RATE: f32 = 30.0;
static DT: f32 = 1.0 / FRAME_RATE;

fn main()
{
    //Create variable to keep track of time elapsed
    let mut current_time: f32 = 0.0;
    let mut current_frame_main: usize = 0;

    //Create world
    let mut world = World::new();
    world.register::<RigidBody>();
    world.register::<KeyboardState>();

    //Create dispatcher to manage system execution
    let mut dispatcher = DispatcherBuilder::new()
    .with(EquationsOfMotion, "EOM", &[])
    .build();
    dispatcher.setup(&mut world);

    //Intialize the airplane

    //Setup airplane with default values and then fill in what we need to calculate mass properties
    let mut myairplane = RigidBody{..Default::default()};

    //test start position (same as c++ version)
    myairplane.v_position.x = -5000.0;
    myairplane.v_position.y = 0.0;
    myairplane.v_position.z = 2000.0; 

    myairplane.v_velocity.x = 60.0;
    myairplane.f_speed = 60.0;
    myairplane.v_forces.x = 500.0;
    myairplane.thrustforce = 500.0;
    myairplane.q_orientation = common::Myquaternion::make_q_from_euler(0.0, 0.0, 0.0);
    myairplane.element = vec![
        PointMass{f_mass: 6.56, v_d_coords: common::Myvec::new(14.5, 12.0, 2.5), v_local_inertia: common::Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 7.31, v_d_coords: common::Myvec::new(14.5, 5.5, 2.5), v_local_inertia: common::Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 7.31, v_d_coords: common::Myvec::new(14.5, -5.5, 2.5), v_local_inertia: common::Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 6.56, v_d_coords: common::Myvec::new(14.5, -12.0, 2.5), v_local_inertia: common::Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 2.62, v_d_coords: common::Myvec::new(3.03, 2.5, 3.0), v_local_inertia: common::Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 2.62, v_d_coords: common::Myvec::new(3.03, -2.5, 3.0), v_local_inertia: common::Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 2.93, v_d_coords: common::Myvec::new(2.25, 0.0, 5.0), v_local_inertia: common::Myvec::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
        PointMass{f_mass: 31.8, v_d_coords: common::Myvec::new(15.25, 0.0, 1.5), v_local_inertia: common::Myvec::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) }
        ];


    //Calculate mass properties on this airplane initialized
    calc_airplane_mass_properties(&mut myairplane);

    //Create plane Entity and populate the Component data using the adata from myairplane (had to do this because i could not calculate mass properties from the entity directly)
    let _plane = world.create_entity()
    .with(RigidBody{
        mass: myairplane.mass,
        m_inertia: myairplane.m_inertia,
        m_inertia_inverse: myairplane.m_inertia_inverse,
        v_position: myairplane.v_position,       
        v_velocity: myairplane.v_velocity,      
        v_euler_angles: myairplane.v_euler_angles, 
        f_speed: myairplane.f_speed,
        v_angular_velocity: myairplane.v_angular_velocity,      
        v_forces: myairplane.v_forces,        
        thrustforce: myairplane.thrustforce, 
        v_moments: myairplane.v_moments,
        v_velocity_body: myairplane.v_velocity_body,     
        stalling: false,
        flaps: false,
        q_orientation: myairplane.q_orientation, 
        element: myairplane.element,
        current_frame: 0,
    })
    .with(KeyboardState{
        thrust_up: false,
        thrust_down: false,
    
        left_rudder: false,
        right_rudder: false,
    
        roll_left: false,
        roll_right: false,

        pitch_up: false,
        pitch_down: false,
    
        flaps_down: false,
        zero_flaps: false,
    })
    .build();

    //Main simulation loop
    for _ in 0..900
    {
        //increment time count
        current_time = current_time + DT;
        current_frame_main = current_frame_main + 1;
        println!("{}", "====================================================");
        println!("time:             {}", current_time);
        println!("frames:           {}", current_frame_main);

        //process this frame
        dispatcher.dispatch(&world); 
        world.maintain();
    
    }

}


//Functions to collect airfoil performance data:
//lift and drag coefficient data is given for a set of discrete attack angles, 
//so then linear interpolation is used to determine the coefficients for the 
//attack angle that falls between the discrete angles.

//Given the angle of attack and status of the flaps,
//return lift angle coefficient for camabred airfoil with 
//plain trailing-edge (+/- 15 degree inflation).
fn lift_coefficient(angle: f32, flaps: i32) -> f32
{
    let clf0 = vec![-0.54, -0.2, 0.2, 0.57, 0.92, 1.21, 1.43, 1.4, 1.0];
    let clfd = vec![0.0, 0.45, 0.85, 1.02, 1.39, 1.65, 1.75, 1.38, 1.17];
    let clfu = vec![-0.74, -0.4, 0.0, 0.27, 0.63, 0.92, 1.03, 1.1, 0.78];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

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


//given angle of attack and flap status, 
//return drag coefficient for cambered airfoil with 
//plain trailing-edge flap (+/- 15 degree deflection).
fn drag_coefficient(angle: f32, flaps: i32) -> f32
{
    let cdf0 = vec![0.01, 0.0074, 0.004, 0.009, 0.013, 0.023, 0.05, 0.12, 0.21];
    let cdfd = vec![0.0065, 0.0043, 0.0055, 0.0153, 0.0221, 0.0391, 0.1, 0.195, 0.3];
    let cdfu = vec![0.005, 0.0043, 0.0055, 0.02601, 0.03757, 0.06647, 0.13, 0.1, 0.25];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f32 = 0.75; //0.5 in book but 0.75 in actual code

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


//Rudder lift and drag coefficients are similar to that of the wing 
//but the coefficients themselves are different and the tail rudder 
//does not include flaps.
//Given attack angle, return lift coefficient for a symmetric (no camber) 
//airfoil without flaps.
fn rudder_lift_coefficient(angle: f32) -> f32
{
    let clf0 = vec![0.16, 0.456, 0.736, 0.968, 1.144, 1.12, 0.8];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cl: f32 = 0.0;
    let aa: f32 = angle.abs();

    for i in 0..6 
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

//Given attack angle, return drag coefficient for a symmetric (no camber) 
//airfoil without flaps.
fn rudder_drag_coefficient(angle: f32) -> f32
{
    let cdf0 = vec![0.0032, 0.0072, 0.0104, 0.0184, 0.04, 0.096, 0.168];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f32 = 0.75; //0.5 in book
    let aa: f32 = angle.abs();

    for i in 0..6  
    {
        if a[i] <= aa && a[i + 1] > aa
        {
            cd = cdf0[i] - (a[i] - aa) * (cdf0[i] - cdf0[i + 1]) / (a[i] - a[i + 1]);
            break;
        }
    }
    return cd;
}
