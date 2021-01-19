//Integration tests for the Bourg-based Flight Dynamics Model

//To run on the command line: cargo test --test bourg_integration

//Float_cmp crate for comparing floats
use float_cmp::*;

//Bring into scope the functions and data as necessary for testing
use fdm_library::bourg_fdm::common::vector::Myvec;
use fdm_library::bourg_fdm::common::matrix::Mymatrix;
use fdm_library::bourg_fdm::common::quaternion::Myquaternion;
use fdm_library::bourg_fdm::components::component_datafdm::*;
use fdm_library::bourg_fdm::systems::system_equations_of_motion::calc_airplane_loads;
use fdm_library::bourg_fdm::systems::system_equations_of_motion::calc_airplane_mass_properties;

#[test]
fn fdm_test() //Can test one flight data variable of the entire FDM at a time. Must change frame #, and get that value to compare from C++ output. Can also add flight controls artificially
{
    let fps = 30.0;
    let dt = 1.0 / fps;
    let d_thrust = 100.0;

    //Create airplane to compute mass properties
    let mut fdm = DataFDM{ 
        element : vec![
            PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, 12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, 5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Myvec::new(14.5, -5.5, 2.5), v_local_inertia: Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 6.56, v_d_coords: Myvec::new(14.5, -12.0, 2.5), v_local_inertia: Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, 2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Myvec::new(3.03, -2.5, 3.0), v_local_inertia: Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.93, v_d_coords: Myvec::new(2.25, 0.0, 5.0), v_local_inertia: Myvec::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 31.8, v_d_coords: Myvec::new(15.25, 0.0, 1.5), v_local_inertia: Myvec::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Myvec::new(0.0, 0.0, 0.0), v_cg_coords: Myvec::new(0.0, 0.0, 0.0) }
        ], 
    
        
    //Define initial flight parameters
    v_position: Myvec{x: -5000.0, y: 0.0, z: 2000.0},
    v_velocity: Myvec{x: 60.0, y: 0.0, z: 0.0},
    f_speed: 60.0,
    v_forces: Myvec{x: 500.0, y: 0.0, z: 0.0},
    thrustforce: 500.0,
    q_orientation: Myquaternion::make_q_from_euler(0.0, 0.0, 0.0),

    //Everything else is zero to begin
    ..Default::default()
    };

    //Calculate mass properties on this airplane
    calc_airplane_mass_properties(&mut fdm);

    let mut current_frame: usize = 0;
    for _ in 0..900 //frame #
    {
        //Increment frame
        current_frame = current_frame + 1;

        //Reset/zero the elevators, rudders, and ailerons every loop
        //Rudder
        fdm.element[6].f_incidence = 0.0;
        //Ailerons
        fdm.element[0].i_flap = 0;
        fdm.element[3].i_flap = 0;
        //Elevators
        fdm.element[4].i_flap = 0;
        fdm.element[5].i_flap = 0;
        //Flaps will be toggled on and off so flaps does not need to be zerod each time

            //TESTS WILL ARTIFICIALLY STIMULATE THE AIRPLANE'S COMPONENTS FOR A GIVEN FRAME

            //Flight component activation guide

            //thrust up
            // fdm.thrustforce = fdm.thrustforce + d_thrust;
            
            //thrust down
            // fdm.thrustforce = fdm.thrustforce - d_thrust;
        
            //left rudder
            // fdm.element[6].f_incidence = 16.0;

            //right rudder
            // fdm.element[6].f_incidence = -16.0;

            //roll left
            // fdm.element[0].i_flap = 1;
            // fdm.element[3].i_flap = -1;

            //roll right
            // fdm.element[0].i_flap = -1;
            // fdm.element[3].i_flap = 1;

            //pitch up
            // fdm.element[4].i_flap = 1;
            // fdm.element[5].i_flap = 1;

            //pitch down
            // fdm.element[4].i_flap = -1;
            // fdm.element[5].i_flap = -1;
        
            //flaps deflected 
            // fdm.element[1].i_flap = -1;
            // fdm.element[2].i_flap = -1;
            // fdm.flaps = true;
        
            //no flaps
            // fdm.element[1].i_flap = 0;
            // fdm.element[2].i_flap = 0;
            // fdm.flaps = false;


            //---------------------------------------
            //TEST 1 (no flight control)

            // //TEST 2 Roll 
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     //increase thrust
            //     fdm.thrustforce = fdm.thrustforce + d_thrust;
            // }
            // else if current_frame >= 6 && current_frame <= 246 //pitch up 8 seconds
            // {
            //     //pitch up
            //     fdm.element[4].i_flap = 1;
            //     fdm.element[5].i_flap = 1;
            // }
            // else if current_frame >= 247 && current_frame <= 307 //roll right for 2 seconds
            // {
            //     //roll right
            //     fdm.element[0].i_flap = -1;
            //     fdm.element[3].i_flap = 1;
            // }


            // //TEST 3 Pitch
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     //increase thrust
            //     fdm.thrustforce = fdm.thrustforce + d_thrust;
            // }
            // //pitch up
            // fdm.element[4].i_flap = 1;
            // fdm.element[5].i_flap = 1;


            // //TEST 4 Yaw
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     //increase thrust
            //     fdm.thrustforce = fdm.thrustforce + d_thrust;
            // }
            // else if current_frame >= 6 && current_frame <= 246 
            // {
            //     //pitch up 8 seconds
            //     fdm.element[4].i_flap = 1;
            //     fdm.element[5].i_flap = 1;
            // }
            // else if current_frame >= 247 && current_frame <= 307 //yaw right for 2 seconds
            // {
            //     //yaw right
            //     fdm.element[6].f_incidence = -16.0;
            // }

            // //TEST 5 Flaps
            // fdm.element[1].i_flap = -1;
            // fdm.element[2].i_flap = -1;
            // fdm.flaps = true;


            //TEST 6 EVERYTHING
            if current_frame >= 1 && current_frame <= 5
            {
                //Increase thrust
                fdm.thrustforce = fdm.thrustforce + d_thrust;
            }
            else if current_frame >= 6 && current_frame <= 246 
            {
                //Pitch up
                fdm.element[4].i_flap = 1;
                fdm.element[5].i_flap = 1;
            }
            else if current_frame >= 247 && current_frame <= 307
            {
                //Yaw left
                fdm.element[6].f_incidence = 16.0;

            }
            else if current_frame >= 308 && current_frame <= 368
            {
                //Yaw right
                fdm.element[6].f_incidence = -16.0;
            }
            else if current_frame >= 369 && current_frame <= 469
            {
                //Roll right
                fdm.element[0].i_flap = -1;
                fdm.element[3].i_flap = 1;
            }
            else if current_frame >= 470 && current_frame <= 530
            {
                //Roll left
                fdm.element[0].i_flap = 1;
                fdm.element[3].i_flap = -1;
            }
            else if current_frame >= 531 && current_frame <= 546
            {
                //pitch down
                fdm.element[4].i_flap = -1;
                fdm.element[5].i_flap = -1;
            }
            else if current_frame >= 547 && current_frame <= 552
            {
                //thrust down
                fdm.thrustforce = fdm.thrustforce - d_thrust;
            }

            else if current_frame >= 553 && current_frame <= 900
            {
                //Flaps deflected
                fdm.element[1].i_flap = -1;
                fdm.element[2].i_flap = -1;
                fdm.flaps = true;
            }





        //Calculate all of the forces and moments on the airplane
        calc_airplane_loads(&mut fdm);

        //Calculate acceleration of airplane in earth space
        let ae: Myvec = Myvec::dividescalar(&fdm.v_forces, fdm.mass);

        //Calculate velocity of airplane in earth space
        fdm.v_velocity = Myvec::addvec(&fdm.v_velocity, &Myvec::multiplyscalar(&ae, dt)); 

        //Calculate position of airplane in earth space
        fdm.v_position = Myvec::addvec(&fdm.v_position, &Myvec::multiplyscalar(&fdm.v_velocity, dt));

        //Calculate angular velocity of airplane in body space
        let one = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia, &fdm.v_angular_velocity);
        let two = Myvec::crossproduct(&fdm.v_angular_velocity, &one);
        let three = Myvec::subtractvec(&fdm.v_moments, &two);
        let four = Mymatrix::multiply_matrix_by_vec(&fdm.m_inertia_inverse, &three);
        let five = Myvec::multiplyscalar(&four, dt);
        fdm.v_angular_velocity = Myvec::addvec(&fdm.v_angular_velocity, &five);

        //Calculate the new rotation quaternion
        let uno = Myquaternion::multiply_quat_by_vec(&fdm.q_orientation, &fdm.v_angular_velocity);
        let dos = Myquaternion::multiplyscalar(&uno, 0.5 * dt);
        fdm.q_orientation = Myquaternion::addquat(&fdm.q_orientation, &dos);

        //Now normalize the orientation quaternion (make into unit quaternion)
        let mag = fdm.q_orientation.magnitude();
        if mag != 0.0
        {
            fdm.q_orientation = Myquaternion::dividescalar(&fdm.q_orientation, mag);
        }

        //Calculate the velocity in body space
        fdm.v_velocity_body = Myquaternion::qvrotate(&Myquaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);

        //Calculate air speed
        fdm.f_speed = fdm.v_velocity.magnitude();

        //Get euler angles
        let euler = Myquaternion::make_euler_from_q(&fdm.q_orientation);
        fdm.v_euler_angles.x = euler.x;
        fdm.v_euler_angles.y = euler.y;
        fdm.v_euler_angles.z = euler.z; 

    }

    //Grab our test data results
    let flight_test_data = vec![fdm.v_position.x, fdm.v_position.y, fdm.v_position.z, fdm.v_euler_angles.x, -fdm.v_euler_angles.y, fdm.v_euler_angles.z, fdm.f_speed /1.688]; //Pitch is negated to represent positive pitch as up
    
    //Copy over the data from the c++ benchmark tests
    //let benchmark_data = vec![-1045.37,-0.0444084, 421.399,-0.00279299,-16.1436,-0.00508352, 85.9906]; //TEST 1 (NOTHING)
    //let benchmark_data = vec![-4794.57, -1924.94, 482.128, 19.3067, -4.04701, -168.199, 100.042]; //TEST 2 (ROLL)
    //let benchmark_data = vec![-2093.35, -3.95869, 2451.71, 0.404863, 34.7533, -0.581428, 51.8425]; //TEST 3 (PITCH)
    //let benchmark_data = vec![-2033.18, -1985.34, 1486.51, 15.1557, -4.85586, -76.4714, 94.558]; //TEST 4 (YAW)
    //let benchmark_data = vec![-1995.63, -0.401624, 1778.12, 0.0124067, 9.38473, -0.0325167, 57.0701]; //TEST 5 (FLAPS)
    let benchmark_data = vec![-2708.89, 59.6766, 540.036, 107.562, 64.6713, -105.868, 59.8944]; //TEST 6 (EVERYTHING)

    println!("Rust/ECS Flight Data  : {:?}", flight_test_data);
    println!("C++ Benchmark         : {:?}", benchmark_data);
   
    //7 Variables to check for each of the 6 tests
    let cmp1 = approx_eq!(f32, flight_test_data[0], benchmark_data[0], epsilon = 0.01); //Pos x
    let cmp2 = approx_eq!(f32, flight_test_data[1], benchmark_data[1], epsilon = 0.01); //Pos y
    let cmp3 = approx_eq!(f32, flight_test_data[2], benchmark_data[2], epsilon = 0.01); //Pos z
    let cmp4 = approx_eq!(f32, flight_test_data[3], benchmark_data[3], epsilon = 0.01); //Roll
    let cmp5 = approx_eq!(f32, flight_test_data[4], benchmark_data[4], epsilon = 0.01); //Pitch
    let cmp6 = approx_eq!(f32, flight_test_data[5], benchmark_data[5], epsilon = 0.01); //Yaw
    let cmp7 = approx_eq!(f32, flight_test_data[6], benchmark_data[6], epsilon = 0.01); //Airspeed

    //If all comparisons are within the epsilon, return true
    if cmp1 == true && cmp2 == true  && cmp3 == true && cmp4 == true && cmp5 == true && cmp6 == true && cmp7 == true
    {
        assert!(true);
    }
    else 
    {
        assert!(false);
    }

        
}




