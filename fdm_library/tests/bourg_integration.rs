//Integration tests for the Bourg-based Flight Dynamics Model

//To run on the command line: cargo test --test bourg_integration

//Float_cmp crate for comparing floats
use float_cmp::*;

//Bring into scope the functions and data as necessary for testing
use fdm_library::bourg::common::vector::Vector;
use fdm_library::bourg::common::quaternion::Quaternion;
use fdm_library::bourg::fdm::structures::DataFDM;
use fdm_library::bourg::fdm::structures::PointMass;
use fdm_library::bourg::fdm::calc_loads::calc_airplane_loads;
use fdm_library::bourg::fdm::mass_properties::calc_airplane_mass_properties;
use fdm_library::bourg::fdm::keypresses::*;

#[test]
fn fdm_test() 
{
    let fps = 30.0;
    let dt = 1.0 / fps;
    let d_thrust = 100.0;

    //Create airplane to compute mass properties
    let mut fdm = DataFDM{ 
        element : vec![
            PointMass{f_mass: 6.56, v_d_coords: Vector::new(14.5, 12.0, 2.5), v_local_inertia: Vector::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector::new(14.5, 5.5, 2.5), v_local_inertia: Vector::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector::new(14.5, -5.5, 2.5), v_local_inertia: Vector::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 6.56, v_d_coords: Vector::new(14.5, -12.0, 2.5), v_local_inertia: Vector::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector::new(3.03, 2.5, 3.0), v_local_inertia: Vector::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector::new(3.03, -2.5, 3.0), v_local_inertia: Vector::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.93, v_d_coords: Vector::new(2.25, 0.0, 5.0), v_local_inertia: Vector::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 31.8, v_d_coords: Vector::new(15.25, 0.0, 1.5), v_local_inertia: Vector::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) }
        ], 
    
        
    //Define initial flight parameters
    v_position: Vector{x: -5000.0, y: 0.0, z: 2000.0},
    v_velocity: Vector{x: 60.0, y: 0.0, z: 0.0},
    f_speed: 60.0,
    v_forces: Vector{x: 500.0, y: 0.0, z: 0.0},
    thrustforce: 500.0,
    q_orientation: Quaternion::make_q_from_euler(0.0, 0.0, 0.0),

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
        zero_rudder(&mut fdm);
        zero_ailerons(&mut fdm);
        zero_elevators(&mut fdm);
        //Flaps will be toggled on and off so flaps does not need to be zerod each time

            //TESTS WILL STIMULATE THE AIRPLANE'S COMPONENTS FOR A GIVEN FRAME

            //---------------------------------------
            //TEST 1 (no flight control)

            // //TEST 2 Roll 
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     thrust_up(&mut fdm, d_thrust);
            // }
            // else if current_frame >= 6 && current_frame <= 246
            // {
            //     pitch_up(&mut fdm);
            // }
            // else if current_frame >= 247 && current_frame <= 307 
            // {
            //     roll_right(&mut fdm);
            // }


            // //TEST 3 Pitch
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     thrust_up(&mut fdm, d_thrust);
            // }
            // pitch_up(&mut fdm);


            // //TEST 4 Yaw
            // if current_frame >= 1 && current_frame <= 5
            // {
            //     thrust_up(&mut fdm, d_thrust);
            // }
            // else if current_frame >= 6 && current_frame <= 246 
            // {
            //     pitch_up(&mut fdm);
            // }
            // else if current_frame >= 247 && current_frame <= 307
            // {
            //     yaw_right(&mut fdm);
            // }


            // //TEST 5 Flaps
            // flaps_down(&mut fdm);


            //TEST 6 EVERYTHING
            if current_frame >= 1 && current_frame <= 5
            {
                thrust_up(&mut fdm, d_thrust);
            }
            else if current_frame >= 6 && current_frame <= 246 
            {
                pitch_up(&mut fdm);
            }
            else if current_frame >= 247 && current_frame <= 307
            {
                yaw_left(&mut fdm);
            }
            else if current_frame >= 308 && current_frame <= 368
            {
                yaw_right(&mut fdm);
            }
            else if current_frame >= 369 && current_frame <= 429
            {
                roll_right(&mut fdm);
            }
            else if current_frame >= 430 && current_frame <= 490
            {
                roll_left(&mut fdm);
            }
            else if current_frame >= 491 && current_frame <= 505
            {
                pitch_down(&mut fdm);
            }
            else if current_frame >= 506 && current_frame <= 511
            {
                thrust_down(&mut fdm, d_thrust);
            }
            else if current_frame >= 512 && current_frame <= 900
            {
                flaps_down(&mut fdm);
            }

        //Calculate all of the forces and moments on the airplane
        calc_airplane_loads(&mut fdm);

        //Calculate acceleration of airplane in earth space
        let ae: Vector = fdm.v_forces / fdm.mass;

        //Calculate velocity of airplane in earth space
        fdm.v_velocity = fdm.v_velocity + ae * dt; 

        //Calculate position of airplane in earth space
        fdm.v_position = fdm.v_position + fdm.v_velocity * dt;

        //Calculate angular velocity of airplane in body space
        fdm.v_angular_velocity = fdm.v_angular_velocity + ((fdm.m_inertia_inverse * 
            (fdm.v_moments - Vector::crossproduct(&fdm.v_angular_velocity, &(fdm.m_inertia * fdm.v_angular_velocity)))) * dt);

        //Calculate the new rotation quaternion
        fdm.q_orientation = fdm.q_orientation + (fdm.q_orientation * fdm.v_angular_velocity) * (0.5 * dt);

        //Now normalize the orientation quaternion (make into unit quaternion)
        let mag = fdm.q_orientation.magnitude();
        if mag != 0.0
        {
            fdm.q_orientation = fdm.q_orientation / mag;
        }

        //Calculate the velocity in body space
        fdm.v_velocity_body = Quaternion::qvrotate(&Quaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);

        //Calculate air speed
        fdm.f_speed = fdm.v_velocity.magnitude();

        //Get euler angles
        let euler = Quaternion::make_euler_from_q(&fdm.q_orientation);
        fdm.v_euler_angles.x = euler.x;
        fdm.v_euler_angles.y = euler.y;
        fdm.v_euler_angles.z = euler.z; 

    }

    //Grab our test data results
    let flight_test_data = vec![fdm.v_position.x, fdm.v_position.y, fdm.v_position.z,
                                fdm.v_euler_angles.x, -fdm.v_euler_angles.y, fdm.v_euler_angles.z, 
                                fdm.f_speed /1.688]; //Pitch is negated to represent positive pitch as up
    
    //Copy over the data from the c++ benchmark tests
    //TEST 1 (NOTHING)
    //let benchmark_data = vec![-1045.37,-0.0444084, 421.399,-0.00279299,-16.1436,-0.00508352, 85.9906]; 

    //TEST 2 (ROLL)
    //let benchmark_data = vec![-4794.57, -1924.94, 482.128, 19.3067, -4.04701, -168.199, 100.042]; 

    //TEST 3 (PITCH)
    //let benchmark_data = vec![-2093.35, -3.95869, 2451.71, 0.404863, 34.7533, -0.581428, 51.8425]; 

    //TEST 4 (YAW)
    //let benchmark_data = vec![-2033.18, -1985.34, 1486.51, 15.1557, -4.85586, -76.4714, 94.558]; 

    //TEST 5 (FLAPS)
    //let benchmark_data = vec![-1995.63, -0.401624, 1778.12, 0.0124067, 9.38473, -0.0325167, 57.0701]; 

    //TEST 6 (EVERYTHING)
    let benchmark_data = vec![-2360.21, 322.768, 587.561, -0.994568, -17.7002, 17.7958, 101.768]; 
    
    

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




