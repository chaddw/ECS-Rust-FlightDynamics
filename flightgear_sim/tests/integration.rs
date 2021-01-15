


    #[test]
    fn fdm_test() //Can test one flight data variable of the entire FDM at a time. Must change frame #, and get that value to compare from C++ output. Can also add flight controls artificially
    {
        let dt = 1.0 / 30.0;
        let d_thrust = 100.0;

        //create airplane to compute mass properties
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
        for _ in 0..5 //frame #
        {
            current_frame = current_frame + 1;

            //Can add flight control artificially to test
            //Roll test parameters
            if current_frame >= 1 && current_frame <= 5
            {
                //increase thrust
                fdm.thrustforce = fdm.thrustforce + d_thrust;
            }
            else if current_frame >= 6 && current_frame <= 246 //pitch up 8 seconds
            {
                //pitch up
                fdm.element[4].i_flap = 1;
                fdm.element[5].i_flap = 1;
            }
            else if current_frame >= 247 && current_frame <= 307 //roll right for 2 seconds
            {
                //roll right
                fdm.element[0].i_flap = -1;
                fdm.element[3].i_flap = 1;
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

        //C++ value to match

        //with no roll
        //let equal = 1999.9642333984375; //altitude (first frame)
        //let equal = 1999.899169921875; //altitude (second frame)
        //let equal = 1999.8056640625; //altitude (third frame) 
        //let equal = 1973.194580078125; //altitude (frame 50) 

        //With roll
        //let equal = 1999.9642333984375; //altitude (frame 1) 
        //let equal = 1999.899169921875; //altitude (frame 2) 
        //let equal =  1999.8056640625; //altitude (frame 3) 
        //let equal =  1999.6846923828125; //altitude (frame 4) 
        let equal =  1999.537109375; //altitude (frame 5) 



        //change variable as needed for testing
         assert_eq!(fdm.v_position.z, equal);
         
    }
   

   

