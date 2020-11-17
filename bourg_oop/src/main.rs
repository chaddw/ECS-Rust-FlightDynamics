
//Vector, Matrix, Quaternion module
mod common;

#[derive(Debug)]
struct PointMass
{
    f_mass: f64,
    v_d_coords: common::Myvec,
    v_local_inertia: common::Myvec,
    f_incidence: f64,
    f_dihedral: f64,
    f_area: f64,
    i_flap: i32,
    v_normal: common::Myvec,
    v_cg_coords: common::Myvec
}

#[derive(Debug, Default)]
struct RigidBody
{
    mass: f64,                                  //total mass
    m_inertia: common::Mymatrix,
    m_inertia_inverse: common::Mymatrix,
    v_position: common::Myvec,                  // position in earth coordinates
    v_velocity: common::Myvec,                  // velocity in earth coordinates
    v_velocity_body: common::Myvec,             // velocity in body coordinates
    v_angular_velocity: common::Myvec,          // angular velocity in body coordinates
    v_euler_angles: common::Myvec,   
    f_speed: f64,                               // speed (magnitude of the velocity)
    stalling: bool,
    flaps: bool,
    q_orientation: common::Myquaternion,        // orientation in earth coordinates 
    v_forces: common::Myvec,                    // total force on body
    thrustforce: f64,                           //magnitude of thrust
    v_moments: common::Myvec,                   // total moment (torque) on body
    element: Vec<PointMass>,                    //vector of point mass elements
    frame_count: f64 
}


impl RigidBody
{
    //quicker way to initialize...
    // fn new() -> RigidBody 
    // {
    //     Self
    //     {

           

    //         mass: 0.0,  

    //        // mmInertia: [[0.0; 3]; 3], //3x3 matrix... was using this before nalgebra

    //        //nalgebra approach
    //         m_inertia: Matrix3::new(0.0, 0.0, 0.0,
    //                                 0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0),
    //         m_inertia_inverse: Matrix3::new(0.0, 0.0, 0.0,
    //                                 0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0),

    //         //set initial position
    //         v_position: Vector3::new(-5000.0, 0.0, 200.0),

    //         //set initial velocity
    //         v_velocity: Vector3::new(60.0, 0.0, 0.0),

    //         v_euler_angles: Vector3::new(0.0, 0.0,0.0), //not defined in book here

    //         f_speed: 60.0,

    //         //set angular velocity
    //         v_angular_velocity: Vector3::new(0.0, 0.0, 0.0),

    //         //set initial thrust, forces, and moments
    //         v_forces: Vector3::new(500.0, 0.0, 0.0),
    //         thrustforce: 500.0,   //this isnt written in the rigid body intiialization for some reason...

    //         v_moments: Vector3::new(0.0, 0.0, 0.0),

    //         //zero the velocity in body space coordinates
    //         v_velocity_body: Vector3::new(0.0, 0.0, 0.0),

    //         //set these to false at first, will control later with keyboard... these are not defined in the structure
    //         stalling: false,
    //         flaps: false,

    //         //set initial orientation
    //         //q_orientation: MakeQFromEulerAngles(0.0, 0.0, 0.0),  
    //         //q_orientation: Quaternion{n: 0.0, v: Vector{x: 0.0, y: 0.0, z: 0.0}}, //default value for now

    //         //nalgebra approach
    //         q_orientation: Quaternion::new(0.0, 0.0, 0.0, 0.0), //from_euler_angles(0.0, 0.0, 0.0),
    //        q_orientation_unit: UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),


    //         //8 elements of the plane taken into account
    //         element: vec![
    //             PointMass{f_mass: 6.56, v_d_coords: Vector3::new(14.5, 12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 7.31, v_d_coords: Vector3::new(14.5, 5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 7.31, v_d_coords: Vector3::new(14.5, -5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 6.56, v_d_coords: Vector3::new(14.5, -12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 2.62, v_d_coords: Vector3::new(3.03, 2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 2.62, v_d_coords: Vector3::new(3.03, -2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 2.93, v_d_coords: Vector3::new(2.25, 0.0, 5.0), v_local_inertia: Vector3::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
    //             PointMass{f_mass: 31.8, v_d_coords: Vector3::new(15.25, 0.0, 1.5), v_local_inertia: Vector3::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) }
    //             ]

           
    //     }

    // }

    fn calc_airplane_mass_properties(&mut self)
    {
        //(element pieces are initialized when the airplane entity is initialized in main)

        let mut inn: f64;
        let mut di: f64;

        //Calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
        for  i in self.element.iter_mut()
        {
            inn = (i.f_incidence).to_radians();
            di = (i.f_dihedral).to_radians();
            i.v_normal = common::Myvec::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
            i.v_normal.normalize(); 
        }

        //Calculate total mass
        let mut total_mass: f64 = 0.0;
        for i in self.element.iter()
        {
            total_mass = total_mass + i.f_mass;
        }

        //Calculate combined center of gravity location
        let mut v_moment = common::Myvec::new(0.0,0.0,0.0);
        for i in self.element.iter()
        {
            let tmp = common::Myvec::multiplyscalar(&i.v_d_coords, i.f_mass);
            
            v_moment = common::Myvec::addvec(&v_moment, &tmp);
        }
        let cg = common::Myvec::dividescalar(&v_moment, total_mass); 

        //Calculate coordinates of each element with respect to the combined CG, relative position
        for i in self.element.iter_mut()
        {
            i.v_cg_coords = common::Myvec::subtractvec(&i.v_d_coords, &cg);
        }

        //Calculate the moments and products of intertia for the combined elements
        let mut ixx: f64 = 0.0;
        let mut iyy: f64 = 0.0;
        let mut izz: f64 = 0.0;
        let mut ixy: f64 = 0.0;
        let mut ixz: f64 = 0.0;
        let mut iyz: f64 = 0.0;

        for i in self.element.iter()
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
        self.mass = total_mass;
        self.m_inertia = common::Mymatrix::new(ixx, -ixy, -ixz,
                                               -ixy, iyy, -iyz,
                                               -ixz, -iyz, izz);
        //Get inverse of matrix
        self.m_inertia_inverse = common::Mymatrix::inverse(&self.m_inertia);
    }

    //calculates all of the forces and moments on the plane at any time
    fn calc_airplane_loads(&mut self)
    {

        let mut fb = common::Myvec::new(0.0, 0.0, 0.0); //total force
        let mut mb = common::Myvec::new(0.0, 0.0, 0.0); //total moment
    
        //Reset forces and moments
        self.v_forces = common::Myvec::new(0.0, 0.0, 0.0);
        self.v_moments = common::Myvec::new(0.0, 0.0, 0.0);
    
        //Define thrust vector, which acts through the plane's center of gravity
        let mut thrust = common::Myvec::new(1.0, 0.0, 0.0);
        thrust = common::Myvec::multiplyscalar(&thrust, self.thrustforce);
    
        //Calculate forces and moments in body space
        let mut v_drag_vector = common::Myvec::new(0.0, 0.0, 0.0);
        let mut v_resultant= common::Myvec::new(0.0, 0.0, 0.0);
    
        self.stalling = false;
    
        //Loop through the 7 lifting elements, skipping the fuselage
        for i in 0..8 
        {
            if i == 6 //Tail rudder. its a special case because it can rotate, so the normal vector is recalculated
            {
                let inn: f64 = (self.element[i].f_incidence).to_radians();
                let di: f64 = (self.element[i].f_dihedral).to_radians();
                self.element[i].v_normal = common::Myvec::new(inn.sin(),
                                                              inn.cos() * di.sin(), 
                                                              inn.cos() * di.cos());
                self.element[i].v_normal.normalize();
            }
           
            //Calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
            let mut vtmp = common::Myvec::crossproduct(&self.v_angular_velocity, &self.element[i].v_cg_coords);
            let v_local_velocity = common::Myvec::addvec(&self.v_velocity_body, &vtmp);
    
            //Calculate local air speed
            let f_local_speed: f64 = self.v_velocity_body.magnitude(); 
    
            //Find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
            if f_local_speed > 1.0
            {
                let v_local_vel_tmp = common::Myvec::reverse_aka_conjugate(&v_local_velocity); //-vLocalVelocity
                v_drag_vector = common::Myvec::dividescalar(&v_local_vel_tmp, f_local_speed);
            }
    
            //Find direction that lift will act. lift is perpendicular to the drag vector
            let lift_tmp = common::Myvec::crossproduct(&v_drag_vector, &self.element[i].v_normal);
            let mut v_lift_vector = common::Myvec::crossproduct(&lift_tmp, &v_drag_vector);
            let mut tmp = v_lift_vector.magnitude(); 
            v_lift_vector.normalize();
      
            //Find the angle of attack. its the angle between the lift vector and element normal vector 
            tmp = common::Myvec::dotproduct(&v_drag_vector, &self.element[i].v_normal);
    
            if tmp > 1.0
            {
                tmp = 1.0;
            }
            if tmp < -1.0
            {
                tmp = -1.0;
            }
    
            let f_attack_angle: f64 = tmp.asin().to_degrees();
    
            //Determine lift and drag force on the element (rho is defined as 0.002376900055)
            tmp = 0.5 * 0.002376900055 * f_local_speed * f_local_speed * self.element[i].f_area;   
    
            if i == 6 //tail/ rudder
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
                let firstpart = common::Myvec::multiplyscalar(&v_lift_vector, lift_coefficient(f_attack_angle, self.element[i].i_flap));
                let secondpart = common::Myvec::multiplyscalar(&v_drag_vector, drag_coefficient(f_attack_angle, self.element[i].i_flap));
                let addtogether = common::Myvec::addvec(&firstpart, &secondpart);
                v_resultant = common::Myvec::multiplyscalar(&addtogether, tmp);
            }
    
            //Check for stall. if the coefficient of lift is 0, stall is occuring.
            if i <= 3
            {
                if lift_coefficient(f_attack_angle, self.element[i].i_flap) == 0.0
                {
                    self.stalling = true; 
                }
            }
    
            //Keep running total of resultant forces (total force)
            fb = common::Myvec::addvec(&fb, &v_resultant);
    
            //Calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
            vtmp = common::Myvec::crossproduct(&self.element[i].v_cg_coords, &v_resultant);
            mb = common::Myvec::addvec(&mb, &vtmp);
        }
    
        //Add thrust
        fb = common::Myvec::addvec(&fb, &thrust);
    
        //Convert forces from model space to earth space. rotates the vector by the unit quaternion (QVRotate function)
        self.v_forces = common::Myquaternion::qvrotate(&self.q_orientation, &fb);
    
        //Apply gravity (g is -32.174 ft/s^2), ONLY APPLY WHEN ALTITUDE IS GREATER THAN ZERO
        self.v_forces.z = self.v_forces.z + (-32.17399979) * self.mass;
    
        self.v_moments = common::Myvec::addvec(&self.v_moments, &mb);
    }

     //equations of motion / update
     fn step_simulation(&mut self, dt: f64)
     {
        //Calculate all of the forces and moments on the airplane
        self.calc_airplane_loads();

        //Calculate acceleration of airplane in earth space
        let ae: common::Myvec = common::Myvec::dividescalar(&self.v_forces, self.mass);

        //Calculate velocity of airplane in earth space
        let ae_mult_dt_tmp = common::Myvec::multiplyscalar(&ae, dt);
        self.v_velocity = common::Myvec::addvec(&self.v_velocity, &ae_mult_dt_tmp); 

        //Calculate position of airplane in earth space
        let vel_mult_dt_tmp = common::Myvec::multiplyscalar(&self.v_velocity, dt);
        self.v_position = common::Myvec::addvec(&self.v_position, &vel_mult_dt_tmp); //add the degrees on lat/lon/and meters (model uses feet for this)

        //Calculate angular velocity of airplane in body space
        let one = common::Mymatrix::multiply_matrix_by_vec(&self.m_inertia, &self.v_angular_velocity);
        let two = common::Myvec::crossproduct(&self.v_angular_velocity, &one);
        let three = common::Myvec::subtractvec(&self.v_moments, &two);
        let four = common::Mymatrix::multiply_matrix_by_vec(&self.m_inertia_inverse, &three);
        let five = common::Myvec::multiplyscalar(&four, dt);
        self.v_angular_velocity = common::Myvec::addvec(&self.v_angular_velocity, &five);

        //Calculate the new rotation quaternion
        let uno = common::Myquaternion::multiply_quat_by_vec(&self.q_orientation, &self.v_angular_velocity);
        let dos = common::Myquaternion::multiplyscalar(&uno, 0.5 * dt);
        self.q_orientation = common::Myquaternion::addquat(&self.q_orientation, &dos);

        //Now normalize the orientation quaternion (make into unit quaternion)
        let mag = self.q_orientation.magnitude();
        if mag != 0.0
        {
            self.q_orientation = common::Myquaternion::dividescalar(&self.q_orientation, mag);
        }

        //Calculate the velocity in body space:
        self.v_velocity_body = common::Myquaternion::qvrotate(&common::Myquaternion::conjugate(&self.q_orientation), &self.v_velocity);

        //Calculate air speed
        self.f_speed = self.v_velocity.magnitude(); 

        //Get euler angles for our info
        let euler = common::Myquaternion::make_euler_from_q(&self.q_orientation);
        //THESE ARENT SUPPOSED TO BE MADE NEGATIVE BUT IT FIXES ALL THE ISSUES, not sure where that went wrong in the model...
        self.v_euler_angles.x = euler.x;
        self.v_euler_angles.y = -euler.y; 
        self.v_euler_angles.z = -euler.z;


     }

    //flight controls

    //thrust
    fn inc_thrust(&mut self)
    {
        if self.thrustforce < MAX_THRUST
        {
            self.thrustforce = self.thrustforce + D_THRUST;
        }  
    }
    fn dec_thrust(&mut self)
    {
        if self.thrustforce > 0.0
        {
            self.thrustforce = self.thrustforce - D_THRUST;
        } 
    }

    //yaw
    fn left_rudder(&mut self)
    {
        self.element[6].f_incidence = 16.0;
    }
    fn right_rudder(&mut self)
    {
        self.element[6].f_incidence = -16.0;
    }
    fn zero_rudder(&mut self)
    {
        self.element[6].f_incidence = 0.0;
    }

    //roll
    fn roll_left(&mut self)
    {
        self.element[0].i_flap = 1;
        self.element[3].i_flap = -1;
    }
    fn roll_right(&mut self)
    {
        self.element[0].i_flap = -1;
        self.element[3].i_flap = 1;
    }
    fn zero_ailerons(&mut self)
    {
        self.element[0].i_flap = 0;
        self.element[3].i_flap = 0;
    }

    //pitch
    fn pitch_up(&mut self)
    {
        self.element[4].i_flap = 1;
        self.element[5].i_flap = 1;
    }
    fn pitch_down(&mut self)
    {
        self.element[4].i_flap = -1;
        self.element[5].i_flap = -1;
    }
    fn zero_elevators(&mut self)
    {
        self.element[4].i_flap = 0;
        self.element[5].i_flap = 0;
    }

    //flaps
    fn flaps_down(&mut self)
    {
        self.element[1].i_flap = -1;
        self.element[2].i_flap = -1;
        self.flaps = true;
    }
    fn zero_flaps(&mut self)
    {
        self.element[1].i_flap = 0;
        self.element[2].i_flap = 0;
        self.flaps = false;
    }
} //end impl

//set frame rate, which will set the delta time
static FRAME_RATE: f64 = 30.0; //0.016; //THE C++ EXAMPLE ALWAYS RUNS AT 1000 FPS CUZ IT LOOPS SO FAST


//set thrust parameters
static D_THRUST: f64 = 100.0;
static MAX_THRUST: f64 = 3000.0;
fn main()
{
    //Intialize the airplane
    //make default values, and fill in what we need at the start, this will be passed to the entity
    let mut myairplane = RigidBody{..Default::default()};
    //c++ program start position
    myairplane.v_position.x = -5000.0;
    myairplane.v_position.y =  0.0;
    myairplane.v_position.z =  2000.0; 

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
    myairplane.q_orientation = common::Myquaternion::make_q_from_euler(0.0, 0.0, 0.0);

    myairplane.calc_airplane_mass_properties(); 
    
    //create objects to be processed
    // let mut objects: Vec<&RigidBody> = Vec::new();
    // for _ in 0..1000 //loop
    // {
    //     objects.push(&myairplane);
    // }


    for _ in 0..1000 //loop
    {

        let dt: f64 = 1.0 / FRAME_RATE;
        //zero everything (this isnt actually needed if we only press 1 key consistently for each frame. this certaintly needed for flightgear simulation)
        myairplane.zero_ailerons();
        myairplane.zero_elevators();
        myairplane.zero_rudder();
        myairplane.zero_flaps(); 

        //flight meanuever goes here
        //myairplane.inc_thrust();

        //update
        myairplane.step_simulation(dt);

        //Print some relevant data
        println!("time:             {}, frames: {}", myairplane.frame_count * dt, myairplane.frame_count);
        println!("roll:             {}", myairplane.v_euler_angles.x);
        println!("pitch:            {}", myairplane.v_euler_angles.y); //c++ has this as negative to make pitching down (negative) to make more sense, but i believe flightgear wants to see this as negative anyway
        println!("yaw:              {}", myairplane.v_euler_angles.z);
        println!("alt:              {}", myairplane.v_position.z);
        println!("thrus:            {}", myairplane.thrustforce);
        println!("speed:            {}", myairplane.f_speed/1.688 );
        println!("pos x:            {}", myairplane.v_position.x);
        println!("pos y:            {}", myairplane.v_position.y);
        println!("pos z:            {}", myairplane.v_position.z);
        println!("{}", "====================================================");

        myairplane.frame_count = myairplane.frame_count + 1.0;

    }

    //println!("{:?}", &objects[69]);
}


//Functions to collect airfoil performance data:
//lift and drag coefficient data is given for a set of discrete attack angles, 
//so then linear interpolation is used to determine the coefficients for the 
//attack angle that falls between the discrete angles.

//Given the angle of attack and status of the flaps,
//return lift angle coefficient for camabred airfoil with 
//plain trailing-edge (+/- 15 degree inflation).
fn lift_coefficient(angle: f64, flaps: i32) -> f64
{
    let clf0 = vec![-0.54, -0.2, 0.2, 0.57, 0.92, 1.21, 1.43, 1.4, 1.0]; 
    let clfd = vec![0.0, 0.45, 0.85, 1.02, 1.39, 1.65, 1.75, 1.38, 1.17];
    let clfu = vec![-0.74, -0.4, 0.0, 0.27, 0.63, 0.92, 1.03, 1.1, 0.78];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cl: f64 = 0.0;

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
fn drag_coefficient(angle: f64, flaps: i32) -> f64
{
    let cdf0 = vec![0.01, 0.0074, 0.004, 0.009, 0.013, 0.023, 0.05, 0.12, 0.21];
    let cdfd = vec![0.0065, 0.0043, 0.0055, 0.0153, 0.0221, 0.0391, 0.1, 0.195, 0.3];
    let cdfu = vec![0.005, 0.0043, 0.0055, 0.02601, 0.03757, 0.06647, 0.13, 0.1, 0.25];
    let a = vec![-8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f64 = 0.75; //0.5 in book but 0.75 in actual code

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
fn rudder_lift_coefficient(angle: f64) -> f64
{
    let clf0 = vec![0.16, 0.456, 0.736, 0.968, 1.144, 1.12, 0.8];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cl: f64 = 0.0;
    let aa: f64 = angle.abs();

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
fn rudder_drag_coefficient(angle: f64) -> f64
{
    let cdf0 = vec![0.0032, 0.0072, 0.0104, 0.0184, 0.04, 0.096, 0.168];
    let a = vec![0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0];

    let mut cd: f64 = 0.75; //0.5 in book
    let aa: f64 = angle.abs();

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
