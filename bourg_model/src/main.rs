//use std::ops::{Add, Sub, Mul, Div}; //operator overload
extern crate nalgebra as na;
//use na::*; 
use na::{Matrix3, Vector3, UnitQuaternion, Unit, Quaternion}; //took off unit


//imports for flight function
#[macro_use]
extern crate crossterm;
use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, Clear, ClearType};
use std::io::{stdout, Write};

use std::thread;


//rigid body structure to encapsulate the required data during the simulation
#[derive(Debug)] //nalgebra does not like default
struct RigidBody
{
    mass: f32, //total mass

    //mmInertia: [[f32;3];3],      // = [[0.0; 3]; 3], //mass moment of inertia
   // mInertiaInverse:[[f32;3];3],// inverse of mass moment in inertia
    
    //Matrix3x3 mIntertia;      //mass moment of inertia
    //Matrix3x3 mInertiaInverse;// inverse of mass moment in inertia
    m_inertia: Matrix3<f32>,
    m_inertia_inverse: Matrix3<f32>,
    //mInertia: Matrix3<f32, na::U3, na::U3, na::ArrayStorage<f32, na::U3, na::U3>>,
   // mInertiaInverse: Matrix3<f32, na::U3, na::U3, na::ArrayStorage<f32, na::U3, na::U3>>,


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


impl RigidBody
{
    //quicker way to initialize...
    fn new() -> RigidBody 
    {
        Self
        {

            //but this seems like a bunch of default values except for vposition and v_velocity, v_forces, ... maybe dont need this new method and can just set defaults in main

            mass: 0.0,  

           // mmInertia: [[0.0; 3]; 3], //3x3 matrix... was using this before nalgebra

           //nalgebra approach
            m_inertia: Matrix3::new(0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0),
            m_inertia_inverse: Matrix3::new(0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0),

            //set initial position
            v_position: Vector3::new(-5000.0, 0.0, 200.0),

            //set initial velocity
            v_velocity: Vector3::new(60.0, 0.0, 0.0),

            v_euler_angles: Vector3::new(0.0, 0.0,0.0), //not defined in book here

            f_speed: 60.0,

            //set angular velocity
            v_angular_velocity: Vector3::new(0.0, 0.0, 0.0),

            //set initial thrust, forces, and moments
            v_forces: Vector3::new(500.0, 0.0, 0.0),
            thrustforce: 500.0,   //this isnt written in the rigid body intiialization for some reason...

            v_moments: Vector3::new(0.0, 0.0, 0.0),

            //zero the velocity in body space coordinates
            v_velocity_body: Vector3::new(0.0, 0.0, 0.0),

            //set these to false at first, will control later with keyboard... these are not defined in the structure
            stalling: false,
            flaps: false,

            //set initial orientation
            //q_orientation: MakeQFromEulerAngles(0.0, 0.0, 0.0),  
            //q_orientation: Quaternion{n: 0.0, v: Vector{x: 0.0, y: 0.0, z: 0.0}}, //default value for now

            //nalgebra approach
            q_orientation: Quaternion::new(0.0, 0.0, 0.0, 0.0), //from_euler_angles(0.0, 0.0, 0.0),
           q_orientation_unit: UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),


            //8 elements of the plane taken into account
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

           
        }

    }

    fn calc_airplane_mass_properties(&mut self)
    {
        println!("{}", "calculating mass properties...");
        let mut inn: f32;
        let mut di: f32;

        //calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
        for  i in self.element.iter_mut()
        {
            inn = (i.f_incidence).to_radians();
            di = (i.f_dihedral).to_radians();
            i.v_normal = Vector3::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
            i.v_normal = i.v_normal.normalize();
        }

        //calculate total mass
        let mut total_mass: f32 = 0.0;
        for i in self.element.iter()
        {
            total_mass = total_mass + i.f_mass;
        }
        //println!("Total mass: {}", total_mass);

        //calculate combined center of gravity location
        let mut first_moment_x: f32 = 0.0;
        let mut first_moment_y: f32 = 0.0;
        let mut first_moment_z: f32 = 0.0;
        for i in self.element.iter()
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
        for i in self.element.iter_mut()
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
    
        //finally, set up airplanes mass and inertia matrix
        self.mass = total_mass;

        //this was setting up matrix with 3x3 array
        // self.mmInertia[0][0] = Ixx;
        // self.mmInertia[0][1] = -Ixy;
        // self.mmInertia[0][2] = -Ixz;
    
        // self.mmInertia[1][0] = -Ixy;
        // self.mmInertia[1][1] = Iyy;
        // self.mmInertia[1][2] = -Iyz;
    
        // self.mmInertia[2][0] = -Ixz;
        // self.mmInertia[2][1] = -Iyz;
        // self.mmInertia[2][2] = Izz;



        //using nalgebra matrix
        self.m_inertia = Matrix3::new(ixx, -ixy, -ixz,
                                     -ixy, iyy, -iyz,
                                     -ixz, -iyz, izz);

        //get inverse of matrix
        self.m_inertia_inverse = self.m_inertia.try_inverse().unwrap();

        
    }

    //calculates all of the forces and moments on the plane at any time
    fn calc_airplane_loads(&mut self)
    {
        //println!("{}", "calculating forces...");

        let mut fb = Vector3::new(0.0, 0.0, 0.0);
        let mut mb = Vector3::new(0.0, 0.0, 0.0);

        //reset forces and moments
        self.v_forces = Vector3::new(0.0, 0.0, 0.0);
        self.v_moments = Vector3::new(0.0, 0.0, 0.0);

        //define thrust vector, which acts throguh the plane's center of gravity
        let mut thrust = Vector3::new(1.0, 0.0, 0.0);
        thrust = thrust * self.thrustforce; 
  
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
                let inn: f32 = (self.element[i].f_incidence).to_radians();
                let di: f32 = (self.element[i].f_dihedral).to_radians();
                self.element[i].v_normal = Vector3::new(inn.sin(), inn.cos() * di.sin(), inn.cos() * di.cos());
                self.element[i].v_normal = self.element[i].v_normal.normalize();
            }

            //calculate local velocity at element. This includes the velocity due to linear motion of the airplane plus the velocity and each element due to rotation
            //rotation part

            vtmp = self.v_angular_velocity.cross(&self.element[i].v_cg_coords); //crossproduct

            v_local_velocity = self.v_velocity_body + vtmp;

            //calculate local air speed
            f_local_speed = self.v_velocity_body.magnitude();

            //find the direction that drag will act. it will be in line with the relative velocity but going in the opposite direction
            if f_local_speed > 1.0
            {
                v_drag_vector = -v_local_velocity / f_local_speed;
            }


            //find direction that lift will act. lift is perpendicular to the drag vector
            //(i think there is a problem with normalizing)
            v_lift_vector = (v_drag_vector.cross(&self.element[i].v_normal)).cross(&v_drag_vector);
            tmp = v_lift_vector.magnitude();
            v_lift_vector = v_lift_vector.normalize();
   
            //find the angle of attack. its the angle between the lfit vector and eelement normal vector. 
            tmp = v_drag_vector.dot(&self.element[i].v_normal);
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
            tmp = 0.5 * 1.225 * f_local_speed * f_local_speed * self.element[i].f_area;
            if i == 6 //tail rudder
            {
                v_resultant = (v_lift_vector * RigidBody::rudder_lift_coefficient(f_attack_angle) + v_drag_vector * RigidBody::rudder_drag_coefficient(f_attack_angle)) * tmp;
            }
            else
            {
                v_resultant = (v_lift_vector * RigidBody::lift_coefficient(f_attack_angle, self.element[i].i_flap) + v_drag_vector * RigidBody::drag_coefficient(f_attack_angle, self.element[i].i_flap)) * tmp;
            }

            //check for stall. if the coefficient of lift is 0, stall is occuring.
            if RigidBody::lift_coefficient(f_attack_angle, self.element[i].i_flap) == 0.0
            {
                self.stalling = true; //probably will need to add stalling to rigid body variables
            }

            //keep running total of resultant forces (total force)
            fb = fb + v_resultant;

            //calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
            vtmp = self.element[i].v_cg_coords.cross(&v_resultant);
            mb = mb + vtmp;
        }


        //add thrust
        fb = fb + thrust;

        //convert forces from model space to earth space. rotates the vector by the unit quaternion (QVRotate function)
        self.v_forces = self.q_orientation_unit.transform_vector(&fb);

        //apply gravity (g is -32.174 ft/s^2)
        self.v_forces.z = self.v_forces.z + (-32.174) * self.mass;

        self.v_moments = self.v_moments + mb;
    }


    //functions to collect airfoil performance data
    //lift and drag coefficient data is given for a set of discrete attack angles, so then linear interpolation is used to determine the coefficients for the attack angle that falls between the discrete angles
   
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




     fn step_simulation(&mut self, dt: f32)
     {
        let mut ae = Vector3::new(0.0, 0.0, 0.0);

        //calculate all of the forces and moments on the airplane
        self.calc_airplane_loads();

        //calculate acceleration of airplane in earth coordinates
        ae = self.v_forces / self.mass;

        //calculate velocity of airplane in earth coordinates
        self.v_velocity = self.v_velocity + ae * dt;

        //calculate position of airplane in earth coordinates
        self.v_position = self.v_position + self.v_velocity * dt;

        //handle rotations
        let mut mag: f32 = 0.0;

        //calculate angular velocity of airplane in body coordinates
        self.v_angular_velocity = self.v_angular_velocity + self. m_inertia_inverse * ( self.v_moments - ( self.v_angular_velocity.cross(&(self.m_inertia * self.v_angular_velocity)))) * dt;


        //calculate the new rotation quaternion

        //we need angular velocity to be in a quaternion form for the multiplication.... ( i think this gives anaccurate result...) because 
        //nalgebra wont let me multiply the vector3 of angular velocity with the unit quaternion ( or a regular quaternion)

        //so create Quaternion based on the angular velocity ( i hope this math works out properly given the work around with nalgbra...) if not ill have to do it by hand
        let qtmp =  Quaternion::new(0.0, self.v_angular_velocity.x, self.v_angular_velocity.y, self.v_angular_velocity.z);                    
        self.q_orientation = self.q_orientation + (self.q_orientation * qtmp) * (0.5 * dt); 

        //now normalize the orientation quaternion (make into unit quaternion)
        self.q_orientation_unit = UnitQuaternion::new_normalize(self.q_orientation);

        //calculate the velocity in body coordinates
        self.v_velocity_body = self.q_orientation_unit.transform_vector(&self.v_velocity);


        //calculate air speed
        self.f_speed = self.v_velocity.magnitude();

        //get euler angles for our info
        let euler = self.q_orientation_unit.euler_angles();
        self.v_euler_angles.x = euler.0; //roll
        self.v_euler_angles.y = euler.1; //pitch
        self.v_euler_angles.z = euler.2; //yaw

        //println!("{}", self.v_euler_angles);

     }



     //flight controls
     fn inc_thrust(&mut self)
     {
        self.thrustforce = self.thrustforce + d_thrust;
        if self.thrustforce > maxthrust
        {
            self.thrustforce = maxthrust;
        }
        //println!("{}","increase thrust");
     }
     fn dec_thrust(&mut self)
     {
        self.thrustforce = self.thrustforce - d_thrust;
        if self.thrustforce < 0.0
        {
            self.thrustforce = 0.0;
        }
       // println!("{}","decrease thrust");
     }

     fn left_rudder(&mut self)
     {
         self.element[6].f_incidence = 16.0;
        // println!("{}","left rudder");
     }
     fn right_rudder(&mut self)
     {
         self.element[6].f_incidence = -16.0;
        // println!("{}","right rudder");
     }
     fn zero_rudder(&mut self)
     {
         self.element[6].f_incidence = 0.0;
       //  println!("{}","zero rudder");
     }



     fn roll_left(&mut self)
     {
         self.element[0].i_flap = 1;
         self.element[3].i_flap = -1;
         //println!("{}","roll left");
     }
     fn roll_right(&mut self)
     {
         self.element[0].i_flap = -1;
         self.element[3].i_flap = 1;
       //  println!("{}","roll right");
     }
     fn zero_ailerons(&mut self)
     {
         self.element[0].i_flap = 0;
         self.element[3].i_flap = 0;
      //   println!("{}","zero ailerons");
     }



     fn pitch_up(&mut self)
     {
         self.element[4].i_flap = 1;
         self.element[5].i_flap = 1;
       //  println!("{}","pitch up");
     }
     fn pitch_down(&mut self)
     {
         self.element[4].i_flap = -1;
         self.element[5].i_flap = -1;
       //  println!("{}","pitch down");
     }
     fn zero_elevators(&mut self)
     {
         self.element[4].i_flap = 0;
         self.element[5].i_flap = 0;
        // println!("{}","zero elevators");
     }

     

     fn flaps_down(&mut self)
     {
         self.element[1].i_flap = -1;
         self.element[2].i_flap = -1;
         self.flaps = true;
       //  println!("{}","flaps down");
     }
     fn zero_flaps(&mut self)
     {
         self.element[1].i_flap = 0;
         self.element[2].i_flap = 0;
         self.flaps = false;
       //  println!("{}","zero flaps");
     }

     //use flight control input
     fn flight(&mut self)
     {
        //println!("{}", "Airplane is ready for motion...");

        let mut stdout = stdout();
        //going into raw mode
        enable_raw_mode().unwrap();
    
        //clearing the screen, going to top left corner and printing welcoming message
        execute!(stdout, Clear(ClearType::All), cursor::MoveTo(0, 0), Print("Fly me!")) .unwrap();
        //ctrl + q to exit, w = pitch down, s = pitch up, a = roll left, d = roll right, t = increase thrust, y = decrease thrust, z = yaw left, x = yaw right, landing flaps up = f, landing flaps down = g 
    
    
        let no_modifiers = KeyModifiers::empty();
    
        //key detection, this needs to happen asynchronously...
        loop {

 
            self.zero_rudder();
            self.zero_ailerons();
            self.zero_elevators();
            //going to top left corner
            execute!(stdout, cursor::MoveTo(0, 0)).unwrap();

            //matching the key
            match read().unwrap() //like a switch statement
            {
                //pitch down
                Event::Key(KeyEvent {
                    code: KeyCode::Char('w'),
                    modifiers: no_modifiers,
                }) => self.pitch_down(),

                //pitch up
                Event::Key(KeyEvent {
                    code: KeyCode::Char('s'),
                    modifiers: no_modifiers,
                }) => self.pitch_up(),

                //roll left
                Event::Key(KeyEvent {
                    code: KeyCode::Char('a'),
                    modifiers: no_modifiers,
                }) => self.roll_left(),

                //roll right
                Event::Key(KeyEvent {
                    code: KeyCode::Char('d'),
                    modifiers: no_modifiers,
                }) => self.roll_right(),

                //increase thrust
                    Event::Key(KeyEvent {
                    code: KeyCode::Char('t'),
                    modifiers: no_modifiers,
                }) => self.inc_thrust(),              

                //decrease thrust
                Event::Key(KeyEvent {
                    code: KeyCode::Char('y'),
                    modifiers: no_modifiers,
                }) => self.dec_thrust(),

                 //yaw left
                 Event::Key(KeyEvent {
                    code: KeyCode::Char('z'),
                    modifiers: no_modifiers,
                }) => self.left_rudder(),    

                //yaw right
                Event::Key(KeyEvent {
                    code: KeyCode::Char('x'),
                    modifiers: no_modifiers,
                }) => self.right_rudder(),

                 //landing flaps down
                 Event::Key(KeyEvent {
                    code: KeyCode::Char('g'),
                    modifiers: no_modifiers,
                }) => self.flaps_down(), 
                
                
                 //landing flaps up
                 Event::Key(KeyEvent {
                    code: KeyCode::Char('f'),
                    modifiers: no_modifiers,
                }) => self.zero_flaps(),               


                //quit
                Event::Key(KeyEvent {
                    code: KeyCode::Char('q'),
                    modifiers: KeyModifiers::CONTROL,
                }) => break,
                _ => (),
            }

            self.step_simulation(1.0);
        }
    

        //disabling raw mode
        disable_raw_mode().unwrap();

     }




} //end impl

static d_thrust: f32 = 100.0;
static maxthrust: f32 = 3000.0;
fn main()
{
              
    //create and initialize airplane
    let mut airplane = RigidBody::new();
    println!("{}", "Airplane initialized...");
    airplane.calc_airplane_mass_properties(); //would be nice to call this automatically in new, having some trouble with getting that to work rn...

    //begin flight loop
    airplane.flight();

   // println!("{:#?}", Airplane);


//     let mut myvec = Vector{x: 1.0, y: 2.0, z: 3.0};
//     println!("{:?}", myvec);
//     let mut myvec2 = Vector{x: 10.0, y: 20.0, z: 30.0};
//     println!("{:?}", myvec2);

//     let vecadd = myvec / 2.0;
//    println!("dividing vector1 by 2: {:?}", vecadd);


    //using static method to initialize... unsuccessful for now
    // let myquaternion = Quaternion::newQuaternion(10.0, vec![0.0, 0.0, 0.0], 1.0, 2.0, 3.0, 4.0 ); //call static methods with double colon
    // let myquaternion2 = Quaternion::newQuaternion(10.0, vec![0.0, 0.0, 0.0], 1.0, 2.0, 3.0, 4.0 );

    //how i was initializing with the newquaternion method
    // let mut myquaternion2 = Quaternion{..Default::default()};
    // myquaternion2.newQuaternion(1.0, 2.0, 3.0, 4.0);
    // println!("{:?}", myquaternion);

 
    //initialize quaternions
    // let mut myquaternion = Quaternion{n: 10.0, v: Vector{x: 1.0, y: 2.0, z: 3.0}};
    // println!("{:?}", myquaternion);

    // let mut myquaternion2 = Quaternion{n: 5.0, v: Vector{x: 5.0, y: 6.0, z: 7.0}};
    //  println!("{:?}", myquaternion2);
    // let quatadd = myquaternion + myquaternion2;
    // println!("adding quaternions: {:?}", quatadd);



}