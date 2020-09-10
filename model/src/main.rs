//use std::ops::{Add, Sub, Mul, Div}; //operator overload
extern crate nalgebra as na;
//use na::*; 
use na::{Matrix3, Vector3, UnitQuaternion}; //took off unit



//commenting out old vector and quaternion class
//////////////////////////// 

/////VECTOR CLASS

// #[allow(non_snake_case)]
// #[derive(Default, Debug)]
// struct Vector
// {
//     x: f32,
//     y: f32,
//     z: f32
// }

// impl Vector
// {


//     //static does not want to work right now to initialize struct
//     //  fn newVec(x: f32, y: f32, z: f32) -> Vector
//     //  {
//     //     Vector{x:x, y:y, z:z};
//     //  }


//     //scalar magnitude
//     fn Magnitude(&self) -> f32
//     {
//         return (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
//     }

//     //convert to unit vector
//     fn Normalize(&mut self) -> ()
//     {
//         const tol: f32 = 0.0001; //tolerance
//         let mut m: f32 = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        
//         if m <= tol
//         {
//             m = 1.0;
//         }

//         self.x = self.x / m;
//         self.y = self.y / m;
//         self.x = self.x / m;

//         if self.x.abs() < tol
//         {
//             self.x = 0.0;
//         }
//         if self.y.abs() < tol
//         {
//             self.y = 0.0;
//         }
//         if self.x.abs() < tol
//         {
//             self.z = 0.0;
//         }
//     }

//     //reverse direction of vector. point in opposite direction
//     fn Reverse(&mut self) -> ()
//     {
//         self.x = -self.x;
//         self.y = -self.y;
//         self.z = -self.z;
//     }
// }

// //vector operator overloading
// impl Add for Vector 
// {
//     type Output = Self;

//     fn add(self, other: Self) -> Self {
//         Self {

//             x: self.x + other.x, 
//             y: self.y + other.y, 
//             z: self.z + other.z
//         }
//     }
// }

// impl Sub for Vector 
// {
//     type Output = Self;

//     fn sub(self, other: Self) -> Self {
//         Self {

//             x: self.x - other.x, 
//             y: self.y - other.y, 
//             z: self.z - other.z
//         }
//     }
// }

// //multiply by a scalar
// impl Mul<f32> for Vector 
// {
//     type Output = Self;

//     fn mul(self, scalar: f32) -> Self {
//         Self {

//             x: self.x * scalar, 
//             y: self.y * scalar, 
//             z: self.z * scalar
//         }
//     }
// }

// //vector by vector.... fix this...
// // impl Mul for Vector 
// // {
// //     type Output = Self;

// //     fn mul(self, other: Self) -> f32 {
// //         f32 {

// //             self.x * other.x + self.y * other.y + self.z *
// //         }
// //     }
// // }

// //divide by a scalar
// impl Div<f32> for Vector
// {
//     type Output = Self;

//     fn div(self, scalar: f32) -> Self {
//         Self {

//             x: self.x / scalar, 
//             y: self.y / scalar, 
//             z: self.z / scalar
//         }
//     }
// }
// //needs cross product ^, dot product *.


// //////QUATERNIONS

// #[allow(non_snake_case)]
// #[derive(Default, Debug)]
// struct Quaternion
// {
//     n: f32,
//     v: Vector,
//    // v: Vec<f32> //v: [f32;3], //this is a "Vector" type defined in appendix A...
//     //e0: f32, e1: f32, e2: f32, e3: f32
// }

// #[allow(non_snake_case)]
// impl Quaternion
// {
//     //instance method with access to struct fields to initialize the struct... maybe this needs to be static...
//     //realized i do not need this to initialize easily
//     // fn newQuaternion(&mut self, e0: f32, e1: f32, e2: f32, e3: f32)
//     // {
//     //     self.v = Vector{x: e1, y: e2, z: e3};
//     //     //self.v = vec![0.0, 0.0, 0.0]; //x, y, z
//     //     self.n = e0;
//     //     //self.v[0] = e1; //x
//     //    // self.v[1] = e2; //y
//     //    // self.v[2] = e3; //z
//     // }

//     //intialize statically. would need to add e1,e2,e3 to the struct variables
//     //  fn newQuaternion(n: f32, mut v: Vec<f32>, e0: f32, e1: f32, e2: f32, e3: f32)
//     //  {
//     //     v = vec![e1, e2, e3]; //x, y, z
//     //     Quaternion {n: n, v: v};
//     //  }

//     //similar to vector but take the scalar into account
//     fn Magnitude(&self) -> f32
//     {
//         return (self.n * self.n + self.v.x + self.v.x + self.v.y * self.v.y + self.v.z * self.v.z).sqrt();
//     }

//     //vector accessor
//     fn GetVector(&self) -> &Vector
//     {
//         //return vec![ self.v[0], self.v[1], self.v[2] ]
//         return &self.v;
//     }

//     //scaler accessor
//     fn GetScalar(&self) -> f32
//     {
//         return self.n;
//     }

// }

// //quaternion operator overloading
// impl Add for Quaternion
// {
//     type Output = Self;

//     fn add(self, other: Self) -> Self
//     {
//         Self {

//             n: self.n + other.n, //scalar
//             v: Vector{x: self.v.x + other.v.x, y: self.v.y + other.v.y, z: self.v.z + other.v.z}
//             //v: vec![self.v[0] + other.v[0], self.v[1] + other.v[1], self.v[2] + other.v[2] ]
//         }
//     }
// }

// impl Sub for Quaternion
// {
//     type Output = Self;

//     fn sub(self, other: Self) -> Self
//     {
//         Self {

//             n: self.n - other.n, //scalar
//             v: Vector{x: self.v.x - other.v.x, y: self.v.y - other.v.y, z: self.v.z - other.v.z}
//             //v: vec![self.v[0] + other.v[0], self.v[1] + other.v[1], self.v[2] + other.v[2] ]
//         }
//     }
// }

// //in quaternion class: need scalar multiply, scalar divide, conjugate.... nalgebra makes this easy



///////////////////////// end old code



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
    q_orientation: UnitQuaternion<f32>,   // orientation in earth coordinates
    vforces: Vector3<f32>,            // total force on body
    thrustforce: f32,           //magnitude of thrust
    v_moments: Vector3<f32>,          // total moment (torque) on body



    element: Vec<PointMass> //vector of point mass elements


}

//the point masses are elements making up the bodystructure 
#[derive(Debug)]
struct PointMass
{
    fmass: f32,
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

            //but this seems like a bunch of default values except for vposition and v_velocity, vforces, ... maybe dont need this new method and can just set defaults in main

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
            vforces: Vector3::new(500.0, 0.0, 0.0),
            thrustforce: 500.0,   //this isnt written in the rigid body intiialization for some reason...

            v_moments: Vector3::new(0.0, 0.0, 0.0),

            //zero the velocity in body space coordinates
            v_velocity_body: Vector3::new(0.0, 0.0, 0.0),

            //set these to false at first, will control later with keyboard... these are not defined in the structure
            //stalling: false,
            //Flaps: false,

            //set initial orientation
            //q_orientation: MakeQFromEulerAngles(0.0, 0.0, 0.0),  
            //q_orientation: Quaternion{n: 0.0, v: Vector{x: 0.0, y: 0.0, z: 0.0}}, //default value for now

            //nalgebra approach
            q_orientation: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),


            //8 elements of the plane taken into account
            element: vec![
                PointMass{fmass: 6.56, v_d_coords: Vector3::new(14.5, 12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 7.31, v_d_coords: Vector3::new(14.5, 5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 7.31, v_d_coords: Vector3::new(14.5, -5.5, 2.5), v_local_inertia: Vector3::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 6.56, v_d_coords: Vector3::new(14.5, -12.0, 2.5), v_local_inertia: Vector3::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 2.62, v_d_coords: Vector3::new(3.03, 2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 2.62, v_d_coords: Vector3::new(3.03, -2.5, 3.0), v_local_inertia: Vector3::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 2.93, v_d_coords: Vector3::new(2.25, 0.0, 5.0), v_local_inertia: Vector3::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) },
                PointMass{fmass: 31.8, v_d_coords: Vector3::new(15.25, 0.0, 1.5), v_local_inertia: Vector3::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Vector3::new(0.0, 0.0, 0.0), v_cg_coords: Vector3::new(0.0, 0.0, 0.0) }
                ]

           
        }

    }

    fn calc_airplane_mass_properties(&mut self)
    {
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
            total_mass = total_mass + i.fmass;
        }
        //println!("Total mass: {}", total_mass);

        //calculate combined center of gravity location
        let mut first_moment_x: f32 = 0.0;
        let mut first_moment_y: f32 = 0.0;
        let mut first_moment_z: f32 = 0.0;
        for i in self.element.iter()
        {
            //X coord
            first_moment_x = first_moment_x + i.fmass * i.v_d_coords.x;
            //y coord
            first_moment_y = first_moment_y + i.fmass * i.v_d_coords.y;
            //z coord
            first_moment_z = first_moment_z + i.fmass * i.v_d_coords.z;
            // vMoment = vMoment * i.fmass * i.v_d_coords; //vector multiplcation not set up properly... oh well. even with nalgebra it panics
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
            ixx = ixx + i.v_local_inertia.x + i.fmass *
                (i.v_cg_coords.y * i.v_cg_coords.y +
                i.v_cg_coords.z * i.v_cg_coords.z);

            iyy = iyy + i.v_local_inertia.y + i.fmass *
                (i.v_cg_coords.z * i.v_cg_coords.z +
                i.v_cg_coords.x * i.v_cg_coords.x);

            izz = izz + i.v_local_inertia.z + i.fmass *
                (i.v_cg_coords.x * i.v_cg_coords.x +
                i.v_cg_coords.y * i.v_cg_coords.y);

            ixy = ixy + i.fmass * (i.v_cg_coords.x * 
                i.v_cg_coords.y);

            ixz = ixz + i.fmass * (i.v_cg_coords.x * 
                i.v_cg_coords.z);
            
            iyz = iyz + i.fmass * (i.v_cg_coords.y *
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



        //using nalgebra matrix. having trouble getting it in the rigid body 
        self.m_inertia = Matrix3::new(ixx, -ixy, -ixz,
                                -ixy, iyy, -iyz,
                                -ixz, -iyz, izz);

        //get inverse of matrix
        self.m_inertia_inverse = self.m_inertia.try_inverse().unwrap();

        
    }

    //calculates all of the forces and moments on the plane at any time
    fn calc_airplane_loads(&mut self)
    {
        let mut fb = Vector3::new(0.0, 0.0, 0.0);
        let mut mb = Vector3::new(0.0, 0.0, 0.0);

        //reset forces and moments
        self.vforces = Vector3::new(0.0, 0.0, 0.0);
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
        let mut stalling: bool = false;

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
                stalling = true; //probably will need to add stalling to rigid body variables
            }

            //keep running total of resultant forces (total force)
            fb = fb + v_resultant;

            //calculate the moment about the center of gravity of this element's force and keep them in a running total of these moments (total moment)
            vtmp = self.element[i].v_cg_coords.cross(&v_resultant);
            mb = mb + vtmp;
        }


        //add thrust
        fb = fb + thrust;

        //convert forces from model space to earth space. rotates the vector by the unit quaternion
        self.vforces = self.q_orientation.transform_vector(&fb);

        //apply gravity (g is -32.174 ft/s^2)
        self.vforces.z = self.vforces.z + (-32.174) * self.mass;

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




} //end impl


fn main()
{
              

    let mut airplane = RigidBody::new();
    airplane.calc_airplane_mass_properties(); //would be nice to call this automatically in new, having some trouble with getting that to work rn...
    airplane.calc_airplane_loads();
    //println!("{:#?}", Airplane);


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