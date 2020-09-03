use std::ops::{Add, Sub, Mul, Div}; //operator overload
extern crate nalgebra as na;
//use na::*; 
use na::{Matrix3, Vector3, UnitQuaternion, Unit};
/////VECTOR CLASS

#[allow(non_snake_case)]
#[derive(Default, Debug)]
struct Vector
{
    x: f32,
    y: f32,
    z: f32
}

impl Vector
{


    //static does not want to work right now to initialize struct
    //  fn newVec(x: f32, y: f32, z: f32) -> Vector
    //  {
    //     Vector{x:x, y:y, z:z};
    //  }


    //scalar magnitude
    fn Magnitude(&self) -> f32
    {
        return (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
    }

    //convert to unit vector
    fn Normalize(&mut self) -> ()
    {
        const tol: f32 = 0.0001; //tolerance
        let mut m: f32 = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        
        if m <= tol
        {
            m = 1.0;
        }

        self.x = self.x / m;
        self.y = self.y / m;
        self.x = self.x / m;

        if self.x.abs() < tol
        {
            self.x = 0.0;
        }
        if self.y.abs() < tol
        {
            self.y = 0.0;
        }
        if self.x.abs() < tol
        {
            self.z = 0.0;
        }
    }

    //reverse direction of vector. point in opposite direction
    fn Reverse(&mut self) -> ()
    {
        self.x = -self.x;
        self.y = -self.y;
        self.z = -self.z;
    }
}

//vector operator overloading
impl Add for Vector 
{
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {

            x: self.x + other.x, 
            y: self.y + other.y, 
            z: self.z + other.z
        }
    }
}

impl Sub for Vector 
{
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {

            x: self.x - other.x, 
            y: self.y - other.y, 
            z: self.z - other.z
        }
    }
}

//multiply by a scalar
impl Mul<f32> for Vector 
{
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {

            x: self.x * scalar, 
            y: self.y * scalar, 
            z: self.z * scalar
        }
    }
}

//vector by vector.... i'll just do this by hand for now on the combined center of gravity
// impl Mul for Vector 
// {
//     type Output = Self;

//     fn mul(self, other: Self) -> f32 {
//         f32 {

//             self.x * other.x + self.y * other.y + self.z *
//         }
//     }
// }

//divide by a scalar
impl Div<f32> for Vector
{
    type Output = Self;

    fn div(self, scalar: f32) -> Self {
        Self {

            x: self.x / scalar, 
            y: self.y / scalar, 
            z: self.z / scalar
        }
    }
}
//needs cross product ^, dot product *.


//////QUATERNIONS

#[allow(non_snake_case)]
#[derive(Default, Debug)]
struct Quaternion
{
    n: f32,
    v: Vector,
   // v: Vec<f32> //v: [f32;3], //this is a "Vector" type defined in appendix A...
    //e0: f32, e1: f32, e2: f32, e3: f32
}

#[allow(non_snake_case)]
impl Quaternion
{
    //instance method with access to struct fields to initialize the struct... maybe this needs to be static...
    //realized i do not need this to initialize easily
    // fn newQuaternion(&mut self, e0: f32, e1: f32, e2: f32, e3: f32)
    // {
    //     self.v = Vector{x: e1, y: e2, z: e3};
    //     //self.v = vec![0.0, 0.0, 0.0]; //x, y, z
    //     self.n = e0;
    //     //self.v[0] = e1; //x
    //    // self.v[1] = e2; //y
    //    // self.v[2] = e3; //z
    // }

    //intialize statically. would need to add e1,e2,e3 to the struct variables
    //  fn newQuaternion(n: f32, mut v: Vec<f32>, e0: f32, e1: f32, e2: f32, e3: f32)
    //  {
    //     v = vec![e1, e2, e3]; //x, y, z
    //     Quaternion {n: n, v: v};
    //  }

    //similar to vector but take the scalar into account
    fn Magnitude(&self) -> f32
    {
        return (self.n * self.n + self.v.x + self.v.x + self.v.y * self.v.y + self.v.z * self.v.z).sqrt();
    }

    //vector accessor
    fn GetVector(&self) -> &Vector
    {
        //return vec![ self.v[0], self.v[1], self.v[2] ]
        return &self.v;
    }

    //scaler accessor
    fn GetScalar(&self) -> f32
    {
        return self.n;
    }

}

//quaternion operator overloading
impl Add for Quaternion
{
    type Output = Self;

    fn add(self, other: Self) -> Self
    {
        Self {

            n: self.n + other.n, //scalar
            v: Vector{x: self.v.x + other.v.x, y: self.v.y + other.v.y, z: self.v.z + other.v.z}
            //v: vec![self.v[0] + other.v[0], self.v[1] + other.v[1], self.v[2] + other.v[2] ]
        }
    }
}

impl Sub for Quaternion
{
    type Output = Self;

    fn sub(self, other: Self) -> Self
    {
        Self {

            n: self.n - other.n, //scalar
            v: Vector{x: self.v.x - other.v.x, y: self.v.y - other.v.y, z: self.v.z - other.v.z}
            //v: vec![self.v[0] + other.v[0], self.v[1] + other.v[1], self.v[2] + other.v[2] ]
        }
    }
}

//in quaternion class: need scalar multiply, scalar divide, conjugate
//in quaternion functions and operators: needs multiplication, division and many more... some seem repeated like addition... weird. 
// will code them when they come up in the model



//rigid body structure to encapsulate the required data during the simulation
#[allow(non_snake_case)]
#[derive(Debug)]
struct RigidBody
{
    mass: f32, //total mass

    //probably will not make a a custom matrix 3x3 struct...
    //mmInertia: [[f32;3];3],      // = [[0.0; 3]; 3], //mass moment of inertia
   // mInertiaInverse:[[f32;3];3],// inverse of mass moment in inertia
    
    //Matrix3x3 mIntertia;      //mass moment of inertia
    //Matrix3x3 mInertiaInverse;// inverse of mass moment in inertia
    mInertia: Matrix3<f32>,
    mInertiaInverse: Matrix3<f32>,
    //mInertia: Matrix3<f32, na::U3, na::U3, na::ArrayStorage<f32, na::U3, na::U3>>,
   // mInertiaInverse: Matrix3<f32, na::U3, na::U3, na::ArrayStorage<f32, na::U3, na::U3>>,


    vPosition: Vector,          // position in earth coordinates
    vVelocity: Vector,          // velocity in earth coordinates
    vVelocityBody: Vector,      // velocity in body coordinates
    vAngularVelocity: Vector,   // angular velocity in body coordinates
    vEulerAngles: Vector,       // Euler angles in body coordinates
    fSpeed: f32,                // speed (magnitude of the velocity)
    qOrientation: Quaternion,   // orientation in earth coordinates
    vForces: Vector,            // total force on body
    vMoments: Vector,          // total moment (torque) on body

    //nalgeabra approach
    Qua: UnitQuaternion<f32>
}


impl RigidBody
{
    //quicker way to initialize...
    fn new() -> RigidBody 
    {
        Self
        {

            //but this seems like a bunch of default values except for vposition and vvelocity, vforces, ... maybe dont need this new method and can just set defaults in main

            mass: 0.0,  

           // mmInertia: [[0.0; 3]; 3], //3x3 matrix... was using this before nalgebra

           //nalgebra approach
            mInertia: Matrix3::new(0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0),
            mInertiaInverse: Matrix3::new(0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0),

            //set initial position
            vPosition: Vector{x: -5000.0, y: 0.0, z: 200.0},

            //set initial velocity
            vVelocity: Vector{x: 60.0, y: 0.0, z: 0.0},

            vEulerAngles: Vector{x: 0.0, y:0.0, z: 0.0}, //not defined in book here

            fSpeed: 60.0,

            //set angular velocity
            vAngularVelocity: Vector{x: 0.0, y: 0.0, z: 0.0},

            //set initial thrust, forces, and moments
            vForces: Vector{x: 500.0, y: 0.0, z: 0.0},
            //Thrustforce: 500.0,   //this is not in the rigid body for some reason...

            vMoments: Vector{x: 0.0, y: 0.0, z: 0.0},

            //zero the velocity in body space coordinates
            vVelocityBody: Vector{x: 0.0, y: 0.0, z: 0.0},

            //set these to false at first, will control later with keyboard... these are not defined in the structure
            //Stalling: false,
            //Flaps: false,

            //set initial orientation
            //qOrientation: MakeQFromEulerAngles(0.0, 0.0, 0.0),  
            qOrientation: Quaternion{n: 0.0, v: Vector{x: 0.0, y: 0.0, z: 0.0}}, //default value for now

 
            //nalgebra approach
            Qua: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),


            //calculate plane's mass properties
            //(wont be called here)


           
        }

    }

    fn CalcAirplaneMassProperties(&mut self)
    {
        //the point masses are elements making up the bodystructure 
        #[allow(non_snake_case)]
        #[derive(Default, Debug)]
        struct PointMass
        {
            fmass: f32,
            vDCoords: Vector, //"design position"
            vLocalInertia: Vector,
            fIncidence: f32,
            fDihedral: f32,
            fArea: f32,
            iFlap: f32,

            vNormal: Vector,
            vCGCoords: Vector //"corrected position"

        }


        //8 elements of the plane taken into account
        let mut Element = vec![
            PointMass{fmass: 6.56, vDCoords: Vector{x: 14.5, y: 12.0, z: 2.5}, vLocalInertia: Vector{x: 13.92, y: 10.50 , z: 24.00}, fIncidence: -3.5, fDihedral: 0.0, fArea: 31.2, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 7.31, vDCoords: Vector{x: 14.5, y: 5.5, z: 2.5}, vLocalInertia: Vector{x: 21.95, y: 12.22, z: 33.67}, fIncidence: -3.5, fDihedral: 0.0, fArea: 36.4, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 7.31, vDCoords: Vector{x: 14.5, y: -5.5, z: 2.5}, vLocalInertia: Vector{x: 21.95, y: 12.22, z: 33.67}, fIncidence: -3.5, fDihedral: 0.0, fArea: 36.4, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 6.56, vDCoords: Vector{x: 14.5, y: -12.0, z: 2.5}, vLocalInertia: Vector{x: 13.92, y: 10.50, z: 24.00}, fIncidence: -3.5, fDihedral: 0.0, fArea: 31.2, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 2.62, vDCoords: Vector{x: 3.03, y: 2.5, z: 3.0}, vLocalInertia: Vector{x: 0.837, y: 0.385, z: 1.206}, fIncidence: 0.0, fDihedral: 0.0, fArea: 10.8, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 2.62, vDCoords: Vector{x: 3.03, y: -2.5, z: 3.0}, vLocalInertia: Vector{x: 0.837, y: 0.385, z: 1.206}, fIncidence: 0.0, fDihedral: 0.0, fArea: 10.8, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 2.93, vDCoords: Vector{x: 2.25, y: 0.0, z: 5.0}, vLocalInertia: Vector{x: 1.262, y: 1.942, z: 0.718}, fIncidence: 0.0, fDihedral: 90.0, fArea: 12.0, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} },
            PointMass{fmass: 31.8, vDCoords: Vector{x: 15.25, y: 0.0, z: 1.5}, vLocalInertia: Vector{x: 66.30, y: 861.9, z: 861.9}, fIncidence: 0.0, fDihedral: 0.0, fArea: 84.0, iFlap: 0.0, vNormal: Vector{x: 0.0, y: 0.0, z: 0.0}, vCGCoords: Vector{x: 0.0, y: 0.0, z: 0.0} }
            ];

        let mut inn: f32;
        let mut di: f32;

        //calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
        for  i in Element.iter_mut()
        {
            inn = (i.fIncidence).to_radians();
            di = (i.fDihedral).to_radians();
            i.vNormal = Vector{x: inn.sin(), y: (inn.cos() * di.sin()), z: (inn.cos() * di.cos())};
            i.vNormal.Normalize();
        }

        //calculate total mass
        let mut TotalMass: f32 = 0.0;
        for i in Element.iter()
        {
            TotalMass = TotalMass + i.fmass;
        }
        println!("Total mass: {}", TotalMass);

        //calculate combined center of gravity location
        let mut FirstMomentX: f32 = 0.0;
        let mut FirstMomentY: f32 = 0.0;
        let mut FirstMomentZ: f32 = 0.0;
        for i in Element.iter()
        {
            //X coord
            FirstMomentX = FirstMomentX + i.fmass * i.vDCoords.x;
            //y coord
            FirstMomentY = FirstMomentY + i.fmass * i.vDCoords.y;
            //z coord
            FirstMomentZ = FirstMomentZ + i.fmass * i.vDCoords.z;
            // vMoment = vMoment * i.fmass * i.vDCoords; //vector multiplcation not set up properly... oh well
        }
        let mut vMoment = Vector{x: FirstMomentX, y: FirstMomentY, z: FirstMomentZ}; //remember there is a vmoments in rigid body.. we'll see how this plays out
        let CG = vMoment / TotalMass; //operator overload works! Vector / scalar
        println!("Combined center of gravity {:?}", CG);

        //calculate coordinates of each element with respect to the combined CG, relative position
        for i in Element.iter_mut()
        {
            i.vCGCoords.x = i.vDCoords.x - CG.x;
            i.vCGCoords.y = i.vDCoords.y - CG.y;
            i.vCGCoords.z = i.vDCoords.z - CG.z;
        }
 
        //calculate the moments and products of intertia for the combined elements
        let mut Ixx: f32 = 0.0;
        let mut Iyy: f32 = 0.0;
        let mut Izz: f32 = 0.0;
        let mut Ixy: f32 = 0.0;
        let mut Ixz: f32 = 0.0;
        let mut Iyz: f32 = 0.0;

        for i in Element.iter()
        {
            Ixx = Ixx + i.vLocalInertia.x + i.fmass *
                (i.vCGCoords.y * i.vCGCoords.y +
                i.vCGCoords.z * i.vCGCoords.z);

            Iyy = Iyy + i.vLocalInertia.y + i.fmass *
                (i.vCGCoords.z * i.vCGCoords.z +
                i.vCGCoords.x * i.vCGCoords.x);

            Izz = Izz + i.vLocalInertia.z + i.fmass *
                (i.vCGCoords.x * i.vCGCoords.x +
                i.vCGCoords.y * i.vCGCoords.y);

            Ixy = Ixy + i.fmass * (i.vCGCoords.x * 
                i.vCGCoords.y);

            Ixz = Ixz + i.fmass * (i.vCGCoords.x * 
                i.vCGCoords.z);
            
            Iyz = Iyz + i.fmass * (i.vCGCoords.y *
                i.vCGCoords.z);
        }
    
        //finally, set up airplanes mass and inertia matrix
        self.mass = TotalMass;

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
        self.mInertia = Matrix3::new(Ixx, -Ixy, -Ixz,
                                -Ixy, Iyy, -Iyz,
                                -Ixz, -Iyz, Izz);

        //get inverse of matrix
        self.mInertiaInverse = self.mInertia.try_inverse().unwrap();


    }
}


fn main()
{
              

    let mut Airplane = RigidBody::new();
    Airplane.CalcAirplaneMassProperties(); //would be nice to call this automatically in new, having some trouble with getting that to work rn...
    println!("{:#?}", Airplane);


    let mut myvec = Vector{x: 1.0, y: 2.0, z: 3.0};
    println!("{:?}", myvec);
    let mut myvec2 = Vector{x: 10.0, y: 20.0, z: 30.0};
    println!("{:?}", myvec2);

    let vecadd = myvec / 2.0;
   println!("dividing vector1 by 2: {:?}", vecadd);


    //using static method to initialize... unsuccessful for now
    // let myquaternion = Quaternion::newQuaternion(10.0, vec![0.0, 0.0, 0.0], 1.0, 2.0, 3.0, 4.0 ); //call static methods with double colon
    // let myquaternion2 = Quaternion::newQuaternion(10.0, vec![0.0, 0.0, 0.0], 1.0, 2.0, 3.0, 4.0 );

    //how i was initializing with the newquaternion method
    // let mut myquaternion2 = Quaternion{..Default::default()};
    // myquaternion2.newQuaternion(1.0, 2.0, 3.0, 4.0);
    // println!("{:?}", myquaternion);

 
    //initialize quaternions
    let mut myquaternion = Quaternion{n: 10.0, v: Vector{x: 1.0, y: 2.0, z: 3.0}};
    println!("{:?}", myquaternion);

    let mut myquaternion2 = Quaternion{n: 5.0, v: Vector{x: 5.0, y: 6.0, z: 7.0}};
     println!("{:?}", myquaternion2);
    let quatadd = myquaternion + myquaternion2;
    println!("adding quaternions: {:?}", quatadd);



}