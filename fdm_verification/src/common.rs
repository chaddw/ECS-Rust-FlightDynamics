#[derive(Debug, Default, PartialEq)]
pub struct Myvec
{
    pub x: f32,
    pub y: f32,
    pub z: f32,

}

impl Myvec
{
    //like a constructor, static method
    pub fn new(x: f32, y: f32, z: f32) -> Myvec
    {
        Myvec { x: x, y: y, z: z}
    }

    pub fn magnitude(&self) -> f32
    {
        return (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
    }

    //mutates in place
    pub fn normalize(&mut self)
    {
        let tol: f32 = 0.000000000000001; //float type tolerance
        let mut m: f32 = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if m <= tol
        {
            m = 1.0;
        }
        self.x /= m;
        self.y /= m;
        self.z /= m;
        
        if self.x.abs() < tol 
        {
            self.x = 0.0;
        }
        if self.y.abs() < tol 
        {
            self.y = 0.0;
        }
        if self.z.abs() < tol
        {
            self.z = 0.0;
        }
    
    }


    pub fn reverse(v: &Myvec) -> Myvec
    {
        let vec = Myvec { 
            x: -v.x,
            y: -v.y,
            z: -v.z,
        };

        return vec;

    }

    pub fn addvec(u: &Myvec, v: &Myvec) -> Myvec
    {
        let vec = Myvec { 
            x: u.x + v.x, 
            y: u.y + v.y,
            z: u.z + v.z,
        };

        return vec;

    }


    pub fn subtractvec(u: &Myvec, v: &Myvec) -> Myvec
    {
        let vec = Myvec { 
            x: u.x - v.x, 
            y: u.y - v.y,
            z: u.z - v.z,
        };

        return vec;

    }

    pub fn multiplyscalar(v: &Myvec, scalar: f32) -> Myvec
    {
        let vec = Myvec { 
            x: v.x * scalar, 
            y: v.y * scalar,
            z: v.z * scalar,
        };

        return vec;
    }
    pub fn dividescalar(v: &Myvec, scalar: f32) -> Myvec
    {
        let vec = Myvec { 
            x: v.x / scalar, 
            y: v.y / scalar,
            z: v.z / scalar,
        };

        return vec;
    }


    pub fn crossproduct(u: &Myvec, v: &Myvec) -> Myvec //^
    {
        let vec = Myvec { 
            x:  u.y*v.z - u.z*v.y,
            y: -u.x*v.z + u.z*v.x,
            z:  u.x*v.y - u.y*v.x,
        };

        return vec;

    }

    pub fn dotproduct(u: &Myvec, v: &Myvec) -> f32 //like multiplying vec
    {

        return u.x*v.x + u.y*v.y + u.z*v.z;

    }
}

#[derive(Debug, Default, PartialEq)]
pub struct Myquaternion
{
    pub n: f32, //scalar part
    pub v: Myvec, //vector part  v.x, v.y, v.z
}



impl Myquaternion
{
    #[allow(dead_code)]
    pub fn new(n: f32, x: f32, y: f32, z: f32) -> Myquaternion
    {
        Myquaternion { 
            
            n: n,
            
            v:  Myvec {
                x: x,
                y: y,
                z: z,
            } }
    }

    pub fn magnitude(&self) -> f32
    {
        return (self.n * self.n + self.v.x * self.v.x + self.v.y * self.v.y + self.v.z * self.v.z).sqrt();
    }

    pub fn conjugate(q: &Myquaternion) -> Myquaternion //~
    {
        let quat = Myquaternion { 
            n: q.n,
            
            v:  Myvec {
                x: -q.v.x, 
                y: -q.v.y,
                z: -q.v.z,
            }

        };

        return quat;
    }

    pub fn addquat(q1: &Myquaternion, q2: &Myquaternion) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n: q1.n + q2.n,
            
            v:  Myvec {
                x: q1.v.x + q2.v.x,
                y: q1.v.y + q2.v.y,
                z: q1.v.z + q2.v.z,
            }

        };

        return quat;
    }
    
    #[allow(dead_code)]
    pub fn subtractquat(q1: &Myquaternion, q2: &Myquaternion) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n: q1.n - q2.n,
            
            v:  Myvec {
                x: q1.v.x - q2.v.x,
                y: q1.v.y - q2.v.y,
                z: q1.v.z - q2.v.z,
            }

        };

        return quat;
    }

    pub fn multiplyscalar(q: &Myquaternion, scalar: f32) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n: q.n * scalar,
            
            v:  Myvec {
                x: q.v.x * scalar,
                y: q.v.y * scalar,
                z: q.v.z * scalar,
            }

        };

        return quat;
    }
    pub fn dividescalar(q: &Myquaternion, scalar: f32) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n: q.n / scalar,
            
            v:  Myvec {
                x: q.v.x / scalar,
                y: q.v.y / scalar,
                z: q.v.z / scalar,
            }

        };

        return quat;
    }

    pub fn multiplyquat(q1: &Myquaternion, q2: &Myquaternion) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n:  q1.n*q2.n - q1.v.x*q2.v.x - q1.v.y*q2.v.y - q1.v.z*q2.v.z,
            
            v:  Myvec {
                x: q1.n*q2.v.x + q1.v.x*q2.n + q1.v.y*q2.v.z - q1.v.z*q2.v.y,
                y: q1.n*q2.v.y + q1.v.y*q2.n + q1.v.z*q2.v.x - q1.v.x*q2.v.z,
                z: q1.n*q2.v.z + q1.v.z*q2.n + q1.v.x*q2.v.y - q1.v.y*q2.v.x,
            }

        };   

        return quat;
    }

    pub fn multiply_quat_by_vec(q: &Myquaternion, v: &Myvec) -> Myquaternion 
    {
        let quat = Myquaternion { 
            n:  -(q.v.x*v.x + q.v.y*v.y + q.v.z*v.z),
            
            v:  Myvec {
                x: q.n*v.x + q.v.y*v.z - q.v.z*v.y,
                y: q.n*v.y + q.v.z*v.x - q.v.x*v.z,
                z: q.n*v.z + q.v.x*v.y - q.v.y*v.x,
            }

        };   

        return quat;
    }
    

    pub fn qvrotate(q: &Myquaternion, v: &Myvec) -> Myvec
    {

        let q_conj = Myquaternion::conjugate(&q);
        let t = Myquaternion::multiply_quat_by_vec(&q, &v);
        let t2 = Myquaternion::multiplyquat(&t, &q_conj);

        let vec = Myvec { 
            x: t2.v.x,
            y: t2.v.y,
            z: t2.v.z,
        };

        return vec;
    }

    pub fn make_euler_from_q(q: &Myquaternion) -> Myvec
    {
        let pi = 3.14159265359;

        let q00: f64 = (q.n * q.n) as f64;
        let q11: f64 = (q.v.x * q.v.x) as f64;
        let q22: f64 = (q.v.y * q.v.y) as f64;
        let q33: f64 = (q.v.z * q.v.z) as f64;
        
        let r11: f64 = q00 + q11 - q22 - q33;
        let r21: f64 = (2.0 * (q.v.x*q.v.y + q.n*q.v.z) ) as f64;
        let r31: f64 = (2.0 * (q.v.x*q.v.z - q.n*q.v.y) ) as f64;
        let r32: f64 = (2.0 * (q.v.y*q.v.z + q.n*q.v.x) ) as f64;
        let r33: f64 = q00 - q11 - q22 + q33;
        
        let mut u: Myvec = Default::default();
        let tmp: f64 = r31.abs();
        if tmp > 0.999999
        {
            let r12: f64 = (2.0 * (q.v.x*q.v.y - q.n*q.v.z) ) as f64;
            let r13: f64 = (2.0 * (q.v.x*q.v.z + q.n*q.v.y) ) as f64;
        
            u.x = rad_to_deg(0.0_f32); //roll //be sure these r right
            u.y = rad_to_deg ((-(pi/2.0) * r31/tmp) as f32); //pitch
            u.z = rad_to_deg(-r12.atan2(-r31*r13) as f32) ; //yaw
            return u;
        }
        
        u.x = rad_to_deg( (r32.atan2(r33)) as f32); //roll
        u.y = rad_to_deg( (-r31.asin()) as f32) ; //pitch
        u.z = rad_to_deg( (r21.atan2(r11)) as f32 ); //yaw
        return u;

    }

    pub fn make_q_from_euler(x: f64, y: f64, z: f64) -> Myquaternion
    {
        let mut q: Myquaternion = Default::default();
        let roll: f64 = deg_to_rad(x as f32) as f64;
        let pitch: f64 = deg_to_rad(y as f32) as f64;
        let yaw: f64 = deg_to_rad(z as f32) as f64;

        let cyaw: f64 = (0.5 * yaw).cos();
        let cpitch: f64 = (0.5 * pitch).cos();
        let croll: f64 = (0.5 * roll).cos();

        let syaw: f64 = (0.5 * yaw).sin();
        let spitch: f64 = (0.5 * pitch).sin();
        let sroll: f64 = (0.5 * roll).sin();
    
        let cyawcpitch: f64 = cyaw * cpitch;
        let syawspitch: f64 = syaw * spitch;
        let cyawspitch: f64 = cyaw * spitch;
        let syawcpitch: f64 = syaw * cpitch;
    
    
        q.n = (cyawcpitch * croll + syawspitch * sroll) as f32;
        q.v.x = (cyawcpitch * sroll - syawspitch * croll) as f32;
        q.v.y = (cyawspitch * croll + syawcpitch * sroll) as f32;
        q.v.z = (syawcpitch * croll - cyawspitch * sroll) as f32;

        return q;
    }
}


#[derive(Debug, Default, PartialEq)]
pub struct Mymatrix
{
    // elements eij: i -> row, j -> column
    pub e11: f32, pub e12: f32, pub e13: f32,
    pub e21: f32, pub e22: f32, pub e23: f32,
    pub e31: f32, pub e32: f32, pub e33: f32,
}


impl Mymatrix
{

    pub fn new( r1c1: f32,  r1c2: f32,  r1c3: f32, 
                r2c1: f32,  r2c2: f32,  r2c3: f32, 
                r3c1: f32,  r3c2: f32,  r3c3: f32) -> Mymatrix
    {
        Mymatrix { 
            e11: r1c1,
            e12: r1c2,
            e13: r1c3,
            e21: r2c1,
            e22: r2c2,
            e23: r2c3,
            e31: r3c1,
            e32: r3c2,
            e33: r3c3,
        }

    }

    pub fn inverse(&self) -> Mymatrix
    {
        let mut d: f32 = self.e11*self.e22*self.e33 -
                    self.e11*self.e32*self.e23 +
                    self.e21*self.e32*self.e13 -
                    self.e21*self.e12*self.e33 +
                    self.e31*self.e12*self.e23 -
                    self.e31*self.e22*self.e13;
    
        if d == 0.0 
        {
                d = 1.0;
        }
    
        let matrix = Mymatrix
        {

            e11: (self.e22*self.e33-self.e23*self.e32)/d,
            e12: -(self.e12*self.e33-self.e13*self.e32)/d,
            e13: (self.e12*self.e23-self.e13*self.e22)/d,
            e21: -(self.e21*self.e33-self.e23*self.e31)/d,
            e22: (self.e11*self.e33-self.e13*self.e31)/d,
            e23: -(self.e11*self.e23-self.e13*self.e21)/d,
            e31: (self.e21*self.e32-self.e22*self.e31)/d,
            e32: -(self.e11*self.e32-self.e12*self.e31)/d,
            e33: (self.e11*self.e22-self.e12*self.e21)/d,

        };
        return matrix;           
    }


    pub fn multiply_matrix_by_vec(m: &Mymatrix, u: &Myvec) -> Myvec
    {

        let vec = Myvec { 
            x: m.e11*u.x + m.e12*u.y + m.e13*u.z,
            y: m.e21*u.x + m.e22*u.y + m.e23*u.z,
            z: m.e31*u.x + m.e32*u.y + m.e33*u.z,
        };

        return vec;
        
    }

}



//Helper functions for degrees <---> radians conversion
pub fn deg_to_rad(deg: f32) -> f32
{
    let pi: f32 = 3.14159265359;
	deg * pi / 180.0
}

pub fn rad_to_deg(rad: f32) -> f32
{	
    let pi: f32 = 3.14159265359;
	rad * 180.0 / pi
}