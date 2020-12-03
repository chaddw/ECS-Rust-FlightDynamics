#[derive(Debug, Default)]
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


    pub fn reverse_aka_conjugate(v: &Myvec) -> Myvec
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


    //how we would set up if we want to use dot operator on the instance
    //could also set up to operator overload but not worried about that now
    // pub fn subtract(&mut self, u: Myvec)
    // {
    //     self.x -= u.x;
    //     self.y -= u.y;
    //     self.z -= u.z;
    // }

    // pub fn multiply(&mut self, u: Myvec)
    // {
    //     self.x *= u.x;
    //     self.y *= u.y;
    //     self.z *= u.z;
    // }

    // pub fn divide(&mut self, u: Myvec)
    // {
    //     self.x /= u.x;
    //     self.y /= u.y;
    //     self.z /= u.z;
    // }

}

#[derive(Debug, Default)]
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

        let q00 = q.n * q.n;
        let q11 = q.v.x * q.v.x;
        let q22 = q.v.y * q.v.y;
        let q33 = q.v.z * q.v.z;
        
        let r11 = q00 + q11 - q22 - q33;
        let r21 = 2.0 * (q.v.x*q.v.y + q.n*q.v.z);
        let r31 = 2.0 * (q.v.x*q.v.z - q.n*q.v.y);
        let r32 = 2.0 * (q.v.y*q.v.z + q.n*q.v.x);
        let r33 = q00 - q11 - q22 + q33;
        
        let mut u: Myvec = Default::default();
        let tmp = r31.abs();
        if tmp > 0.999999
        {
            let r12 = 2.0 * (q.v.x*q.v.y - q.n*q.v.z);
            let r13 = 2.0 * (q.v.x*q.v.z + q.n*q.v.y);
        
            u.x = 0.0_f32.to_degrees();                                           // roll
            u.y = (-(pi/2.0) * r31/tmp).to_degrees();  // pitch
            u.z = (-r12.atan2(-r31*r13)).to_degrees(); // yaw
            return u;
        }
        
        u.x = (r32.atan2(r33)).to_degrees(); // roll
        u.y = (-r31.asin()).to_degrees();    // pitch
        u.z = (r21.atan2(r11)).to_degrees(); // yaw
        return u;

    }

    pub fn make_q_from_euler(x: f32, y: f32, z: f32) -> Myquaternion
    {
        let mut q: Myquaternion = Default::default();
        let roll: f32 = x.to_radians();
        let pitch: f32 = y.to_radians();
        let yaw: f32 = z.to_radians();

        let cyaw: f32 = (0.5 * yaw).cos();
        let cpitch: f32 = (0.5 * pitch).cos();
        let croll: f32 = (0.5 * roll).cos();

        let syaw: f32 = (0.5 * yaw).sin();
        let spitch: f32 = (0.5 * pitch).sin();
        let sroll: f32 = (0.5 * roll).sin();
    
        let cyawcpitch: f32 = cyaw * cpitch;
        let syawspitch: f32 = syaw * spitch;
        let cyawspitch: f32 = cyaw * spitch;
        let syawcpitch: f32 = syaw * cpitch;
    
    
        q.n = cyawcpitch * croll + syawspitch * sroll;
        q.v.x = cyawcpitch * sroll - syawspitch * croll;
        q.v.y = cyawspitch * croll + syawcpitch * sroll;
        q.v.z = syawcpitch * croll - cyawspitch * sroll;

        return q;
    }




}


#[derive(Debug, Default)]
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

            e11:  (self.e22*self.e33-self.e23*self.e32)/d,
            e12:  -(self.e12*self.e33-self.e13*self.e32)/d,
            e13: (self.e12*self.e23-self.e13*self.e22)/d,
            e21: -(self.e21*self.e33-self.e23*self.e31)/d,
            e22:  (self.e11*self.e33-self.e13*self.e31)/d,
            e23:  -(self.e11*self.e23-self.e13*self.e21)/d,
            e31:   (self.e21*self.e32-self.e22*self.e31)/d,
            e32:  -(self.e11*self.e32-self.e12*self.e31)/d,
            e33:  (self.e11*self.e22-self.e12*self.e21)/d,

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