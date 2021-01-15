use crate::common::vector::Myvec;
use crate::common::mathutils::rad_to_deg;
use crate::common::mathutils::deg_to_rad;

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
        
            u.x = rad_to_deg(0.0_f32); //roll
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