//To run unit tests for the Quaternion module from the command line: 
//cargo test --lib quaternion

use crate::bourg::common::vector::Vector;
use crate::bourg::common::math_utils::rad_to_deg;
use crate::bourg::common::math_utils::deg_to_rad;
use crate::bourg::common::constants::PI;

//Operator overloads
use std::ops::{Add, Sub, Div, Mul};

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct Quaternion
{
    pub n: f32, //scalar part
    pub v: Vector, //vector part v.x, v.y, v.z
}


//Quaternion addition
impl Add for Quaternion
{
    type Output = Self;

    fn add(self, q2: Self) -> Self 
    {
        let quat = Quaternion { 
            n: self.n + q2.n,
            
            v:  Vector {
                x: self.v.x + q2.v.x,
                y: self.v.y + q2.v.y,
                z: self.v.z + q2.v.z,
            }
        };
        
        return quat;
    }
}

//Quaternion subtraction
impl Sub for Quaternion
{
    type Output = Self;

    fn sub(self, q2: Self) -> Self 
    {
        let quat = Quaternion { 
            n: self.n + q2.n,
            
            v:  Vector {
                x: self.v.x - q2.v.x,
                y: self.v.y - q2.v.y,
                z: self.v.z - q2.v.z,
            }
        };
        
        return quat;
    }
}


//Quaternion multiplcation
impl Mul for Quaternion
{
    type Output = Self;

    fn mul(self, q2: Self) -> Self 
    {
        let quat = Quaternion { 
            n:  self.n*q2.n - self.v.x*q2.v.x - self.v.y*q2.v.y - self.v.z*q2.v.z,
            
            v:  Vector {
                x: self.n*q2.v.x + self.v.x*q2.n + self.v.y*q2.v.z - self.v.z*q2.v.y,
                y: self.n*q2.v.y + self.v.y*q2.n + self.v.z*q2.v.x - self.v.x*q2.v.z,
                z: self.n*q2.v.z + self.v.z*q2.n + self.v.x*q2.v.y - self.v.y*q2.v.x,
            }
        };   

        return quat;
    }
}

//Quaternion multiplcation by scalar
impl Mul<f32> for Quaternion
{
    type Output = Self;

    fn mul(self, scalar: f32) -> Self 
    {
        let quat = Quaternion { 
            n: self.n * scalar,
            
            v:  Vector {
                x: self.v.x * scalar,
                y: self.v.y * scalar,
                z: self.v.z * scalar,
            }
        };

        return quat;
    }
}


//Quaternion division by scalar
impl Div<f32> for Quaternion
{
    type Output = Self;

    fn div(self, scalar: f32) -> Self 
    {
        let quat = Quaternion { 
            n: self.n / scalar,
            
            v:  Vector {
                x: self.v.x / scalar,
                y: self.v.y / scalar,
                z: self.v.z / scalar,
            }
        };

        return quat;
    }
}

//Quaternion multiplcation by Vector
impl Mul<Vector> for Quaternion
{
    type Output = Self;

    fn mul(self, v: Vector) -> Self 
    {
        let quat = Quaternion { 
            n:  -(self.v.x*v.x + self.v.y*v.y + self.v.z*v.z),
            
            v:  Vector {
                x: self.n*v.x + self.v.y*v.z - self.v.z*v.y,
                y: self.n*v.y + self.v.z*v.x - self.v.x*v.z,
                z: self.n*v.z + self.v.x*v.y - self.v.y*v.x,
            }

        };   

        return quat;
    }
}



impl Quaternion
{
    pub fn new(n: f32, x: f32, y: f32, z: f32) -> Quaternion
    {
        Quaternion { 
            
            n: n,
            
            v:  Vector {
                x: x,
                y: y,
                z: z,
            } }
    }

    pub fn magnitude(&self) -> f32
    {
        return (self.n * self.n + self.v.x * self.v.x + self.v.y * self.v.y + self.v.z * self.v.z).sqrt();
    }

    pub fn conjugate(q: &Quaternion) -> Quaternion
    {
        let quat = Quaternion { 
            n: q.n,
            
            v:  Vector {
                x: -q.v.x, 
                y: -q.v.y,
                z: -q.v.z,
            }
        };
        return quat;
    }

    
    pub fn qvrotate(q: &Quaternion, v: &Vector) -> Vector
    {

        let q2 = Quaternion::conjugate(&q);

        let q1 = Quaternion { //multiply quaternion by vector
            n:  -(q.v.x*v.x + q.v.y*v.y + q.v.z*v.z),
            
            v:  Vector {
                x: q.n*v.x + q.v.y*v.z - q.v.z*v.y,
                y: q.n*v.y + q.v.z*v.x - q.v.x*v.z,
                z: q.n*v.z + q.v.x*v.y - q.v.y*v.x,
            }
        };   


        let t2 = Quaternion { //multiply quaternions
            n:  q1.n*q2.n - q1.v.x*q2.v.x - q1.v.y*q2.v.y - q1.v.z*q2.v.z,
            
            v:  Vector {
                x: q1.n*q2.v.x + q1.v.x*q2.n + q1.v.y*q2.v.z - q1.v.z*q2.v.y,
                y: q1.n*q2.v.y + q1.v.y*q2.n + q1.v.z*q2.v.x - q1.v.x*q2.v.z,
                z: q1.n*q2.v.z + q1.v.z*q2.n + q1.v.x*q2.v.y - q1.v.y*q2.v.x,
            }
        };   

        let vector = Vector { 
            x: t2.v.x,
            y: t2.v.y,
            z: t2.v.z,
        };

        return vector;
    }

    pub fn make_euler_from_q(q: &Quaternion) -> Vector
    {
        let q00: f64 = (q.n * q.n) as f64;
        let q11: f64 = (q.v.x * q.v.x) as f64;
        let q22: f64 = (q.v.y * q.v.y) as f64;
        let q33: f64 = (q.v.z * q.v.z) as f64;
        
        let r11: f64 = q00 + q11 - q22 - q33;
        let r21: f64 = (2.0 * (q.v.x*q.v.y + q.n*q.v.z) ) as f64;
        let r31: f64 = (2.0 * (q.v.x*q.v.z - q.n*q.v.y) ) as f64;
        let r32: f64 = (2.0 * (q.v.y*q.v.z + q.n*q.v.x) ) as f64;
        let r33: f64 = q00 - q11 - q22 + q33;
        
        let mut u: Vector = Default::default();
        let tmp: f64 = r31.abs();
        if tmp > 0.999999
        {
            let r12: f64 = (2.0 * (q.v.x*q.v.y - q.n*q.v.z) ) as f64;
            let r13: f64 = (2.0 * (q.v.x*q.v.z + q.n*q.v.y) ) as f64;
        
            u.x = rad_to_deg(0.0_f32); //roll
            u.y = rad_to_deg ((-(PI as f64/2.0) * r31/tmp) as f32); //pitch
            u.z = rad_to_deg(-r12.atan2(-r31*r13) as f32) ; //yaw
            return u;
        }
        
        u.x = rad_to_deg( (r32.atan2(r33)) as f32); //roll
        u.y = rad_to_deg( (-r31.asin()) as f32) ; //pitch
        u.z = rad_to_deg( (r21.atan2(r11)) as f32 ); //yaw
        return u;

    }

    pub fn make_q_from_euler(x: f64, y: f64, z: f64) -> Quaternion
    {
        let mut q: Quaternion = Default::default();
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


#[cfg(test)]
mod tests
{
    use super::*;

    //QUATERNION TESTS

    #[test]
    fn q_magnitude_test1()   
    { 
        let q = Quaternion::new(2.0, 2.0, 2.0, 2.0);
        
        let equal = 4.0;
        assert_eq!(q.magnitude(), equal);
    }

    #[test]
    fn q_magnitude_test2()   
    { 
        let q = Quaternion::new(8.1, 15.25, 0.1, 2.89);
        
        let equal = 17.5081310272216796875;
        assert_eq!(q.magnitude(), equal);
    }
        
    #[test]
    fn q_add_quaternion_test1()   
    {
        let mut q1 = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Quaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = q1 + q2;

        let equal = Quaternion::new(5.0, 2.0, 4.0, 6.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_add_quaternion_test2()   
    {
        let mut q1 = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = Quaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        q1 = q1 + q2;

        let equal = Quaternion::new(1.201631069183349609375, -0.375548690557479858398438, -0.0742167681455612182617188, -0.873655140399932861328125);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_scalar_test1()   
    {

        let mut q = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        q = q * 3.0;

        let equal = Quaternion::new(6.0, 3.0, 6.0, 9.0);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_multiply_by_scalar_test2()   
    {

        let mut q = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = q * 0.016666668;

        let equal = Quaternion::new(0.0166363250464200973510742, -0.000379226228687912225723267, -0.000666224921587854623794556, 0.00065028353128582239151001);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_divide_by_scalar_test1()   
    {
        let mut q = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        q = q / 3.0;

        let equal = Quaternion::new(0.666666686534881591796875, 0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_divide_by_scalar_test2()   
    {
        let mut q = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = q / 0.016666668;

        let equal = Quaternion::new(59.8907623291015625,-1.36521422863006591796875, -2.3984096050262451171875, 2.3410205841064453125);
        assert_eq!(q, equal);
    }
        
    #[test]
    fn q_conjugate_test1()   
    {
        let q1 = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Quaternion::conjugate(&q1);

        let equal = Quaternion::new(2.0, -1.0, -2.0, -3.0);
        assert_eq!(q2, equal);
    }
        
    #[test]
    fn q_conjugate_test2()   
    {
        let q1 = Quaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let q2 = Quaternion::conjugate(&q1);

        let equal = Quaternion::new(0.319945394992828369140625, -0.0418616682291030883789062, 0.26201188564300537109375, 0.90952312946319580078125);
        assert_eq!(q2, equal);
    }

    #[test]
    fn q_multiply_by_quaternion_test1()
    {
        let mut q1 = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = Quaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = q1 * q2;

        let equal = Quaternion::new(-8.0, 5.0, 10.0 , 15.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_quaternion_test2()
    {
        let mut q1 = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = Quaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        q1 = q1 * q2;

        let equal = Quaternion::new(0.345328778028488159179688, 0.0810852870345115661621094, -0.293385803699493408203125, -0.8877489566802978515625);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_vec_test1()
    {
        let mut q1 = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = Vector::new(3.0, 1.0, 2.0);
        q1 = q1 * v;

        let equal = Quaternion::new(-11.0, 7.0, 9.0, -1.0);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_multiply_by_vec_test2()
    {
        let mut q1 = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let v = Vector::new(127.105736, -13.1427, -10.31398);
        q1 = q1 * v;

        let equal = Quaternion::new(2.7691707611083984375, 127.79940032958984375, -8.39416790008544921875, -4.91529941558837890625);
        assert_eq!(q1, equal);
    }

    #[test]
    fn q_rotate_quaternion_by_vector_test1()
    {
        let q1 = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let mut v = Vector::new(3.0, 1.0, 2.0);
        v = Quaternion::qvrotate(&q1, &v);

        let equal = Vector::new(-4.0, 62.0, 26.0);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_rotate_quaternion_by_vector_test2()
    {
        let q1 = Quaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let mut v = Vector::new(127.105736, -13.1427, -10.31398);
        v = Quaternion::qvrotate(&q1, &v);

        let equal = Vector::new(128.1537475585937, -3.393682956695556640625, 0.285190761089324951171875);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_quaternion_from_euler_angles_test1()
    {
        let q = Quaternion::make_q_from_euler(5.0, 7.0, 10.0);

        let equal = Quaternion::new(0.993622303009033203125,  0.0380566865205764770507812, 0.0645529404282569885253906, 0.0842576175928115844726562);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_make_quaternion_from_euler_angles_test2()
    {
        let q = Quaternion::make_q_from_euler(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);

        let equal = Quaternion::new(0.998179376125335693359375, -0.0227535702288150787353516, -0.0399734899401664733886719, 0.0390170104801654815673828);
        assert_eq!(q, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test1()
    {
        let q = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = Quaternion::make_euler_from_q(&q);

        let equal = Vector::new(0.0, 90.0, 15.9453945159912109375);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test2()
    {
        let q = Quaternion::new(0.99817944, -0.022753572, -0.039973494,0.03901701);
        let v = Quaternion::make_euler_from_q(&q);

        let equal = Vector::new(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test3()
    {
        let q = Quaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let v = Quaternion::make_euler_from_q(&q);

        let equal = Vector::new(30.3658618927001953125, -5.25052165985107421875, -142.664825439453125);
        assert_eq!(v, equal);
    }

    #[test]
    fn q_make_euler_angles_from_quaternion_test4()
    {
        let q = Quaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        let v = Quaternion::make_euler_from_q(&q);

        let equal = Vector::new(-6.178070545196533203125, -41.140392303466796875, -152.5458831787109375);
        assert_eq!(v, equal);
    }
}