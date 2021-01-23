//To run the unit tests for the Matrix module from the command line: 
//cargo test --lib matrix

use crate::bourg_fdm::common::vector::Vector;

//Operator overloads
use std::ops::{Mul};

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct Matrix
{
    // elements eij: i -> row, j -> column
    pub e11: f32, pub e12: f32, pub e13: f32,
    pub e21: f32, pub e22: f32, pub e23: f32,
    pub e31: f32, pub e32: f32, pub e33: f32,
}


//Multiplication by vector
impl Mul<Vector> for Matrix
{
    type Output = Vector;

    fn mul(self, u: Vector) -> Vector 
    {
        let vector = Vector { 
            x: self.e11*u.x + self.e12*u.y + self.e13*u.z,
            y: self.e21*u.x + self.e22*u.y + self.e23*u.z,
            z: self.e31*u.x + self.e32*u.y + self.e33*u.z,
        };

        return vector;

    }
}


impl Matrix
{

    pub fn new( r1c1: f32,  r1c2: f32,  r1c3: f32, 
                r2c1: f32,  r2c2: f32,  r2c3: f32, 
                r3c1: f32,  r3c2: f32,  r3c3: f32) -> Matrix
    {
        Matrix { 
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

    pub fn inverse(&self) -> Matrix
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
    
        let matrix = Matrix
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

}

#[cfg(test)]
mod tests
{

    use super::*;

    //MATRIX TESTS
    #[test]
    fn m_inverse_test1()
    {  
        let m = Matrix::new(1.0, 2.0, 3.0, 
                            4.0, 5.0, 6.0,
                            7.0, 8.0, 9.0).inverse();

        let equal = Matrix::new(-3.0, 6.0, -3.0,
                                6.0, -12.0, 6.0,
                                -3.0, 6.0, -3.0);
        assert_eq!(m, equal);
    }


    #[test]
    fn m_inverse_test2()
    { 
        let m = Matrix::new(2549.629150390625, -0.0, 166.91925048828125, 
                            -0.0, 2024.4990234375, -0.0, 
                            166.91925048828125, -0.0, 4414.73388671875).inverse();

        let equal = Matrix::new(0.0003931871615350246429443359375, -0.0, -1.48662438732571899890899658203125e-05,
                                -0.0, 0.0004939493373967707157135009765625, -0.0,
                                -1.48662438732571899890899658203125e-05, -0.0, 0.0002270762925036251544952392578125);
        assert_eq!(m, equal);
    }

        
    #[test]
    fn m_multiply_by_vec_test1()
    {  
        let m = Matrix::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let v = m * Vector::new(3.0, 3.0, 3.0);

        let equal = Vector::new(18.0, 45.0, 72.0);
        assert_eq!(v, equal);

    }

    #[test]
    fn m_multiply_by_vec_test2()
    {  
        let m = Matrix::new(2549.629150390625, -0.0, 166.91925048828125, 
                            -0.0, 2024.4990234375, -0.0, 
                            166.91925048828125, -0.0, 4414.73388671875);

        let v = m * Vector::new(0.000029893242754042148590087890625, 0.063622482120990753173828125, -0.000000184518285095691680908203125);

        let equal = Vector::new(0.076185882091522216796875, 128.80364990234375, 0.00417515868321061134338379);
        assert_eq!(v, equal);

    }

}