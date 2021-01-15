
#[derive(Debug, Default, PartialEq)]
pub struct Myvec
{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Myvec
{
    pub fn new(x: f32, y: f32, z: f32) -> Myvec
    {
        Myvec {x: x, y: y, z: z}
    }

    pub fn magnitude(&self) -> f32
    {
        return (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
    }

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

    pub fn dotproduct(u: &Myvec, v: &Myvec) -> f32
    {

        return u.x*v.x + u.y*v.y + u.z*v.z;

    }
}


#[cfg(test)]
mod tests
{

    use super::*;

    //VECTOR TESTS
    #[test]
    fn v_magnitude_test1() 
    {
        let v = Myvec::new(1.0, 1.0, 1.0);
        assert_eq!(v.magnitude(), 1.73205077648162841796875);
    }

    #[test]
    fn v_magnitude_test2()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        assert_eq!(v.magnitude(), 3.7416574954986572265625);
    }

    #[test]
    fn v_magnitude_test3()
    {
        let v = Myvec::new(63.73516082763671875, -0.004545502364635467529296875, -8.11302661895751953125);
        assert_eq!(v.magnitude(), 64.24945068359375);
    }


    #[test]
    fn v_normalize_test1()
    {
        let mut v = Myvec::new(1.0, 1.0, 1.0);
        v.normalize();

        let equal = Myvec::new(0.57735025882720947265625, 0.57735025882720947265625, 0.57735025882720947265625);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test2()
    {
        let mut v = Myvec::new(3.0, 3.0, 3.0);
        v.normalize();

        let equal = Myvec::new(0.577350318431854248046875, 0.577350318431854248046875,0.577350318431854248046875);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test3()
    {
        let mut v = Myvec::new(1.0, 2.0, 3.0);
        v.normalize();

        let equal = Myvec::new(0.267261236906051635742188, 0.534522473812103271484375, 0.80178368091583251953125);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_normalize_test4()
    {
        let mut v = Myvec::new(-0.061048544943332672119140625, 0.0, 0.998134791851043701171875);
        v.normalize();

        let equal = Myvec::new(-0.0610485486686229705810546875, 0.0, 0.9981348514556884765625);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_reverse_test1()
    {
        let mut v = Myvec::new(1.0, 2.0, 3.0);
        v = Myvec::reverse(&v);

        let equal = Myvec::new(-1.0, -2.0, -3.0);
        assert_eq!(v, equal);
    }

    #[test]
    fn v_add_test1()
    {
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::addvec(&v, &u);

        let equal = Myvec::new(11.0, 22.0, 33.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_add_test2()
    {
        let v = Myvec::new(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.0007719345740042626857757568359375);

        let u = Myvec::new(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

        let uv = Myvec::addvec(&v, &u);

        let equal = Myvec::new(0.000301808759104460477828979, -0.00649327132850885391235352, 0.000772719329688698053359985);


        assert_eq!(uv, equal);
    }
        
    #[test]
    fn v_subtract_test1()
    {
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::subtractvec(&v, &u);

        let equal = Myvec::new(9.0, 18.0, 27.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_subtract_test2()
    {
        let v = Myvec::new(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.0007719345740042626857757568359375);

        let u = Myvec::new(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

        let uv = Myvec::subtractvec(&v, &u);

        let equal = Myvec::new(0.000281422457192093133926392, 0.00410096254199743270874023, 0.000771149818319827318191528);


        assert_eq!(uv, equal);
    }




    #[test]
    fn v_cross_product_test1()
    {
        
        let v = Myvec::new(1.0, 0.0, 0.0);
        let u = Myvec::new(0.0, 1.0, 1.0);
        let uv = Myvec::crossproduct(&v, &u);

        let equal = Myvec::new(0.0, -1.0, 1.0);
        assert_eq!(uv, equal);
    }

    #[test]
    fn v_cross_product_test2()
    {
        
        let v = Myvec::new( 1.01931509561836719512939453125e-05,
                            -0.005297116935253143310546875,
                            7.84755684435367584228515625e-07);

        let u = Myvec::new(1.81550502777099609375,
                        0.0,
                        -0.6772263050079345703125);

        let uv = Myvec::crossproduct(&v, &u);

        let equal = Myvec::new(0.0035873469896614551544189453125,
                            8.327797331730835139751434326171875e-06,
                            0.009616942144930362701416015625);
                            
        assert_eq!(uv, equal);

    }

    #[test]
    fn v_dot_product_test1()
    {
        
        let v = Myvec::new(10.0, 20.0, 30.0);
        let u = Myvec::new(1.0, 2.0, 3.0);
        let uv = Myvec::dotproduct(&v, &u);

        assert_eq!(uv, 140.0);
    }
        


    #[test]
    fn v_dot_product_test2()
    {
        
        let v = Myvec::new(-0.9932043552398681640625,
            -3.16645900966250337660312652587890625e-06,
            0.116383351385593414306640625);

        let u = Myvec::new(0.0, 0.0, 1.0);

        let uv = Myvec::dotproduct(&v, &u);

        assert_eq!(uv, 0.116383351385593414306640625);
    }

    #[test]
    fn v_multiply_by_scalar_test1()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        let u = Myvec::multiplyscalar(&v, 3.0);

        let equal = Myvec::new(3.0, 6.0, 9.0);
        assert_eq!(u, equal);
    }
        
    #[test]
    fn v_divide_by_scalar_test1()
    {
        let v = Myvec::new(1.0, 2.0, 3.0);
        let u = Myvec::dividescalar(&v, 3.0);

        let equal = Myvec::new(0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(u, equal);
    }


}