
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



