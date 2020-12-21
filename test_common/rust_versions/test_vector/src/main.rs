//Import Vector, Matrix, Quaternion module
mod common;

fn main() 
{
        //Precision is edited to match the default 6 significant digits c++ percision in bourgs tests
        
        //VECTOR TEST
        println!("{}", "--------------"); 
        println!("{}", "Tests: Methods");
        println!("{}", "--------------"); 
     
        {
                let v = common::Myvec::new(1.0, 1.0, 1.0);
                println!("magnitude v(1,1,1): {:.5}", v.magnitude());
        }
     
        {
                let v = common::Myvec::new(1.0, 2.0, 3.0);
                println!("magnitude v(1,2,3): {:.5}", v.magnitude());
        }
     
        {
                let mut v = common::Myvec::new(1.0, 1.0, 1.0);
                v.normalize();
                println!("normalize v(1,1,1): v.x: {:.5} v.y: {:.5} v.z: {:.5}", v.x, v.y, v.z);
        }
     
        {
                let mut v = common::Myvec::new(3.0, 3.0, 3.0);
                v.normalize();
                println!("normalize v(3,3,3): v.x: {:.5} v.y: {:.5} v.z: {:.5}", v.x, v.y, v.z);
        }
     
        {
                let mut v = common::Myvec::new(1.0, 2.0, 3.0);
                v.normalize();
                println!("normalize v(1,2,3): v.x: {:.6} v.y: {:.6} v.z: {:.6}", v.x, v.y, v.z);
        }
     
        {
                let mut v = common::Myvec::new(1.0, 2.0, 3.0);
                v = common::Myvec::reverse_aka_conjugate(&v);
                println!("reverse v(1,2,3): v.x: {} v.y: {} v.z: {}", v.x, v.y, v.z);
        }
     
     
        println!("{}", "--------------"); 
        println!("{}", "Tests: Functions and Operators");
        println!("{}", "--------------"); 

        {
                let mut v = common::Myvec::new(10.0, 20.0, 30.0);
                let mut u = common::Myvec::new(1.0, 2.0, 3.0);
                let uv = common::Myvec::addvec(&v, &u);
                println!("addvec : uv = v(10,20,30) + u(1,2,3) uv.x: {} uv.y: {} uv.z: {}", uv.x, uv.y, uv.z);
        }
     
        {
                let mut v = common::Myvec::new(10.0, 20.0, 30.0);
                let mut u = common::Myvec::new(1.0, 2.0, 3.0);
                let uv = common::Myvec::subtractvec(&v, &u);
                println!("subtractvec : uv = v(10,20,30) - u(1,2,3) uv.x: {} uv.y: {} uv.z: {}", uv.x, uv.y, uv.z);
        }
     
        {
                //crossproduct
                let mut v = common::Myvec::new(1.0, 0.0, 0.0);
                let mut u = common::Myvec::new(0.0, 1.0, 1.0);
                let uv = common::Myvec::crossproduct(&v, &u);
                println!("crossproduct : uv = v(1,0,0) ^ u(0,1,1) uv.x: {} uv.y: {} uv.z: {}", uv.x, uv.y, uv.z);
        }
     
        {
                //dot product
                let mut v = common::Myvec::new(10.0, 20.0, 30.0);
                let mut u = common::Myvec::new(1.0, 2.0, 3.0);
                let uv = common::Myvec::dotproduct(&v, &u);
                println!("dotproduct : uv = v(10,20,30) * u(1,2,3) u.v: {}", uv);
        }
     
        {
                let mut v = common::Myvec::new(1.0, 2.0, 3.0);
                let u = common::Myvec::multiplyscalar(&v, 3.0);
                println!("multiplyscalar : u = 3 * v(1,2,3) u.x: {} u.y: {} u.z: {}", u.x, u.y, u.z);

        }
     
        {
                let mut v = common::Myvec::new(1.0, 2.0, 3.0);
                let u = common::Myvec::dividescalar(&v, 3.0);
                println!("dividescalar : u = 3 / v(1,2,3) u.x: {:.6} u.y: {:.6} u.z: {}", u.x, u.y, u.z);
        }


}
