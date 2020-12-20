//Import Vector, Matrix, Quaternion module
mod common;


fn main() 
{
        //Precision is edited to match the default 6 significant digits c++ percision in bourgs tests
    
        //TEST QUATERNION
        println!("{}", "--------------"); 
        println!("{}", "Tests: Methods");
        println!("{}", "--------------"); 
     
        {  // Magnitude

                let q = common::Myquaternion::new(2.0, 2.0, 2.0, 2.0);

                println!("Magnitude of q(2,2,2,2) : {}", q.magnitude());
        }
     
     
        {  // addquat
                let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let q2 = common::Myquaternion::new(3.0, 1.0, 2.0, 3.0);
                q1 = common::Myquaternion::addquat(&q1, &q2);

                println!("addquat        : {} {} {}", q1.v.x, q1.v.y, q1.v.z);
                println!("               : {}", q1.n);
        }
     
        {  // multiplyscalar

                let mut q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                q = common::Myquaternion::multiplyscalar(&q, 3.0);
                println!("multiplyscalar : {} {} {}", q.v.x, q.v.y, q.v.z);
                println!("               : {}", q.n);
        }
     
        {  // dividescalar

                let mut q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                q = common::Myquaternion::dividescalar(&q, 3.0);
                println!("dividescalar   : {:.6} {:.6} {}", q.v.x, q.v.y, q.v.z);
                println!("               : {:.6}", q.n);
        }
     
        {  // conjugate
                let q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let q2 = common::Myquaternion::conjugate(&q1);
                println!("conjugate      : {} {} {}", q2.v.x, q2.v.y, q2.v.z);
                println!("               : {}", q2.n);
        }
     
        println!("{}", "--------------"); 
        println!("{}", "Tests: Functions and Operators");
        println!("{}", "--------------"); 

        //multiplyquat
        {
                let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let q2 = common::Myquaternion::new(3.0, 1.0, 2.0, 3.0);
                q1 = common::Myquaternion::multiplyquat(&q1, &q2);

                println!("multiplyquat   : {} {} {}", q1.v.x, q1.v.y, q1.v.z);
                println!("               : {}", q1.n);
        }

        //multiply_quat_by_vec
        {
                let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let v = common::Myvec::new(3.0, 1.0, 2.0);
                q1 = common::Myquaternion::multiply_quat_by_vec(&q1, &v);

                println!("multply quaternion by vector : {} {} {}", q1.v.x, q1.v.y, q1.v.z);
                println!("                             : {}", q1.n);
        }

        //qvrotate rotate quaternion by vector
        {
                let q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let mut v = common::Myvec::new(3.0, 1.0, 2.0);
                v = common::Myquaternion::qvrotate(&q1, &v);

                println!("qvrotate       : {} {} {}", v.x, v.y, v.z);
        }

        //make_q_from_euler
        {
                let q = common::Myquaternion::make_q_from_euler(5.0, 7.0, 10.0);
                println!("make q from euler angles : {:.7} {:.7} {:.7}", q.v.x, q.v.y, q.v.z);
                println!("                         : {:.6}", q.n);
        }


        //make_euler_from_q
        {
                let mut q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
                let v = common::Myquaternion::make_euler_from_q(&q);
                println!("make euler angles from q : {} {} {:.4}", v.x, v.y, v.z);
 
        }
}
