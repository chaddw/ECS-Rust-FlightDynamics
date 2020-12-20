//Import Vector, Matrix, Quaternion module
mod common;


fn print_matrix(m: &common::Mymatrix)
{
        println!("{} {} {}", m.e11, m.e12, m.e13);
        println!("{} {} {}", m.e21, m.e22, m.e23);
        println!("{} {} {}", m.e31, m.e32, m.e33);
}



fn main() 
{

        //TEST MATRIX
        println!("{}", "--------------"); 
        println!("{}", "Tests: Methods");
        println!("{}", "--------------"); 
     
     
        {  // inverse
                let m = common::Mymatrix::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
                println!("{}", "Inverse Test: Matrix3x3:"); 
                print_matrix(&m);
                println!("{}", "Inverse:"); 
                print_matrix(&m.inverse());
        }
     
        println!("{}", "--------------"); 
        println!("{}", "Tests: Functions and Operators");
        println!("{}", "--------------"); 
     
        {  // multiply matrix by vector

                let m = common::Mymatrix::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
                println!("{}", "Multiply matrix by vector test: Matrix3x3"); 
                print_matrix(&m);

                println!("{}", "Vector: (3,3,3)"); 

                let mut v = common::Mymatrix::multiply_matrix_by_vec(&m, &common::Myvec::new(3.0, 3.0, 3.0));
        
                 println!("Matrix * Vector : v.x: {} v.y: {} v.z: {}", v.x, v.y, v.z);
        }

}
