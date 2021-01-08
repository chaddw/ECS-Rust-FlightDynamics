//Import Vector, Matrix, Quaternion module
mod common;

#[cfg(test)]



//MATRIX TESTS
#[test]
fn m_inverse_test1()
{  
        let m = common::Mymatrix::new(1.0, 2.0, 3.0, 
                                          4.0, 5.0, 6.0,
                                          7.0, 8.0, 9.0).inverse();

        let equal = common::Mymatrix::new(-3.0, 6.0, -3.0,
                                          6.0, -12.0, 6.0,
                                          -3.0, 6.0, -3.0);
        assert_eq!(m, equal);
}
     
     
#[test]
fn m_multiply_by_vec_test1()
{  
        let m = common::Mymatrix::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let v = common::Mymatrix::multiply_matrix_by_vec(&m, &common::Myvec::new(3.0, 3.0, 3.0));

        let equal = common::Myvec::new(18.0, 45.0, 72.0);
        assert_eq!(v, equal);

}