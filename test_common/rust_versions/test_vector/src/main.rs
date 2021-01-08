//Import Vector, Matrix, Quaternion module
mod common;

#[cfg(test)]



//VECTOR TESTS
#[test]
fn v_magnitude_test1() 
{
        let v = common::Myvec::new(1.0, 1.0, 1.0);
        assert_eq!(v.magnitude(), 1.73205077648162841796875);
}

#[test]
fn v_magnitude_test2()
{
        let v = common::Myvec::new(1.0, 2.0, 3.0);
        assert_eq!(v.magnitude(), 3.7416574954986572265625);
}

#[test]
fn v_normalize_test1()
{
        let mut v = common::Myvec::new(1.0, 1.0, 1.0);
        v.normalize();

        let equal = common::Myvec::new(0.57735025882720947265625, 0.57735025882720947265625, 0.57735025882720947265625);
        assert_eq!(v, equal);
}

#[test]
fn v_normalize_test2()
{
        let mut v = common::Myvec::new(3.0, 3.0, 3.0);
        v.normalize();

        let equal = common::Myvec::new( 0.577350318431854248046875, 0.577350318431854248046875,0.577350318431854248046875);
        assert_eq!(v, equal);
}

#[test]
fn v_normalize_test3()
{
        let mut v = common::Myvec::new(1.0, 2.0, 3.0);
        v.normalize();

        let equal = common::Myvec::new(0.267261236906051635742188, 0.534522473812103271484375, 0.80178368091583251953125);
        assert_eq!(v, equal);

}

#[test]
fn v_reverse_test1()
{
        let mut v = common::Myvec::new(1.0, 2.0, 3.0);
        v = common::Myvec::reverse(&v);

        let equal = common::Myvec::new(-1.0, -2.0, -3.0);
        assert_eq!(v, equal);
}



#[test]
fn v_add_test1()
{
        let v = common::Myvec::new(10.0, 20.0, 30.0);
        let u = common::Myvec::new(1.0, 2.0, 3.0);
        let uv = common::Myvec::addvec(&v, &u);

        let equal = common::Myvec::new(11.0, 22.0, 33.0);
        assert_eq!(uv, equal);
}
     
#[test]
fn v_subtract_test1()
{
        let v = common::Myvec::new(10.0, 20.0, 30.0);
        let u = common::Myvec::new(1.0, 2.0, 3.0);
        let uv = common::Myvec::subtractvec(&v, &u);

        let equal = common::Myvec::new(9.0, 18.0, 27.0);
        assert_eq!(uv, equal);
}
     
#[test]
fn v_cross_product_test1()
{
        //crossproduct
        let v = common::Myvec::new(1.0, 0.0, 0.0);
        let u = common::Myvec::new(0.0, 1.0, 1.0);
        let uv = common::Myvec::crossproduct(&v, &u);

        let equal = common::Myvec::new(0.0, -1.0, 1.0);
        assert_eq!(uv, equal);
}
  
#[test]
fn v_dot_product_test1()
{
        //dot product
        let v = common::Myvec::new(10.0, 20.0, 30.0);
        let u = common::Myvec::new(1.0, 2.0, 3.0);
        let uv = common::Myvec::dotproduct(&v, &u);

        assert_eq!(uv, 140.0);
}
     

#[test]
fn v_multiply_by_scalar_test1()
{
        let v = common::Myvec::new(1.0, 2.0, 3.0);
        let u = common::Myvec::multiplyscalar(&v, 3.0);

        let equal = common::Myvec::new(3.0, 6.0, 9.0);
        assert_eq!(u, equal);
}
     
#[test]
fn v_divide_by_scalar_test1()
{
        let v = common::Myvec::new(1.0, 2.0, 3.0);
        let u = common::Myvec::dividescalar(&v, 3.0);

        let equal = common::Myvec::new(0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(u, equal);
}


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