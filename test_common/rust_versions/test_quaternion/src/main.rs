//Import Vector, Matrix, Quaternion module
mod common;

#[cfg(test)]


//QUATERNION TESTS

#[test]
fn q_magnitude_test1()   
{ 
        let q = common::Myquaternion::new(2.0, 2.0, 2.0, 2.0);
        
        let equal = 4.0;
        assert_eq!(q.magnitude(), equal);
}

#[test]
fn q_magnitude_test2()   
{ 
        let q = common::Myquaternion::new(8.1, 15.25, 0.1, 2.89);
        
        let equal = 17.5081310272216796875;
        assert_eq!(q.magnitude(), equal);
}
     
#[test]
fn q_add_quaternion_test1()   
{
        let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = common::Myquaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = common::Myquaternion::addquat(&q1, &q2);

        let equal = common::Myquaternion::new(5.0, 2.0, 4.0, 6.0);
        assert_eq!(q1, equal);
}

#[test]
fn q_add_quaternion_test2()   
{
        let mut q1 = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = common::Myquaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        q1 = common::Myquaternion::addquat(&q1, &q2);

        let equal = common::Myquaternion::new(1.201631069183349609375, -0.375548690557479858398438, -0.0742167681455612182617188, -0.873655140399932861328125);
        assert_eq!(q1, equal);
}

#[test]
fn q_multiply_by_scalar_test1()   
{

        let mut q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        q = common::Myquaternion::multiplyscalar(&q, 3.0);

        let equal = common::Myquaternion::new(6.0, 3.0, 6.0, 9.0);
        assert_eq!(q, equal);
}

#[test]
fn q_multiply_by_scalar_test2()   
{

        let mut q = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = common::Myquaternion::multiplyscalar(&q, 0.016666668);

        let equal = common::Myquaternion::new(0.0166363250464200973510742, -0.000379226228687912225723267, -0.000666224921587854623794556, 0.00065028353128582239151001);
        assert_eq!(q, equal);
}

#[test]
fn q_divide_by_scalar_test1()   
{
        let mut q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        q = common::Myquaternion::dividescalar(&q, 3.0);

        let equal = common::Myquaternion::new(0.666666686534881591796875, 0.333333343267440795898438, 0.666666686534881591796875, 1.0);
        assert_eq!(q, equal);
}

#[test]
fn q_divide_by_scalar_test2()   
{
        let mut q = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        q = common::Myquaternion::dividescalar(&q, 0.016666668 );

        let equal = common::Myquaternion::new(59.8907623291015625,-1.36521422863006591796875, -2.3984096050262451171875, 2.3410205841064453125);
        assert_eq!(q, equal);
}
     
#[test]
fn q_conjugate_test1()   
{
        let q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = common::Myquaternion::conjugate(&q1);

        let equal = common::Myquaternion::new(2.0, -1.0, -2.0, -3.0);
        assert_eq!(q2, equal);
}
     
#[test]
fn q_conjugate_test2()   
{
        let q1 = common::Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let q2 = common::Myquaternion::conjugate(&q1);

        let equal = common::Myquaternion::new(0.319945394992828369140625, -0.0418616682291030883789062, 0.26201188564300537109375, 0.90952312946319580078125);
        assert_eq!(q2, equal);
}

#[test]
fn q_multiply_by_quaternion_test1()
{
        let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let q2 = common::Myquaternion::new(3.0, 1.0, 2.0, 3.0);
        q1 = common::Myquaternion::multiplyquat(&q1, &q2);

        let equal = common::Myquaternion::new(-8.0, 5.0, 10.0 , 15.0);
        assert_eq!(q1, equal);
}

#[test]
fn q_multiply_by_quaternion_test2()
{
        let mut q1 = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let q2 = common::Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        q1 = common::Myquaternion::multiplyquat(&q1, &q2);

        let equal = common::Myquaternion::new(0.345328778028488159179688, 0.0810852870345115661621094, -0.293385803699493408203125, -0.8877489566802978515625);
        assert_eq!(q1, equal);
}

#[test]
fn q_multiply_by_vec_test1()
{
        let mut q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = common::Myvec::new(3.0, 1.0, 2.0);
        q1 = common::Myquaternion::multiply_quat_by_vec(&q1, &v);

        let equal = common::Myquaternion::new(-11.0, 7.0, 9.0, -1.0);
        assert_eq!(q1, equal);
}

#[test]
fn q_multiply_by_vec_test2()
{
        let mut q1 = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let v = common::Myvec::new(127.105736, -13.1427, -10.31398);
        q1 = common::Myquaternion::multiply_quat_by_vec(&q1, &v);

        let equal = common::Myquaternion::new(2.7691707611083984375, 127.79940032958984375, -8.39416790008544921875, -4.91529941558837890625);
        assert_eq!(q1, equal);
}

#[test]
fn q_rotate_quaternion_by_vector_test1()
{
        let q1 = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let mut v = common::Myvec::new(3.0, 1.0, 2.0);
        v = common::Myquaternion::qvrotate(&q1, &v);

        let equal = common::Myvec::new(-4.0, 62.0, 26.0);
        assert_eq!(v, equal);
}

#[test]
fn q_rotate_quaternion_by_vector_test2()
{
        let q1 = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494, 0.03901701);
        let mut v = common::Myvec::new(127.105736, -13.1427, -10.31398);
        v = common::Myquaternion::qvrotate(&q1, &v);

        let equal = common::Myvec::new(128.1537475585937, -3.393682956695556640625, 0.285190761089324951171875);
        assert_eq!(v, equal);
}



#[test]
fn q_make_quaternion_from_euler_angles_test1()
{
        let q = common::Myquaternion::make_q_from_euler(5.0, 7.0, 10.0);

        let equal = common::Myquaternion::new(0.993622303009033203125,  0.0380566865205764770507812, 0.0645529404282569885253906, 0.0842576175928115844726562);
        assert_eq!(q, equal);
}

#[test]
fn q_make_quaternion_from_euler_angles_test2()
{
        let q = common::Myquaternion::make_q_from_euler(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);

        let equal = common::Myquaternion::new(0.998179376125335693359375, -0.0227535702288150787353516, -0.0399734899401664733886719, 0.0390170104801654815673828);
        assert_eq!(q, equal);
}


//error!
#[test]
fn q_make_euler_angles_from_quaternion_test1()
{
        let q = common::Myquaternion::new(2.0, 1.0, 2.0, 3.0);
        let v = common::Myquaternion::make_euler_from_q(&q);

        let equal = common::Myvec::new(0.0, 90.0, 15.9453945159912109375);
        assert_eq!(v, equal);
}

//error!
#[test]
fn q_make_euler_angles_from_quaternion_test2()
{
        let q = common::Myquaternion::new(0.99817944, -0.022753572, -0.039973494,0.03901701);
        let v = common::Myquaternion::make_euler_from_q(&q);

        let equal = common::Myvec::new(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);
        assert_eq!(v, equal);
}

//error!
#[test]
fn q_make_euler_angles_from_quaternion_test3()
{
        let q = common::Myquaternion::new(0.3199454, 0.04186167, -0.2620119, -0.9095231);
        let v = common::Myquaternion::make_euler_from_q(&q);

        let equal = common::Myvec::new(30.3658618927001953125, -5.25052165985107421875, -142.664825439453125);
        assert_eq!(v, equal);
}

//error!
#[test]
fn q_make_euler_angles_from_quaternion_test4()
{
        let q = common::Myquaternion::new(0.20345166, -0.35279512, -0.034243274, -0.91267216);
        let v = common::Myquaternion::make_euler_from_q(&q);

        let equal = common::Myvec::new(-6.178070545196533203125, -41.140392303466796875, -152.5458831787109375);
        assert_eq!(v, equal);
}