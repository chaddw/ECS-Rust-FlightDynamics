//Calculate mass properties based on the airplane's different body pieces
//This is called from inside main before the airplane Entity is created


//Vector, Matrix
use crate::bourg_fdm::common::vector::Vector;
use crate::bourg_fdm::common::matrix::Matrix;

//Math utils
use crate::bourg_fdm::common::math_utils::deg_to_rad;

//DataFDM Component
use crate::bourg_fdm::components::component_datafdm::DataFDM;

pub fn calc_airplane_mass_properties(fdm: &mut DataFDM)
{
    let mut inc: f32;
    let mut di: f32;

    //Calculate the normal (perpendicular) vector to each lifting surface. This is needed for relative air velocity to find lift and drag.
    for  i in fdm.element.iter_mut()
    {
        inc = deg_to_rad(i.f_incidence);
        di = deg_to_rad(i.f_dihedral);
        i.v_normal = Vector::new(inc.sin(), inc.cos() * di.sin(), inc.cos() * di.cos());
        i.v_normal.normalize(); 
    }

    //Calculate total mass
    let mut total_mass: f32 = 0.0;
    for i in fdm.element.iter()
    {
        total_mass = total_mass + i.f_mass;
    }

    //Calculate combined center of gravity location
    let mut v_moment = Vector::new(0.0,0.0,0.0);
    for i in fdm.element.iter()
    {
        let _tmp = i.v_d_coords * i.f_mass;
        v_moment = v_moment + _tmp;
    }
    let cg = v_moment / total_mass; 

    //Calculate coordinates of each element with respect to the combined CG, relative position
    for i in fdm.element.iter_mut()
    {
        i.v_cg_coords = i.v_d_coords - cg;
    }

    //Calculate the moments and products of intertia for the combined elements
    let mut ixx: f32 = 0.0;
    let mut iyy: f32 = 0.0;
    let mut izz: f32 = 0.0;
    let mut ixy: f32 = 0.0;
    let mut ixz: f32 = 0.0;
    let mut iyz: f32 = 0.0;

    for i in fdm.element.iter()
    {
        ixx = ixx + i.v_local_inertia.x + i.f_mass *
            (i.v_cg_coords.y * i.v_cg_coords.y +
            i.v_cg_coords.z * i.v_cg_coords.z);

        iyy = iyy + i.v_local_inertia.y + i.f_mass *
            (i.v_cg_coords.z * i.v_cg_coords.z +
            i.v_cg_coords.x * i.v_cg_coords.x);

        izz = izz + i.v_local_inertia.z + i.f_mass *
            (i.v_cg_coords.x * i.v_cg_coords.x +
            i.v_cg_coords.y * i.v_cg_coords.y);

        ixy = ixy + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.y);

        ixz = ixz + i.f_mass * (i.v_cg_coords.x * 
            i.v_cg_coords.z);
        
        iyz = iyz + i.f_mass * (i.v_cg_coords.y *
            i.v_cg_coords.z);
    }

    //Finally, set up airplanes mass and inertia Matrix
    fdm.mass = total_mass;
    fdm.m_inertia = Matrix::new(ixx, -ixy, -ixz,
                                  -ixy, iyy, -iyz,
                                  -ixz, -iyz, izz);

    //Get inverse of Matrix
    fdm.m_inertia_inverse = Matrix::inverse(&fdm.m_inertia);
}


