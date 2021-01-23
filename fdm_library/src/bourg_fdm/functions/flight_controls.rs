//Flight controls


use crate::bourg_fdm::components::component_datafdm::DataFDM;


pub fn thrust_up(fdm: &mut DataFDM, d_thrust: f32)
{
    fdm.thrustforce = fdm.thrustforce + d_thrust;
}

pub fn thrust_down(fdm: &mut DataFDM, d_thrust: f32)
{
    fdm.thrustforce = fdm.thrustforce - d_thrust;
}

pub fn yaw_left(fdm: &mut DataFDM)
{
    fdm.element[6].f_incidence = 16.0;
}

pub fn yaw_right(fdm: &mut DataFDM)
{
    fdm.element[6].f_incidence = -16.0;
}

pub fn roll_left(fdm: &mut DataFDM)
{
    fdm.element[0].i_flap = 1;
    fdm.element[3].i_flap = -1;
}

pub fn roll_right(fdm: &mut DataFDM)
{
    fdm.element[0].i_flap = -1;
    fdm.element[3].i_flap = 1;
}

pub fn pitch_up(fdm: &mut DataFDM)
{
    fdm.element[4].i_flap = 1;
    fdm.element[5].i_flap = 1;
}

pub fn pitch_down(fdm: &mut DataFDM)
{
    fdm.element[4].i_flap = -1;
    fdm.element[5].i_flap = -1;
}

pub fn flaps_down(fdm: &mut DataFDM)
{
    fdm.element[1].i_flap = -1;
    fdm.element[2].i_flap = -1;
    fdm.flaps = true;
}

pub fn zero_flaps(fdm: &mut DataFDM)
{
    fdm.element[1].i_flap = 0;
    fdm.element[2].i_flap = 0;
    fdm.flaps = false;
}

pub fn zero_ailerons(fdm: &mut DataFDM)
{
    fdm.element[0].i_flap = 0;
    fdm.element[3].i_flap = 0;
}

pub fn zero_rudder(fdm: &mut DataFDM)
{
    fdm.element[6].f_incidence = 0.0;
}

pub fn zero_elevators(fdm: &mut DataFDM)
{
    fdm.element[4].i_flap = 0;
    fdm.element[5].i_flap = 0;
}


