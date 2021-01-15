
//Helper functions for degrees <---> radians conversion
pub fn deg_to_rad(deg: f32) -> f32
{
    let pi: f32 = 3.14159265359;
	deg * pi / 180.0
}

pub fn rad_to_deg(rad: f32) -> f32
{	
    let pi: f32 = 3.14159265359;
	rad * 180.0 / pi
}