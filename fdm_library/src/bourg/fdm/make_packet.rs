//Function to load the FGNetFDM structure with updated data

//Get data needed for the System to work
use crate::bourg::fdm::structures::DataFDM;
use crate::flightgear::FGNetFDM;

pub fn load_fgnetfdm(fdm: &DataFDM, fgnet: &mut FGNetFDM)
{
    //All data passed into the FGNetFDM struct is converted to network byte order
    
    //Set Roll, Pitch, Yaw in radians
    //Negate to compensate for coordinate handedness differences
    let roll: f32 = fdm.v_euler_angles.x.to_radians() as f32;
    let pitch: f32 = -fdm.v_euler_angles.y.to_radians() as f32; 
    let yaw: f32 = (90.0 + -fdm.v_euler_angles.z).to_radians() as f32;

    //Lat and lon degrees need to be converted to radians for FlightGear
    let lat: f64 = fdm.v_position.x.to_radians() as f64;
    let lon: f64 = fdm.v_position.y.to_radians() as f64;
    let alt: f64 = fdm.v_position.z as f64; 

    //Set lat, long, alt
    fgnet.latitude = f64::from_be_bytes(lat.to_ne_bytes());
    fgnet.longitude = f64::from_be_bytes(lon.to_ne_bytes()); 
    fgnet.altitude = f64::from_be_bytes(alt.to_ne_bytes()); 
                                                                            
    //Roll, Pitch, Yaw
    fgnet.phi = f32::from_be_bytes(roll.to_ne_bytes());
    fgnet.theta = f32::from_be_bytes(pitch.to_ne_bytes()); 
    fgnet.psi = f32::from_be_bytes(yaw.to_ne_bytes());

    //Other airplane data
    let fg_net_fdm_version = 24_u32;
    fgnet.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

}