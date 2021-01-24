//Function to load the FGNetFDM structure with updated data

//Get data needed for the System to work
use crate::palmer::fdm::structures::DataFDM;
use crate::flightgear::FGNetFDM;

//System to make packet based on fgnetfdm structure required by FlightGear 
pub fn load_fgnetfdm(fdm: &DataFDM, fgnet: &mut FGNetFDM)
{
    //All data passed into the FGNetFDM struct is converted to network byte order

    //Set Roll, Pitch, Yaw in radians
    //Negate to compensate for coordinate differences
    let roll: f32 = fdm.bank.to_radians() as f32;
    let pitch: f32 = fdm.alpha.to_radians() as f32;
    let yaw: f32 = 90.0_f32.to_radians() + -fdm.heading_angle as f32; //heading angle is already in radians

    //Lat and lon degrees need to be converted to radians for FlightGear
    let lat = fdm.position.x.to_radians();
    let lon = fdm.position.y.to_radians();
    let alt = fdm.position.z;

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