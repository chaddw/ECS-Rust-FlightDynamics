//This file contains the MakePacket System

//SPECS
use specs::prelude::*;

//Get data needed for the System to work
use crate::palmer_fdm::components::component_packet::Packet;
use crate::palmer_fdm::components::component_datafdm::DataFDM;

//System to make packet based on fgnetfdm structure required by FlightGear 
pub struct MakePacket;
impl<'a> System<'a> for MakePacket
{
    type SystemData = (
        ReadStorage<'a, DataFDM>,
        WriteStorage<'a, Packet>,
    );

    fn run(&mut self, (datafdm, mut packet): Self::SystemData) 
    {

        for (fdm, mut pckt) in (&datafdm, &mut packet).join() 
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
            let alt = fdm.position.z; //add starting elevation offset to altitude

            //Set lat, long, alt
            pckt.fgnetfdm.latitude = f64::from_be_bytes(lat.to_ne_bytes()); 
            pckt.fgnetfdm.longitude = f64::from_be_bytes(lon.to_ne_bytes()); 
            pckt.fgnetfdm.altitude = f64::from_be_bytes(alt.to_ne_bytes()); 

            //Roll, Pitch, Yaw
            pckt.fgnetfdm.phi = f32::from_be_bytes(roll.to_ne_bytes());
            pckt.fgnetfdm.theta = f32::from_be_bytes(pitch.to_ne_bytes());
            pckt.fgnetfdm.psi = f32::from_be_bytes(yaw.to_ne_bytes());

            //Other airplane data
            let fg_net_fdm_version = 24_u32;
            pckt.fgnetfdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

            //Convert struct to array of u8 bytes
            pckt.bytes = bincode::serialize(&pckt.fgnetfdm).unwrap();

        }
    }
}

