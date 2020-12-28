//This file contains the MakePacket System

//SPECS
use specs::prelude::*;

//Get data needed for the System to work
use crate::data::Packet;
use crate::data::DataFDM;

//System to make packets
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

            //Set Roll, Pitch, Yaw
            let roll: f32 = fdm.bank as f32;
            let pitch: f32 = fdm.alpha as f32;
            let yaw: f32 = (90.0 + -fdm.heading_angle.to_degrees()) as f32; //heading on the longitude x axis

            //Set lat, lon, and alt. Lat and lon need to be converted to radians for FlightGear
            let lat = fdm.position[0].to_radians();
            let lon = fdm.position[1].to_radians();
            let alt = fdm.position[2] + fdm.q[5]; //add starting elevation offset to altitude

            //Set lat, long, alt
            pckt.fgnetfdm.latitude = f64::from_be_bytes(lat.to_ne_bytes()); 
            pckt.fgnetfdm.longitude = f64::from_be_bytes(lon.to_ne_bytes()); 
            pckt.fgnetfdm.altitude = f64::from_be_bytes(alt.to_ne_bytes()); 

            //Roll, Pitch, Yaw
            pckt.fgnetfdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
            pckt.fgnetfdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes());
            pckt.fgnetfdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

            //Other airplane data
            let fg_net_fdm_version = 24_u32;
            pckt.fgnetfdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

            //Convert struct to array of u8 bytes
            pckt.bytes = bincode::serialize(&pckt.fgnetfdm).unwrap();

        }
    }
}

