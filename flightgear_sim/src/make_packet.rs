//This file contains the MakePacket System

//SPECS
use specs::prelude::*;

//Get data needed for the System to work
use crate::data::Packet;
use crate::data::DataFDM;

//System to make a packet based on fgnetfdm structure required by FlightGear 
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
            let roll: f32 = -fdm.v_euler_angles.x.to_radians() as f32;
            let pitch: f32 = -fdm.v_euler_angles.y.to_radians() as f32; 
            let yaw: f32 = fdm.v_euler_angles.z.to_radians() as f32;

            //TRYING TO USE ECEF COORDINATES AND CONVERTING TO LLA
            //This does not work, all kinds of odd behavior takes place
            // let nalgebra_ecef_pos = Vector3::new(fdm.v_position.x, fdm.v_position.y, fdm.v_position.z);
            // let lla = geo::ecef2lla(&nalgebra_ecef_pos, &ELLIPSOID); 
            // pckt.fgnetfdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            // pckt.fgnetfdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); 
            // pckt.fgnetfdm.altitude = f64::from_be_bytes(lla.z.to_ne_bytes()); 

            //Set lat, long, alt
            pckt.fgnetfdm.latitude = f64::from_be_bytes((fdm.v_position.x as f64).to_ne_bytes());
            pckt.fgnetfdm.longitude = f64::from_be_bytes((fdm.v_position.y as f64).to_ne_bytes()); 
            pckt.fgnetfdm.altitude = f64::from_be_bytes((fdm.v_position.z as f64).to_ne_bytes()); //flightgear wants meters here, but on screen it displays feet
                                                                                    
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