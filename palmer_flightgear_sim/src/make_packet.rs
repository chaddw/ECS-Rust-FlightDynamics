//This file contains the MakePacket System

//Specs
use specs::prelude::*;

//Coordinate conversions
use coord_transforms::prelude::*;

//Get data needed for the System to work
use crate::data::Packet;
use crate::data::DataFDM; //this actually isnt needed for the system to work... having an issue only using the one keyboardstate writestorage component
use crate::data::FGNetFDM;

//Global ellipsoid variable inside of lazy static macro in main
use crate::ELLIPSOID;

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

            //Create fdm instance
            let mut fg_fdm: FGNetFDM = Default::default();

            //Set Roll, Pitch, Yaw
            let roll: f32 = fdm.bank as f32;
            let pitch: f32 = fdm.alpha as f32; //Will use angle of attack because its "easier"
            let yaw: f32 = 90.0; //Only need to face in one direction

            //Coordinate conversion: cartesian to geodetic
            let lla = geo::ecef2lla(&fdm.ecef_vec, &ELLIPSOID); 

            //Set lat, long, alt
            fg_fdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            fg_fdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
            fg_fdm.altitude = f64::from_be_bytes(fdm.q[5].to_ne_bytes()); //lla.z seems to increase altitude artificially...

            //Roll, Pitch, Yaw
            fg_fdm.phi = f32::from_be_bytes((roll.to_radians()).to_ne_bytes());
            fg_fdm.theta = f32::from_be_bytes((pitch.to_radians()).to_ne_bytes()); //will use angle of attack because its "easier"
            fg_fdm.psi = f32::from_be_bytes((yaw.to_radians()).to_ne_bytes());

            //Other airplane data
            let fg_net_fdm_version = 24_u32;
            fg_fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());

            //Convert struct to array of u8 bytes
            pckt.bytes = bincode::serialize(&fg_fdm).unwrap();

        }//end for
    }//end run
}//end system

