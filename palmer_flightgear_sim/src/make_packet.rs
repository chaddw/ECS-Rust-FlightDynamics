//This file contains the MakePacket System

//SPECS
use specs::prelude::*;

//Coordinate conversions
use coord_transforms::prelude::*;

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
            let yaw: f32 = 90.0; //Only need to face in one direction

            //Define earth ellipsoid for coordinate conversion
            let ellipsoid: coord_transforms::structs::geo_ellipsoid::geo_ellipsoid = geo_ellipsoid::geo_ellipsoid::new(geo_ellipsoid::WGS84_SEMI_MAJOR_AXIS_METERS, geo_ellipsoid::WGS84_FLATTENING);

            //Coordinate conversion: cartesian to geodetic
            let lla = geo::ecef2lla(&fdm.ecef_vec, &ellipsoid); 

            //Set lat, long, alt
            pckt.fgnetfdm.latitude = f64::from_be_bytes(lla.x.to_ne_bytes());
            pckt.fgnetfdm.longitude = f64::from_be_bytes(lla.y.to_ne_bytes()); //this stays fixed
            pckt.fgnetfdm.altitude = f64::from_be_bytes(fdm.q[5].to_ne_bytes());

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

