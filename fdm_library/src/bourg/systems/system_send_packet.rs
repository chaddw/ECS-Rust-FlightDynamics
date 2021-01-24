//This file contains the SendPacket System

//SPECS
use specs::prelude::*;

//FGNetFDM structure
use crate::flightgear::FGNetFDM;

//Get function to call
use crate::bourg::fdm::send_packet::send;

//System to send packets after they are made
pub struct SendPacket;
impl<'a> System<'a> for SendPacket
{
    type SystemData = ReadStorage<'a, FGNetFDM>;

    fn run(&mut self, fgnetfdm: Self::SystemData) 
    {
        for fgnet in fgnetfdm.join() 
        {
            send(fgnet);
        }
    }
}