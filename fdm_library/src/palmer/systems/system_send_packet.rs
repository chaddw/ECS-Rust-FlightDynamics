//This file contains the SendPacket System

//SPECS
use specs::prelude::*;

//FGNetFDM structure
use crate::flightgear::FGNetFDM;

//Function to call
use crate::palmer::fdm::send_packet::send;

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
