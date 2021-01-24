//This file contains the MakePacket System

//SPECS
use specs::prelude::*;

//Get data needed for the System to work
use crate::bourg::fdm::structures::DataFDM;
use crate::flightgear::FGNetFDM;

//Get function to call
use crate::bourg::fdm::make_packet::load_fgnetfdm;

//System to make a packet based on fgnetfdm structure required by FlightGear 
pub struct MakePacket;
impl<'a> System<'a> for MakePacket
{
    type SystemData = (
        ReadStorage<'a, DataFDM>,
        WriteStorage<'a, FGNetFDM>,
    );

    fn run(&mut self, (datafdm, mut fgnetfdm): Self::SystemData) 
    {
        for (fdm, mut fgnet) in (&datafdm, &mut fgnetfdm).join() 
        {
            //Call function to load updated data into FGNetFDM
            load_fgnetfdm(fdm, &mut fgnet);
        }
    }
}