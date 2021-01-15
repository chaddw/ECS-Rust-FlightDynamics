//Packet Component containing the FGNetFDM structure, along with its serialized version

//SPECS
use specs::prelude::*;

//FGNetFDM structure
use crate::net_fdm::FGNetFDM;

//Component containg the the FGNetFDM structure, and its conversion into bytes
#[derive(Debug, Default)]
pub struct Packet
{
    pub fgnetfdm: FGNetFDM,
    pub bytes: Vec<u8>,
}
impl Component for Packet
{
    type Storage = VecStorage<Self>;
}