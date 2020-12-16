//This file contains the SendPacket System

//SPECS
use specs::prelude::*;

//Get data needed for the System
use crate::data::Packet;

//Global socket variable inside lazy static macro in main
use crate::SOCKET;

pub struct SendPacket;
impl<'a> System<'a> for SendPacket 
{
    type SystemData = ReadStorage<'a, Packet>;

    fn run(&mut self, packet: Self::SystemData) {

        for pckt in packet.join() 
        {
             //Finally send &[u8] of bytes over socket connected on FlightGear
             SOCKET.send(&pckt.bytes).expect("couldn't send packet");
        }
    }
}
