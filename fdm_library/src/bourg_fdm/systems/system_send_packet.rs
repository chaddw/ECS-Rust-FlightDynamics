//This file contains the SendPacket System

//SPECS
use specs::prelude::*;

//Networking
use std::net::UdpSocket;

//Get data needed for the System
use crate::bourg_fdm::components::component_packet::Packet;

//System to send packets after they are made
pub struct SendPacket;
impl<'a> System<'a> for SendPacket
{
    type SystemData = ReadStorage<'a, Packet>;

    fn run(&mut self, packet: Self::SystemData) 
    {
        for pckt in packet.join() 
        {
            //Open a socket
            let socket: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");

            //Connect to the socket on FlightGear
            socket.connect("127.0.0.1:5500").expect("connect function failed");

            //Finally send &[u8] of bytes over socket connected on FlightGear
            socket.send(&pckt.bytes).expect("couldn't send packet");
        }
    }
}