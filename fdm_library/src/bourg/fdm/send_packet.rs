//Function to serialize and send FGNetFDM packet

//Networking
use std::net::UdpSocket;

//FGNetFDM structure
use crate::flightgear::FGNetFDM;

pub fn send(fgnet: &FGNetFDM)
{
    //Convert struct to array of u8 bytes
    let bytes = bincode::serialize(&fgnet).unwrap();

    //Open a socket
    let socket: std::net::UdpSocket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");

    //Connect to the socket on FlightGear
    socket.connect("127.0.0.1:5500").expect("connect function failed");

    //Finally send &[u8] of bytes over socket connected on FlightGear
    socket.send(&bytes).expect("couldn't send packet");
}