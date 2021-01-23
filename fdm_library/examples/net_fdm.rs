//Net_fdm example
//This code file is a simple example that sends a packet to flightgear to roll the airplane 5 degrees

//FlightGear is ran with this line of command argumments on the fgfs executable:
//fgfs.exe --aircraft=ufo --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --timeofday=noon --native-fdm=socket,in,30,,5500,udp

//Cessna Skyhawk visual
//fgfs.exe --disable-panel --disable-sound --enable-hud --disable-random-objects --fdm=null --timeofday=noon --native-fdm=socket,in,30,,5500,udp

//Networking
use std::net::UdpSocket;

//Main loop
use std::{thread, time};

//FGNetFDM structure
use fdm_library::flightgear::FGNetFDM;

fn main()
{
    //Sleep time for the loop
    let millis = time::Duration::from_millis(1000);

    //Set numerical values for this example
    let latitude: f64 = 45.59823;
    let longitude: f64= -120.6902;
    let altitude: f64 = 150.0;
    let mut roll: f32 = 0.0;
    let pitch: f32 = 0.0;
    let yaw: f32 = 0.0;
    
    let fg_net_fdm_version = 24_u32;

    //Create fdm instance
    let mut fdm: FGNetFDM = Default::default();

    loop
    {
        thread::sleep(millis);

        //All data passed into the FGNetFDM struct is converted to network byte order
        fdm.version = u32::from_be_bytes(fg_net_fdm_version.to_ne_bytes());
        fdm.latitude = f64::from_be_bytes(latitude.to_radians().to_ne_bytes());
        fdm.longitude = f64::from_be_bytes(longitude.to_radians().to_ne_bytes());
        fdm.altitude = f64::from_be_bytes(altitude.to_ne_bytes());
        fdm.phi = f32::from_be_bytes(roll.to_radians().to_ne_bytes());
        fdm.theta = f32::from_be_bytes(pitch.to_radians().to_ne_bytes());
        fdm.psi = f32::from_be_bytes(yaw.to_radians().to_ne_bytes());

        //Create socket and connect to flightgear
        let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
        socket.connect("127.0.0.1:5500").expect("connect function failed");

        //Convert struct array of u8 of bytes
        let bytes = bincode::serialize(&fdm).unwrap();
        println!("{:?}", bytes);

        //Finally send &[u8] of bytes to flight gear
        socket.send(&bytes).expect("couldn't send message");

        //Roll 5 degrees
        roll = roll + 5.0; 
        if roll > 20.0
        {
            roll = 0.0;
        }
        println!("Roll: {}", roll);
    }
}