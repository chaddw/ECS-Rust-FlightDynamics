use std::net::UdpSocket;

// const uint32_t FG_NET_FDM_VERSION = 24;


struct FGNetFDM{ //recreate the structure

    altitude: f64, 

}

    


fn main()
{
    let D2R = 3.14159 / 180.0;
    let _altitude = 150.0;

    //create fdm data structure instance
    let _fdm: FGNetFDM = FGNetFDM { altitude: 10.0*D2R}; 

    //convert to ordered bytes
    let bytes = (_fdm.altitude).to_ne_bytes();
    //let and_back = f64::from_ne_bytes(bytes);



    //create socket and connect
    let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
    socket.connect("127.0.0.1:5500").expect("connect function failed");

    //send
    socket.send(&bytes).expect("couldn't send message");



}



