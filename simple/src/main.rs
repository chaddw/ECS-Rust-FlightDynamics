use std::net::UdpSocket;

fn main() {
    let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
    socket.connect("127.0.0.1:5500").expect("connect function failed");
    socket.send(&[0, 1, 2]).expect("couldn't send message");
}
