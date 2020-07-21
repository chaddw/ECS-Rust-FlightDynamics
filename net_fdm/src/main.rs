use std::net::UdpSocket;


//commented out variables, not worried about these for now
// const uint32_t FG_NET_FDM_VERSION = 24;


struct FGNetFDM{ //recreate the structure

    altitude: u8, 


//     enum {
//         FG_MAX_ENGINES = 4,
//         FG_MAX_WHEELS = 3,
//         FG_MAX_TANKS = 4
//     };

//     version: u32, // increment when data values change
//     padding: u32, // padding

//     // Positions
//     let double longitude;		// geodetic (radians)
//     let double latitude;		// geodetic (radians)
//     let double altitude;		// above sea level (meters)
//     let float agl;			// above ground level (meters)
//     let float phi;			// roll (radians)
//     let float theta;		// pitch (radians)
//     let float psi;			// yaw or true heading (radians)
//     let float alpha;                // angle of attack (radians)
//     let float beta;                 // side slip angle (radians)

//     // Velocities
//     let float phidot;		// roll rate (radians/sec)
//     let float thetadot;		// pitch rate (radians/sec)
//     let float psidot;		// yaw rate (radians/sec)
//     let float vcas;		        // calibrated airspeed
//     let float climb_rate;		// feet per second
//     let float v_north;              // north velocity in local/body frame, fps
//     let float v_east;               // east velocity in local/body frame, fps
//     let float v_down;               // down/vertical velocity in local/body frame, fps
//     let float v_body_u;    // ECEF velocity in body frame
//     let float v_body_v;    // ECEF velocity in body frame 
//     let float v_body_w;    // ECEF velocity in body frame
    
//     // Accelerations
//     let float A_X_pilot;		// X accel in body frame ft/sec^2
//     let float A_Y_pilot;		// Y accel in body frame ft/sec^2
//     let float A_Z_pilot;		// Z accel in body frame ft/sec^2

//     // Stall
//     let float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
//     let float slip_deg;		// slip ball deflection

//     // Pressure
    
//     // Engine status
//     let uint32_t num_engines;	     // Number of valid engines
//     let uint32_t eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
//     let float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
//     let float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
//     let float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
//     let float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
//     let float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
//     let float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
//     let float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
//     let float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
//     let float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

//     // Consumables
//     let uint32_t num_tanks;		// Max number of fuel tanks
//     let float fuel_quantity[FG_MAX_TANKS];

//     // Gear status
//     let uint32_t num_wheels;
//     let uint32_t wow[FG_MAX_WHEELS];
//     let float gear_pos[FG_MAX_WHEELS];
//     let float gear_steer[FG_MAX_WHEELS];
//     let float gear_compression[FG_MAX_WHEELS];

//     // Environment
//     let uint32_t cur_time;           // current unix time
//                                  // FIXME: make this uint64_t before 2038
//     let int32_t warp;                // offset in seconds to unix time
//     let float visibility;            // visibility in meters (for env. effects)

//     // Control surface positions (normalized values)
//     let float elevator;
//     let float elevator_trim_tab;
//     let float left_flap;
//     let float right_flap;
//     let float left_aileron;
//     let float right_aileron;
//     let float rudder;
//     let float nose_wheel;
//     let float speedbrake;
//     let float spoilers;


}


fn main()
{
    //create fdm data structure instance
    let fdm: FGNetFDM = FGNetFDM { altitude: 10}; //


    let socket = UdpSocket::bind("127.0.0.1:1337").expect("couldn't bind to address");
    socket.connect("127.0.0.1:5500").expect("connect function failed");

    //need to figure out the byte order for altitude so flight gear understands
    socket.send(&[ fdm.altitude]).expect("couldn't send message");
}



