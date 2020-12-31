//This main file sets up the ECS architecture, creates the Entity, and runs the main simulation loop

//SPECS
use specs::prelude::*;

//Main loop
use std::{thread, time};

//Import Components and resources
mod data;

//Component
use crate::data::DataFDM;

//Structures inside Components
use crate::data::PointMass;

//Resources
use crate::data::DeltaTime;
use crate::data::MaxThrust;
use crate::data::DeltaThrust;

//Import System
mod equations_of_motion;

//Import Vector, Matrix, Quaternion module
mod common;

//Import calculate mass properties function for use when the airplane Entity is created
use crate::equations_of_motion::calc_airplane_mass_properties;

fn main()
{
    //Create world
    let mut world = World::new();

    //Register the Components to the world
    world.register::<DataFDM>();

    //Choose frame rate, which will calculate delta time
    let frame_rate: f32 = 30.0;
    let dt: f32 = 1.0 / frame_rate; //seconds

    //Add dt as a SPECS resource
    world.insert(DeltaTime(dt));

    //Choose max thrust potential and increment amount
    let max_thrust: f32 = 3000.0;
    let d_thrust: f32 = 100.0;

    //Add max_thrust and d_thrust as resources
    world.insert(MaxThrust(max_thrust));
    world.insert(DeltaThrust(d_thrust));

    //Create variable to keep track of time elapsed
    let mut current_time: f64 = 0.0;
    let mut current_frame_main: usize = 0;

    //Create a dispatcher to manage system execution
    let mut dispatcher = DispatcherBuilder::new()
    .with(equations_of_motion::EquationsOfMotion, "EOM", &[])
    .build();
    dispatcher.setup(&mut world);

    //Intialize the airplane

    //Setup an airplane with the PointMass elements defined, and everything else defaulted to zero
    //This object is soley used to pass to the calculate_mass_properties function to determine total mass, and the inertia matrix
    let mut myairplane = DataFDM{ 
        element : vec![
            PointMass{f_mass: 6.56, v_d_coords: common::Myvec::new(14.5, 12.0, 2.5), v_local_inertia: common::Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: common::Myvec::new(14.5, 5.5, 2.5), v_local_inertia: common::Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: common::Myvec::new(14.5, -5.5, 2.5), v_local_inertia: common::Myvec::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 6.56, v_d_coords: common::Myvec::new(14.5, -12.0, 2.5), v_local_inertia: common::Myvec::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: common::Myvec::new(3.03, 2.5, 3.0), v_local_inertia: common::Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: common::Myvec::new(3.03, -2.5, 3.0), v_local_inertia: common::Myvec::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.93, v_d_coords: common::Myvec::new(2.25, 0.0, 5.0), v_local_inertia: common::Myvec::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 31.8, v_d_coords: common::Myvec::new(15.25, 0.0, 1.5), v_local_inertia: common::Myvec::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: common::Myvec::new(0.0, 0.0, 0.0), v_cg_coords: common::Myvec::new(0.0, 0.0, 0.0) }
        ], ..Default::default()};

    //Calculate mass properties on this airplane
    calc_airplane_mass_properties(&mut myairplane);

    //Create an airplane Entity and populate its DataFDM Component with the mass properties computed
    //Additionally, define the starting flight values and position
    let _plane = world.create_entity()
    .with(DataFDM{

        //Copy over the mass properties calculated
        mass: myairplane.mass,
        m_inertia: myairplane.m_inertia,
        m_inertia_inverse: myairplane.m_inertia_inverse,

        //Define initial flight parameters
        v_position: common::Myvec{x: -5000.0, y: 0.0, z: 2000.0},
        v_velocity: common::Myvec{x: 60.0, y: 0.0, z: 0.0},
        f_speed: 60.0,
        v_forces: common::Myvec{x: 500.0, y: 0.0, z: 0.0},
        thrustforce: 500.0,
        q_orientation: common::Myquaternion::make_q_from_euler(0.0, 0.0, 0.0),
    
        //Copy over the defined PointMass elements
        element: myairplane.element,

        //Everything else is zero to begin
        ..Default::default()
    })
    .build();

    //Create time type with the dt in milliseconds
    let timestep = time::Duration::from_millis((dt * 1000.0) as u64);

    //Main simulation loop
    while current_time < 30.0 //seconds
    {
        //Get current time
        let start = time::Instant::now();

        //Increment time count
        current_time = current_time + dt as f64;
        current_frame_main = current_frame_main + 1;
        println!("{}", "====================================================");
        println!("Time: {}, Frames: {}", current_time as f32, current_frame_main);

        //Process this frame
        dispatcher.dispatch(&world); 
        world.maintain();

        //Find difference in time elapsed this loop versus the timestep
        // let sleep_time = timestep.checked_sub(time::Instant::now().duration_since(start));

        // //Sleep for extra time if calculation took less time than the DT time step
        // if sleep_time != None 
        // {
        //     thread::sleep(sleep_time.unwrap());
        // }
    }
}




