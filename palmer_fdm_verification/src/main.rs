//This main file sets up the ECS architecture, creates the entity, and runs the main simulation loop

//Specs
use specs::prelude::*;

//Import Components and resources
mod data;
use crate::data::DataFDM;
use crate::data::PerformanceData;
use crate::data::DeltaTime;

//Import Systems
mod equations_of_motion;

fn main()
{
    //Create variable to keep track of time elapsed
    let mut current_time = 0.0;
    let mut current_frame_main: usize = 0;

    //Create world
    let mut world = World::new();
    
    //Register Components in the world
    world.register::<DataFDM>();

    //Choose frame rate, which will calculate delta time
    let frame_rate: f64 = 2.0;
    let dt: f64 = 1.0 / frame_rate; //seconds

    //Add dt as a specs resource
    world.insert(DeltaTime(dt)); 

    //Create dispatcher of the Systems
    let mut dispatcher = DispatcherBuilder::new()
    .with(equations_of_motion::EquationsOfMotion, "EOM", &[])
    .build();
    dispatcher.setup(&mut world);

    //Create airplane Entity with Components
    let _plane = world.create_entity()
    .with(DataFDM{
        //Parameters altered for equivalency tests
        throttle: 1.0, //throttle percentage
        alpha: 4.0,//angle of attack
        bank: 0.0, //bank angle
        flap: 0.0,  //flap deflection amount

        current_frame: 0,
        q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0], //will store ODE results
        airspeed: 0.0,
        mass_properties: PerformanceData{
            wing_area: 16.2,            //  wing wetted area, m^2
            wing_span: 10.9,            //  wing span, m
            tail_area: 2.0,             //  tail wetted area, m^2
            cl_slope0: 0.0889,          //  slope of Cl-alpha curve
            cl0: 0.178,                 //  Cl value when alpha = 0
            cl_slope1: -0.1,            //  slope of post-stall Cl-alpha curve
            cl1: 3.2,                   //  intercept of post-stall Cl-alpha curve
            alpha_cl_max: 16.0,         //  alpha at Cl(max)
            cdp: 0.034,                 //  parasitic drag coefficient
            eff: 0.77,                  //  induced drag efficiency coefficient
            mass: 1114.0,               //  airplane mass, kg
            engine_power: 119310.0,     //  peak engine power, W
            engine_rps: 40.0,           //  engine turnover rate, rev/s
            prop_diameter: 1.905,       //  propeller diameter, m
            a: 1.83,                    //  propeller efficiency curve fit coefficient
            b:-1.32,                    //  propeller efficiency curve fit coefficient
        }
    })
    .build();


    //Loop simulation
    while current_time < 40.0
    {

        //Increment time count
        current_time = current_time + dt;
        current_frame_main = current_frame_main + 1;
        println!("{}", "====================================================");
        println!("Time (seconds): {}, frames: {}", current_time, current_frame_main);

        //Process this frame
        dispatcher.dispatch(&world);
        world.maintain();

    }
}