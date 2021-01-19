//Integration tests for the Palmer-based Flight Dynamics Model

//To run on the command line: cargo test --test palmer_integration

//Float_cmp crate for comparing floats
use float_cmp::*;

//Bring into scope the functions and data as necessary for testing
use fdm_library::palmer_fdm::components::component_datafdm::*;
use fdm_library::palmer_fdm::systems::system_equations_of_motion::plane_right_hand_side;

#[test]
fn fdm_test()
{

    //Create variable to keep track of time elapsed
    let mut current_time = 0.0;

    //Choose FPS to get dt
    let fps = 10.0;
    let dt = 1.0 / fps;

    //Create airplane
    let mut fdm = DataFDM{
    
        //Parameters altered for equivalency tests
        throttle: 1.0, //throttle percentage (0.0 - 1.0)
        alpha: 4.0,//angle of attack degrees (-16.0 - 20.0)
        bank: 0.0, //bank angle degrees (-20.0 - 20.0)
        flap: 20.0,  //flap deflection amount degrees (20.0 or 40.0)

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
        },

        //Everything else is zero to begin
        ..Default::default()
    };



    while current_time < 60.0 //seconds
    {

        //Increment time count
        current_time = current_time + dt;

        let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        //Retrieve value of dependent variable
        let mut q = fdm.q.clone();

        //Get the static time variable DT
        let ds = dt;

        // Compute the four Runge-Kutta steps, The return 
        // value of plane_right_hand_side method is an array
        // of delta-q values for each of the four steps.
        plane_right_hand_side(&mut fdm, &mut q, &mut qcopy,  &ds, 0.0, &mut dq1);
        plane_right_hand_side(&mut fdm, &mut q, &mut dq1,    &ds, 0.5, &mut dq2);
        plane_right_hand_side(&mut fdm, &mut q, &mut dq2,    &ds, 0.5, &mut dq3);
        plane_right_hand_side(&mut fdm, &mut q, &mut dq3,    &ds, 1.0, &mut dq4);

        //  Update the dependent and independent variable values
        //  at the new dependent variable location and store the
        //  values in the ODE object arrays.
        for i in 0..6
        {
            q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
            fdm.q[i] = q[i];
        }

        //Calculate airspeed
        fdm.airspeed = (fdm.q[0] * fdm.q[0] + fdm.q[2] * fdm.q[2] + fdm.q[4] * fdm.q[4]).sqrt();

    }



    //Grab our test data results
    let flight_test_data = vec![fdm.q[1], fdm.q[3], fdm.q[5], fdm.airspeed];
    
    //Copy over the data from the c benchmark tests
    //let benchmark_data = vec![2245.945807, 0.000000, 157.184886, 44.737909]; //TEST 1 (ANGLE OF ATTACK)

    //let benchmark_data = vec![1824.183957, -1148.138559, 155.741858, 44.905979]; //TEST 2 (BANK)

    let benchmark_data = vec![1924.086822, 0.000000, 187.228017, 38.606508]; //TEST 3 (FLAPS)


    println!("Rust/ECS Flight Data  : {:?}", flight_test_data);
    println!("C Benchmark           : {:?}", benchmark_data);

    //7 Variables to check for each of the 3 tests
    let cmp1 = approx_eq!(f64, flight_test_data[0], benchmark_data[0], epsilon = 0.000001); //Pos x
    let cmp2 = approx_eq!(f64, flight_test_data[1], benchmark_data[1], epsilon = 0.000001); //Pos y
    let cmp3 = approx_eq!(f64, flight_test_data[2], benchmark_data[2], epsilon = 0.000001); //Pos z
    let cmp4 = approx_eq!(f64, flight_test_data[3], benchmark_data[3], epsilon = 0.000001); //Airspeed (m/s)


    //If all comparisons are within the epsilon, return true
    if cmp1 == true && cmp2 == true  && cmp3 == true && cmp4 == true
    {
        assert!(true);
    }
    else 
    {
        assert!(false);
    }


}
   

   

