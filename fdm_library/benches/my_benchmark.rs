//Benchmark Bourg vs Palmer
//To run the benches: cargo bench
//Results are posted in the target/criterion/report

/*
The airplanes in both models are flying and pitching up for every frame of the bench.
We set up the two functions, bourg and palmer,
that take in the Frames Per Second and # of frames to execute and then run the function for that many frames
*/
use criterion::{criterion_group, criterion_main, Criterion, BenchmarkId};

//Bring into scope the functions and data as necessary for Bourg
use fdm_library::bourg::common::vector::Vector;
use fdm_library::bourg::common::quaternion::Quaternion;
use fdm_library::bourg::fdm::structures::DataFDM;
use fdm_library::bourg::fdm::structures::PointMass;
use fdm_library::bourg::fdm::calc_loads::calc_airplane_loads;
use fdm_library::bourg::fdm::mass_properties::calc_airplane_mass_properties;
use fdm_library::bourg::fdm::keypresses::*;

//Bring into scope the functions and structures as necessary for Palmer
use fdm_library::palmer::fdm::plane_right_hand_side::plane_rhs;
use fdm_library::palmer::fdm::structures::DataFDM as OtherDataFDM;
use fdm_library::palmer::fdm::structures::PerformanceData;


fn bourg(fps: f32, frames: usize) 
{
    let dt = 1.0 / fps;

    //Create airplane to compute mass properties
    let mut fdm = DataFDM{ 
        element : vec![
            PointMass{f_mass: 6.56, v_d_coords: Vector::new(14.5, 12.0, 2.5), v_local_inertia: Vector::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector::new(14.5, 5.5, 2.5), v_local_inertia: Vector::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 7.31, v_d_coords: Vector::new(14.5, -5.5, 2.5), v_local_inertia: Vector::new(21.95, 12.22, 33.67), f_incidence: -3.5, f_dihedral: 0.0, f_area: 36.4, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 6.56, v_d_coords: Vector::new(14.5, -12.0, 2.5), v_local_inertia: Vector::new(13.92, 10.50, 24.00), f_incidence: -3.5, f_dihedral: 0.0, f_area: 31.2, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector::new(3.03, 2.5, 3.0), v_local_inertia: Vector::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.62, v_d_coords: Vector::new(3.03, -2.5, 3.0), v_local_inertia: Vector::new(0.837, 0.385, 1.206), f_incidence: 0.0, f_dihedral: 0.0, f_area: 10.8, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 2.93, v_d_coords: Vector::new(2.25, 0.0, 5.0), v_local_inertia: Vector::new(1.262, 1.942, 0.718), f_incidence: 0.0, f_dihedral: 90.0, f_area: 12.0, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) },
            PointMass{f_mass: 31.8, v_d_coords: Vector::new(15.25, 0.0, 1.5), v_local_inertia: Vector::new(66.30, 861.9, 861.9), f_incidence: 0.0, f_dihedral: 0.0, f_area: 84.0, i_flap: 0, v_normal: Vector::new(0.0, 0.0, 0.0), v_cg_coords: Vector::new(0.0, 0.0, 0.0) }
        ], 
    
        
    //Define initial flight parameters
    v_position: Vector{x: 0.0, y: 0.0, z: 2000.0},
    v_velocity: Vector{x: 60.0, y: 0.0, z: 0.0},
    f_speed: 60.0,
    v_forces: Vector{x: 500.0, y: 0.0, z: 0.0},
    thrustforce: 500.0,
    q_orientation: Quaternion::make_q_from_euler(0.0, 0.0, 0.0),

    //Everything else is zero to begin
    ..Default::default()
    };

    //Calculate mass properties on this airplane
    calc_airplane_mass_properties(&mut fdm);

    for _ in 0..frames //frame #
    {

        //Reset/zero the elevators, rudders, and ailerons every loop
        zero_rudder(&mut fdm);
        zero_ailerons(&mut fdm);
        zero_elevators(&mut fdm);
        //Flaps will be toggled on and off so flaps does not need to be zerod each time

        
        //Pitch up for the duration of the benchmark
        pitch_up(&mut fdm);



        //Calculate all of the forces and moments on the airplane
        calc_airplane_loads(&mut fdm);

        //Calculate acceleration of airplane in earth space
        let ae: Vector = fdm.v_forces / fdm.mass;

        //Calculate velocity of airplane in earth space
        fdm.v_velocity = fdm.v_velocity + ae * dt; 

        //Calculate position of airplane in earth space
        fdm.v_position = fdm.v_position + fdm.v_velocity * dt;

        //Calculate angular velocity of airplane in body space
        fdm.v_angular_velocity = fdm.v_angular_velocity + ((fdm.m_inertia_inverse * 
            (fdm.v_moments - Vector::crossproduct(&fdm.v_angular_velocity, &(fdm.m_inertia * fdm.v_angular_velocity)))) * dt);

        //Calculate the new rotation quaternion
        fdm.q_orientation = fdm.q_orientation + (fdm.q_orientation * fdm.v_angular_velocity) * (0.5 * dt);

        //Now normalize the orientation quaternion (make into unit quaternion)
        let mag = fdm.q_orientation.magnitude();
        if mag != 0.0
        {
            fdm.q_orientation = fdm.q_orientation / mag;
        }

        //Calculate the velocity in body space
        fdm.v_velocity_body = Quaternion::qvrotate(&Quaternion::conjugate(&fdm.q_orientation), &fdm.v_velocity);

        //Calculate air speed
        fdm.f_speed = fdm.v_velocity.magnitude();

        //Get euler angles
        let euler = Quaternion::make_euler_from_q(&fdm.q_orientation);
        fdm.v_euler_angles.x = euler.x;
        fdm.v_euler_angles.y = euler.y;
        fdm.v_euler_angles.z = euler.z; 

    }
        
}




fn palmer(fps: f64, frames: usize)
{
    let dt = 1.0 / fps;

    //Create airplane
    let mut fdm = OtherDataFDM{
    
        //Parameters that get modified by keypress
        throttle: 0.15, //throttle percentage (0.0 - 1.0)
        alpha: 20.0,//angle of attack degrees (-16.0 - 20.0)
        bank: 0.0, //bank angle degrees (-20.0 - 20.0)
        flap: 0.0,  //flap deflection amount degrees (20.0 or 40.0)

        q: vec![35.0, 1000.0, 0.0, 0.0, 5.0, 93.0], //will store ODE results, start with airplane flying
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

    for _ in 0..frames //frame #
    {

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
        plane_rhs(&mut fdm, &mut q, &mut qcopy,  &ds, 0.0, &mut dq1);
        plane_rhs(&mut fdm, &mut q, &mut dq1,    &ds, 0.5, &mut dq2);
        plane_rhs(&mut fdm, &mut q, &mut dq2,    &ds, 0.5, &mut dq3);
        plane_rhs(&mut fdm, &mut q, &mut dq3,    &ds, 1.0, &mut dq4);

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

}


//Benching as a group

fn bench_fdms(c: &mut Criterion) 
{
    let mut group = c.benchmark_group("FDMs");
    let frame_count = 100;

    for i in [10.0, 30.0, 60.0, 100.0].iter() //Input Frames Per Second that will determine the time step
    {
        group.bench_with_input(BenchmarkId::new("Bourg", i), i, 
            |b, i| b.iter(|| bourg(*i, frame_count)));
        group.bench_with_input(BenchmarkId::new("Palmer", i), i, 
            |b, i| b.iter(|| palmer(*i as f64, frame_count)));
    }
    group.finish();
}

criterion_group!(benches, bench_fdms);
criterion_main!(benches);