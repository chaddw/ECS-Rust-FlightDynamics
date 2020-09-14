
//********************************************
//  This structure defines the data required
//  to model a plane.
//********************************************

#[derive(Debug)]
struct Plane 
{

    numEqns: usize, //int
    s: f64,

  //  q: [f64; 6],
    q: Vec<f64>,
    // dq1: Vec<f64>,
    // dq2: Vec<f64>,
    // dq3: Vec<f64>,
    // dq4: Vec<f64>,
 
    bank: f64,
    alpha: f64,//  angle of attack
    throttle: f64,
    wingArea: f64,
    wingSpan: f64,
    tailArea: f64,
    clSlope0: f64,   // slope of Cl-alpha curve
    cl0: f64,         // intercept of Cl-alpha curve
    clSlope1: f64,    // post-stall slope of Cl-alpha curve
    cl1: f64,        // post-stall intercept of Cl-alpha curve
    alphaClMax: f64,  // alpha when Cl=Clmax
    cdp: f64,         // parasite drag coefficient
    eff: f64,         // induced drag efficiency coefficient
    mass: f64,
    enginePower: f64,
    engineRps: f64,   // revolutions per second
    propDiameter: f64,
    a: f64,           //  propeller efficiency coefficient
    b: f64,           //  propeller efficiency coefficient
    flap: String //&'static str,        //  flap deflection amount (pointer in c)

}


//initialize plane and solves for the plane motion with range-kutta
fn main()
{
    let mut x: f64 = 0.0;
    let mut z: f64 = 0.0;
    let mut v: f64 = 0.0;
    let mut time: f64 = 0.0;
    let dt = 0.5; 

    //create plane, set airplane data
    let mut plane = Plane {
    wingArea: 16.2,             //  wing wetted area, m^2
    wingSpan: 10.9,             //  wing span, m
    tailArea: 2.0,              //  tail wetted area, m^2
    clSlope0: 0.0889,           //  slope of Cl-alpha curve
    cl0: 0.178,                 //  Cl value when alpha = 0
    clSlope1: -0.1,             //  slope of post-stall Cl-alpha curve
    cl1: 3.2,                   //  intercept of post-stall Cl-alpha curve
    alphaClMax: 16.0,           //  alpha at Cl(max)
    cdp: 0.034,                 //  parasitic drag coefficient
    eff: 0.77,                  //  induced drag efficiency coefficient
    mass: 1114.0,               //  airplane mass, kg
    enginePower: 119310.0,      //  peak engine power, W
    engineRps: 40.0,            //  engine turnover rate, rev/s
    propDiameter: 1.905,        //  propeller diameter, m
    a: 1.83,                    //  propeller efficiency curve fit coefficient
    b:-1.32,                    //  propeller efficiency curve fit coefficient
    bank: 0.0,
    alpha: 4.0, 
    throttle: 1.0, 
    flap: String::from("0"),    //  Flap setting
    numEqns: 6, 
    s: 0.0,                     //  time
   // q: [0.0;6]
     q: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                //  vx, x, vy, y, vz, z
    // dq1: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    // dq2: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    // dq31: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    // dq4: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    };



  
    //accelerate the plane for 40 seconds
    while plane.s < 40.0
    {
        plane.planeRungeKutta4( dt);

        time = plane.s;
        x = plane.q[1];
        z = plane.q[5];
        v = (plane.q[0] * plane.q[0] + plane.q[2] * plane.q[2] + plane.q[4] * plane.q[4]).sqrt();
    
 
        println!("time = {}, x = {}, altitude = {}, airspeed = {} ", time, x, z, v);

        //println!("{:#?}", plane);
    }

}

//************************************************************
//  This method solves for the plane motion using a
//  4th-order Runge-Kutta solver
//************************************************************

impl Plane
{


    fn planeRungeKutta4(&mut self, mut ds: f64)
    {

        // let mut q: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq1: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq2: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq3: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // let mut dq4: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let mut q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut qcopy = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq1 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq2 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq3 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq4 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


        //retrieve value of dependent variable
        q = self.q.clone();

        // Compute the four Runge-Kutta steps, The return 
        // value of planeRightHandSide method is an array
        // of delta-q values for each of the four steps.

        self.planeRightHandSide( &mut q, &mut qcopy, &mut ds, 0.0, &mut dq1);
        self.planeRightHandSide( &mut q, &mut dq1,   &mut ds, 0.5, &mut dq2);
        self.planeRightHandSide( &mut q, &mut dq2,   &mut ds, 0.5, &mut dq3);
        self.planeRightHandSide( &mut q, &mut dq3,   &mut ds, 1.0, &mut dq4);


        //  Update the dependent and independent variable values
        //  at the new dependent variable location and store the
        //  values in the ODE object arrays.
        self.s = self.s + ds;

        for i in 0..self.numEqns
        {
            q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
            self.q[i] = q[i];
        }

    }


//*************************************************************
//  This method loads the right-hand sides for the plane ODEs
//*************************************************************
    fn planeRightHandSide(&mut self, q: &mut Vec<f64>, deltaQ: &mut Vec<f64>, &mut ds: &mut f64, qScale:  f64, mut dq: &mut Vec<f64>)
    {

    //  q[0] = vx = dxdt
    //  q[1] = x
    //  q[2] = vy = dydt
    //  q[3] = y
    //  q[4] = vz = dzdt
    //  q[5] = z
        let mut newQ = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; //[f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 

        let yo = -1.0_f64;
        let pi = yo.acos();

        let G: f64 = -9.81;
    
        let mut cl: f64 = 0.0;

        let mut cosP: f64 = 0.0;   //  climb angle
        let mut sinP: f64= 0.0;   //  climb angle
        let mut cosT: f64 = 0.0;   //  heading angle
        let mut sinT: f64 = 0.0;   //  heading angle
       
        //  Convert bank angle from degrees to radians
        //  Angle of attack is not converted because the
        //  Cl-alpha curve is defined in terms of degrees.
        self.bank = self.bank.to_radians();

        //  Compute the intermediate values of the 
        //  dependent variables.

        for i in 0..6
        {
            newQ[i] = q[i] + qScale * deltaQ[i]; 
        }

        //  Assign convenenience variables to the intermediate 
        //  values of the locations and velocities.
        let vx: f64 = newQ[0];
        let vy: f64 = newQ[2];
        let vz: f64 = newQ[4];
        let x: f64 = newQ[1];
        let y: f64 = newQ[3];
        let z: f64 = newQ[5];
        let vh: f64 = (vx * vx + vy * vy).sqrt();
        let vtotal: f64 = (vx * vx + vy * vy + vz * vz).sqrt();

        //  Compute the air density
        let temperature: f64 = 288.15 - 0.0065 * z;
        let grp: f64 = 1.0 - 0.0065 * z / 288.15;
        let pressure: f64 = 101325.0 * (grp.powf(5.25));
        let density: f64 = 0.00348 * pressure / temperature;

        //  Compute power drop-off factor
        let omega: f64 = density / 1.225;
        let factor: f64 = (omega - 0.12)/  0.88;

        //  Compute thrust
        let advanceRatio: f64 = vtotal / (self.engineRps * self.propDiameter);
        let thrust: f64 = self.throttle * factor * self.enginePower * (self.a + self.b * advanceRatio * advanceRatio) / (self.engineRps * self.propDiameter);

        //  Compute lift coefficient. The Cl curve is 
        //  modeled using two straight lines.
        if  self.alpha < self.alphaClMax
        {
            cl = self.clSlope0 * self.alpha + self.cl0;
        }
        else 
        {
            cl = self.clSlope1 * self.alpha + self.cl1;
        }

        //  Include effects of flaps and ground effects.
        //  Ground effects are present if the plane is
        //  within 5 meters of the ground.
        if self.flap == "20"
        {
            cl += 0.25;
        }
        if self.flap == "40"
        {
            cl += 0.5;
        }
        if z < 5.0
        {
            cl += 0.25;
        }

        //  Compute lift
        let lift: f64 = 0.5 * cl * density * vtotal * vtotal * self.wingArea;

        // //  Compute drag coefficient
        let aspectRatio: f64 = self.wingSpan * self.wingSpan / self.wingArea;
        let cd = self.cdp + cl * cl / (pi * aspectRatio * self.eff);
        
        // //  Compute drag force
        let drag: f64 = 0.5 * cd * density * vtotal * vtotal * self.wingArea;


        //  Define some shorthand convenience variables
        //  for use with the rotation matrix.
        //  Compute the sine and cosines of the climb angle,
        //  bank angle, and heading angle;
        let cosW: f64 = self.bank.cos(); 
        let sinW: f64 = self.bank.sin(); 

        if  vtotal == 0.0
        {
            cosP = 1.0;
            sinP = 0.0;
        }
        else
        {
            cosP = vh / vtotal;  
            sinP = vz / vtotal;  
        }
        
        if vh == 0.0
        {
            cosT = 1.0;
            sinT = 0.0;
        }
        else
        {
            cosT = vx / vh;
            sinT = vy / vh;
        }


        //  Convert the thrust, drag, and lift forces into
        //  x-, y-, and z-components using the rotation matrix.
        let Fx: f64 = cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift;
        let Fy: f64 = sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift;
        let mut Fz: f64 = sinP * (thrust - drag) + cosP * cosW * lift;

        //  Add the gravity force to the z-direction force.
        Fz = Fz + self.mass * G;

        //  Since the plane can't sink into the ground, if the
        //  altitude is less than or equal to zero and the z-component
        //  of force is less than zero, set the z-force
        //  to be zero.
        if  z <= 0.0 && Fz <= 0.0  
        {
            Fz = 0.0;
        }

        //  Load the right-hand sides of the ODE's
        dq[0] = ds * (Fx / self.mass);
        dq[1] = ds * vx;
        dq[2] = ds * (Fy / self.mass);
        dq[3] = ds * vy;
        dq[4] = ds * (Fz / self.mass);
        dq[5] = ds * vz;

 
    }

}




