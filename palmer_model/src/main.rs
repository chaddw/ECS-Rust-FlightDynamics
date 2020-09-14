
//********************************************
//  This structure defines the data required
//  to model a plane.
//********************************************

#[derive(Debug)]
struct Plane 
{

    numEqns: usize, //int
    s: f64,
    q: [f64;6],
 
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
    let mut x: f64;
    let mut z: f64;
    let mut v: f64;
    let mut time: f64;
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
    q: [0.0; 6]                 //  vx, x, vy, y, vz, z
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


    fn planeRungeKutta4(&mut self, ds: f64)
    {

       // let mut q: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq1: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq2: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq3: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq4: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


       // q = self.q; //same as doing this for loop...

        // for i in 0..self.numEqns
        // {
        //     //q[i] = self.q[i];
        //     println!("{}",q[i]);
        // }


        // Compute the four Runge-Kutta steps, The return 
        // value of planeRightHandSide method is an array
        // of delta-q values for each of the four steps.

        self.planeRightHandSide(self.q,self.q,ds, 0.0, dq1);
        self.planeRightHandSide(self.q, dq1, ds, 0.5, dq2);
        self.planeRightHandSide(self.q, dq2, ds, 0.5, dq3);
        self.planeRightHandSide(self.q, dq3, ds, 1.0, dq4);

        //  Update the dependent and independent variable values
        //  at the new dependent variable location and store the
        //  values in the ODE object arrays.
        self.s = self.s + ds;

        for i in 0..self.numEqns
        {
            self.q[i] = self.q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
            self.q[i] = self.q[i];

        }


    }


//*************************************************************
//  This method loads the right-hand sides for the plane ODEs
//*************************************************************
    fn planeRightHandSide(&mut self, q: [f64;6], deltaQ: [f64;6], ds: f64, qScale: f64, mut dq: [f64;6])
    {
    //  q[0] = vx = dxdt
    //  q[1] = x
    //  q[2] = vy = dydt
    //  q[3] = y
    //  q[4] = vz = dzdt
    //  q[5] = z
        let mut newQ: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values 

        let yo = -1.0_f64;
        let pi = yo.acos();

        let G: f64 = -9.81;
    
        let vx: f64;
        let vy: f64;
        let vz: f64;
        let x: f64;
        let y: f64;
        let z: f64;
        let vh: f64;
        let vtotal: f64;
        let temperature: f64;
        let grp: f64;
        let pressure: f64;
        let density: f64;
        let omega: f64;
        let factor: f64;
        let advanceRatio: f64;
        let thrust: f64;
        let mut cl: f64;
        let lift: f64;
        let aspectRatio: f64;
        let cd: f64;
        let drag: f64;
        let cosW: f64;   //  bank angle
        let sinW: f64;   //  bank angle
        let cosP: f64;   //  climb angle
        let sinP: f64;   //  climb angle
        let cosT: f64;   //  heading angle
        let sinT: f64;   //  heading angle
        let Fx: f64;
        let Fy: f64;
        let mut Fz: f64;
       

        // let mut bank: f64;
        // let alpha: f64;  
        // let throttle: f64;
        // let wingArea: f64;
        // let wingSpan: f64;
        // let tailArea: f64;
        // let clSlope0: f64;    
        // let cl0: f64;       
        // let clSlope1: f64;    
        // let cl1: f64;         
        // let alphaClMax: f64;  
        // let cdp: f64;         
        // let eff: f64;         
        // let mass: f64;
        // let enginePower: f64;
        // let engineRps: f64;   
        // let propDiameter: f64;
        // let a: f64;          
        // let b: f64;          

    //  Convert bank angle from degrees to radians
    //  Angle of attack is not converted because the
    //  Cl-alpha curve is defined in terms of degrees.
    self.bank = self.bank.to_radians();
   // println!("{}", self.bank);

    //  Compute the intermediate values of the 
    //  dependent variables.

    for i in 0..6
    {
        newQ[i] = q[i] + qScale * deltaQ[i]; 
        //println!("{}", newQ[i]);
    }

    //  Assign convenenience variables to the intermediate 
    //  values of the locations and velocities.
    vx = newQ[0];
    vy = newQ[2];
    vz = newQ[4];
    x = newQ[1];
    y = newQ[3];
    z = newQ[5];
    vh = (vx * vx + vy * vy).sqrt();
    vtotal = (vx * vx + vy * vy + vz * vz).sqrt();

    //  Compute the air density
    temperature = 288.15 - 0.0065 * z;
    grp = 1.0 - 0.0065 * z / 288.15;
    pressure = 101325.0 * (grp.powf(5.25));
    density = 0.00348 * pressure / temperature;

    //  Compute power drop-off factor
    omega = density / 1.225;
    factor = (omega - 0.12)/  0.88;

    //  Compute thrust
    advanceRatio = vtotal / (self.engineRps * self.propDiameter);
    thrust = self.throttle * factor * self.enginePower * (self.a + self.b * advanceRatio * advanceRatio) / (self.engineRps * self.propDiameter);

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
    lift = 0.5 * cl * density * vtotal * vtotal * self.wingArea;

    // //  Compute drag coefficient
    aspectRatio = self.wingSpan * self.wingSpan / self.wingArea;
    cd = self.cdp + cl * cl / (pi * aspectRatio * self.eff);
    
    // //  Compute drag force
    drag = 0.5 * cd * density * vtotal * vtotal * self.wingArea;


    //  Define some shorthand convenience variables
    //  for use with the rotation matrix.
    //  Compute the sine and cosines of the climb angle,
    //  bank angle, and heading angle;
    cosW = self.bank.cos(); 
    sinW = self.bank.sin(); 

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
    Fx = cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift;
    Fy = sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift;
    Fz = sinP * (thrust - drag) + cosP * cosW * lift;

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




