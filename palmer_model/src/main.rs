//Based on palmers code found here 
//https://github.com/Apress/physics-for-game-programmers/blob/master/Code/C_Code/Chapter10_Planes/FlightSimulator.c
//********************************************
//  This structure defines the data required
//  to model a plane.
//********************************************
#[allow(non_snake_case)]
#[derive(Debug)]
struct Plane {

    numEqns: usize, //int
    s: f64,
    q: [f64;6],
 
    bank: f64,
    alpha: f64,  //  angle of attack
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

fn main()
{
    let mut x: f64;
    let mut z: f64;
    let mut v: f64;
    let mut time: f64;
    let dt = 0.5; 

    //create plane, set airplane data
    let mut plane = Plane  {
    wingArea: 16.2, wingSpan: 10.9, tailArea: 2.0, clSlope0: 0.0889, 
    cl0: 0.178, clSlope1: -0.1, cl1: 3.2, alphaClMax: 16.0, cdp: 0.034, eff: 0.77, mass: 1114.0,
    enginePower: 119310.0, engineRps: 40.0, propDiameter: 1.905, a: 1.83, b:-1.32, bank: 0.0,
    alpha: 4.0, throttle: 1.0, flap: String::from("0"), numEqns: 6, s: 0.0, q: [0.0;6]
    };



    //  Set airplane data
    // plane.wingArea = 16.2;     //  wing wetted area, m^2
    // plane.wingSpan = 10.9;     //  wing span, m
    // plane.tailArea = 2.0;      //  tail wetted area, m^2
    // plane.clSlope0 = 0.0889;   //  slope of Cl-alpha curve
    // plane.cl0 = 0.178;         //  Cl value when alpha = 0
    // plane.clSlope1 = -0.1;     //  slope of post-stall Cl-alpha curve
    // plane.cl1 = 3.2;           //  intercept of post-stall Cl-alpha curve
    // plane.alphaClMax = 16.0;      //  alpha at Cl(max)
    // plane.cdp = 0.034;            //  parasitic drag coefficient
    // plane.eff = 0.77;             //  induced drag efficiency coefficient
    // plane.mass = 1114.0;          //  airplane mass, kg
    // plane.enginePower = 119310.0; //  peak engine power, W
    // plane.engineRps = 40.0;       //  engine turnover rate, rev/s
    // plane.propDiameter = 1.905;   //  propeller diameter, m
    // plane.a = 1.83;     //  propeller efficiency curve fit coefficient
    // plane.b = -1.32;    //  propeller efficiency curve fit coefficient
    // plane.bank = 0.0;         
    // plane.alpha = 4.0;        
    // plane.throttle = 1.0;   
    // plane.flap = "0";         //  Flap setting

    // plane.numEqns = 6;
    // plane.s = 0.0;      //  time 
    // plane.q[0] = 0.0;   //  vx 
    // plane.q[1] = 0.0;   //  x  
    // plane.q[2] = 0.0;   //  vy 
    // plane.q[3] = 0.0;   //  y  
    // plane.q[4] = 0.0;   //  vz 
    // plane.q[5] = 0.0;   //  z  
  

    while plane.s < 40.0
    {
           plane.planeRungeKutta4( dt);

        time = plane.s;
        x = plane.q[1];
        z = plane.q[5];
        v = (( plane.q[0]*plane.q[0] + plane.q[2]*plane.q[2] + plane.q[4]*plane.q[4] ).sqrt()).abs();
    
        println!("------------------");
        println!("time = {}", time);
        println!("x = {}", x);
        println!("altitude = {}", z);
        println!("airspeed = {}", v);


    }
    println!("{:#?}", p);
}

//************************************************************
//  This method solves for the plane motion using a
//  4th-order Runge-Kutta solver
//************************************************************
impl Plane{


    fn planeRungeKutta4(&mut self, ds: f64) -> ()
    {

        //let j: i32;
        let numEqns: usize;
        let s: f64;
        let mut q: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq1: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq2: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq3: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut dq4: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        //  Define a convenience variable to make the code more readable
        numEqns = self.numEqns;

        //  Retrieve the current values of the dependent and independent variables, (had a for loop for q)
        s = self.s;
        for j in 0..numEqns
        {
            q[j] = self.q[j];
           // println!("{}",self.q[j]);
        }
    
        // Compute the four Runge-Kutta steps, The return 
        // value of planeRightHandSide method is an array
        // of delta-q values for each of the four steps.

        self.planeRightHandSide( q, q,   ds, 0.0, dq1);
        self.planeRightHandSide(q, dq1, ds, 0.5, dq2);
        self.planeRightHandSide(q, dq2, ds, 0.5, dq3);
        self.planeRightHandSide(q, dq3, ds, 1.0, dq4);

        //  Update the dependent and independent variable values
        //  at the new dependent variable location and store the
        //  values in the ODE object arrays.
        self.s = self.s + ds;

         for j in 0..numEqns
        {
            q[j] = q[j] + (dq1[j] + 2.0*dq2[j] + 2.0*dq3[j] + dq4[j])/6.0;
            self.q[j] = q[j];

        }


    }
}

//*************************************************************
//  This method loads the right-hand sides for the plane ODEs
//*************************************************************
#[allow(non_snake_case)]
impl Plane{

    fn planeRightHandSide(& mut self, q: [f64;6], deltaQ: [f64;6], ds: f64, qScale: f64, mut dq: [f64;6]) -> ()
    {
    //  q[0] = vx = dxdt
    //  q[1] = x
    //  q[2] = vy = dydt
    //  q[3] = y
    //  q[4] = vz = dzdt
    //  q[5] = z
        let mut newQ: [f64;6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // intermediate dependent variable values.
        let mut bank: f64;
        let alpha: f64;  
        let throttle: f64;
        let wingArea: f64;
        let wingSpan: f64;
        let tailArea: f64;
        let clSlope0: f64;    
        let cl0: f64;       
        let clSlope1: f64;    
        let cl1: f64;         
        let alphaClMax: f64;  
        let cdp: f64;         
        let eff: f64;         
        let mass: f64;
        let enginePower: f64;
        let engineRps: f64;   
        let propDiameter: f64;
        let a: f64;          
        let b: f64;           

        let i: usize;

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

        bank = self.bank;
        alpha = self.alpha;  
        throttle = self.throttle;
        wingArea = self.wingArea;
        wingSpan = self.wingSpan;
        tailArea = self.tailArea;
        clSlope0 = self.clSlope0;    
        cl0 = self.cl0;       
        clSlope1 = self.clSlope1;    
        cl1 = self.cl1;         
        alphaClMax = self.alphaClMax;  
        cdp = self.cdp;         
        eff = self.eff;         
        mass = self.mass;
        enginePower = self.enginePower;
        engineRps = self.engineRps;   
        propDiameter = self.propDiameter;
        a = self.a;          
        b = self.b;           


    //  Convert bank angle from degrees to radians
    //  Angle of attack is not converted because the
    //  Cl-alpha curve is defined in terms of degrees.
    bank = bank*pi/180.0;

    //  Compute the intermediate values of the 
    //  dependent variables.

    for i in 0..6
    {
        newQ[i] = q[i] + qScale*deltaQ[i]; 
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
    vh = (vx*vx + vy*vy).sqrt();
    vtotal = (vx*vx + vy*vy + vz*vz).sqrt();

    //  Compute the air density
    temperature = 288.15 - 0.0065*z;
    grp = 1.0 - 0.0065*z/288.15;
    pressure = 101325.0*(grp.powf(5.25));
    density = 0.00348*pressure/temperature;

    //  Compute power drop-off factor
    omega = density/1.225;
    factor = (omega - 0.12)/0.88;

    //  Compute thrust
    advanceRatio = vtotal/(engineRps*propDiameter);
    thrust = throttle*factor*enginePower*
            (a + b*advanceRatio*advanceRatio)/(engineRps*propDiameter);

    //  Compute lift coefficient. The Cl curve is 
    //  modeled using two straight lines.
    if  alpha < alphaClMax  {
        cl = clSlope0*alpha + cl0;
    }
    else {
        cl = clSlope1*alpha + cl1;
    }

        //  Include effects of flaps and ground effects.
    //  Ground effects are present if the plane is
    //  within 5 meters of the ground.
        if self.flap == "20"
        {
            cl +=0.25;
        }
        if self.flap == "40"
        {
            cl +=0.5;
        }
        if z < 5.0
        {
            cl += 0.25;
        }

        //  Compute lift
        lift = 0.5*cl*density*vtotal*vtotal*wingArea;

        // //  Compute drag coefficient
        aspectRatio = wingSpan*wingSpan/wingArea;
        cd = cdp + cl*cl/(pi*aspectRatio*eff);
        

        // //  Compute drag force
        drag = 0.5*cd*density*vtotal*vtotal*wingArea;


        //  Define some shorthand convenience variables
        //  for use with the rotation matrix.
        //  Compute the sine and cosines of the climb angle,
        //  bank angle, and heading angle;
        cosW = bank.cos(); 
        sinW = bank.sin(); 

        if  vtotal == 0.0  {
            cosP = 1.0;
            sinP = 0.0;
        }
        else {
            cosP = vh/vtotal;  
            sinP = vz/vtotal;  
        }
        
        if vh == 0.0  {
            cosT = 1.0;
            sinT = 0.0;
        }
        else {
            cosT = vx/vh;
            sinT = vy/vh;
        }


        //  Convert the thrust, drag, and lift forces into
    //  x-, y-, and z-components using the rotation matrix.
        Fx = cosT*cosP*(thrust - drag) + (sinT*sinW - cosT*sinP*cosW)*lift;
        Fy = sinT*cosP*(thrust - drag) + (-cosT*sinW - sinT*sinP*cosW)*lift;
        Fz = sinP*(thrust - drag) + cosP*cosW*lift;



        //  Add the gravity force to the z-direction force.
    Fz = Fz + mass*G;


    //  Since the plane can't sink into the ground, if the
    //  altitude is less than or equal to zero and the z-component
    //  of force is less than zero, set the z-force
    //  to be zero.
    if  z <= 0.0 && Fz <= 0.0  
    {
        Fz = 0.0;
    }

    //  Load the right-hand sides of the ODE's
    dq[0] = ds*(Fx/mass);
    dq[1] = ds*vx;
    dq[2] = ds*(Fy/mass);
    dq[3] = ds*vy;
    dq[4] = ds*(Fz/mass);
    dq[5] = ds*vz;




    }
}
