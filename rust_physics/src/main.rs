//goal: go through bourgs code examples to learn about the-
//basics and also attempt to code them in rust

//Chapter 1 basics: mass properties

//mass moment of inertia of a body is a quantitative measure of the radial distribution of
//the mass of a body about a given axis of rotation
//also known as rotiational intertia, its a measure of a bodys resistence to rotational motion

//the point masses making up the body
//are represented by an array of structures where each structure contains the point mass’s
//design coordinates and mass.

#[allow(non_snake_case)]
#[derive(Default, Debug)]
struct _PointMass
{
    mass: u32,
    designPosition: Vec<u32>, //coordinates
    correctedPosition: Vec<u32>,
    localIntertia: Vec<f32>//element’s local moment of inertia
}

#[allow(non_snake_case)]
fn main() 
{
    let mut elements = vec![_PointMass{mass: 100, designPosition: vec![3, 2, 1], correctedPosition: vec![0,0,0], localIntertia: vec![0.0,0.0,0.0]},
     _PointMass{mass: 200, designPosition: vec![7, 8, 9], correctedPosition: vec![0,0,0], localIntertia: vec![0.0,0.0,0.0]}];// ..Default::default()}]; //..Default::default()}
    //elements.push(_PointMass{mass: 100, ..Default::default()})
    
    let mut TotalMass = 0;
    let mut CombinedCG = vec![0,0,0];
   // let mut FirstMoment = vec![0,0,0];
    let mut FirstMomentX: u32 = 0;
    let mut FirstMomentY: u32 = 0;
    let mut FirstMomentZ: u32 = 0;
   // let mut Moments: Vec<Vec<u32>> = vec![Vec::with_capacity(3); 0] ; //vector of vectors
    //let mut vec = vec![vec!['#'; 80]; 24]; //example..

    for i in elements.iter()
    {
        TotalMass = TotalMass + i.mass;
    }
    println!("Total mass: {}", TotalMass);


    for i in elements.iter()
    {
                //x coord
                FirstMomentX = FirstMomentX + i.mass * i.designPosition[0];
               //y coord
                FirstMomentY = FirstMomentY + i.mass * i.designPosition[1];
                //z coord
                FirstMomentZ = FirstMomentZ + i.mass * i.designPosition[2];
    } 
  

    CombinedCG[0] = FirstMomentX / TotalMass;
    CombinedCG[1] = FirstMomentY / TotalMass;
    CombinedCG[2] = FirstMomentZ / TotalMass;
    println!("Coordinates for the center of mass: {:?}", CombinedCG);

    // now calculate relative position for each point mass...
    for i in elements.iter_mut()
    {
        i.correctedPosition[0] = u32::wrapping_add(i.designPosition[0], CombinedCG[0]);
        i.correctedPosition[1] = u32::wrapping_add(i.designPosition[1], CombinedCG[1]);
        i.correctedPosition[2] = u32::wrapping_add(i.designPosition[2], CombinedCG[2]);
    }



//To calculate mass moment of inertia, you need to take the second moment of each
//elemental mass making up the body about each coordinate axis.

//A tensor is a mathematical expression that has magnitude and direction, but its magnitude
//may not be unique depending on the direction.

// the struct holds local moment of inertia
//vector to represent the thee local moment of intertia terms and we assume 
//that local products of intertia are zero for each element

//calculate inertia tensor given component elements

    let mut Ixx: f32 = 0.0;
    let mut Iyy: f32 = 0.0;
    let mut Izz: f32 = 0.0;
    let mut Ixy: f32 = 0.0;
    let mut Ixz: f32 = 0.0;
    let mut Iyz: f32 = 0.0;

    for i in elements.iter()
    {
       Ixx = Ixx + i.localIntertia[0] +
       (i.mass as f32) * ((i.correctedPosition[1] as f32) *
        (i.correctedPosition[1] as f32) +
        (i.correctedPosition[2] as f32) *
        (i.correctedPosition[2] as f32));

        Iyy = Iyy + i.localIntertia[1] +
        (i.mass as f32) * ((i.correctedPosition[2] as f32) *
         (i.correctedPosition[2] as f32) +
         (i.correctedPosition[0] as f32) *
         (i.correctedPosition[0] as f32));

         Izz = Izz + i.localIntertia[2] +
         (i.mass as f32) * ((i.correctedPosition[0] as f32) *
          (i.correctedPosition[0] as f32) +
          (i.correctedPosition[1] as f32) *
          (i.correctedPosition[1] as f32));

          Ixy = Ixy + (i.mass as f32) * ((i.correctedPosition[0] as f32) *
          (i.correctedPosition[1] as f32));

          Ixz = Ixz + (i.mass as f32) * ((i.correctedPosition[0] as f32) *
          (i.correctedPosition[2] as f32));

          Iyz = Iyz + (i.mass as f32) * ((i.correctedPosition[1] as f32) *
          (i.correctedPosition[2] as f32));
    }

    //print out point mass struct
   for i in elements.iter()
   {
       println!("Pointmass struct {:?}", i);
   }

    let mut inertiaTensor: [[f32;3];3] = [[0.0; 3]; 3]; //3x3 matrix
    //inertiaTensor[2][0] = 5; //left bracket is which row, right is column of that row

    inertiaTensor[0][0] = Ixx;
    inertiaTensor[0][1] = -Ixy;
    inertiaTensor[0][2] = -Ixz;

    inertiaTensor[1][0] = -Ixy;
    inertiaTensor[1][1] = Iyy;
    inertiaTensor[1][2] = -Iyz;

    inertiaTensor[2][0] = -Ixz;
    inertiaTensor[2][1] = -Iyz;
    inertiaTensor[2][2] = Izz;

    println!("Inertia tensor: {:?}", inertiaTensor);
}
