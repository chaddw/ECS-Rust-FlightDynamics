//goal: go through bourgs code examples to learn about the-
//basics and also attempt to code them in rust

//mass properties

//mass moment of inertia of a body is a quantitative measure of the radial distribution of
//the mass of a body about a given axis of rotation
//also known as rotiational intertia, its a measure of a bodys resistence to rotational motion



//the point masses making up the body
//are represented by an array of structures where each structure contains the point mass’s
//design coordinates and mass.

#[derive(Default)]
struct _PointMass
{
    mass: u32,
    designPosition: Vec<u32>, //coordinates
    correctedPosition: Vec<u32>,
    localIntertia: Vec<f64>
}
const NUMELEMENTS: usize = 2;


fn main() 
{
    let mut elements = vec![_PointMass{mass: 100, designPosition: vec![3, 2, 1], correctedPosition: vec![0,0,0], ..Default::default()},
     _PointMass{mass: 200, designPosition: vec![7, 8, 9], correctedPosition: vec![0,0,0], ..Default::default()}];
    //elements.push(_PointMass{mass: 100, ..Default::default()})
    
    let mut TotalMass = 0;
   // let mut CombinedCG = vec![0,0,0];
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
    let totalX = FirstMomentX / TotalMass;
    let totalY = FirstMomentY / TotalMass;
    let totalZ = FirstMomentZ / TotalMass;

    println!("Coordinates for the center of mass: {}, {}, {}", totalX, totalY, totalZ);

    // calculate relative position for each point mass...

    let mut CorrectedX: u32 = 0;
    let mut CorrectedtY: u32 = 0;
    let mut CorrectedZ: u32 = 0;

    for i in elements.iter()
    {


        
           // i.correctedPosition[0] =//i.designPosition[0] - totalX;
           // i.correctedPosition[1] = i.designPosition[1] - totalY;
           // i.correctedPosition[2] = i.designPosition[2] - totalZ;



    }

//To calculate mass moment of inertia, you need to take the second moment of each
//elemental mass making up the body about each coordinate axis.

//A tensor is a mathematical expression that has magnitude and direction, but its magnitude
//may not be unique depending on the direction.



// the struct holds local moment of inertia
//vector to represent the thee local moment of intertia terms and we assume 
//that local products of intertia are zero for each element

//calculate inertia tensor given component elements


}
