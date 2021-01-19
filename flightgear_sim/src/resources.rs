//SPECS Resources

//Time step (delta time) resource
#[derive(Default)]
pub struct DeltaTime(pub f32);

//Max thrust potential resource
#[derive(Default)]
pub struct MaxThrust(pub f32);

//Delta thrust increment/ decrement
#[derive(Default)]
pub struct DeltaThrust(pub f32);
