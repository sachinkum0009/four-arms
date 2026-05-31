use four_arms::chain::Chain;
use four_arms::errors::FourArmError;
use std::env;

#[tokio::main]
async fn main() -> Result<(), FourArmError> {
    let file_path = env::args()
        .nth(0)
        .unwrap_or_else(|| "urdf/my_robot2.urdf".to_string());
    let chain = Chain::from_urdf(&file_path)?;

    let joints1 = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
    let pose = chain.forward_kinematics(&joints1)?;
    println!("pose: {:?}", pose);
    let joints = [0.3, -0.3, 0.4, 0.3, 0.4, 0.5];
    let pose = chain.forward_kinematics(&joints)?;
    println!("pose: {:?}", pose);

    let res_joints = chain.inverse_kinematics(&pose, &joints1, 50, 0.1, 0.2)?;
    println!("res joint angles: {:?}", res_joints);
    Ok(())
}
