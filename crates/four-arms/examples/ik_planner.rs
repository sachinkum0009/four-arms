use four_arms::chain::Chain;
use four_arms::errors::FourArmError;
use four_arms::planner::{CHOMP, PRM, Planner, RRT, RRTStar};
use four_arms::robot::Pose;
use rerun::{RecordingStreamBuilder, RecordingStreamError};
use std::env;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let file_path = "/Users/mac/zzzzz/rust/robotics/robotics/urdf/my_robot2.urdf".to_string();
    let file_path = env::args()
        .nth(0)
        .unwrap_or_else(|| "urdf/my_robot2.urdf".to_string());
    let chain = Chain::from_urdf(&file_path)?;
    let rec = RecordingStreamBuilder::new("ik_planner_example")
        .recording_id("run-1")
        .connect_grpc()?;

    for t in 0..10 {
        rec.set_time_sequence("step", t);
        let tf = t as f64;
        rec.log("/arm/shoulder", &rerun::Scalars::single((tf * 0.5).sin()))?;
        rec.log("/arm/elbow", &rerun::Scalars::single((tf * 0.5).cos()))?;
    }

    let joints1 = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
    let pose = chain.forward_kinematics(&joints1)?;
    println!("pose: {:?}", pose);
    // let joints = [0.3, -0.3, 0.4, 0.3, 0.4, 0.5];
    // let pose = chain.forward_kinematics(&joints)?;
    // println!("pose: {:?}", pose);

    // let res_joints = chain.inverse_kinematics(&pose, &joints1, 50, 0.1, 0.2)?;
    // let target_pose = Pose::default();
    // println!("res joint angles: {:?}", res_joints);

    // let rrt = RRT::default();
    // // let traj = rrt.plan(&pose, &target_pose)?;
    //
    //
    // println!("trajectory of joints: {:?}", traj);

    // let joint_limits = chain.get_joint_limits();
    // let start_joints = [0.0, 0.2, 0.3, 0.3];
    // let goal_joints = [1.0, 1.3, 1.4, 1.3];
    // let rrt = RRT::new(0.2, 100, joint_limits);
    // let traj = rrt.plan_traj(&start_joints, &goal_joints)?;
    // for joint in traj {
    //     println!("joints: {:?}", joint);
    // }
    Ok(())
}
