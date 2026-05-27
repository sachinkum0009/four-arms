use planner_core::error::PlannerError;
use planner_core::urdf_loader::UrdfLoader;
use planner_core::utils::{Joints, Pose};
use planner_ik::fk_solver::FkSolver;
use planner_ik::ik_solver::{IkSolve, IkSolver};
use planner_ik::jacobian_ik_solver::JacobianIkSolver;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf_path = "/Users/mac/zzzzz/rust/robotics/robotics/urdf/my_robot3.urdf".to_string();

    // let ik_solver = IkSolver::<6>::default();
    // let target_pose = Pose::default();
    // let current_joints = Joints::zeros();
    // let num_steps = 10;
    // let traj = ik_solver.compute_traj(current_joints, target_pose, num_steps)?;
    // for joints in traj {
    //     println!("joints: {:?}", joints);
    // }

    let fk_solver = FkSolver::from_urdf(urdf_path)?;

    let joints = Joints::new([0.3, 0.3, 0.4, 0.5, 0.5, 0.1]);
    let pose = fk_solver.calculate_pose(&joints.get_joints())?;
    println!("pose: {:?}", pose);
    let joints2 = Joints::new([0.6, 1.5, 0.1, 0.5, 3.0, 0.1]);
    let pose2 = fk_solver.calculate_pose(&joints2.get_joints());
    match pose2 {
        Ok(pose) => println!("pose: {:?}", pose),
        Err(err) => match err {
            PlannerError::FkError(log) => println!("failed to calc fk: {}", log),
            PlannerError::IkError(log) => println!("failed to calc ik: {}", log),
        },
    }

    // let urdf_loader = UrdfLoader::from_urdf(&urdf_path)?;
    // let links = urdf_loader.get_links();
    // for link in links {
    //     println!("link: {:?}", link.name);
    // }
    // let joints = urdf_loader.get_joints();
    // for joint in joints {
    //     println!("joint: {:?}", joint.axis.xyz.0);
    // }

    Ok(())
}
