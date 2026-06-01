use four_arms::chain::Chain;
use four_arms::planner::{Planner, RRT};
use four_arms::robot::Pose;
use rerun::RecordingStreamBuilder;
use rerun::external::re_importer::UrdfTree;
use rerun::external::{re_log, urdf_rs};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    re_log::setup_logging();
    let urdf_path = "/Users/mac/zzzzz/rust/robotics/robotics/urdf/my_robot2.urdf";
    let chain = Chain::from_urdf(urdf_path)?;
    let rec = RecordingStreamBuilder::new("ik_planner_example")
        .recording_id("run-1")
        .connect_grpc()?;
    rec.log_file_from_path(urdf_path, None, true)?;
    let urdf = UrdfTree::from_file_path(urdf_path, None)?;

    // Log ee_path and ee_marker directly under base_link in the TF tree
    // so Rerun can resolve the transform chain without any TF lookup needed
    let ee_path_entity = "/six_dof_arm/visual_geometries/base_link/ee_path";
    let ee_marker_entity = "/six_dof_arm/visual_geometries/base_link/ee_marker";

    let joint_limits = chain.get_joint_limits();
    let start_joints = [1.5, 0.2, 0.3, 0.3, 0.0, 0.0];
    let goal_joints = [-1.5, 1.3, 1.4, 1.3, 1.0, 1.5];
    let rrt = RRT::new(0.2, 500, joint_limits);
    let traj = rrt.plan_traj(&start_joints, &goal_joints)?;

    let mut ee_path: Vec<[f32; 3]> = Vec::new();

    for (step, planned_joints) in traj.iter().enumerate() {
        rec.set_time_sequence("step", step as i64);

        // Animate URDF joints
        let revolute_joints: Vec<_> = urdf
            .joints()
            .filter(|j| j.joint_type == urdf_rs::JointType::Revolute)
            .collect();
        for (joint, &angle) in revolute_joints.iter().zip(planned_joints.iter()) {
            let joint_transform = urdf.compute_joint_transform(joint, angle, true)?;
            rec.log("/transforms", &joint_transform)?;
        }

        // Forward kinematics → end-effector position
        if let Ok(pose) = chain.forward_kinematics(planned_joints) {
            let p = pose.position;
            let pos = [p[0] as f32, p[1] as f32, p[2] as f32];
            ee_path.push(pos);

            // Current EE marker — child of base_link so TF resolves
            rec.log(
                ee_marker_entity,
                &rerun::Points3D::new([pos])
                    .with_radii([0.02])
                    .with_colors([[255_u8, 100, 30]]),
            )?;

            // Growing line — child of base_link so TF resolves
            if ee_path.len() >= 2 {
                rec.log(
                    ee_path_entity,
                    &rerun::LineStrips3D::new([ee_path.clone()])
                        .with_colors([[80_u8, 200, 255]])
                        .with_radii([0.005]),
                )?;
            }
        }
    }

    // Final complete path as static
    if !ee_path.is_empty() {
        rec.log_static(
            ee_path_entity,
            &rerun::LineStrips3D::new([ee_path])
                .with_colors([[255_u8, 220, 50]])
                .with_radii([0.005]),
        )?;
    }

    println!("Done!");
    Ok(())
}
