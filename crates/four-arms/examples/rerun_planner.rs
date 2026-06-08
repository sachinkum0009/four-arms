use four_arms::chain::Chain;
use four_arms::planner::{RRT, RRTConnect, RRTStar};
use four_arms::robot::Pose;
use four_arms::smoother::CubicSplineSmoother;
use rerun::RecordingStreamBuilder;
use rerun::external::re_importer::UrdfTree;
use rerun::external::{re_log, urdf_rs};
use tracing::{Level, Span, error, info, warn};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // tracing_subscriber::fmt().init();
    re_log::setup_logging();
    let urdf_path = "/Users/mac/zzzzz/rust/robotics/robotics/urdf/my_robot2.urdf";

    let chain = Chain::from_urdf(urdf_path)?;
    let rec = RecordingStreamBuilder::new("ik_planner_example")
        .recording_id("run-1")
        .connect_grpc()?;

    rec.log_file_from_path(urdf_path, None, true)?;
    let urdf = UrdfTree::from_file_path(urdf_path, None)?;

    let ee_path_entity = "/six_dof_arm/base_link/ee_path";
    let ee_marker_entity = "/six_dof_arm/base_link/ee_marker";
    let smooth_ee_path_entity = "/six_dof_arm/base_link/smooth_ee_path";

    let joint_limits = chain.get_joint_limits();
    let start_pose = Pose::new(
        [0.20900953541769549, 0.2, 1.1938682645080299],
        [
            0.07664507071083379,
            0.33035508016638776,
            0.37440638599328097,
            0.863024282550289,
        ],
    );
    let initial_joints = [0.0; 6];
    let start_joints = chain.inverse_kinematics(&start_pose, &initial_joints, 50, 0.1, 0.2)?; // [1.5, 0.2, 0.3, 0.3, 0.0, 0.0];
    let goal_pose = Pose::new(
        [0.20900953541769549, -0.2, 0.5],
        [
            0.07664507071083379,
            0.33035508016638776,
            0.37440638599328097,
            0.863024282550289,
        ],
    );
    let goal_joints = chain.inverse_kinematics(&goal_pose, &initial_joints, 50, 0.1, 0.2)?; // [-1.5, 1.3, 1.4, 1.3, 1.0, 1.5];
    let rrt = RRTConnect::new(0.3, 500, joint_limits);
    // let rrt = RRTStar::new(0.3, 500, joint_limits, 0.2);
    let traj = rrt.plan_traj(&start_joints, &goal_joints)?;
    info!("original traj: {:?}", traj.clone());

    let cubic_spline_smoother = CubicSplineSmoother {};
    let smooth_traj = cubic_spline_smoother.smooth_traj(traj.clone(), 1.0, 2.0)?;
    info!("smooth traj: {:?}", smooth_traj);

    // Compute end-effector paths for both original and smooth trajectories
    let mut pose_path = Vec::new();
    for joints in traj.clone() {
        let pose = chain.forward_kinematics(&joints)?.position;
        pose_path.push(pose);
    }

    let mut smooth_pose_path = Vec::new();
    for joints in traj.clone() {
        let pose = chain.forward_kinematics(&joints)?.position;
        let pos = [pose[0] as f32, pose[1] as f32, pose[2] as f32];
        smooth_pose_path.push(pos);
    }

    // Register coordinate frame archetypes to ensure proper transform chain resolution
    rec.log_static(
        "/six_dof_arm/base_link/ee_path",
        &rerun::archetypes::CoordinateFrame::new("base_link"),
    )?;
    rec.log_static(
        "/six_dof_arm/base_link/ee_marker",
        &rerun::archetypes::CoordinateFrame::new("base_link"),
    )?;
    rec.log_static(
        smooth_ee_path_entity,
        &rerun::archetypes::CoordinateFrame::new("base_link"),
    )?;

    // Log the smooth trajectory path in RED
    if !smooth_pose_path.is_empty() {
        rec.log_static(
            smooth_ee_path_entity,
            &rerun::LineStrips3D::new([smooth_pose_path])
                .with_colors([[255_u8, 0, 0]]) // Red color
                .with_radii([0.006]), // Slightly thicker to distinguish from original path
        )?;
    }

    rec.log(
        "/six_dof_arm/base_link/ee_path",
        &rerun::LineStrips3D::new([pose_path.clone()])
            .with_colors([[80_u8, 200, 255]])
            .with_radii([0.005]),
    )?;

    let mut ee_path: Vec<[f32; 3]> = Vec::new();

    for (step, planned_joints) in smooth_traj.iter().enumerate() {
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

        // Forward kinematics → original end-effector position animation
        if let Ok(pose) = chain.forward_kinematics(planned_joints) {
            let p = pose.position;
            let pos = [p[0] as f32, p[1] as f32, p[2] as f32];
            ee_path.push(pos);

            // Current EE marker
            rec.log(
                ee_marker_entity,
                &rerun::Points3D::new([pos])
                    .with_radii([0.02])
                    .with_colors([[255_u8, 100, 30]]),
            )?;

            // Growing line for the raw trajectory
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

    // Final complete raw path as static
    if !ee_path.is_empty() {
        rec.log_static(
            ee_path_entity,
            &rerun::LineStrips3D::new([ee_path])
                .with_colors([[255_u8, 220, 50]])
                .with_radii([0.005]),
        )?;
    }

    info!("Done!");
    Ok(())
}
