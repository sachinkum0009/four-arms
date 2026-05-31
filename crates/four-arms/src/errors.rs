/*!
 * Provides the enum for managing different errors
 */
use thiserror::Error;

#[derive(Error, Debug)]
/// Top-level error type for robotic arm operations, covering kinematics,
/// file parsing, configuration mismatches, and motion planning.
pub enum FourArmError {
    /// Triggered when forward kinematics (FK) calculations fail.
    #[error("Failed to calc Forward Kinematics: {0}")]
    FkError(String),

    /// Triggered when inverse kinematics (IK) analytical or numerical solvers fail to converge.
    #[error("Failed to calc Inverse Kinematics: {0}")]
    IkError(String),

    /// Triggered when the input URDF or XML robot description file is malformed or unreadable.
    #[error("Failed to parse xml or urdf file: {0}")]
    ParseError(String),

    /// Triggered when the provided joint state vector configuration size does not match the robot's physical degrees of freedom (DoF).
    #[error("Joint size doesn't match: {0} (Received: {1})")]
    JointMismatch(String, usize),

    /// Triggered when the motion planner fails to find a valid collision-free path or trajectory.
    #[error("Failed to plan the trajectory: {0}")]
    TrajPlanError(String),
}
