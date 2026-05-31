use thiserror::Error;

#[derive(Error, Debug)]
pub enum FourArmError {
    #[error("Failed to calc Forward Kinematics: {0}")]
    FkError(String),
    #[error("Failed to calc Inverse Kinematics: {0}")]
    IkError(String),
    #[error("Failed to parse xml or urdf file: {0}")]
    ParseError(String),
    #[error("Joint size doesn't match: {0} (Received: {1})")]
    JointMismatch(String, usize),
}
