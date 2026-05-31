use crate::{
    errors::FourArmError,
    robot::{Joint, Pose},
};

/// Rapidly Random exploring Tree
///
/// Planning algorithm to plan trajectory to reach
/// target goal.
pub struct RRT {
    step_size: f64,
    max_iter: usize,
}

impl RRT {
    /// initialized the RRT
    pub fn new(step_size: f64, max_iter: usize) -> Self {
        Self {
            step_size,
            max_iter,
        }
    }

    /// Plans the trajectory from start pose
    /// to goal pose
    pub fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
        Err(FourArmError::TrajPlanError(
            "Failed to plan the trajectory".to_string(),
        ))
    }
}

impl Default for RRT {
    /// Initializes RRT with default params
    /// step_size: 0.1
    /// max_iter: 50
    fn default() -> Self {
        Self {
            step_size: 0.1,
            max_iter: 50,
        }
    }
}
