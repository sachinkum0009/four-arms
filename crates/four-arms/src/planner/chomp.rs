use crate::{
    errors::FourArmError,
    planner::Planner,
    robot::{Joint, Pose},
};

/// **CHOMP** Planner
///
/// for plannig the trajectory
pub struct CHOMP {}

impl Planner for CHOMP {
    fn new(step_size: f64, max_iter: usize) -> Self {
        Self {}
    }

    fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
        Err(FourArmError::TrajPlanError(
            "failed to plan traj".to_string(),
        ))
    }
}

impl Default for CHOMP {
    fn default() -> Self {
        Self {}
    }
}
