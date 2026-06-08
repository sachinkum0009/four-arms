use crate::planner::Planner;

/// PRM algorithm
///
/// Used for planning trajectory for the robot.
pub struct PRM {}

impl Planner for PRM {
    fn plan(
        &self,
        start_pos: &crate::robot::Pose,
        goal_pos: &crate::robot::Pose,
    ) -> Result<Vec<crate::robot::Joint>, crate::errors::FourArmError> {
        Err(crate::errors::FourArmError::FunctionNotImplemented)
    }
    fn new(step_size: f64, max_iter: usize) -> Self {
        Self {}
    }
}

impl Default for PRM {
    fn default() -> Self {
        Self {}
    }
}
