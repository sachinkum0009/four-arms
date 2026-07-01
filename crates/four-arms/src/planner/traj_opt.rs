use crate::{errors::FourArmError, planner::Trajectory};

#[derive(Debug)]
pub struct TrajOpt {
    num_points: usize,
    max_iter: usize,
    dt: f64,
    smooth_weight: f64,
    obstacle_weight: f64,
    collision_threshold: f64,
}

impl TrajOpt {
    pub fn new(
        num_points: usize,
        max_iter: usize,
        dt: f64,
        smooth_weight: f64,
        obstacle_weight: f64,
        collision_threshold: f64,
    ) -> Self {
        Self {
            num_points,
            max_iter,
            dt,
            smooth_weight,
            obstacle_weight,
            collision_threshold,
        }
    }
    /// Plans the trajectory using [TrajOpt] algorithm
    ///
    /// # Arguments
    /// - start_joints
    /// - goal_joints
    ///
    /// # Returns
    /// Result<Trajectory, FourArmError>
    ///
    pub fn plan_traj(
        &self,
        start_joints: &[f64],
        goal_joints: &[f64],
    ) -> Result<Trajectory, FourArmError> {
        Err(FourArmError::FunctionNotImplemented)
    }

    fn obstacle_cost(&self, trajectory: &Trajectory) -> f64 {
        // TODO: replace the code later
        1.0
    }

    fn smoothness_cost(&self, trajectory: &Trajectory) -> f64 {
        // TODO: replace the code later
        1.0
    }

    fn path_length_cost(&self, trajectory: &Trajectory) -> f64 {
        // TODO: replace the code later
        1.0
    }

    /// Calculates the total cost for the trajectory execution
    ///
    /// # Arguments
    /// - trajectory: [Trajectory]
    ///
    /// # Returns
    /// f64
    fn total_cost(&self, trajectory: &Trajectory) -> f64 {
        self.path_length_cost(trajectory)
            + self.smoothness_cost(trajectory)
            + self.obstacle_cost(trajectory)
    }
}

impl Default for TrajOpt {
    fn default() -> Self {
        Self {
            num_points: 50,
            max_iter: 200,
            dt: 0.1,
            smooth_weight: 1.0,
            obstacle_weight: 1.0,
            collision_threshold: 1.0,
        }
    }
}
