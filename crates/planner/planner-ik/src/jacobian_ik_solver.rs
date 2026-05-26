use crate::ik_solver::IkSolve;

use planner_core::utils::{Joints, Pose};

/// Jacobian Ik Solver
pub struct JacobianIkSolver {
    max_iterations: usize,
    convergence_threshold: f64,
    damping: f64,
}

impl JacobianIkSolver {
    pub fn new(max_iterations: usize, convergence_threshold: f64, damping: f64) -> Self {
        Self {
            max_iterations,
            convergence_threshold,
            damping,
        }
    }
}

impl Default for JacobianIkSolver {
    fn default() -> Self {
        Self {
            max_iterations: 200,
            convergence_threshold: 1e-4,
            damping: 0.01,
        }
    }
}

impl<const N: usize> IkSolve<N> for JacobianIkSolver {
    fn compute_traj(
        &self,
        current_joints: &Joints<N>,
        target_pose: &Pose,
    ) -> Result<Vec<Joints<N>>, String> {
        let joints = Joints::<N>::zeros();
        let mut trajectory = Vec::new();
        trajectory.push(joints);
        Ok(trajectory)
    }
}
