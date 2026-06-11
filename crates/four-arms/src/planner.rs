/*!
 * # Planners
 *
 * Helps to plan the trajectory from start to goal pose
 */

mod chomp;
pub mod dijstra;
mod prm;
mod rrt;
mod rrt_connect;
mod rrt_star;

pub use chomp::CHOMP;
pub use prm::PRM;
pub use rrt::RRT;
pub use rrt_connect::RRTConnect;
pub use rrt_star::RRTStar;

use crate::{
    errors::FourArmError,
    robot::{Joint, Pose},
};

pub trait Planner: Default {
    fn new(step_size: f64, max_iter: usize) -> Self;
    /// Plans the trajectory from start pose to goal pose
    ///
    /// # Arguments
    /// - start_pos: Pose
    /// - goal_pos: Pose
    ///
    /// # Return
    /// - Result<Vec<Joint>, FourArmError>
    fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError>;
}

/// Extension trait providing geometry helpers for joint-space configs.
trait ConfigExt {
    /// Euclidean distance between two configs.
    fn distance(&self, other: &[f64]) -> f64;

    /// Steps from `self` toward `target` by at most `step_size`.
    /// Returns `None` when `self` already equals `target`.
    fn step_towards(&self, target: &[f64], step_size: f64) -> Option<Vec<f64>>;

    /// Collision check stub — always returns `false` (no collision)
    /// until a real collision checker is wired in.
    fn is_in_collision(&self) -> bool;
}

impl ConfigExt for [f64] {
    fn distance(&self, other: &[f64]) -> f64 {
        self.iter()
            .zip(other.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    fn step_towards(&self, target: &[f64], step_size: f64) -> Option<Vec<f64>> {
        let dist = self.distance(target);
        if dist == 0.0 {
            return None;
        }
        if dist <= step_size {
            return Some(target.to_vec());
        }
        let scale = step_size / dist;
        Some(
            self.iter()
                .zip(target.iter())
                .map(|(a, b)| a + (b - a) * scale)
                .collect(),
        )
    }

    fn is_in_collision(&self) -> bool {
        // TODO: integrate with the robot's collision model
        false
    }
}

// types
pub type JointState = Vec<f64>;
pub type Trajectory = Vec<JointState>;
