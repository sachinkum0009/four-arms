/*!
 * # Planners
 *
 * Helps to plan the trajectory from start to goal pose
 */

mod chomp;
mod prm;
mod rrt;
mod rrt_connect;
mod rrt_star;
pub mod smoother;

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
    fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError>;
}
