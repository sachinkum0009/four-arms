use crate::{
    errors::FourArmError,
    planner::Planner,
    robot::{Joint, Pose},
};

use rand::RngExt;

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

/// RRT Node
struct RRTNode {
    config: Vec<f64>,
    parent_idx: Option<usize>,
}

/// Rapidly Random exploring Tree
///
/// Planning algorithm to plan trajectory to reach
/// target goal.
pub struct RRT {
    step_size: f64,
    max_iter: usize,
    joint_limits: Vec<(f64, f64)>,
}

impl RRT {
    /// initialized the RRT
    pub fn new(step_size: f64, max_iter: usize, joint_limits: Vec<(f64, f64)>) -> Self {
        Self {
            step_size,
            max_iter,
            joint_limits,
        }
    }

    /// Plans the trajectory from start pose
    /// to goal pose
    fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
        Err(FourArmError::TrajPlanError(
            "Failed to plan the trajectory".to_string(),
        ))
    }
}

impl RRT {
    pub fn plan_traj(
        &self,
        start_joints: &[f64],
        goal_joints: &[f64],
    ) -> Result<Vec<Vec<f64>>, FourArmError> {
        if start_joints.len() != goal_joints.len() {
            return Err(FourArmError::JointMismatch(
                "joint of size doesn't match".to_string(),
                goal_joints.len(),
            ));
        }

        let mut tree: Vec<RRTNode> = vec![RRTNode {
            config: start_joints.to_vec(),
            parent_idx: None,
        }];

        let mut rng = rand::rng();

        let mut goal_reached = false;
        let goal_bias = 0.10; // 10% chance to sample the goal directly to speed up convergence

        for _ in 0..self.max_iter {
            let rand_config = if rng.random_bool(goal_bias) {
                goal_joints.to_vec()
            } else {
                self.joint_limits
                    .iter()
                    .map(|&(min, max)| rng.random_range(min..=max))
                    .collect::<Vec<f64>>()
            };

            // Find the closest node in the current tree to the sampled configuration
            let (nearest_idx, nearest_node) = tree
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let dist_a = a.config.distance(&rand_config);
                    let dist_b = b.config.distance(&rand_config);
                    dist_a
                        .partial_cmp(&dist_b)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();

            // Step from the nearest node towards the random configuration by `step_size`
            if let Some(new_config) = nearest_node
                .config
                .step_towards(&rand_config, self.step_size)
            {
                // Collision check: only add the node if the path/configuration is safe
                if !new_config.is_in_collision() {
                    tree.push(RRTNode {
                        config: new_config.clone(),
                        parent_idx: Some(nearest_idx),
                    });

                    if new_config.distance(goal_joints) < self.step_size {
                        // Add final step exactly to the goal
                        if !goal_joints.is_in_collision() {
                            tree.push(RRTNode {
                                config: goal_joints.to_vec(),
                                parent_idx: Some(tree.len() - 1),
                            });
                            goal_reached = true;
                            break;
                        }
                    }
                }
            }
        }

        // Reconstruct the path if the goal was successfully reached
        if goal_reached {
            let mut path = Vec::new();
            let mut current_idx = tree.len() - 1;

            while let Some(node) = tree.get(current_idx) {
                path.push(node.config.clone());
                if let Some(parent) = node.parent_idx {
                    current_idx = parent;
                } else {
                    break;
                }
            }

            // Path is constructed from goal to start, so reverse it
            path.reverse();
            Ok(path)
        } else {
            Err(FourArmError::TrajPlanError(
                "RRT failed to find a valid trajectory within max_iter bounds.".to_string(),
            ))
        }
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
            ..Default::default()
        }
    }
}
