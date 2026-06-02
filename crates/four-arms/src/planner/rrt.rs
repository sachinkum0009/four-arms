use crate::{
    errors::FourArmError,
    planner::{ConfigExt, JointState, Planner, Trajectory},
    robot::{Joint, Pose},
};

use rand::RngExt;

/// RRT Node
struct RRTNode {
    config: JointState,
    parent_idx: Option<usize>,
}

/// Rapidly Random exploring Tree
///
/// Planning algorithm to plan trajectory to reach
/// target goal.
/// Steps
/// 1. Initialize Tree
/// 2. Random Sampling
/// 3. Find Nearest Node
/// 4. Steer towards Sample
/// 5. Check for goal reached
/// 6. Repeat Until sucess
/// 7. Extract the Path
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
    ) -> Result<Trajectory, FourArmError> {
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
            joint_limits: vec![
                (-1.571, 1.571),
                (-1.571, 1.571),
                (-1.571, 1.571),
                (-1.571, 1.571),
                (-1.571, 1.571),
                (-1.571, 1.571),
            ],
        }
    }
}
