use rand::RngExt;

use crate::{
    errors::FourArmError,
    planner::{ConfigExt, JointState, Trajectory},
    robot::{Joint, Pose},
};


struct RRTNode {
    config: JointState,
    parent_idx: Option<usize>,
    cost: f64,
}

/// Rapidly Random exploring Tree
///
/// Planning algorithm to plan trajectory to reach
/// target goal.
pub struct RRTStar {
    step_size: f64,
    max_iter: usize,
    joint_limits: Vec<(f64, f64)>,
    radius: f64,
}

impl RRTStar {
    /// initialized the RRT
    pub fn new(
        step_size: f64,
        max_iter: usize,
        joint_limits: Vec<(f64, f64)>,
        radius: f64,
    ) -> Self {
        Self {
            step_size,
            max_iter,
            joint_limits,
            radius,
        }
    }

    /// Plans the trajectory from start pose
    /// to goal pose
    pub fn plan(&self, _start_pos: &Pose, _goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
        Err(FourArmError::TrajPlanError(
            "Failed to plan the trajectory".to_string(),
        ))
    }

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

        let mut tree = vec![RRTNode {
            config: start_joints.to_vec(),
            parent_idx: None,
            cost: 0.0, // initial cost 0
        }];

        let mut rng = rand::rng();

        let mut goal_reached = false;
        let goal_bias = 0.10;

        for _ in 0..self.max_iter {
            let rand_config = if rng.random_bool(goal_bias) {
                goal_joints.to_vec()
            } else {
                self.joint_limits
                    .iter()
                    .map(|&(min, max)| rng.random_range(min..=max))
                    .collect::<JointState>()
            };

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

            let nearest_config = nearest_node.config.clone();
            let new_config = match nearest_config.step_towards(&rand_config, self.step_size) {
                Some(c) => c,
                None => continue,
            };

            if new_config.is_in_collision() {
                continue;
            }

            let near_indices: Vec<usize> = tree
                .iter()
                .enumerate()
                .filter(|(_, node)| node.config.distance(&new_config) <= self.radius)
                .map(|(i, _)| i)
                .collect();

            let (best_parent_idx, best_cost) = near_indices.iter().fold(
                (nearest_idx, f64::INFINITY),
                |(best_idx, best_cost), &i| {
                    let edge_cost = tree[i].config.distance(&new_config);
                    let candidate_cost = tree[i].cost + edge_cost;
                    // Only consider collision-free edges
                    if candidate_cost < best_cost
                    // && !self.path_in_collision(&tree[i].config, &new_config)
                    {
                        (i, candidate_cost)
                    } else {
                        (best_idx, best_cost)
                    }
                },
            );

            let (best_parent_idx, best_cost) = if best_cost == f64::INFINITY {
                let edge_cost = nearest_config.distance(&new_config);
                let cost = tree[nearest_idx].cost + edge_cost;
                (nearest_idx, cost)
            } else {
                (best_parent_idx, best_cost)
            };

            let new_idx = tree.len();
            tree.push(RRTNode {
                config: new_config.clone(),
                parent_idx: Some(best_parent_idx),
                cost: best_cost,
            });

            for &near_idx in &near_indices {
                let edge_cost = new_config.distance(&tree[near_idx].config);
                let potential_cost = best_cost + edge_cost;
                if potential_cost < tree[near_idx].cost {
                    tree[near_idx].parent_idx = Some(new_idx);
                    tree[near_idx].cost = potential_cost;
                }
            }

            if new_config.distance(goal_joints) < self.step_size {
                let goal_cost = best_cost + new_config.distance(goal_joints);
                tree.push(RRTNode {
                    config: goal_joints.to_vec(),
                    parent_idx: Some(new_idx),
                    cost: goal_cost,
                });
                goal_reached = true;
                break;
            }
        }

        if goal_reached {
            let path = RRTStar::extract_path(&tree);
            Ok(path)
        } else {
            Err(FourArmError::TrajPlanError(
                "RRT* failed to find a valid trajectory within max_iter bounds.".to_string(),
            ))
        }
    }

    /// Extract the path
    /// # Arguments
    /// tree: &[RRTNode]
    ///
    /// # Result
    /// Trajectory
    fn extract_path(tree: &[RRTNode]) -> Trajectory {
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
        path.reverse();
        path
    }
}

impl Default for RRTStar {
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
            radius: 0.2,
        }
    }
}
