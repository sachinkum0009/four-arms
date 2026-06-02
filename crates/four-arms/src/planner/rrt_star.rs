use crate::{
    errors::FourArmError,
    planner::{ConfigExt, JointState, Trajectory},
    robot::{Joint, Pose},
};

use rand::RngExt;

struct RRTNode {
    config: JointState,
    parent_idx: Option<usize>,
    // cost: f64,
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
    pub fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
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

            if let Some(new_config) = nearest_node
                .config
                .step_towards(&rand_config, self.step_size)
            {
                if !new_config.is_in_collision() {
                    tree.push(RRTNode {
                        config: new_config.clone(),
                        parent_idx: Some(nearest_idx),
                    });

                    if new_config.distance(goal_joints) < self.step_size {
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
            path.reverse();
            Ok(path)
        } else {
            Err(FourArmError::TrajPlanError(
                "RRT failed to find a valid trajectory within max_iter bounds.".to_string(),
            ))
        }
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
