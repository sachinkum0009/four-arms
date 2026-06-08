use rand::RngExt;

use crate::{
    errors::FourArmError,
    planner::{ConfigExt, JointState, Planner, Trajectory},
    robot::{Joint, Pose},
};

/// RRT Node
struct RRTNode {
    config: JointState,
    parent_idx: Option<usize>,
}

/// Rapidly Random exploring Tree
///
/// Planning algorithm to plan trajectory to reach
/// target goal.
pub struct RRTConnect {
    step_size: f64,
    max_iter: usize,
    joint_limits: Vec<(f64, f64)>,
}

impl RRTConnect {
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
    ///
    fn plan(&self, start_pos: &Pose, goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
        Err(FourArmError::TrajPlanError(
            "Failed to plan the trajectory".to_string(),
        ))
    }
}

impl RRTConnect {
    /// Plans the trajectory for the robot
    /// from start_joints to goal_joints
    ///
    /// # Arguments
    /// start_joints: &[f64]
    ///
    /// goal_joints: &[f64]
    ///
    /// # Result
    /// Result<Trajectory, FourArmsError>
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

        let mut start_tree: Vec<RRTNode> = vec![RRTNode {
            config: start_joints.to_vec(),
            parent_idx: None,
        }];
        let mut goal_tree: Vec<RRTNode> = vec![RRTNode {
            config: goal_joints.to_vec(),
            parent_idx: None,
        }];

        let mut rng = rand::rng();
        let mut goal_reached = false;
        let goal_bias = 0.10;

        for _ in 0..self.max_iter {
            let start_rand_config = if rng.random_bool(goal_bias) {
                goal_joints.to_vec()
            } else {
                self.joint_limits
                    .iter()
                    .map(|&(min, max)| rng.random_range(min..=max))
                    .collect::<Vec<f64>>()
            };

            let (start_nearest_idx, start_nearest_config) = {
                let (idx, node) = start_tree
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let dist_a = a.config.distance(&start_rand_config);
                        let dist_b = b.config.distance(&start_rand_config);
                        dist_a
                            .partial_cmp(&dist_b)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .ok_or(FourArmError::EmptyTree("start_tree is empty".to_string()))?;
                (idx, node.config.clone())
            };

            let start_extended = if let Some(new_config) =
                start_nearest_config.step_towards(&start_rand_config, self.step_size)
            {
                if !new_config.is_in_collision() {
                    start_tree.push(RRTNode {
                        config: new_config.clone(),
                        parent_idx: Some(start_nearest_idx),
                    });
                    Some(new_config)
                } else {
                    None
                }
            } else {
                None
            };

            if let Some(new_start_node) = start_extended {
                let (goal_nearest_idx, goal_nearest_config) = {
                    let (idx, node) = goal_tree
                        .iter()
                        .enumerate()
                        .min_by(|(_, a), (_, b)| {
                            a.config
                                .distance(&new_start_node)
                                .partial_cmp(&b.config.distance(&new_start_node))
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .ok_or(FourArmError::EmptyTree("goal_tree is empty".to_string()))?;
                    (idx, node.config.clone())
                };

                if let Some(goal_new_config) =
                    goal_nearest_config.step_towards(&new_start_node, self.step_size)
                {
                    if !goal_new_config.is_in_collision() {
                        goal_tree.push(RRTNode {
                            config: goal_new_config.clone(),
                            parent_idx: Some(goal_nearest_idx),
                        });

                        // Check if the trees have met
                        if goal_new_config.distance(&new_start_node) <= self.step_size {
                            // return Ok(self.reconstruct_path(&start_tree, &goal_tree));
                            goal_reached = true;
                            break;
                        }
                    }
                }
            }
        }
        if goal_reached {
            let path = self.reconstruct_path(&start_tree, &goal_tree);
            Ok(path)
        } else {
            Err(FourArmError::TrajPlanError(
                "failed to plan trajectory".to_string(),
            ))
        }
    }

    fn reconstruct_path(&self, start_tree: &[RRTNode], goal_tree: &[RRTNode]) -> Trajectory {
        let mut start_path = Vec::new();
        let mut goal_path = Vec::new();

        let mut current_idx = start_tree.len() - 1;
        while let Some(node) = start_tree.get(current_idx) {
            start_path.push(node.config.clone());
            if let Some(parent) = node.parent_idx {
                current_idx = parent;
            } else {
                break;
            }
        }

        current_idx = goal_tree.len() - 1;
        while let Some(node) = goal_tree.get(current_idx) {
            goal_path.push(node.config.clone());
            if let Some(parent) = node.parent_idx {
                current_idx = parent;
            } else {
                break;
            }
        }

        start_path.reverse();
        start_path.extend(goal_path.into_iter().skip(1));
        start_path
    }
}

impl Default for RRTConnect {
    /// Initializes RRTConnect with default params
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
