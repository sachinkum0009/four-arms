use rand::RngExt;

use crate::{
    errors::FourArmError,
    planner::{ConfigExt, JointState, Trajectory},
    robot::Pose,
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
    #[allow(dead_code)]
    fn plan(
        &self,
        _start_pos: &Pose,
        _goal_pos: &Pose,
    ) -> Result<Vec<crate::robot::Joint>, FourArmError> {
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
        let goal_bias = 0.10;
        let mut swapped = false;

        for _ in 0..self.max_iter {
            let (extend_tree, connect_tree) = if swapped {
                (&mut goal_tree, &mut start_tree)
            } else {
                (&mut start_tree, &mut goal_tree)
            };

            // --- SAMPLE & EXTEND ---
            let rand_config = if rng.random_bool(goal_bias) {
                goal_joints.to_vec()
            } else {
                self.joint_limits
                    .iter()
                    .map(|&(min, max)| rng.random_range(min..=max))
                    .collect::<Vec<f64>>()
            };

            let ext_nearest_idx = {
                let (idx, _) = extend_tree
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        a.config
                            .distance(&rand_config)
                            .partial_cmp(&b.config.distance(&rand_config))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .ok_or(FourArmError::EmptyTree("extend tree is empty".to_string()))?;
                idx
            };

            let ext_new = if let Some(cfg) = extend_tree[ext_nearest_idx]
                .config
                .step_towards(&rand_config, self.step_size)
            {
                if !cfg.is_in_collision() {
                    let idx = extend_tree.len();
                    extend_tree.push(RRTNode {
                        config: cfg.clone(),
                        parent_idx: Some(ext_nearest_idx),
                    });
                    Some((idx, cfg))
                } else {
                    None
                }
            } else {
                None
            };

            // --- CONNECT (repeatedly extend the other tree towards the new node) ---
            if let Some((_, ext_new_config)) = ext_new
                && self.connect_trees(connect_tree, &ext_new_config)
            {
                let path = self.reconstruct_path(&start_tree, &goal_tree, swapped);
                return Ok(path);
            }

            swapped = !swapped;
        }

        Err(FourArmError::TrajPlanError(
            "RRT-Connect failed to find a valid trajectory within max_iter".to_string(),
        ))
    }

    /// Repeatedly extend `tree` towards `target` until either the target
    /// is reached (returns `true`) or an obstacle blocks further progress
    /// (returns `false`).
    fn connect_trees(&self, tree: &mut Vec<RRTNode>, target: &[f64]) -> bool {
        loop {
            let nearest_idx = {
                let (idx, _) = tree
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        a.config
                            .distance(target)
                            .partial_cmp(&b.config.distance(target))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .expect("connect_trees: tree should not be empty");
                idx
            };

            match tree[nearest_idx]
                .config
                .step_towards(target, self.step_size)
            {
                Some(next) if !next.is_in_collision() => {
                    let dist_to_target = next.distance(target);
                    tree.push(RRTNode {
                        config: next,
                        parent_idx: Some(nearest_idx),
                    });
                    if dist_to_target <= self.step_size {
                        return true;
                    }
                }
                _ => return false,
            }
        }
    }

    fn reconstruct_path(
        &self,
        start_tree: &[RRTNode],
        goal_tree: &[RRTNode],
        swapped: bool,
    ) -> Trajectory {
        let (tree_a, tree_b, swap_order) = if swapped {
            (goal_tree, start_tree, true)
        } else {
            (start_tree, goal_tree, false)
        };

        let mut path_a = Vec::new();
        let mut current_idx = tree_a.len() - 1;
        while let Some(node) = tree_a.get(current_idx) {
            path_a.push(node.config.clone());
            match node.parent_idx {
                Some(p) => current_idx = p,
                None => break,
            }
        }

        let mut path_b = Vec::new();
        current_idx = tree_b.len() - 1;
        while let Some(node) = tree_b.get(current_idx) {
            path_b.push(node.config.clone());
            match node.parent_idx {
                Some(p) => current_idx = p,
                None => break,
            }
        }

        if swap_order {
            path_a.reverse();
            path_b.reverse();
            path_b.extend(path_a.into_iter().skip(1));
            path_b
        } else {
            path_a.reverse();
            path_a.extend(path_b.into_iter().skip(1));
            path_a
        }
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
