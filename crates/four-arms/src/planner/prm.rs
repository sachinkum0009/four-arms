use crate::errors::FourArmError;
use crate::kdtree::{KDNode, KDTree, Point};
use rand::{Rng, RngExt};
use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;

// --- PRM Structures ---
#[derive(Debug, Clone)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
}

#[derive(Debug, Clone)]
pub struct PRMNode {
    pub config: Vec<f64>,
    pub parent_idx: Option<usize>,
}

#[derive(Debug, Default)]
pub struct PRM {
    pub max_iter: usize,
    pub step_size: f64,
    pub k_nearest: usize,
    pub kd_tree: Option<KDTree>,
    joint_limits: Vec<(f64, f64)>,
}

impl PRM {
    pub fn new() -> Self {
        Self {
            max_iter: 1000,
            step_size: 0.1,
            k_nearest: 5,
            kd_tree: None,
            joint_limits: vec![
                (-PI, PI), // Joint 1
                (-PI, PI), // Joint 2
                (-PI, PI), // Joint 3
                (-PI, PI), // Joint 4
                (-PI, PI), // Joint 5
                (-PI, PI), // Joint 6
            ],
        }
    }

    /// Sample a random configuration within joint limits
    fn sample_config(&self) -> Vec<f64> {
        let mut rng = rand::rng();
        self.joint_limits
            .iter()
            .map(|&(min, max)| rng.random_range(min..=max))
            .collect()
    }

    /// Check if a configuration is collision-free (placeholder)
    fn is_collision_free(&self, _config: &[f64]) -> bool {
        // Replace with your actual collision checking logic
        true
    }

    /// Check if the path between two configurations is collision-free
    fn is_path_collision_free(&self, from: &[f64], to: &[f64]) -> bool {
        let steps = 10;
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let interpolated: Vec<f64> = from
                .iter()
                .zip(to.iter())
                .map(|(&a, &b)| a + t * (b - a))
                .collect();
            if !self.is_collision_free(&interpolated) {
                return false;
            }
        }
        true
    }

    /// Build the roadmap by sampling and connecting nodes
    pub fn build_roadmap(&mut self) {
        if self.kd_tree.is_none() {
            self.kd_tree = Some(KDTree::new(self.k_nearest));
        }

        if let Some(tree) = self.kd_tree.as_mut() {
            tree.clear();
        }

        // Sample nodes
        for _ in 0..self.max_iter {
            let config = self.sample_config();
            if self.is_collision_free(&config) {
                if let Some(tree) = self.kd_tree.as_mut() {
                    tree.insert(Point::new(config));
                }
            }
        }
    }

    /// Find the nearest node in the roadmap to a given configuration
    fn find_nearest(&self, config: &[f64]) -> Option<Point> {
        self.kd_tree
            .as_ref()
            .and_then(|tree| tree.nearest_neighbor(&Point::new(config.to_vec())))
            .map(|point| point)
    }

    /// Query the roadmap for a path between start and goal
    pub fn query(&self, start: &[f64], goal: &[f64]) -> Result<Vec<Vec<f64>>, FourArmError> {
        if !self.is_collision_free(start) || !self.is_collision_free(goal) {
            return Err(FourArmError::TrajPlanError("Collsion detected".to_string()));
        }

        let tree = self
            .kd_tree
            .as_ref()
            .ok_or(FourArmError::TrajPlanError("not path".to_string()))?;
        let start_idx = self
            .find_nearest(start)
            .ok_or(FourArmError::TrajPlanError("not path".to_string()))?;
        let goal_idx = self
            .find_nearest(goal)
            .ok_or(FourArmError::TrajPlanError("not path".to_string()))?;

        // BFS to find a path in the roadmap
        let mut queue = VecDeque::new();
        let mut visited = vec![false; self.max_iter];
        let mut parent = vec![None; self.max_iter];

        queue.push_back(start_idx);
        visited[start_idx] = true;

        while let Some(current) = queue.pop_front() {
            if current == goal_idx {
                // Reconstruct path
                let mut path = Vec::new();
                let mut node = current;
                while let Some(p) = parent[node] {
                    path.push(tree.get_point(node).unwrap().coords.clone());
                    node = p;
                }
                path.push(tree.get_point(start_idx).unwrap().coords.clone());
                path.reverse();
                path.push(goal.to_vec());
                return Ok(path);
            }

            // Get neighbors (k-nearest)
            let neighbors = tree(
                &Point::new(tree.get_point(current).unwrap().coords.clone()),
                self.k_nearest,
            );

            for (neighbor_idx, _) in neighbors {
                if !visited[neighbor_idx] {
                    let from = tree.get_point(current).unwrap().coords.clone();
                    let to = tree.get_point(neighbor_idx).unwrap().coords.clone();
                    if self.is_path_collision_free(&from, &to) {
                        visited[neighbor_idx] = true;
                        parent[neighbor_idx] = Some(current);
                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        Err(FourArmError::TrajPlanError(
            "Failed to plan trajectory".to_string(),
        ))
    }
}

// --- Planner Trait ---
pub trait Planner {
    fn plan(&mut self, start: &[f64], goal: &[f64]) -> Result<Vec<Vec<f64>>, FourArmError>;
}

impl Planner for PRM {
    fn plan(&mut self, start: &[f64], goal: &[f64]) -> Result<Vec<Vec<f64>>, FourArmError> {
        self.build_roadmap();
        // self.query(start, goal)
        Err(FourArmError::FunctionNotImplemented)
    }
}
