use crate::errors::FourArmError;
use crate::kdtree::{KDTree, Point};
use rand::RngExt;
use std::collections::VecDeque;
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
    pub nodes: Vec<Point>,
    joint_limits: Vec<(f64, f64)>,
}

impl PRM {
    pub fn new(
        max_iter: usize,
        step_size: f64,
        k_nearest: usize,
        kd_tree: Option<KDTree>,
        joint_limits: Vec<(f64, f64)>,
    ) -> Self {
        Self {
            max_iter,
            step_size,
            k_nearest,
            kd_tree,
            nodes: Vec::new(),
            joint_limits,
        }
    }
    pub fn default() -> Self {
        Self {
            max_iter: 1000,
            step_size: 0.1,
            k_nearest: 5,
            kd_tree: None,
            nodes: Vec::new(),
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
        self.nodes.clear();
        for _ in 0..self.max_iter {
            let config = self.sample_config();
            if self.is_collision_free(&config) {
                let point = Point::new(config);
                if let Some(tree) = self.kd_tree.as_mut() {
                    tree.insert(point.clone());
                }
                self.nodes.push(point);
            }
        }
    }

    /// Find the index of the nearest node in the roadmap to a given configuration
    fn find_nearest_idx(&self, config: &[f64]) -> Option<usize> {
        let query = Point::new(config.to_vec());
        let nearest = self.kd_tree.as_ref()?.nearest_neighbor(&query)?;
        self.nodes.iter().position(|n| *n == nearest)
    }

    /// Find k-nearest neighbor indices (brute force)
    fn k_nearest_neighbors(&self, config: &[f64], k: usize) -> Vec<usize> {
        let query = Point::new(config.to_vec());
        let mut indices: Vec<usize> = (0..self.nodes.len()).collect();
        indices.sort_by(|a, b| {
            let da = self.nodes[*a].distance_squared(&query);
            let db = self.nodes[*b].distance_squared(&query);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });
        indices.truncate(k);
        indices
    }

    /// Query the roadmap for a path between start and goal
    pub fn query(&self, start: &[f64], goal: &[f64]) -> Result<Vec<Vec<f64>>, FourArmError> {
        if !self.is_collision_free(start) || !self.is_collision_free(goal) {
            return Err(FourArmError::TrajPlanError(
                "Collision detected".to_string(),
            ));
        }

        if self.nodes.is_empty() {
            return Err(FourArmError::TrajPlanError(
                "Roadmap is empty, call build_roadmap first".to_string(),
            ));
        }

        let start_idx = self
            .find_nearest_idx(start)
            .ok_or(FourArmError::TrajPlanError(
                "Could not find start node".to_string(),
            ))?;
        let goal_idx = self
            .find_nearest_idx(goal)
            .ok_or(FourArmError::TrajPlanError(
                "Could not find goal node".to_string(),
            ))?;

        // BFS to find a path in the roadmap
        let num_nodes = self.nodes.len();
        let mut queue = VecDeque::new();
        let mut visited = vec![false; num_nodes];
        let mut parent = vec![None; num_nodes];

        queue.push_back(start_idx);
        visited[start_idx] = true;

        while let Some(current) = queue.pop_front() {
            if current == goal_idx {
                // Reconstruct path
                let mut path = Vec::new();
                let mut node = current;
                while let Some(p) = parent[node] {
                    path.push(self.nodes[node].coords.clone());
                    node = p;
                }
                path.push(self.nodes[start_idx].coords.clone());
                path.reverse();
                path.push(goal.to_vec());
                return Ok(path);
            }

            // Get neighbors (k-nearest)
            let neighbors = self.k_nearest_neighbors(&self.nodes[current].coords, self.k_nearest);

            for &neighbor_idx in &neighbors {
                if neighbor_idx == current {
                    continue;
                }
                if !visited[neighbor_idx] {
                    if self.is_path_collision_free(
                        &self.nodes[current].coords,
                        &self.nodes[neighbor_idx].coords,
                    ) {
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

    pub fn plan_traj(
        &mut self,
        start: &[f64],
        goal: &[f64],
    ) -> Result<Vec<Vec<f64>>, FourArmError> {
        self.build_roadmap();
        self.query(start, goal)
    }
}

// --- Planner Trait ---
// pub trait Planner {
//     fn plan_traj(&mut self, start: &[f64], goal: &[f64]) -> Result<Vec<Vec<f64>>, FourArmError>;
// }

// impl Planner for PRM {

// }
