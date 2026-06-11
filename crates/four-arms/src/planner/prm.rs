use crate::kdtree::{KDNode, KDTree, Point};
use crate::{errors::FourArmError, planner::Planner};
use rand::{Rng, RngExt};
use std::collections::{HashMap, VecDeque};

/// Edge between two nodes in the roadmap
#[derive(Debug, Clone)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
}

/// Node in the PRM roadmap
#[derive(Debug, Clone)]
pub struct PRMNode {
    pub config: Vec<f64>,
    pub parent_idx: Option<usize>,
}

/// Probabilistic Roadmap (PRM) planner
pub struct PRM {
    pub max_iter: usize,
    pub step_size: f64,
    pub k_nearest: usize,
    pub kd_tree: Option<KDTree>,
    joint_limits: Vec<(f64, f64)>,
}

impl Default for PRM {
    fn default() -> Self {
        Self {
            max_iter: 1000,
            step_size: 0.1,
            k_nearest: 5,
            kd_tree: None,
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

impl PRM {
    pub fn preprocess(&mut self) {
        if self.kd_tree.is_none() {
            self.kd_tree = Some(KDTree::new(self.k_nearest));
        } else {
            if let Some(ref mut tree) = self.kd_tree {
                tree.clear();
                let mut rng = rand::rng();

                for _ in 0..self.max_iter {
                    let point = Point::new(
                        self.joint_limits
                            .iter()
                            .map(|&(min, max)| rng.random_range(min..=max))
                            .collect::<Vec<f64>>(),
                    );
                    tree.insert(point);
                }
            }
        }
    }

    fn build_roadmap(&self) {}

    /// Plan a trajectory from `start_joints` to `goal_joints` using PRM.
    /// Returns a vector of joint configurations representing the path.
    pub fn plan_traj(
        &mut self,
        start_joints: &[f64],
        goal_joints: &[f64],
    ) -> Result<Vec<Vec<f64>>, FourArmError> {
        self.kd_tree.as_mut().map(|tree| {
            let start_point = Point::new(start_joints.to_vec());
            let goal_point = Point::new(goal_joints.to_vec());
            tree.insert(start_point);
            tree.insert(goal_point);
        });

        Err(FourArmError::FunctionNotImplemented)
    }
    //         // Initialize roadmap with start and goal nodes
    //         let mut nodes = vec![
    //             PRMNode {
    //                 config: start_joints.to_vec(),
    //                 parent_idx: None,
    //             },
    //             PRMNode {
    //                 config: goal_joints.to_vec(),
    //                 parent_idx: None,
    //             },
    //         ];

    //         // Initialize KDTree for nearest-neighbor searches
    //         let mut kd_tree = KDTree::new(start_joints.len());
    //         kd_tree.insert(start_joints.to_vec());
    //         kd_tree.insert(goal_joints.to_vec());

    //         // Sample random configurations
    //         let mut rng = rand::thread_rng();
    //         for _ in 0..self.max_iter {
    //             let random_config: Vec<f64> = (0..start_joints.len())
    //                 .map(|_| rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI))
    //                 .collect();
    //             }

    //             // Add to nodes and KDTree
    //             let new_idx = nodes.len();
    //             nodes.push(PRMNode {
    //                 config: random_config.clone(),
    //                 parent_idx: None,
    //             });
    //             kd_tree.insert(random_config);

    //             // Connect to k-nearest neighbors
    //             let nearest_indices =
    //                 self.find_k_nearest(&kd_tree, &nodes[new_idx].config, self.k_nearest);
    //             for &neighbor_idx in &nearest_indices {
    //                 if self.is_path_collision_free(
    //                     &nodes[new_idx].config,
    //                     &nodes[neighbor_idx].config,
    //                     // Bidirectional edge (undirected graph)
    //                     // Note: In practice, you'd maintain an adjacency list
    //                 }
    //             }
    //         }

    //         // Build adjacency list for BFS
    //         let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
    //         for i in 0..nodes.len() {
    //             for j in i + 1..nodes.len() {
    //                 if self.is_path_collision_free(
    //                     &nodes[i].config,
    //                     &nodes[j].config,
    //                     adj.entry(i).or_default().push(j);
    //                     adj.entry(j).or_default().push(i);
    //                 }
    //             }
    //         }

    //         // Find path from start (index 0) to goal (index 1) using BFS
    //         self.find_path(&nodes, &adj)
    //     }

    //     /// Find the k-nearest neighbors of a point using the KDTree
    //     fn find_k_nearest(&self, kd_tree: &KDTree, point: &[f64], k: usize) -> Vec<usize> {
    //         // Note: This is a placeholder. In practice, you'd use KDTree::k_nearest_neighbors
    //         // For now, we'll use a brute-force approach (replace with KDTree method)
    //         let mut distances: Vec<(usize, f64)> = kd_tree
    //             .points()
    //             .iter()
    //             .enumerate()
    //             .map(|(i, p)| {
    //                 let distance = self.euclidean_distance(point, p);
    //                 (i, distance)
    //             })
    //             .collect();

    //         distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    //         distances.into_iter().take(k).map(|(i, _)| i).collect()
    //     }

    //     /// Euclidean distance between two configurations
    //     fn euclidean_distance(&self, a: &[f64], b: &[f64]) -> f64 {
    //         a.iter()
    //             .zip(b.iter())
    //             .map(|(&x, &y)| (x - y).powi(2))
    //             .sum::<f64>()
    //             .sqrt()
    //     }

    //     /// Check if the path between two configurations is collision-free
    //     // fn is_path_collision_free(
    //     //     &self,
    //     //     a: &[f64],
    //     //     b: &[f64],
    //     //     let steps = (self.euclidean_distance(a, b) / self.step_size).ceil() as usize;
    //     //     for i in 0..=steps {
    //     //         let ratio = i as f64 / steps as f64;
    //     //         let interpolated: Vec<f64> = a
    //     //             .iter()
    //     //             .zip(b.iter())
    //     //             .map(|(&x, &y)| x + ratio * (y - x))
    //     //             .collect();
    //     //         }
    //     //     }
    //     //     true
    //     // }

    //     /// Find a path from start to goal using BFS
    //     fn find_path(
    //         &self,
    //         nodes: &[PRMNode],
    //         adj: &HashMap<usize, Vec<usize>>,
    //     ) -> Result<Vec<Vec<f64>>, FourArmError> {
    //         let mut queue = VecDeque::new();
    //         let mut visited = vec![false; nodes.len()];
    //         let mut parent = vec![None; nodes.len()];

    //         queue.push_back(0);
    //         visited[0] = true;

    //         while let Some(current) = queue.pop_front() {
    //             if current == 1 {
    //                 // Reconstruct path
    //                 let mut path = Vec::new();
    //                 let mut idx = current;
    //                 while let Some(p) = parent[idx] {
    //                     path.push(nodes[idx].config.clone());
    //                     idx = p;
    //                 }
    //                 path.push(nodes[0].config.clone());
    //                 path.reverse();
    //                 return Ok(path);
    //             }

    //             if let Some(neighbors) = adj.get(&current) {
    //                 for &neighbor in neighbors {
    //                     if !visited[neighbor] {
    //                         visited[neighbor] = true;
    //                         parent[neighbor] = Some(current);
    //                         queue.push_back(neighbor);
    //                     }
    //                 }
    //             }
    //         }

    //         Err(FourArmError::PathNotFound)
    //     }
}

// // Assume the KDTree module is defined elsewhere in your crate
// mod kdtree {
//     #[derive(Debug)]
//     pub struct KDTree {
//         points: Vec<Vec<f64>>,
//         k: usize,
//     }

//     impl KDTree {
//         pub fn new(k: usize) -> Self {
//             KDTree {
//                 points: Vec::new(),
//                 k,
//             }
//         }

//         pub fn insert(&mut self, point: Vec<f64>) {
//             assert_eq!(
//                 point.len(),
//                 self.k,
//                 "Point dimension must match KDTree dimension"
//             );
//             self.points.push(point);
//         }

//         pub fn points(&self) -> &[Vec<f64>] {
//             &self.points
//         }

//         // Placeholder for k-nearest neighbors (implement this in your actual KDTree)
//         pub fn k_nearest_neighbors(&self, _point: &[f64], _k: usize) -> Vec<usize> {
//             vec![] // Replace with actual implementation
//         }
//     }
// }

// // Assume the errors and planner modules are defined elsewhere
// mod errors {
//     #[derive(Debug)]
//     pub enum FourArmError {
//         PathNotFound,
//         // Other error variants...
//     }
// }

// mod planner {
//     pub trait Planner {
//         type Error;
//         fn plan_traj(
//             &mut self,
//             start: &[f64],
//             goal: &[f64],
//     }
// }

// impl planner::Planner for PRM {
//     type Error = errors::FourArmError;

//     fn plan_traj(
//         &mut self,
//         start: &[f64],
//         goal: &[f64],
// }
