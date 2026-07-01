use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

type Node = u32;
type Weight = u32;

/// Graph
pub struct Graph {
    edges: HashMap<Node, Vec<(Node, Weight)>>,
}

impl Graph {
    pub fn new() -> Self {
        Self {
            edges: HashMap::new(),
        }
    }

    pub fn add_edge(&mut self, from: Node, to: Node, weight: Weight) {
        self.edges.entry(from).or_default().push((to, weight));
    }
}

/// Dijstra
pub struct Dijstra {}

impl Dijstra {
    /// initialize new instance
    pub fn new() -> Self {
        Self {}
    }

    pub fn plan(&self, graph: &Graph, start: Node, goal: Node) -> Option<Vec<Node>> {
        let mut pq = BinaryHeap::new();
        let mut distances = HashMap::new();

        let mut predecessors = HashMap::new();

        distances.insert(start, 0);
        pq.push(State {
            node: start,
            distance: 0,
        });

        while let Some(State { node, distance }) = pq.pop() {
            if node == goal {
                return Self::reconstruct_path(predecessors, start, goal);
            }

            if let Some(&known_distance) = distances.get(&node) {
                if distance > known_distance {
                    continue;
                }
            }

            // Explore neighbors
            if let Some(neighbors) = graph.edges.get(&node) {
                for &(neighbor, weight) in neighbors {
                    let new_distance = distance + weight;
                    let is_shorter = distances.get(&neighbor).map_or(true, |&d| new_distance < d);

                    if is_shorter {
                        distances.insert(neighbor, new_distance);
                        predecessors.insert(neighbor, node);
                        pq.push(State {
                            node: neighbor,
                            distance: new_distance,
                        });
                    }
                }
            }
        }
        None
    }

    /// reconstruct the path
    fn reconstruct_path(
        predecessors: HashMap<Node, Node>,
        start: Node,
        goal: Node,
    ) -> Option<Vec<Node>> {
        let mut path = Vec::new();
        let mut current = goal;
        while current != start {
            path.push(current);
            current = *predecessors.get(&current)?;
        }
        path.push(start);
        path.reverse();
        Some(path)
    }
}

// State for the priority queue
#[derive(Debug, PartialEq, Eq)]
struct State {
    node: Node,
    distance: Weight,
}

// Implement ordering for the priority queue (min-heap)
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.distance.cmp(&self.distance)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
