use std::cmp::Ordering;
use std::f64;

/// Point
#[derive(Debug, Clone)]
pub struct Point {
    pub coords: Vec<f64>,
}

impl Point {
    pub fn new(coords: Vec<f64>) -> Self {
        Self { coords }
    }
    fn dim(&self) -> usize {
        self.coords.len()
    }

    fn distance_squared(&self, other: &Point) -> f64 {
        self.coords
            .iter()
            .zip(other.coords.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum()
    }
}

/// KDNode
#[derive(Debug)]
pub struct KDNode {
    pub point: Point,
    pub left: Option<Box<KDNode>>,
    pub right: Option<Box<KDNode>>,
}

impl KDNode {
    pub fn new(point: Point) -> Self {
        Self {
            point,
            left: None,
            right: None,
        }
    }

    fn insert(&mut self, point: Point, depth: usize) {
        let axis = depth % point.dim();
        match point.coords[axis].partial_cmp(&self.point.coords[axis]) {
            Some(Ordering::Less) => {
                if let Some(ref mut left) = self.left {
                    left.insert(point, depth + 1);
                } else {
                    self.left = Some(Box::new(KDNode::new(point)));
                }
            }
            Some(Ordering::Greater) => {
                if let Some(ref mut right) = self.right {
                    right.insert(point, depth + 1);
                } else {
                    self.right = Some(Box::new(KDNode::new(point)));
                }
            }
            _ => {
                if let Some(ref mut left) = self.left {
                    left.insert(point, depth + 1);
                } else {
                    self.left = Some(Box::new(KDNode::new(point)));
                }
            }
        }
    }

    fn nearest_neighbor(
        &self,
        query: &Point,
        depth: usize,
        best: &mut Option<Point>,
        best_dist: &mut f64,
    ) {
        let dist = self.point.distance_squared(query);
        if best.is_none() || dist < *best_dist {
            *best = Some(self.point.clone());
            *best_dist = dist;
        }

        let axis = depth % query.dim();
        let diff = query.coords[axis] - self.point.coords[axis];

        let (first, second) = if diff <= 0.0 {
            (&self.left, &self.right)
        } else {
            (&self.right, &self.left)
        };

        if let Some(node) = first {
            node.nearest_neighbor(query, depth + 1, best, best_dist);
        }

        if diff.powi(2) < *best_dist {
            if let Some(node) = second {
                node.nearest_neighbor(query, depth + 1, best, best_dist);
            }
        }
    }
}

/// KDTree is a space partitioning data structure for organizing points in a k-dimensional space.
#[derive(Debug)]
pub struct KDTree {
    pub root: Option<KDNode>,
    k: usize,
}

impl KDTree {
    pub fn new(k: usize) -> Self {
        Self { root: None, k }
    }
    pub fn insert(&mut self, point: Point) {
        if point.dim() != self.k {
            panic!("Point dimension does not match tree dimension");
        }

        if let Some(ref mut root) = self.root {
            root.insert(point, 0);
        } else {
            self.root = Some(KDNode::new(point));
        }
    }

    pub fn nearest_neighbor(&self, query: &Point) -> Option<Point> {
        if query.dim() != self.k {
            panic!("Query point dimension does not match tree dimension");
        }
        let mut best: Option<Point> = None;
        let mut best_dist = f64::INFINITY;
        if let Some(ref root) = self.root {
            root.nearest_neighbor(query, 0, &mut best, &mut best_dist);
        }
        best
    }
    pub fn clear(&mut self) -> bool {
        if self.root.is_some() {
            self.root = None;
            true
        } else {
            false
        }
    }
    pub fn dim(self) -> usize {
        self.k
    }
}
