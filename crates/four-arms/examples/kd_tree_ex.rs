use four_arms::kdtree::{KDNode, KDTree, Point};
use rand::RngExt;
use rerun::{
    LineStrips3D, RecordingStreamBuilder,
    components::{Color, Position3D, Radius},
    datatypes::Float32,
    external::re_log,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    re_log::setup_logging();
    let rec = RecordingStreamBuilder::new("kd_tree")
        .recording_id("run-2")
        .connect_grpc()?;

    let mut rng = rand::rng();

    let mut kd_tree = KDTree::new(3);
    for _ in 0..100 {
        let x = rng.random_range(0.0..100.0);
        let y = rng.random_range(0.0..100.0);
        let z = rng.random_range(0.0..100.0);
        let point = Point::new(vec![x, y, z]);
        kd_tree.insert(point);
    }

    let new_point = Point::new(vec![10.0, 5.0, 2.0]);
    let best_point = kd_tree.nearest_neighbor(&new_point);
    match best_point {
        Some(point) => println!("best point is {:?}", point.coords),
        None => println!("best point not found"),
    };

    // Log the KDTree structure
    if let Some(root) = &kd_tree.root {
        let mut node_id = 0;
        log_kdtree_node(&rec, root, None, 0, &mut node_id);
    }

    Ok(())
}

// Helper function to log a point as a 3D entity in Rerun
fn log_point(rec: &rerun::RecordingStream, point: &Point, label: &str, color: Color) {
    let position = Position3D::new(
        point.coords[0] as f32,
        point.coords[1] as f32,
        point.coords[2] as f32,
    );
    let radius = Radius(Float32::from(0.15));

    let _ = rec.log(
        format!("world/points/{}", label),
        &rerun::Points3D::new([position])
            .with_colors([color])
            .with_radii([radius]),
    );
}

// Recursively traverse the KDTree and log nodes/edges
#[allow(clippy::only_used_in_recursion)]
fn log_kdtree_node(
    rec: &rerun::RecordingStream,
    node: &KDNode,
    parent_pos: Option<[f32; 3]>,
    depth: usize,
    node_id: &mut usize,
) {
    let node_color = Color::from_rgb(255, 100, 100);
    let label = format!("N{}", *node_id);
    log_point(rec, &node.point, &label, node_color);

    let current_pos = [
        node.point.coords[0] as f32,
        node.point.coords[1] as f32,
        node.point.coords[2] as f32,
    ];

    // Log edge from parent to current node
    if let Some(parent) = parent_pos {
        let points = [parent, current_pos];
        rec.log(
            format!("world/edges/{}", label),
            &LineStrips3D::new([points]), // .with_colors([node_color, node_color].into_iter()),
        )
        .unwrap();
    }

    *node_id += 1;

    // Recursively log left and right children
    if let Some(left) = &node.left {
        log_kdtree_node(rec, left, Some(current_pos), depth + 1, node_id);
    }
    if let Some(right) = &node.right {
        log_kdtree_node(rec, right, Some(current_pos), depth + 1, node_id);
    }
}
