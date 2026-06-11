use four_arms::planner::dijstra::{Dijstra, Graph};

fn main() {
    let mut graph = Graph::new();
    graph.add_edge(0, 1, 4);
    graph.add_edge(0, 2, 1);
    graph.add_edge(1, 3, 1);
    graph.add_edge(2, 1, 2);
    graph.add_edge(2, 3, 5);
    graph.add_edge(3, 4, 3);

    let dijkstra = Dijstra::new();
    let path = dijkstra.plan(&graph, 0, 4);

    println!("Shortest path: {:?}", path);
}
