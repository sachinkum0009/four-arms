struct RRT {}
impl RRT {
    fn new() -> Self {
        Self {}
    }
    fn do_something(&self) {
        println!("doing something");
    }
}

fn main() {
    let rrt = RRT::new();
    rrt.do_something();
}
