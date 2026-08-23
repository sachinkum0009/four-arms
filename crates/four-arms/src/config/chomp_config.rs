pub struct CHOMPConfig {
    pub step_size: f64,
    pub max_iter: usize,
    pub smooth_weight: f64,
    pub obstacle_weight: f64,
    pub learning_rate: f64,
    pub n_waypoints: usize,
    pub regularization: f64,
}

impl Default for CHOMPConfig {
    fn default() -> Self {
        Self {
            step_size: 0.1,
            max_iter: 100,
            smooth_weight: 1.0,
            obstacle_weight: 1.0,
            learning_rate: 0.1,
            n_waypoints: 50,
            regularization: 1e-3,
        }
    }
}
