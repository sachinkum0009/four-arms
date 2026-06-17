use crate::{
    errors::FourArmError,
    planner::{Planner, Trajectory},
    robot::{Joint, Pose},
};
use nalgebra::{DMatrix, DVector};

/// # CHOMP Planner
///
/// Covariant Hamiltonian Optimization for Motion Planning.
///
/// A gradient-based trajectory optimizer that iteratively refines an
/// initial straight-line trajectory by minimising a cost functional
/// with smoothness and obstacle-avoidance terms.
///
/// ### Cost function
///   U(ξ) = λ · F_smooth(ξ) + w_obs · F_obs(ξ)
///
/// where ξ is the flattened trajectory, F_smooth penalises acceleration
/// (via second-order finite differences) and F_obs penalises proximity
/// to obstacles.
///
/// ### Update rule
/// For each joint dimension j, solve the regularised linear system
///   (A + ε·I) · Δξ_j = −η · ( λ · A · ξ_j + w_obs · ∇F_obs(ξ_j) )
///
/// then apply  ξ_j ← ξ_j + Δξ_j  and re-clamp the first/last waypoints.
///
/// A = KᵀK (the smoothness prior), η the learning rate, λ the
/// smoothness weight, w_obs the obstacle weight, and ε a small
/// regularisation constant that makes the system positive-definite.
///
/// ### Algorithm steps
/// 1. Initialise a straight-line trajectory from start to goal
/// 2. Build the smoothness matrix A = KᵀK
/// 3. For each iteration:
///    a. Compute smoothness gradient: λ · A · ξ
///    b. Compute obstacle gradient: w_obs · ∇F_obs(ξ)
///    c. Solve (A + ε·I) · Δξ = −η · grad
///    d. Apply Δξ and re-clamp start/goal waypoints
/// 4. Return optimised trajectory
pub struct CHOMP {
    step_size: f64,
    max_iter: usize,
    smooth_weight: f64,
    obstacle_weight: f64,
    learning_rate: f64,
    n_waypoints: usize,
    regularization: f64,
}

// impl Planner for CHOMP {

//     fn plan(&self, _start_pos: &Pose, _goal_pos: &Pose) -> Result<Vec<Joint>, FourArmError> {
//         Err(FourArmError::TrajPlanError(
//             "CHOMP requires joint-space waypoints; use plan_traj instead".to_string(),
//         ))
//     }
// }

impl Default for CHOMP {
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

impl CHOMP {
    pub fn new(
        step_size: f64,
        max_iter: usize,
        smooth_weight: f64,
        obstacle_weight: f64,
        learning_rate: f64,
        n_waypoints: usize,
        regularization: f64,
    ) -> Self {
        Self {
            step_size,
            max_iter: max_iter.min(200),
            smooth_weight,
            obstacle_weight,
            learning_rate,
            n_waypoints,
            regularization,
        }
    }
    /// Configure the smoothness cost weight (λ).
    pub fn smooth_weight(mut self, weight: f64) {
        self.smooth_weight = weight;
    }

    /// Configure the obstacle cost weight (w_obs).
    pub fn obstacle_weight(mut self, weight: f64) {
        self.obstacle_weight = weight;
    }

    /// Configure the learning rate (η).
    pub fn learning_rate(mut self, eta: f64) {
        self.learning_rate = eta;
    }

    /// Configure the number of waypoints in the trajectory.
    pub fn n_waypoints(mut self, n: usize) {
        self.n_waypoints = n.max(4);
    }

    /// Plan a smooth, collision-free trajectory using the CHOMP algorithm.
    ///
    /// # Arguments
    /// * `start_joints` – Starting joint configuration
    /// * `goal_joints`  – Goal joint configuration
    ///
    /// # Returns
    /// An optimised `Trajectory` (list of waypoints, each a `Vec<f64>` of joint angles).
    pub fn plan_traj(
        &self,
        start_joints: &[f64],
        goal_joints: &[f64],
    ) -> Result<Trajectory, FourArmError> {
        let dof = start_joints.len();
        if dof == 0 || start_joints.len() != goal_joints.len() {
            return Err(FourArmError::JointMismatch(
                "start and goal joint vectors must be non-empty and match in length".to_string(),
                goal_joints.len(),
            ));
        }

        let n = self.n_waypoints;
        let mut traj = Self::init_trajectory(start_joints, goal_joints, n);

        // N×N smoothness prior A = KᵀK  (singular — rank N−2).
        // Its nullspace is span(constant, linear) — second-differences of
        // those sequences are zero.  We project the gradient orthogonal to
        // this nullspace before solving to prevent 1/ε blow-up.
        let a = self.build_smoothness_matrix(n);

        // Orthonormal basis for the nullspace of A  (N-vectors)
        let n1 = DVector::from_element(n, 1.0_f64).normalize();
        let n2_base = DVector::from_fn(n, |i, _| i as f64);
        let n2 = &n2_base - &n1 * (n2_base.dot(&n1));
        let n2 = n2.normalize();

        // Regularised system matrix:  A_reg = A + ε·I  (positive-definite)
        let a_reg = &a + DMatrix::<f64>::identity(n, n) * self.regularization;
        let lu = a_reg.lu();

        for _ in 0..self.max_iter {
            for j in 0..dof {
                // N-vector for joint j across all waypoints
                let mut x_j = DVector::zeros(n);
                for i in 0..n {
                    x_j[i] = traj[i][j];
                }

                // Smoothness gradient:  λ · A · x_j
                let mut total = &a * &x_j * self.smooth_weight;

                // Obstacle gradient at each waypoint for this joint
                for i in 0..n {
                    total[i] += self.obstacle_gradient_value(traj[i][j]) * self.obstacle_weight;
                }

                // Project total orthogonal to the nullspace of A
                total -= &n1 * (total.dot(&n1));
                total -= &n2 * (total.dot(&n2));

                // Solve  (A + ε·I) · Δξ = −η · projected_gradient
                let delta = lu.solve(&(total * -self.learning_rate)).ok_or_else(|| {
                    FourArmError::TrajPlanError(
                        "CHOMP linear solve failed during iteration".to_string(),
                    )
                })?;

                for i in 0..n {
                    traj[i][j] += delta[i];
                }
            }

            // Re-clamp first and last waypoints to the fixed endpoints
            for j in 0..dof {
                traj[0][j] = start_joints[j];
                traj[n - 1][j] = goal_joints[j];
            }
        }

        Ok(traj)
    }

    // ── internal helpers ──────────────────────────────────────────────

    /// Straight-line interpolation between start and goal with N waypoints.
    fn init_trajectory(start: &[f64], goal: &[f64], n: usize) -> Trajectory {
        let mut traj = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / (n - 1).max(1) as f64;
            let wp: Vec<f64> = start
                .iter()
                .zip(goal.iter())
                .map(|(s, g)| s + (g - s) * t)
                .collect();
            traj.push(wp);
        }
        traj
    }

    /// Build the N×N smoothness matrix A = KᵀK where K is the
    /// (N−2)×N second-order finite-difference matrix scaled by 1/h².
    fn build_smoothness_matrix(&self, n: usize) -> DMatrix<f64> {
        let h = self.step_size;
        let h2 = h * h;
        let mut k = DMatrix::<f64>::zeros(n - 2, n);
        for i in 0..n - 2 {
            k[(i, i)] = 1.0 / h2;
            k[(i, i + 1)] = -2.0 / h2;
            k[(i, i + 2)] = 1.0 / h2;
        }
        k.transpose() * k
    }

    /// Scalar obstacle-gradient value for a single joint angle.
    ///
    /// Repels the joint from its nearest limit boundary (modelled as 0
    /// and 2π).  In a production system this would be replaced with a
    /// collision-checker query.
    fn obstacle_gradient_value(&self, q: f64) -> f64 {
        let sigma = 0.5;
        let two_pi = 2.0 * std::f64::consts::PI;
        let q_norm = q.rem_euclid(two_pi);
        let d_lo = q_norm;
        let d_hi = two_pi - q_norm;
        let d = d_lo.min(d_hi);

        if d >= sigma {
            return 0.0;
        }

        let s = (-d * d / (sigma * sigma)).exp();
        let sign = if d_lo < d_hi { 1.0 } else { -1.0 };
        -2.0 * d * s / (sigma * sigma) * sign
    }
}
