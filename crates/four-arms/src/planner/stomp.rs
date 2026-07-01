use rand::RngExt;

use crate::{errors::FourArmError, planner::Trajectory};
use nalgebra::{DMatrix, DVector};

/// # STOMP Planner
///
/// **Stochastic Trajectory Optimization for Motion Planning**
///
/// STOMP is a probabilistic gradient-free trajectory optimizer.
/// At each iteration it generates a set of noisy perturbations,
/// evaluates their cost, and updates the trajectory toward the
/// weighted average of low-cost perturbations.
///
/// ### Algorithm (per iteration)
/// 1. Generate `K` random noise vectors drawn from N(0, σ²I)
///    at each interior waypoint.
/// 2. Smooth each noise vector via the regularised smoothness
///    prior  (A + εI)⁻¹  so perturbations respect the acceleration
///    cost.
/// 3. For each perturbed trajectory  θ + ε̃ₖ  evaluate the total
///    cost  U = λ · F_smooth(θ+ε̃ₖ) + w_obs · F_obs(θ+ε̃ₖ).
/// 4. Compute softmax weights from the costs:
///       wₖ = exp(−(Cₖ − Cₘᵢₙ) / T) / Σ exp(−(Cₖ − Cₘᵢₙ) / T)
/// 5. Update  θ ← θ + Σₖ wₖ · ε̃ₖ.
/// 6. Clamp start/goal waypoints to the fixed endpoints.
///
/// Reference: Kalakrishnan et al., 2011.
pub struct STOMP {
    step_size: f64,
    max_iter: usize,
    noise_std: f64,
    temperature: f64,
    smooth_weight: f64,
    obstacle_weight: f64,
    n_waypoints: usize,
    regularization: f64,
    n_perturbations: usize,
}

impl Default for STOMP {
    fn default() -> Self {
        Self {
            step_size: 0.1,
            max_iter: 100,
            noise_std: 0.5,
            temperature: 0.1,
            smooth_weight: 1.0,
            obstacle_weight: 1.0,
            n_waypoints: 50,
            regularization: 1e-3,
            n_perturbations: 20,
        }
    }
}

impl STOMP {
    pub fn new(
        step_size: f64,
        max_iter: usize,
        noise_std: f64,
        temperature: f64,
        smooth_weight: f64,
        obstacle_weight: f64,
        n_waypoints: usize,
        regularization: f64,
        n_perturbations: usize,
    ) -> Self {
        Self {
            step_size,
            max_iter: max_iter.min(200),
            noise_std,
            temperature,
            smooth_weight,
            obstacle_weight,
            n_waypoints: n_waypoints.max(4),
            regularization,
            n_perturbations: n_perturbations.max(2),
        }
    }

    /// Plan a trajectory using the STOMP stochastic optimisation algorithm.
    pub fn plan_traj(
        &self,
        start_joints: &[f64],
        goal_joints: &[f64],
    ) -> Result<Trajectory, FourArmError> {
        let dof = start_joints.len();
        if dof == 0 || start_joints.len() != goal_joints.len() {
            return Err(FourArmError::JointMismatch(
                "start and goal joint vectors must be non-empty and match".to_string(),
                goal_joints.len(),
            ));
        }

        let n = self.n_waypoints;
        let k = self.n_perturbations;
        let mut traj = Self::init_trajectory(start_joints, goal_joints, n);

        // Smoothness prior A = KᵀK and its regularised LU factorisation
        let a = self.build_smoothness_matrix(n);
        let a_reg = &a + DMatrix::<f64>::identity(n, n) * self.regularization;
        let lu = a_reg.lu();

        // Orthonormal nullspace basis (constant & linear)
        let n1 = DVector::from_element(n, 1.0_f64).normalize();
        let n2_base = DVector::from_fn(n, |i, _| i as f64);
        let n2 = &n2_base - &n1 * (n2_base.dot(&n1));
        let n2 = n2.normalize();

        let mut rng = rand::rng();

        for _ in 0..self.max_iter {
            // ── 1. Generate K perturbations ──────────────────────
            let mut perturbations: Vec<Vec<DVector<f64>>> = (0..k)
                .map(|_| (0..dof).map(|_| DVector::zeros(n)).collect())
                .collect();

            for p in 0..k {
                for j in 0..dof {
                    let mut raw = DVector::zeros(n);
                    for i in 1..n - 1 {
                        raw[i] = gaussian_sample(&mut rng, self.noise_std);
                    }
                    // Project away nullspace so smoothing is well-behaved
                    raw -= &n1 * (raw.dot(&n1));
                    raw -= &n2 * (raw.dot(&n2));

                    let smoothed = lu.solve(&raw).ok_or_else(|| {
                        FourArmError::TrajPlanError(
                            "STOMP linear solve failed".to_string(),
                        )
                    })?;
                    perturbations[p][j] = smoothed;
                }
            }

            // ── 2. Evaluate cost of each perturbed trajectory ────
            let mut costs = vec![0.0_f64; k];
            for p in 0..k {
                let mut perturbed = traj.clone();
                for j in 0..dof {
                    for i in 0..n {
                        perturbed[i][j] += perturbations[p][j][i];
                    }
                }
                for j in 0..dof {
                    perturbed[0][j] = start_joints[j];
                    perturbed[n - 1][j] = goal_joints[j];
                }
                costs[p] = self.trajectory_cost(&perturbed, &a);
            }

            // ── 3. Softmax weights ───────────────────────────────
            let weights = self.softmax_weights(&costs);

            // ── 4. Weighted update ────────────────────────────────
            for j in 0..dof {
                let mut delta = DVector::zeros(n);
                for p in 0..k {
                    delta += &perturbations[p][j] * weights[p];
                }
                for i in 0..n {
                    traj[i][j] += delta[i];
                }
            }

            for j in 0..dof {
                traj[0][j] = start_joints[j];
                traj[n - 1][j] = goal_joints[j];
            }
        }

        Ok(traj)
    }

    // ── helpers ─────────────────────────────────────────────────

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

    fn trajectory_cost(&self, traj: &Trajectory, a: &DMatrix<f64>) -> f64 {
        let n = traj.len();
        let dof = traj[0].len();
        let mut cost = 0.0;

        // Smoothness:  ½ · λ · Σⱼ xⱼᵀ · A · xⱼ
        for j in 0..dof {
            let col: Vec<f64> = (0..n).map(|i| traj[i][j]).collect();
            let x_j = DVector::from_vec(col);
            cost += 0.5 * self.smooth_weight * x_j.dot(&(a * &x_j));
        }

        // Obstacle:  w_obs · Σᵢ,ⱼ c(qᵢ,ⱼ)
        for i in 0..n {
            for j in 0..dof {
                cost += self.obstacle_weight * obstacle_cost(traj[i][j]);
            }
        }

        cost
    }

    fn softmax_weights(&self, costs: &[f64]) -> Vec<f64> {
        let c_min = costs.iter().cloned().fold(f64::NAN, |a, b| a.min(b));
        let mut exps: Vec<f64> = costs
            .iter()
            .map(|c| (-(c - c_min) / self.temperature.max(1e-12)).exp())
            .collect();
        let sum: f64 = exps.iter().sum();
        if sum > 0.0 {
            for e in &mut exps {
                *e /= sum;
            }
        }
        exps
    }
}

fn obstacle_cost(q: f64) -> f64 {
    let sigma = 0.5;
    let two_pi = 2.0 * std::f64::consts::PI;
    let q_norm = q.rem_euclid(two_pi);
    let d = (q_norm).min(two_pi - q_norm);
    if d >= sigma {
        0.0
    } else {
        (-d * d / (sigma * sigma)).exp()
    }
}

fn gaussian_sample(rng: &mut impl rand::RngExt, sigma: f64) -> f64 {
    let u1: f64 = rng.random_range(0.0..1.0);
    let u2: f64 = rng.random_range(0.0..1.0);
    let z = (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos();
    z * sigma
}

const TAU: f64 = 6.283185307179586;
