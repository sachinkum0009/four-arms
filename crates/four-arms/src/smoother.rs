use nalgebra::DMatrix;

use crate::{errors::FourArmError, planner::Trajectory};

/// Cubic smoothing splines
///
/// doc: <https://www.centerspace.net/smoothing-cubic-splines>
///
/// <https://docs.scipy.org/doc/scipy/tutorial/interpolate/smoothing_splines.html>
///
pub struct CubicSplineSmoother {}

impl Default for CubicSplineSmoother {
    fn default() -> Self {
        Self::new()
    }
}

impl CubicSplineSmoother {
    /// creates the Cubic Spline Smoother
    ///
    pub fn new() -> Self {
        Self {}
    }
    pub fn smooth_traj(
        &self,
        trajectory: Trajectory,
        step_size: f64,
        knob: f64,
    ) -> Result<Trajectory, FourArmError> {
        let n = trajectory.len();

        if n < 3 {
            // Can't smooth a trajectory with fewer than 3 waypoints using a cubic spline
            return Ok(trajectory);
        }
        let num_joints = trajectory[0].len();

        // Flatten the Vec<Vec<f64>> row-by-row into a flat vector for nalgebra
        let flat_data: Vec<f64> = trajectory.iter().flatten().cloned().collect();
        // Create matrix from row-major data layout
        let y_matrix = DMatrix::from_row_slice(n, num_joints, &flat_data);

        let mut q = DMatrix::<f64>::zeros(n, n - 2);

        for i in 0..(n - 2) {
            q[(i, i)] = 1.0 / step_size;
            q[(i + 1, i)] = -2.0 / step_size;
            q[(i + 2, i)] = 1.0 / step_size;
        }

        let mut r = DMatrix::<f64>::zeros(n - 2, n - 2);
        for i in 0..(n - 2) {
            r[(i, i)] = 4.0 * step_size / 6.0;
            if i < n - 3 {
                r[(i, i + 1)] = step_size / 6.0;
                r[(i + 1, i)] = step_size / 6.0;
            }
        }

        // Instead of inverting R, we solve (R \ Q^T)
        // nalgebra's LU decompisition is perfect for this
        let lu_r = r.lu();
        let inv_r_qt = lu_r
            .solve(&q.transpose())
            .ok_or(FourArmError::TrajSmoothError(
                "Failed to solve lu".to_string(),
            ))?;

        // K = Q * inv(R) * Q^T
        let k = &q * inv_r_qt;

        let i_mat = DMatrix::<f64>::identity(n, n);
        let a = i_mat + knob * &k;

        let lu_a = a.lu();
        let smoothed_matrix = lu_a.solve(&y_matrix).ok_or(FourArmError::TrajSmoothError(
            "Failed to smooth traj".to_string(),
        ))?;
        let mut smoothed_traj = Vec::with_capacity(n);
        for i in 0..n {
            // Extract each row as a standard vector
            let row_vec: Vec<f64> = smoothed_matrix.row(i).iter().cloned().collect();
            smoothed_traj.push(row_vec);
        }
        Ok(smoothed_traj)
    }
}

pub struct ShortcutSmoother {}

// pub trait Smoother {
//     fn smooth_traj(
//         &self,
//         trajectory: Trajectory,
//         step_size: f64,
//         knob: f64,
//     ) -> Result<Trajectory, FourArmError>;
// }
