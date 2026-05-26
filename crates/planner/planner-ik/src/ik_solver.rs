use crate::IkSolverMethod;
use planner_core::utils::{Joints, Pose};

pub struct IkSolver<const N: usize> {
    max_iterations: usize,
    position_tolerance: f64,
    orientation_tolerance: f64,
}

impl<const N: usize> IkSolver<N> {
    pub fn new(max_iterations: usize, position_tolerance: f64, orientation_tolerance: f64) -> Self {
        Self {
            max_iterations,
            position_tolerance,
            orientation_tolerance,
        }
    }

    pub fn default() -> Self {
        Self {
            max_iterations: 100,
            position_tolerance: 0.25,
            orientation_tolerance: 0.3,
        }
    }

    /// Compute a trajectory of joint configuration to transition froma correct pose to a target pose using linear interpolation
    ///
    /// This method calculates the necessary intermediary joint positions required to safety
    /// navigate the robot or kinematic chain between two given pose.
    pub fn compute_traj(
        &self,
        current_joints: Joints<N>,
        target_pose: Pose,
        num_steps: usize,
    ) -> Result<Vec<[f64; N]>, String> {
        let desired_joints = self.calculate_joint_angles(target_pose)?;
        //Joints::new([1.5, 3.3, 2.3, 3.6, 0.3, 5.0]);
        let mut trajectory = Vec::with_capacity(num_steps);

        let current_joint_angles = current_joints.get_joints();
        let desired_joint_angles = desired_joints.get_joints();

        println!("current joints: {:?}", current_joint_angles);
        println!("desired joints: {:?}", desired_joint_angles);

        for step in 1..=num_steps {
            let mut joint_angles = [0.0; N];
            let t = step as f64 / num_steps as f64;

            for i in 0..N {
                joint_angles[i] = current_joint_angles[i]
                    + t * (desired_joint_angles[i] - current_joint_angles[i]);
            }
            trajectory.push(joint_angles);
        }

        Ok(trajectory)
    }

    pub fn calculate_joint_angles(&self, pose: Pose) -> Result<Joints<N>, String> {
        let joints = Joints::<N>::zeros(); // Joints::new([1.5, 3.3, 2.3, 3.6, 0.3, 5.0]);
        Ok(joints)
        // Err("Failed to calculate joint angles".to_string())
    }
}

pub trait IkSolve<const N: usize> {
    fn compute_traj(
        &self,
        current_joints: &Joints<N>,
        target_pose: &Pose,
        method: IkSolverMethod,
    ) -> Result<Vec<Joints<N>>, String>;
}
