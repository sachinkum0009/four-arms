use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use planner_core::urdf_loader::UrdfLoader;
use planner_core::utils::{Joints, Pose};

pub struct FkSolver<const N: usize> {
    urdf: UrdfLoader,
}

impl<const N: usize> FkSolver<N> {
    pub fn from_urdf(urdf_file_path: String) -> Result<Self, String> {
        let urdf_loader = UrdfLoader::from_urdf(&urdf_file_path)
            .map_err(|e| format!("Failed to load URDF: {:?}", e))?;
        Ok(Self { urdf: urdf_loader })
    }

    /// calculates the pose based on the joint angles
    pub fn calculate_pose(&self, joint: Joints<N>) -> Result<Pose, String> {
        let joint_angles = joint.get_joints();
        let joints = self.urdf.get_joints();

        let mut transform = Isometry3::<f64>::identity();

        for (i, urdf_joint) in joints.iter().enumerate() {
            // Static transform (origin)
            let translation = Translation3::new(
                urdf_joint.origin.xyz.0[0],
                urdf_joint.origin.xyz.0[1],
                urdf_joint.origin.xyz.0[2],
            );
            let rotation = UnitQuaternion::from_euler_angles(
                urdf_joint.origin.rpy.0[0],
                urdf_joint.origin.rpy.0[1],
                urdf_joint.origin.rpy.0[2],
            );
            let static_transform = Isometry3::from_parts(translation.into(), rotation.into());

            // Dynamic transform (joint rotation)
            let axis = Vector3::new(
                urdf_joint.axis.xyz.0[0],
                urdf_joint.axis.xyz.0[1],
                urdf_joint.axis.xyz.0[2],
            );
            let joint_axis = nalgebra::Unit::new_normalize(axis);
            let joint_rotation = UnitQuaternion::from_axis_angle(&joint_axis, joint_angles[i]);
            let dynamic_transform =
                Isometry3::from_parts(Translation3::identity().into(), joint_rotation.into());
            transform = transform * static_transform * dynamic_transform;
        }

        // Convert the final 4x4 matrix to Pose
        Ok(self.matrix_to_pose(transform))
    }

    /// Helper to convert a 4x4 transformation matrix to Pose
    fn matrix_to_pose(&self, transform: Isometry3<f64>) -> Pose {
        // Extract translation
        let translation = transform.translation;

        // Extract rotation (upper 3x3 submatrix)
        let rotation = transform.rotation;
        let pose = Pose::from_parts(translation, rotation);
        pose
    }
}
