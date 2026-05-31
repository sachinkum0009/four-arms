use crate::errors::FourArmError;
use crate::robot::{
    Collision, Inertia, Inertial, Joint, Limit, Link, Material, Pose, UrdfRobot, Visual,
    parse_geometry, parse_origin,
};
use nalgebra::{
    DMatrix, Isometry3, Quaternion, Translation, Translation3, UnitQuaternion, Vector3, Vector6,
};
use quick_xml::de::from_str;
use std::fs;

pub struct Chain {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    pub materials: Vec<Material>,
}

impl Chain {
    pub fn new(name: String) -> Self {
        Self {
            name,
            links: Vec::new(),
            joints: Vec::new(),
            materials: Vec::new(),
        }
    }

    pub fn get_links(&self) -> &Vec<Link> {
        &self.links
    }

    pub fn get_joints(&self) -> &Vec<Joint> {
        &self.joints
    }

    pub fn from_urdf(file_path: &str) -> Result<Self, FourArmError> {
        let urdf_content =
            fs::read_to_string(file_path).map_err(|e| FourArmError::ParseError(e.to_string()))?;
        let robot: UrdfRobot =
            from_str(&urdf_content).map_err(|e| FourArmError::ParseError(e.to_string()))?;

        let mut chain = Chain::new(robot.name.unwrap_or_default());

        // Parse materials
        for material in robot.materials {
            let color = material.color.map(|c| {
                let rgba: Vec<f64> = c
                    .rgba
                    .split_whitespace()
                    .map(|s| s.parse().unwrap_or(0.0))
                    .collect();
                [
                    rgba[0],
                    rgba[1],
                    rgba[2],
                    rgba.get(3).copied().unwrap_or(1.0),
                ]
            });
            chain.materials.push(Material {
                name: material.name,
                color,
            });
        }

        // Parse links
        for urdf_link in robot.links {
            let inertial = urdf_link.inertial.map(|i| Inertial {
                mass: i.mass.map_or(0.0, |m| m.value),
                origin: parse_origin(i.origin),
                inertia: Inertia {
                    ixx: i.inertia.as_ref().map_or(0.0, |i| i.ixx),
                    ixy: i.inertia.as_ref().map_or(0.0, |i| i.ixy),
                    ixz: i.inertia.as_ref().map_or(0.0, |i| i.ixz),
                    iyy: i.inertia.as_ref().map_or(0.0, |i| i.iyy),
                    iyz: i.inertia.as_ref().map_or(0.0, |i| i.iyz),
                    izz: i.inertia.as_ref().map_or(0.0, |i| i.izz),
                },
            });

            let visuals = urdf_link
                .visuals
                .into_iter()
                .map(|v| Visual {
                    origin: parse_origin(v.origin),
                    geometry: parse_geometry(v.geometry),
                    material: v.material.map(|m| m.name),
                })
                .collect();

            let collisions = urdf_link
                .collisions
                .into_iter()
                .map(|c| Collision {
                    origin: parse_origin(c.origin),
                    geometry: parse_geometry(c.geometry),
                })
                .collect();

            chain.links.push(Link {
                name: urdf_link.name,
                inertial,
                visuals,
                collisions,
            });
        }

        // Parse joints
        for urdf_joint in robot.joints {
            let origin = parse_origin(urdf_joint.origin);
            let axis = urdf_joint.axis.map(|a| {
                let xyz: Vec<f64> = a
                    .xyz
                    .split_whitespace()
                    .map(|s| s.parse().unwrap_or(0.0))
                    .collect();
                [xyz[0], xyz[1], xyz[2]]
            });

            let limit = urdf_joint.limit.map(|l| Limit {
                lower: l.lower,
                upper: l.upper,
                effort: l.effort,
                velocity: l.velocity,
            });

            chain.joints.push(Joint {
                name: urdf_joint.name,
                joint_type: urdf_joint.r#type,
                parent_link: urdf_joint.parent.link,
                child_link: urdf_joint.child.link,
                origin,
                axis,
                limit,
            });
        }

        Ok(chain)
    }

    /// Calculates the forward kinematics
    ///
    /// It uses linear transpose to calculate the final pose
    /// of end effector
    pub fn forward_kinematics(&self, joints: &[f64]) -> Result<Pose, FourArmError> {
        if joints.len() != self.get_joints().len() {
            return Err(FourArmError::JointMismatch(
                format!(
                    "Expected {} joints from URDF",
                    self.get_joint_limits().len()
                ),
                joints.len(),
            ));
        }

        let mut transform = Isometry3::<f64>::identity();

        for (i, urdf_joint) in self.joints.iter().enumerate() {
            let translation = Translation3::new(
                urdf_joint.origin.position[0],
                urdf_joint.origin.position[1],
                urdf_joint.origin.position[2],
            );
            let q = Quaternion::new(
                urdf_joint.origin.rotation[3],
                urdf_joint.origin.rotation[0],
                urdf_joint.origin.rotation[1],
                urdf_joint.origin.rotation[2],
            );
            let rotation = UnitQuaternion::from_quaternion(q);
            let static_transform = Isometry3::from_parts(translation, rotation);

            let urdf_axis = urdf_joint.axis.unwrap();
            let axis = Vector3::new(urdf_axis[0], urdf_axis[1], urdf_axis[2]);

            let joint_axis = nalgebra::Unit::new_normalize(axis);
            let joint_rotation = UnitQuaternion::from_axis_angle(&joint_axis, joints[i]);

            let dynamic_transform = Isometry3::from_parts(Translation::identity(), joint_rotation);
            transform = transform * static_transform * dynamic_transform;
        }

        Ok(self.matrix_pose(transform))
    }

    /// Calculates joint angles from the pose
    ///
    /// Using Jacobian (Iterative) Method
    pub fn inverse_kinematics(
        &self,
        target_pose: &Pose,
        joints: &[f64],
        max_iterations: usize,
        tolerance: f64,
        damping: f64,
    ) -> Result<Vec<f64>, FourArmError> {
        let mut joint_angles = joints.to_vec();
        for _ in 0..max_iterations {
            let current_pose = self.forward_kinematics(&joint_angles)?;
            let error = self.compute_pose_error(&current_pose, target_pose);
            if error.norm() < tolerance {
                return Ok(joint_angles);
            }

            let jacobian = self.compute_jacobian(&joint_angles)?;

            let n = joint_angles.len();
            let lambda_sq = damping * damping;

            let jt = jacobian.transpose(); // N x 6
            let jjt = &jacobian * &jt; // 6 x 6
            let damped = jjt + DMatrix::identity(6, 6) * lambda_sq; // 6 x 6

            // Solve the 6x6 linear system: (J*J^T + λ²I) * x = error
            let decomp = damped.lu();
            let x = decomp
                .solve(&error)
                .ok_or_else(|| "LU decomposition failed — matrix may be singular".to_string())
                .map_err(FourArmError::IkError)?;

            // Joint update: delta_q = J^T * x
            let delta_q = &jt * x;
            // Apply update
            for i in 0..n {
                joint_angles[i] += delta_q[i];
            }

            // Clamp joints to their limits if defined
            let limits = self.get_joint_limits();
            for (i, limit) in limits.iter().enumerate() {
                if let (Some(lower), Some(upper)) = (limit.lower, limit.upper) {
                    joint_angles[i] = joint_angles[i].clamp(lower, upper);
                }
            }
        }

        Err(FourArmError::IkError(format!(
            "Inverse kinematics did not converge within {} iterations",
            max_iterations
        )))
    }

    /// Returns the Vector containing limits
    fn get_joint_limits(&self) -> Vec<Limit> {
        self.joints
            .iter()
            .filter_map(|joint| {
                joint.limit.as_ref().map(|limit| Limit {
                    lower: limit.lower,
                    upper: limit.upper,
                    effort: limit.effort,
                    velocity: limit.velocity,
                })
            })
            .collect()
    }

    fn compute_jacobian(&self, joint_angles: &[f64]) -> Result<DMatrix<f64>, FourArmError> {
        let joint_size = joint_angles.len();

        // Get end-effector position once — used as the lever arm target for all columns
        let end_effector_pose = self.forward_kinematics(joint_angles)?;
        let p_e = Vector3::new(
            end_effector_pose.position[0],
            end_effector_pose.position[1],
            end_effector_pose.position[2],
        );

        let mut jacobian = DMatrix::<f64>::zeros(6, joint_size);

        // Rebuild the transform chain incrementally — mirrors forward_kinematics exactly,
        // but we sample the intermediate frame at each joint i before applying joint i's rotation.
        let mut transform = Isometry3::<f64>::identity();

        for i in 0..joint_size {
            let urdf_joint = &self.joints[i];

            // --- Static transform (link offset from URDF origin) ---
            let translation = Translation3::new(
                urdf_joint.origin.position[0],
                urdf_joint.origin.position[1],
                urdf_joint.origin.position[2],
            );
            let q = Quaternion::new(
                urdf_joint.origin.rotation[3],
                urdf_joint.origin.rotation[0],
                urdf_joint.origin.rotation[1],
                urdf_joint.origin.rotation[2],
            );
            let rotation = UnitQuaternion::new_normalize(q);
            let static_transform = Isometry3::from_parts(translation, rotation);

            // Apply the static part to reach joint i's origin in world frame
            transform *= static_transform;

            // --- Sample joint i frame AFTER static transform, BEFORE joint rotation ---
            // p_i: origin of joint i in world frame
            let p_i = transform.translation.vector;

            // z_i: rotation axis of joint i expressed in world frame
            // Joint axis from URDF (local frame), default to Z if missing
            let local_axis = urdf_joint
                .axis
                .map(|a| Vector3::new(a[0], a[1], a[2]))
                .unwrap_or_else(Vector3::z);

            // Rotate the local axis into world frame using the accumulated rotation
            let z_i = transform.rotation * local_axis;

            // --- Geometric Jacobian column i ---
            // Linear part:  z_i × (p_e - p_i)   (revolute joint)
            // Angular part: z_i
            let r = p_e - p_i;
            let linear = z_i.cross(&r);

            jacobian[(0, i)] = linear[0];
            jacobian[(1, i)] = linear[1];
            jacobian[(2, i)] = linear[2];
            jacobian[(3, i)] = z_i[0];
            jacobian[(4, i)] = z_i[1];
            jacobian[(5, i)] = z_i[2];

            // --- Apply dynamic (joint angle) rotation to continue the chain ---
            let joint_axis = nalgebra::Unit::new_normalize(local_axis);
            let joint_rotation = UnitQuaternion::from_axis_angle(&joint_axis, joint_angles[i]);
            let dynamic_transform = Isometry3::from_parts(Translation3::identity(), joint_rotation);
            transform *= dynamic_transform;
        }

        Ok(jacobian)
    }

    fn compute_pose_error(&self, current_pose: &Pose, target_pose: &Pose) -> Vector6<f64> {
        let linear_error = Vector3::new(
            target_pose.position[0] - current_pose.position[0],
            target_pose.position[1] - current_pose.position[1],
            target_pose.position[2] - current_pose.position[2],
        );

        let current_q = Quaternion::new(
            current_pose.rotation[3],
            current_pose.rotation[0],
            current_pose.rotation[1],
            current_pose.rotation[2],
        );
        let target_q = Quaternion::new(
            target_pose.rotation[3],
            target_pose.rotation[0],
            target_pose.rotation[1],
            target_pose.rotation[2],
        );

        // Angular error (orientation difference as axis-angle)
        let current_quat = UnitQuaternion::new_normalize(current_q);
        let target_quat = UnitQuaternion::new_normalize(target_q);

        // Compute the relative rotation from current to target
        let mut relative_quat = target_quat * current_quat.inverse();

        // by flipping the quaternion if its scalar part is negative
        if relative_quat.w < 0.0 {
            relative_quat = UnitQuaternion::new_normalize(-relative_quat.into_inner());
        }

        // Extract axis-angle from the relative quaternion
        let angular_error = if relative_quat.angle() > 1e-6 {
            // Fix 3: call .into_inner() to unwrap Unit<Vector3> → Vector3
            let axis = relative_quat
                .axis()
                .map(|a| a.into_inner())
                .unwrap_or(Vector3::zeros());
            axis * relative_quat.angle()
        } else {
            Vector3::zeros()
        };

        // Combine linear and angular errors into a 6D vector
        Vector6::new(
            linear_error[0],
            linear_error[1],
            linear_error[2],
            angular_error[0],
            angular_error[1],
            angular_error[2],
        )
    }

    fn matrix_pose(&self, transform: Isometry3<f64>) -> Pose {
        let translation = transform.translation;
        let rotation = transform.rotation;
        Pose::from_parts(translation, rotation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{Joint, Limit, Link, Pose};

    // Helper function to build a mock 2-DOF chain for mathematical verification
    fn create_mock_chain() -> Chain {
        let mut chain = Chain::new("test_robot".to_string());

        // Create 2 links
        chain.links.push(Link {
            name: "link1".to_string(),
            inertial: None,
            visuals: vec![],
            collisions: vec![],
        });
        chain.links.push(Link {
            name: "link2".to_string(),
            inertial: None,
            visuals: vec![],
            collisions: vec![],
        });

        // Joint 1 sits at world origin
        chain.joints.push(Joint {
            name: "joint1".to_string(),
            joint_type: "revolute".to_string(),
            parent_link: "base_link".to_string(),
            child_link: "link1".to_string(),
            origin: Pose {
                position: [0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion [x, y, z, w]
            },
            axis: Some([0.0, 0.0, 1.0]), // Z-axis rotation
            limit: Some(Limit {
                lower: Some(-std::f64::consts::PI),
                upper: Some(std::f64::consts::PI),
                effort: Some(10.0),
                velocity: Some(1.0),
            }),
        });

        // Joint 2 is offset by 1.0 meter along the X axis
        chain.joints.push(Joint {
            name: "joint2".to_string(),
            joint_type: "revolute".to_string(),
            parent_link: "link1".to_string(),
            child_link: "link2".to_string(),
            origin: Pose {
                position: [1.0, 0.0, 0.0], // 1 meter offset
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            axis: Some([0.0, 0.0, 1.0]), // Z-axis rotation
            limit: Some(Limit {
                lower: Some(-std::f64::consts::PI),
                upper: Some(std::f64::consts::PI),
                effort: Some(10.0),
                velocity: Some(1.0),
            }),
        });

        chain
    }

    #[test]
    fn test_joint_mismatch() {
        let chain = create_mock_chain();

        // The mock chain expects 2 joint inputs, we pass 3
        let wrong_joints = vec![0.0, 0.1, 0.2];
        let result = chain.forward_kinematics(&wrong_joints);

        assert!(result.is_err());
        match result.unwrap_err() {
            FourArmError::JointMismatch(msg, received) => {
                assert_eq!(received, 3);
                assert!(msg.contains("Expected 2 joints"));
            }
            _ => panic!("Expected a JointMismatch variant error!"),
        }
    }

    #[test]
    fn test_forward_kinematics_straight() {
        let chain = create_mock_chain();

        // Joints at 0.0 radians should extend the arm straight along the X-axis
        let joint_angles = vec![0.0, 0.0];
        let pose = chain.forward_kinematics(&joint_angles).unwrap();

        // Position should be at X = 1.0 (from joint2's relative origin)
        assert!((pose.position[0] - 1.0).abs() < 1e-6);
        assert!(pose.position[1].abs() < 1e-6);
        assert!(pose.position[2].abs() < 1e-6);
    }

    #[test]
    fn test_forward_kinematics_rotation() {
        let chain = create_mock_chain();

        // Rotate first joint 90 degrees (PI / 2)
        let joint_angles = vec![std::f64::consts::FRAC_PI_2, 0.0];
        let pose = chain.forward_kinematics(&joint_angles).unwrap();

        // The 1.0m link should now point straight up along the Y axis
        assert!(
            pose.position[0].abs() < 1e-6,
            "Expected X near 0, got {}",
            pose.position[0]
        );
        assert!(
            (pose.position[1] - 1.0).abs() < 1e-6,
            "Expected Y near 1, got {}",
            pose.position[1]
        );
    }

    #[test]
    fn test_inverse_kinematics_convergence() {
        let chain = create_mock_chain();

        // 1. Pick a realistic target pose reachable by this arm (e.g., X=0, Y=1)
        let target_pose = Pose {
            position: [0.0, 1.0, 0.0],
            // Facing 90 degrees rotated around Z axis
            rotation: [0.0, 0.0, 0.7071067811865475, 0.7071067811865476],
        };

        // 2. Supply a seed/initial guess near the solution to avoid local minima
        let initial_guess = vec![0.5, 0.0];

        // 3. Run IK solver
        let result = chain.inverse_kinematics(&target_pose, &initial_guess, 50, 1e-3, 0.1);

        assert!(
            result.is_ok(),
            "IK solver failed to converge: {:?}",
            result.err()
        );
        let solved_joints = result.unwrap();

        // 4. Run FK on the result to confirm it reaches the target destination
        let final_pose = chain.forward_kinematics(&solved_joints).unwrap();

        let distance_error = ((final_pose.position[0] - target_pose.position[0]).powi(2)
            + (final_pose.position[1] - target_pose.position[1]).powi(2))
        .sqrt();

        assert!(
            distance_error < 1e-2,
            "Target position missed! Final position was: {:?}",
            final_pose.position
        );
    }
}
