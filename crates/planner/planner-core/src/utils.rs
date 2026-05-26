use nalgebra::{Translation3, UnitQuaternion};

#[derive(Debug, Clone, PartialEq)]
pub struct Joints<const N: usize> {
    joints: [f64; N],
}

impl<const N: usize> Joints<N> {
    pub fn zeros() -> Self {
        Self { joints: [0.0; N] }
    }
    pub fn new(joints: [f64; N]) -> Self {
        Self { joints }
    }
    pub fn get_joints(&self) -> [f64; N] {
        self.joints
    }
    pub fn increment_joints(&mut self, value: f64) -> [f64; N] {
        for i in 0..N {
            self.joints[i] += value
        }
        self.joints
    }
}

#[derive(Default, Copy, Clone, PartialEq, Debug)]
pub struct Position {
    x: f64,
    y: f64,
    z: f64,
}

impl Position {
    pub fn get_x(&self) -> f64 {
        self.x
    }
    pub fn get_y(&self) -> f64 {
        self.y
    }
    pub fn get_z(&self) -> f64 {
        self.z
    }
    pub fn get_position(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
    pub fn from_translation(translation: Translation3<f64>) -> Self {
        Position {
            x: translation.x,
            y: translation.y,
            z: translation.z,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Orientation {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}
impl Orientation {
    pub fn get_x(&self) -> f64 {
        self.x
    }
    pub fn get_y(&self) -> f64 {
        self.y
    }
    pub fn get_z(&self) -> f64 {
        self.z
    }
    pub fn get_w(&self) -> f64 {
        self.w
    }
    /// Get euler angles from the orientation
    pub fn get_euler(&self) -> (f64, f64, f64) {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        // Clamp to avoid NaN errors if floating-point drift goes slightly outside [-1.0, 1.0]
        let sinp = sinp.clamp(-1.0, 1.0);
        let pitch = sinp.asin();

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        // Returns angles in radians as (roll, pitch, yaw)
        (roll, pitch, yaw)
    }
    pub fn from_rotation(rotation: UnitQuaternion<f64>) -> Self {
        Self {
            x: rotation.i,
            y: rotation.j,
            z: rotation.k,
            w: rotation.w,
        }
    }
}

impl Default for Orientation {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}
#[derive(Default, Debug)]
pub struct Pose {
    position: Position,
    orientation: Orientation,
}

impl Pose {
    pub fn get_position(&self) -> Position {
        self.position
    }
    pub fn get_orientation(&self) -> Orientation {
        self.orientation
    }
    pub fn from_parts(translation: Translation3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        let position = Position::from_translation(translation);
        let orientation = Orientation::from_rotation(rotation);
        Self {
            position,
            orientation,
        }
    }
}
