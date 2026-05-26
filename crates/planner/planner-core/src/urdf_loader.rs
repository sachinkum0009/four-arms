use urdf_rs::{Joint, Link, Robot};

pub struct UrdfLoader {
    robot: Robot,
}

impl UrdfLoader {
    pub fn from_urdf(file_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let robot = urdf_rs::read_file(file_path)?;
        Ok(Self { robot })
    }
    /// Get links of URDF
    pub fn get_links(&self) -> &Vec<Link> {
        let links = &self.robot.links;
        links
    }
    /// Get joints of URDF
    pub fn get_joints(&self) -> &Vec<Joint> {
        let joints = &self.robot.joints;
        joints
    }
}
