use nalgebra::{Translation3, UnitQuaternion};
use serde::Deserialize;

// ========== URDF XML Structs (Deserialization) ==========
#[derive(Debug, Deserialize)]
#[serde(rename = "robot")]
pub struct UrdfRobot {
    #[serde(rename = "@name")]
    pub name: Option<String>,
    #[serde(rename = "link", default)]
    pub links: Vec<UrdfLink>,
    #[serde(rename = "joint", default)]
    pub joints: Vec<UrdfJoint>,
    #[serde(rename = "material", default)]
    pub materials: Vec<UrdfMaterial>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfLink {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "inertial", default)]
    pub inertial: Option<UrdfInertial>,
    #[serde(rename = "visual", default)]
    pub visuals: Vec<UrdfVisual>,
    #[serde(rename = "collision", default)]
    pub collisions: Vec<UrdfCollision>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfInertial {
    #[serde(rename = "mass", default)]
    pub mass: Option<UrdfMass>,
    #[serde(rename = "origin", default)]
    pub origin: Option<UrdfOrigin>,
    #[serde(rename = "inertia", default)]
    pub inertia: Option<UrdfInertia>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfMass {
    #[serde(rename = "@value")]
    pub value: f64,
}

#[derive(Debug, Deserialize)]
pub struct UrdfInertia {
    #[serde(rename = "@ixx")]
    pub ixx: f64,
    #[serde(rename = "@ixy")]
    pub ixy: f64,
    #[serde(rename = "@ixz")]
    pub ixz: f64,
    #[serde(rename = "@iyy")]
    pub iyy: f64,
    #[serde(rename = "@iyz")]
    pub iyz: f64,
    #[serde(rename = "@izz")]
    pub izz: f64,
}

#[derive(Debug, Deserialize)]
pub struct UrdfVisual {
    #[serde(rename = "origin", default)]
    pub origin: Option<UrdfOrigin>,
    #[serde(rename = "geometry", default)]
    pub geometry: Option<UrdfGeometry>,
    #[serde(rename = "material", default)]
    pub material: Option<UrdfMaterialRef>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfCollision {
    #[serde(rename = "origin", default)]
    pub origin: Option<UrdfOrigin>,
    #[serde(rename = "geometry", default)]
    pub geometry: Option<UrdfGeometry>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfGeometry {
    #[serde(rename = "box", default)]
    pub box_: Option<UrdfBox>,
    #[serde(rename = "cylinder", default)]
    pub cylinder: Option<UrdfCylinder>,
    #[serde(rename = "sphere", default)]
    pub sphere: Option<UrdfSphere>,
    #[serde(rename = "mesh", default)]
    pub mesh: Option<UrdfMesh>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfBox {
    #[serde(rename = "@size")]
    pub size: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfCylinder {
    #[serde(rename = "@radius")]
    pub radius: f64,
    #[serde(rename = "@length")]
    pub length: f64,
}

#[derive(Debug, Deserialize)]
pub struct UrdfSphere {
    #[serde(rename = "@radius")]
    pub radius: f64,
}

#[derive(Debug, Deserialize)]
pub struct UrdfMesh {
    #[serde(rename = "@filename")]
    pub filename: String,
    #[serde(rename = "@scale", default)]
    pub scale: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfMaterial {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "color", default)]
    pub color: Option<UrdfColor>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfColor {
    #[serde(rename = "@rgba")]
    pub rgba: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfMaterialRef {
    #[serde(rename = "@name")]
    pub name: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfJoint {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "@type")]
    pub r#type: String,
    #[serde(rename = "parent")]
    pub parent: UrdfLinkRef,
    #[serde(rename = "child")]
    pub child: UrdfLinkRef,
    #[serde(rename = "origin", default)]
    pub origin: Option<UrdfOrigin>,
    #[serde(rename = "axis", default)]
    pub axis: Option<UrdfAxis>,
    #[serde(rename = "limit", default)]
    pub limit: Option<UrdfLimit>,
}

#[derive(Debug, Deserialize)]
pub struct UrdfLinkRef {
    #[serde(rename = "@link")]
    pub link: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfOrigin {
    #[serde(rename = "@xyz", default)]
    pub xyz: String,
    #[serde(rename = "@rpy", default)]
    pub rpy: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfAxis {
    #[serde(rename = "@xyz")]
    pub xyz: String,
}

#[derive(Debug, Deserialize)]
pub struct UrdfLimit {
    #[serde(rename = "@lower")]
    pub lower: Option<f64>,
    #[serde(rename = "@upper")]
    pub upper: Option<f64>,
    #[serde(rename = "@effort")]
    pub effort: Option<f64>,
    #[serde(rename = "@velocity")]
    pub velocity: Option<f64>,
}

// ========== Rust Structs (Domain Model) ==========
#[derive(Debug)]
pub struct Link {
    pub name: String,
    pub inertial: Option<Inertial>,
    pub visuals: Vec<Visual>,
    pub collisions: Vec<Collision>,
}

#[derive(Debug)]
pub struct Inertial {
    pub mass: f64,
    pub origin: Pose,
    pub inertia: Inertia,
}

#[derive(Debug)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug)]
pub struct Visual {
    pub origin: Pose,
    pub geometry: Geometry,
    pub material: Option<String>,
}

#[derive(Debug)]
pub struct Collision {
    pub origin: Pose,
    pub geometry: Geometry,
}

#[derive(Debug)]
pub enum Geometry {
    Box {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Sphere {
        radius: f64,
    },
    Mesh {
        filename: String,
        scale: Option<[f64; 3]>,
    },
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub joint_type: String,
    pub parent_link: String,
    pub child_link: String,
    pub origin: Pose,
    pub axis: Option<[f64; 3]>,
    pub limit: Option<Limit>,
}

#[derive(Debug)]
pub struct Limit {
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub effort: Option<f64>,
    pub velocity: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub position: [f64; 3],
    pub rotation: [f64; 4], // Quaternion (x, y, z, w)
}

impl Pose {
    pub fn from_parts(translation: Translation3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        let position = [translation.x, translation.y, translation.z];
        let rotation = [rotation.i, rotation.j, rotation.k, rotation.w];
        Self { position, rotation }
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Debug)]
pub struct Material {
    pub name: String,
    pub color: Option<[f64; 4]>, // RGBA
}

// ========== Helper Functions ==========
pub fn parse_origin(origin: Option<UrdfOrigin>) -> Pose {
    if let Some(o) = origin {
        let xyz: Vec<f64> = o
            .xyz
            .split_whitespace()
            .map(|s| s.parse().unwrap_or(0.0))
            .collect();
        let rpy: Vec<f64> = o
            .rpy
            .split_whitespace()
            .map(|s| s.parse().unwrap_or(0.0))
            .collect();

        // Convert RPY to quaternion (simplified placeholder)
        let position = [
            xyz.first().copied().unwrap_or(0.0),
            xyz.get(1).copied().unwrap_or(0.0),
            xyz.get(2).copied().unwrap_or(0.0),
        ];
        let rotation = [
            rpy.first().copied().unwrap_or(0.0),
            rpy.get(1).copied().unwrap_or(0.0),
            rpy.get(2).copied().unwrap_or(0.0),
            1.0, // Placeholder for quaternion w
        ];
        Pose { position, rotation }
    } else {
        Pose {
            position: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

pub fn parse_geometry(geometry: Option<UrdfGeometry>) -> Geometry {
    if let Some(g) = geometry {
        if let Some(box_) = g.box_ {
            let size: Vec<f64> = box_
                .size
                .split_whitespace()
                .map(|s| s.parse().unwrap_or(0.0))
                .collect();
            Geometry::Box {
                size: [
                    size.first().copied().unwrap_or(0.0),
                    size.get(1).copied().unwrap_or(0.0),
                    size.get(2).copied().unwrap_or(0.0),
                ],
            }
        } else if let Some(cylinder) = g.cylinder {
            Geometry::Cylinder {
                radius: cylinder.radius,
                length: cylinder.length,
            }
        } else if let Some(sphere) = g.sphere {
            Geometry::Sphere {
                radius: sphere.radius,
            }
        } else if let Some(mesh) = g.mesh {
            let scale = mesh.scale.map(|s| {
                let scale_vec: Vec<f64> = s
                    .split_whitespace()
                    .map(|s| s.parse().unwrap_or(1.0))
                    .collect();
                [
                    scale_vec.first().copied().unwrap_or(1.0),
                    scale_vec.get(1).copied().unwrap_or(1.0),
                    scale_vec.get(2).copied().unwrap_or(1.0),
                ]
            });
            Geometry::Mesh {
                filename: mesh.filename,
                scale,
            }
        } else {
            Geometry::Box {
                size: [0.0, 0.0, 0.0],
            } // Default
        }
    } else {
        Geometry::Box {
            size: [0.0, 0.0, 0.0],
        } // Default
    }
}
