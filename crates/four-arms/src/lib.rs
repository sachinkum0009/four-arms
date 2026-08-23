/*!
# Four Arms

A rust crate to manipulate Robot Arm

1. Helps to easily calculate forward kinematics and inverse kinematics
2. Provides various planners to plan the trajectorys
3. Smoothers to smooth the trajectory before executing on the real robot

*/

pub mod chain;
/// Contains configuration for planners
pub mod config;
pub mod errors;
/// Contains the implementation of KDTree
pub mod kdtree;
pub mod planner;
/// Contains the struct for Link, Joint, UrdfRobot
pub mod robot;
/// Containers algorithm like Cubic Smoother Splines
pub mod smoother;
