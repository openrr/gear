/*
Copyright 2017 Takashi Ogura

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
//! # Motion Planning Library for Robotics
//!
//! Get the collision free trajectory of joint angles. `ncollide` is used to check the
//! collision between the robot and the environment.
//!
//! # Example
//!
//! ```
//! extern crate gear;
//! extern crate nalgebra as na;
//! extern crate ncollide;
//!
//! use ncollide::shape::{Compound3, Cuboid, ShapeHandle3};
//!
//! fn main() {
//!     // Create path planner with loading urdf file and set end link name
//!     let planner = gear::JointPathPlannerBuilder::try_from_urdf_file("sample.urdf", "l_wrist2")
//!         .unwrap()
//!         .collision_check_margin(0.01)
//!         .finalize();
//!     // Create inverse kinematics solver
//!     let solver = gear::JacobianIKSolverBuilder::<f64>::new()
//!         .num_max_try(1000)
//!         .allowable_target_distance(0.01)
//!         .move_epsilon(0.00001)
//!         .jacobian_move_epsilon(0.001)
//!         .finalize();
//!     let solver = gear::RandomInitializeIKSolver::new(solver, 100);
//!     // Create path planner with IK solver
//!     let mut planner = gear::JointPathPlannerWithIK::new(planner, solver);
//!
//!     // Create obstacles, or use `gear::create_compound_from_urdf("obstacles.urdf")`
//!     let obstacle_shape1 = ShapeHandle3::new(Cuboid::new(na::Vector3::new(0.20, 0.4, 0.1)));
//!     let obstacle_pose1 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.1), na::zero());
//!
//!     let obstacle_shape2 = ShapeHandle3::new(Cuboid::new(na::Vector3::new(0.20, 0.3, 0.1)));
//!     let obstacle_pose2 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.6), na::zero());
//!
//!     let obstacles = Compound3::new(vec![
//!         (obstacle_pose1, obstacle_shape1),
//!         (obstacle_pose2, obstacle_shape2),
//!     ]);
//!
//!     // Set IK target transformation
//!     let mut ik_target_pose = na::Isometry3::from_parts(
//!         na::Translation3::new(0.40, 0.20, 0.3),
//!         na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
//!     );
//!     // Plan the path, path is the vector of joint angles for root to "l_wrist2"
//!     let plan1 = planner.plan_with_ik(&ik_target_pose, &obstacles).unwrap();
//!     println!("plan1 = {:?}", plan1);
//!     ik_target_pose.translation.vector[2] += 0.50;
//!     // plan the path from previous result
//!     let plan2 = planner.plan_with_ik(&ik_target_pose, &obstacles).unwrap();
//!     println!("plan2 = {:?}", plan2);
//! }

#[cfg(feature = "assimp")]
extern crate assimp;

extern crate k;
#[macro_use]
extern crate log;
extern crate nalgebra as na;
extern crate ncollide;
extern crate rand;
extern crate rrt;
extern crate urdf_rs;

mod errors;
pub use errors::*;

mod collision_checker;
pub use collision_checker::*;

mod funcs;
pub use funcs::*;

mod ik;
pub use ik::*;

mod path_planner;
pub use path_planner::*;

mod ik_planner;
pub use ik_planner::*;

// re-export k::IK modules
pub use k::{InverseKinematicsSolver, JacobianIKSolver, JacobianIKSolverBuilder};
