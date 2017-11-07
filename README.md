# gear [![Build Status](https://travis-ci.org/OTL/gear.svg?branch=master)](https://travis-ci.org/OTL/gear) [![crates.io](https://img.shields.io/crates/v/gear.svg)](https://crates.io/crates/gear)

Collision Avoidance Path Planning for robotics in Rust-lang

## Code example

```rust
extern crate gear;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate urdf_rs;

use ncollide::shape::{Compound, Cuboid, ShapeHandle};
use k::InverseKinematicsSolver;
use test::Bencher;
use std::path::Path;

// Load urdf file
let input_path = Path::new("sample.urdf");
let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(input_path).unwrap();

// Create collision checker
let checker_for_planner =
    gear::CollisionChecker::<f64>::from_urdf_robot(&urdf_robot, 0.01);
// Create kinematic robot model
let robot_for_planner = k::urdf::create_tree::<f64>(&urdf_robot);

// Create arm object from kinematic robot model for inverse kinematics
let mut arms = k::create_kinematic_chains_with_dof_limit(&robot_for_planner, 7);

// Create motion planner
let mut planner = gear::CollisionAvoidJointPathPlannerBuilder::new(
    arms.pop().expect("no arms found"),
    robot_for_planner,
    checker_for_planner,
).max_try(5000)
    .finalize();

// Create obstacles
let obstacle_shape1 = Cuboid::new(na::Vector3::new(0.20, 0.4, 0.1));
let obstacle_pose1 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.1), na::zero());

let obstacle_shape2 = Cuboid::new(na::Vector3::new(0.20, 0.3, 0.1));
let obstacle_pose2 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.6), na::zero());

let mut shapes = Vec::new();
let handle1 = ShapeHandle::new(obstacle_shape1);
shapes.push((obstacle_pose1, handle1));
let handle2 = ShapeHandle::new(obstacle_shape2);
shapes.push((obstacle_pose2, handle2));
let target_objects = Compound::new(shapes);

// Create IK solver
let solver = k::JacobianIKSolverBuilder::<f64>::new()
    .num_max_try(1000)
    .allowable_target_distance(0.01)
    .move_epsilon(0.00001)
    .jacobian_move_epsilon(0.001)
    .finalize();
let solver = gear::RandomInitializeIKSolver::new(solver, 100);

// Store initial joint angles for planning
let initial = planner.get_joint_angles();

// Set IK target transformation
let mut ik_target_pose = na::Isometry3::from_parts(
    na::Translation3::new(0.40, 0.20, 0.3),
    na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
);    

// Solve IK, this will update kinematic robot model
solver
    .solve(&mut planner.moving_arm, &ik_target_pose)
    .unwrap();
// Store the joint angles as a goal
let goal1 = planner.get_joint_angles();

// Plan the path
let plan1 = planner.plan(&initial, &goal1, &target_objects).unwrap();
println!("{:?}", plan1);
```

## Run example with GUI

```bash
$ cargo run --release --example reach
```

then,

* Up/Down/Left/Right/`f`/`b` to move IK target
* type `g` to move the end of the arm to the target
* type `m` to use current pose as init pose
* type `i` to reach the target
* type `p` to plan

* type `r` to set random pose
* type `c` to check collision


The example can handle any urdf files (sample.urdf is used as default)

```
$ cargo run --release --example reach YOUR_URDF_FILE_PATH
```

[Video](https://www.youtube.com/watch?v=fYfZR1f2HW0)
