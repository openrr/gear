# gear [![Build Status](https://travis-ci.org/OTL/gear.svg?branch=master)](https://travis-ci.org/OTL/gear) [![crates.io](https://img.shields.io/crates/v/gear.svg)](https://crates.io/crates/gear)

Collision Avoidance Path Planning for robotics in Rust-lang

## Code example

```rust
extern crate gear;
extern crate nalgebra as na;
extern crate ncollide;

use ncollide::shape::{Compound3, Cuboid, ShapeHandle3};

fn main() {
    // Create path planner with loading urdf file and set end link name
    let planner = gear::get_joint_path_planner_builder_from_urdf("sample.urdf", "l_wrist2")
        .unwrap()
        .collision_check_margin(0.01)
        .finalize();
    // Create inverse kinematics solver
    let solver = gear::JacobianIKSolverBuilder::<f64>::new()
        .num_max_try(1000)
        .allowable_target_distance(0.01)
        .move_epsilon(0.00001)
        .jacobian_move_epsilon(0.001)
        .finalize();
    let solver = gear::RandomInitializeIKSolver::new(solver, 100);
    // Create path planner with IK solver
    let mut planner = gear::JointPathPlannerWithIK::new(planner, solver);

    // Create obstacles
    let obstacle_shape1 = ShapeHandle3::new(Cuboid::new(na::Vector3::new(0.20, 0.4, 0.1)));
    let obstacle_pose1 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.1), na::zero());

    let obstacle_shape2 = ShapeHandle3::new(Cuboid::new(na::Vector3::new(0.20, 0.3, 0.1)));
    let obstacle_pose2 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.6), na::zero());

    let target_objects = Compound3::new(vec![
        (obstacle_pose1, obstacle_shape1),
        (obstacle_pose2, obstacle_shape2),
    ]);

    // Set IK target transformation
    let mut ik_target_pose = na::Isometry3::from_parts(
        na::Translation3::new(0.40, 0.20, 0.3),
        na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
    );
    // Plan the path
    let plan1 = planner
        .plan_with_ik(&ik_target_pose, &target_objects)
        .unwrap();
    println!("plan1 = {:?}", plan1);
    ik_target_pose.translation.vector[2] += 0.50;
    // move again
    let plan2 = planner
        .plan_with_ik(&ik_target_pose, &target_objects)
        .unwrap();
    println!("plan2 = {:?}", plan2);
}
```

## Run example with GUI

```bash
$ cargo run --release --example reach
```

then,

* Up/Down/Left/Right/`f`/`b` to move IK target
* type `g` to move the end of the arm to the target

* type `i` to just solve inverse kinematics for the target
* type `r` to set random pose
* type `c` to check collision


The example can handle any urdf files (sample.urdf is used as default)

```
$ cargo run --release --example reach YOUR_URDF_FILE_PATH
```

[Video](https://www.youtube.com/watch?v=fYfZR1f2HW0)
