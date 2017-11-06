#![feature(test)]
// run with `$ rustup run nightly cargo bench`

extern crate gear;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate test;
extern crate urdf_rs;

use ncollide::shape::{Compound, Cuboid, ShapeHandle};
use k::InverseKinematicsSolver;
use test::Bencher;
use std::path::Path;

#[bench]
fn bench_ik_and_plan(b: &mut Bencher) {
    let input_path = Path::new("sample.urdf");
    let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(input_path).unwrap();

    let checker_for_planner =
        gear::CollisionChecker::<f64>::from_urdf_robot(&urdf_robot, 0.01);
    let robot_for_planner = k::urdf::create_tree::<f64>(&urdf_robot);

    let mut arms = k::create_kinematic_chains_with_dof_limit(&robot_for_planner, 7);
    let mut planner = gear::CollisionAvoidJointPathPlannerBuilder::new(
        arms.pop().expect("no arms found"),
        robot_for_planner,
        checker_for_planner,
    ).max_try(5000)
        .finalize();

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

    let solver = k::JacobianIKSolverBuilder::<f64>::new()
        .num_max_try(1000)
        .allowable_target_distance(0.01)
        .move_epsilon(0.00001)
        .jacobian_move_epsilon(0.001)
        .finalize();
    let solver = gear::RandomInitializeIKSolver::new(solver, 100);
    let initial = planner.get_joint_angles();
    b.iter(move || {
        let mut ik_target_pose = na::Isometry3::from_parts(
            na::Translation3::new(0.40, 0.20, 0.3),
            na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
        );        
        planner.set_joint_angles(&initial).unwrap();
        solver
            .solve(&mut planner.moving_arm, &ik_target_pose)
            .unwrap();
        let goal1 = planner.get_joint_angles();
        let plan1 = planner.plan(&initial, &goal1, &target_objects).unwrap();
        assert!(plan1.len() > 2);
        ik_target_pose.translation.vector[2] += 0.50;
        solver
            .solve(&mut planner.moving_arm, &ik_target_pose)
            .unwrap();
        let goal2 = planner.get_joint_angles();
        let plan2 = planner.plan(&goal1, &goal2, &target_objects).unwrap();
        assert!(plan2.len() > 2);
    });
}
