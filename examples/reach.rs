extern crate env_logger;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate structopt;
extern crate urdf_rs;
extern crate urdf_viz;
extern crate rand;
extern crate ugok;

use glfw::{Action, WindowEvent, Key};
use std::path::Path;
use ncollide::shape::{Cuboid, Compound};
use k::JointContainer;


struct CollisionAvoidApp<'a> {
    robot: k::LinkTree<f32>,
    viewer: urdf_viz::Viewer<'a>,
    link_names: Vec<String>,
    target_objects: Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ik_target_pose: na::Isometry3<f64>,
    planner: ugok::CollisionAvoidJointPathPlanner,
}

impl<'a> CollisionAvoidApp<'a> {
    fn new(urdf_robot: &'a urdf_rs::Robot, base_dir: &Path) -> Self {
        let mut robot = k::urdf::create_tree::<f32>(urdf_robot);
        let mut viewer = urdf_viz::Viewer::new(urdf_robot);

        viewer.setup(base_dir, false);
        let base_transform = na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57),
        );
        robot.set_root_transform(base_transform);

        let checker_for_planner = ugok::CollisionChecker::<f64>::new(urdf_robot, base_dir, 0.03);
        let mut robot_for_planner = k::urdf::create_tree::<f64>(urdf_robot);
        robot_for_planner.set_root_transform(na::convert(base_transform));
        let planner =
            ugok::CollisionAvoidJointPathPlanner::new(robot_for_planner, checker_for_planner);

        viewer.add_axis_cylinders("origin", 1.0);
        if let Some(obj) = viewer.scenes.get_mut("origin") {
            obj.0.set_local_transformation(base_transform);
        }
        let mut link_names = robot
            .iter_for_joints_link()
            .map(|link| link.name.to_string())
            .collect::<Vec<String>>();
        link_names.push("root".to_owned());

        let target_shape = Cuboid::new(na::Vector3::new(0.20, 0.3, 0.1));
        let base64_pose: na::Isometry3<f64> = na::convert(base_transform);
        let target_pose = base64_pose *
            na::Isometry3::new(na::Vector3::new(0.6, 0.5, 0.0), na::zero());
        let mut cube = viewer.window.add_cube(
            target_shape.half_extents()[0] as f32 * 2.0,
            target_shape.half_extents()[1] as f32 * 2.0,
            target_shape.half_extents()[2] as f32 * 2.0,
        );
        cube.set_local_transformation(na::convert(target_pose));
        cube.set_color(0.5, 0.0, 0.5);

        let ik_target_pose = base64_pose *
            na::Isometry3::from_parts(
                na::Translation3::new(0.60, 0.40, 0.3),
                na::UnitQuaternion::from_euler_angles(0.0, -1.57, 0.0),
            );
        viewer.add_axis_cylinders("ik_target", 0.3);
        if let Some(obj) = viewer.scenes.get_mut("ik_target") {
            obj.0.set_local_transformation(na::convert(ik_target_pose));
        }

        CollisionAvoidApp {
            viewer: viewer,
            link_names: link_names,
            robot: robot,
            target_objects: ugok::wrap_compound(target_shape, target_pose),
            planner: planner,
            ik_target_pose: ik_target_pose,
        }
    }
    fn init(&mut self) {
        self.update_robot();
    }
    fn update_robot(&mut self) {
        self.viewer.update(&self.robot);
    }
    fn run(&mut self) {
        let mut plans: Vec<Vec<f64>> = Vec::new();
        let solver = k::JacobianIKSolverBuilder::<f32>::new()
            .num_max_try(1000)
            .allowable_target_distance(0.01)
            .move_epsilon(0.00001)
            .jacobian_move_epsilon(0.001)
            .finalize();
        let mut arms = k::create_kinematic_chains_with_dof_limit(&self.robot, 6);
        println!("{} {:?}", arms.len(), arms[0].get_joint_angles());
        while self.viewer.render() {
            if !plans.is_empty() {
                let vec: Vec<f32> = plans.pop().unwrap().into_iter().map(|x| x as f32).collect();
                self.robot.set_joint_angles(&vec).unwrap();
                self.update_robot();
            }

            for event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        match code {
                            Key::Up => {
                                self.ik_target_pose.translation.vector[1] += 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::Down => {
                                self.ik_target_pose.translation.vector[1] -= 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::Left => {
                                self.ik_target_pose.translation.vector[0] -= 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::Right => {
                                self.ik_target_pose.translation.vector[0] += 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::A => {
                                self.ik_target_pose.translation.vector[2] += 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::B => {
                                self.ik_target_pose.translation.vector[2] -= 0.05;
                                if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
                                    obj.0.set_local_transformation(
                                        na::convert(self.ik_target_pose),
                                    );
                                }
                            }
                            Key::I => {
                                let result = ugok::solve_ik_with_random_initialize(
                                    &solver,
                                    &mut arms[0],
                                    &na::convert(self.ik_target_pose),
                                    100,
                                );
                                if result.is_ok() {
                                    self.update_robot();
                                } else {
                                    println!("fail!!");
                                }
                            }

                            Key::P => {
                                let goal: Vec<f64> = self.robot
                                    .get_joint_angles()
                                    .into_iter()
                                    .map(|x| x as f64)
                                    .collect();
                                let mut initial = Vec::<f64>::new();
                                initial.resize(goal.len(), 0.0);
                                self.planner.set_joint_angles(&initial).unwrap();
                                let obj = &self.target_objects.shapes()[0];
                                let result = self.planner.plan(&goal, &*obj.1, &obj.0);
                                match result {
                                    Ok(mut plan) => {
                                        plan.reverse();
                                        for i in 0..(plan.len() - 1) {
                                            let mut interpolated_angles =
                                                ugok::interpolate(&plan[i], &plan[i + 1], 0.1);
                                            plans.append(&mut interpolated_angles);
                                        }
                                        //plans.push(plan.last().unwrap());
                                    }
                                    Err(err) => {
                                        println!("{:?}", err);
                                    }
                                }
                            }
                            Key::R => {
                                // ugok::set_random_joint_angles(&mut arms[0]).unwrap_or(());
                                ugok::set_random_joint_angles(&mut self.robot).unwrap_or(());
                                self.update_robot();
                                for name in &self.link_names {
                                    self.viewer.reset_temporal_color(name);
                                }
                            }
                            Key::C => {
                                let angles: Vec<f64> = self.robot
                                    .get_joint_angles()
                                    .into_iter()
                                    .map(|x| x as f64)
                                    .collect();
                                self.planner.set_joint_angles(&angles).unwrap();
                                for obj in self.target_objects.shapes() {
                                    let names =
                                        self.planner.get_colliding_link_names(&*obj.1, &obj.0);
                                    for name in &names {
                                        println!("{}", name);
                                        self.viewer.set_temporal_color(name, 0.8, 0.8, 0.6);
                                    }
                                }
                                println!("===========");
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn main() {
    env_logger::init().unwrap();
    let input_path = Path::new("sample.urdf");
    let base_dir = input_path.parent().unwrap();
    let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(input_path).unwrap();
    let mut app = CollisionAvoidApp::new(&urdf_robot, base_dir);
    app.init();
    app.run();
}
