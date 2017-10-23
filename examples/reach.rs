extern crate env_logger;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate rand;
extern crate structopt;
extern crate ugok;
extern crate urdf_rs;
extern crate urdf_viz;

use glfw::{Action, Key, WindowEvent};
use std::path::Path;
use ncollide::shape::{Compound, Cuboid};
use k::JointContainer;


struct CollisionAvoidApp<'a> {
    robot: k::LinkTree<f32>,
    viewer: urdf_viz::Viewer<'a>,
    target_objects: Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ik_target_pose: na::Isometry3<f64>,
    robot_for_planner: k::LinkTree<f64>,
    planner: ugok::CollisionAvoidJointPathPlanner<k::RefKinematicChain<f64>>,
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
        let mut arms = k::create_kinematic_chains_with_dof_limit(&robot_for_planner, 6);
        let planner =
            ugok::CollisionAvoidJointPathPlanner::new(arms.pop().expect("no arm"), checker_for_planner);

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
            robot: robot,
            robot_for_planner: robot_for_planner,
            target_objects: ugok::wrap_compound(target_shape, target_pose),
            planner: planner,
            ik_target_pose: ik_target_pose,
        }
    }
    fn init(&mut self) {
        self.update_robot();
    }
    fn update_robot(&mut self) {
        let current: Vec<f32> = self.robot_for_planner
            .get_joint_angles()
            .into_iter()
            .map(|x| x as f32)
            .collect();
        self.robot.set_joint_angles(&current).unwrap();
        self.viewer.update(&self.robot);
    }
    fn update_ik_target(&mut self) {
        if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
            obj.0.set_local_transformation(
                na::convert(self.ik_target_pose),
            );
        }
    }
    fn set_current_robot_to_planner(&mut self) -> Result<(), k::JointError> {
        let current: Vec<f64> = self.robot
            .get_joint_angles()
            .into_iter()
            .map(|x| x as f64)
            .collect();
        self.robot_for_planner.set_joint_angles(&current)
    }
    fn run(&mut self) {
        let mut plans: Vec<Vec<f64>> = Vec::new();
        let solver = k::JacobianIKSolverBuilder::<f64>::new()
            .num_max_try(1000)
            .allowable_target_distance(0.01)
            .move_epsilon(0.00001)
            .jacobian_move_epsilon(0.001)
            .finalize();
        let mut initial = Vec::<f64>::new();
        initial.resize(self.planner.robot.get_joint_angles().len(), 0.0);
        //println!("len = {}", self.robot.get_joint_angles().len());
        self.planner.set_joint_angles(&initial).unwrap();

        while self.viewer.render() {
            if !plans.is_empty() {
                //let vec: Vec<f32> = plans.pop().unwrap().into_iter().map(|x| x as f32).collect();
                self.planner.set_joint_angles(&plans.pop().unwrap()).unwrap();
                self.update_robot();
            }

            for event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        match code {
                            Key::Up => {
                                self.ik_target_pose.translation.vector[1] += 0.05;
                                self.update_ik_target();
                            }
                            Key::Down => {
                                self.ik_target_pose.translation.vector[1] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::Left => {
                                self.ik_target_pose.translation.vector[0] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::Right => {
                                self.ik_target_pose.translation.vector[0] += 0.05;
                                self.update_ik_target();
                            }
                            Key::B => {
                                self.ik_target_pose.translation.vector[2] += 0.05;
                                self.update_ik_target();
                            }
                            Key::F => {
                                self.ik_target_pose.translation.vector[2] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::I => {
                                let result = ugok::solve_ik_with_random_initialize(
                                    &solver,
                                    &mut self.planner.robot,
                                    &na::convert(self.ik_target_pose),
                                    100,
                                );
                                if result.is_ok() {
                                    self.update_robot();
                                } else {
                                    println!("fail!!");
                                }
                            }
                            Key::M => {
                                initial = self.planner.robot.get_joint_angles();
                            }
                            Key::P => {
                                /*
                                let goal: Vec<f64> = self.robot
                                    .get_joint_angles()
                                    .into_iter()
                                    .map(|x| x as f64)
                                    .collect();
                                 */
                                let goal = self.planner.robot.get_joint_angles();
                                let obj = &self.target_objects.shapes()[0];
                                self.planner.set_joint_angles(&initial).unwrap();
                                let result = self.planner.plan(&goal, &*obj.1, &obj.0);
                                match result {
                                    Ok(mut plan) => {
                                        plan.reverse();
                                        for i in 0..(plan.len() - 1) {
                                            let mut interpolated_angles =
                                                ugok::interpolate(&plan[i], &plan[i + 1], 0.1);
                                            plans.append(&mut interpolated_angles);
                                        }
                                    }
                                    Err(err) => {
                                        println!("{:?}", err);
                                    }
                                }
                            }
                            Key::R => {
                                ugok::set_random_joint_angles(&mut self.planner.robot).unwrap();
//                                self.set_current_robot_to_planner(().unwrap();
                                self.update_robot();
                            }
                            Key::C => {
                                self.set_current_robot_to_planner().unwrap();
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
