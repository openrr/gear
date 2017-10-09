extern crate env_logger;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate rand;
extern crate structopt;
extern crate urdf_rs;
extern crate urdf_viz;
extern crate ugok;

use glfw::{Action, WindowEvent, Key};
//use k::KinematicChain;
use std::path::Path;
use structopt::StructOpt;
use ncollide::shape::{Cuboid, Compound};


fn move_joint_by_random(robot: &mut k::LinkTree<f32>) -> Result<(), k::JointError> {
    let angles_vec = robot
        .iter_for_joints_link()
        .map(|link| match link.joint.limits {
                 Some(ref range) => (range.max - range.min) * rand::random::<f32>() + range.min,
                 None => (rand::random::<f32>() - 0.5) * 2.0,
             })
        .collect::<Vec<f32>>();
    robot.set_joint_angles(&angles_vec)
}

struct CollisionAvoidApp<'a> {
    robot: k::LinkTree<f32>,
    viewer: urdf_viz::Viewer<'a>,
    checker: ugok::CollisionChecker<f32>,
    link_names: Vec<String>,
    target_objects: Compound<na::Point3<f32>, na::Isometry3<f32>>,
}

impl<'a> CollisionAvoidApp<'a> {
    fn new(urdf_robot: &'a urdf_rs::Robot, base_dir: &Path) -> Self {
        let mut robot = k::urdf::create_tree::<f32>(urdf_robot);
        let mut viewer = urdf_viz::Viewer::new(urdf_robot);
        let checker = ugok::CollisionChecker::new(urdf_robot, base_dir, 0.0);

        viewer.setup(base_dir, false);
        let base_transform =
            na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                      na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57));
        robot.set_root_transform(base_transform);
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
        let target_pose = base_transform *
                          na::Isometry3::new(na::Vector3::new(0.2, 0.5, 0.0), na::zero());
        let mut cube = viewer
            .window
            .add_cube(target_shape.half_extents()[0] * 2.0,
                      target_shape.half_extents()[1] * 2.0,
                      target_shape.half_extents()[2] * 2.0);
        cube.set_local_transformation(target_pose);
        cube.set_color(0.5, 0.0, 0.5);
        viewer.add_axis_cylinders("axis", 0.5);
        CollisionAvoidApp {
            viewer: viewer,
            checker: checker,
            link_names: link_names,
            robot: robot,
            target_objects: ugok::wrap_compound(target_shape, target_pose),
        }
    }
    fn init(&mut self) {
        self.update_robot();
    }
    fn update_robot(&mut self) {
        self.viewer.update(&self.robot);
    }
    fn run(&mut self) {
        while self.viewer.render() {
            for event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        match code {
                            Key::R => {
                                move_joint_by_random(&mut self.robot).unwrap_or(());
                                self.update_robot();
                                for name in &self.link_names {
                                    self.viewer.reset_temporal_color(name);
                                }
                            }
                            Key::C => {
                                for obj in self.target_objects.shapes() {
                                    let names =
                                        self.checker
                                            .get_collision_link_names(&self.robot, &*obj.1, &obj.0);
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
    let opt = urdf_viz::Opt::from_args();
    let input_path = Path::new(&opt.input_urdf_or_xacro);
    let base_dir = input_path
        .parent()
        .unwrap_or_else(|| {
                            panic!("failed to get base dir of {}", opt.input_urdf_or_xacro);
                        });
    let urdf_robot =
        if input_path
                   .extension()
                   .unwrap_or_else(|| panic!("failed to get extension")) == "xacro" {
                let urdf_utf =
                        urdf_rs::utils::convert_xacro_to_urdf(input_path)
                            .unwrap_or_else(|err| panic!("failed to convert xacro {:?}", err));
                urdf_rs::read_from_string(&urdf_utf)
            } else {
                urdf_rs::read_file(&input_path)
            }
            .unwrap_or_else(|err| panic!("failed to read file {:?}: {}", input_path, err));

    let mut app = CollisionAvoidApp::new(&urdf_robot, base_dir);
    app.init();
    app.run();
}
