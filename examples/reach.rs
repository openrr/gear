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
extern crate env_logger;
extern crate gear;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate urdf_viz;

use k::JointContainer;
use glfw::{Action, Key, WindowEvent};
use ncollide::shape::{Compound3, Cuboid3, ShapeHandle3};

fn add_shape_in_viewer(
    viewer: &mut urdf_viz::Viewer,
    shape: &ShapeHandle3<f64>,
    pose: &na::Isometry3<f64>,
    r: f32,
    g: f32,
    b: f32,
) {
    let mut scene_node = if let Some(cube) = shape.as_shape::<Cuboid3<f64>>() {
        viewer.window.add_cube(
            cube.half_extents()[0] as f32 * 2.0,
            cube.half_extents()[1] as f32 * 2.0,
            cube.half_extents()[2] as f32 * 2.0,
        )
    } else {
        panic!("not support shape");
    };
    scene_node.set_local_transformation(na::convert(*pose));
    scene_node.set_color(r, g, b);
}


struct CollisionAvoidApp<I>
where
    I: gear::InverseKinematicsSolver<f64>,
{
    planner: gear::JointPathPlannerWithIK<I>,
    target_objects: Compound3<f64>,
    ik_target_pose: na::Isometry3<f64>,
    colliding_link_names: Vec<String>,
    viewer: urdf_viz::Viewer,
}

impl<I> CollisionAvoidApp<I>
where
    I: gear::InverseKinematicsSolver<f64>,
{
    fn new(planner: gear::JointPathPlannerWithIK<I>) -> Self {
        let mut viewer = urdf_viz::Viewer::new();
        viewer.setup(planner.urdf_robot().as_ref().unwrap());
        viewer.add_axis_cylinders("origin", 1.0);

        let obstacle_shape1 = ShapeHandle3::new(Cuboid3::new(na::Vector3::new(0.20, 0.4, 0.1)));
        let obstacle_pose1 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.1), na::zero());
        add_shape_in_viewer(
            &mut viewer,
            &obstacle_shape1,
            &obstacle_pose1,
            0.5,
            0.0,
            0.5,
        );

        let obstacle_shape2 = ShapeHandle3::new(Cuboid3::new(na::Vector3::new(0.20, 0.3, 0.1)));
        let obstacle_pose2 = na::Isometry3::new(na::Vector3::new(0.7, 0.0, 0.6), na::zero());
        add_shape_in_viewer(
            &mut viewer,
            &obstacle_shape2,
            &obstacle_pose2,
            0.5,
            0.5,
            0.0,
        );

        let target_objects = Compound3::new(vec![
            (obstacle_pose1, obstacle_shape1),
            (obstacle_pose2, obstacle_shape2),
        ]);
        let ik_target_pose = na::Isometry3::from_parts(
            na::Translation3::new(0.40, 0.20, 0.3),
            na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
        );
        viewer.add_axis_cylinders("ik_target", 0.3);
        CollisionAvoidApp {
            viewer,
            target_objects,
            ik_target_pose,
            colliding_link_names: Vec::new(),
            planner,
        }
    }
    fn update_robot(&mut self) {
        self.viewer.update(&self.planner);
    }
    fn update_ik_target(&mut self) {
        if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
            obj.0.set_local_transformation(
                na::convert(self.ik_target_pose),
            );
        }
    }
    fn reset_colliding_link_colors(&mut self) {
        for link in &self.colliding_link_names {
            self.viewer.reset_temporal_color(link);
        }
    }
    fn run(&mut self) {
        self.update_robot();
        self.update_ik_target();
        let mut plans: Vec<Vec<f64>> = Vec::new();
        while self.viewer.render() {
            if !plans.is_empty() {
                self.planner
                    .set_joint_angles(&plans.pop().unwrap())
                    .unwrap();
                self.update_robot();
            }

            for event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        match code {
                            Key::Up => {
                                self.ik_target_pose.translation.vector[2] += 0.05;
                                self.update_ik_target();
                            }
                            Key::Down => {
                                self.ik_target_pose.translation.vector[2] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::Left => {
                                self.ik_target_pose.translation.vector[1] += 0.05;
                                self.update_ik_target();
                            }
                            Key::Right => {
                                self.ik_target_pose.translation.vector[1] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::B => {
                                self.ik_target_pose.translation.vector[0] -= 0.05;
                                self.update_ik_target();
                            }
                            Key::F => {
                                self.ik_target_pose.translation.vector[0] += 0.05;
                                self.update_ik_target();
                            }
                            Key::I => {
                                self.reset_colliding_link_colors();
                                let result = self.planner.solve_ik(&self.ik_target_pose);
                                if result.is_ok() {
                                    self.update_robot();
                                } else {
                                    println!("fail!!");
                                }
                            }
                            Key::G => {
                                self.reset_colliding_link_colors();
                                match self.planner.plan_with_ik(
                                    &self.ik_target_pose,
                                    &self.target_objects,
                                ) {
                                    Ok(mut plan) => {
                                        plan.reverse();
                                        for i in 0..(plan.len() - 1) {
                                            let mut interpolated_angles =
                                                gear::interpolate(&plan[i], &plan[i + 1], 0.1);
                                            plans.append(&mut interpolated_angles);
                                        }
                                    }
                                    Err(error) => {
                                        println!("failed to reach!! {}", error);
                                    }
                                };
                            }
                            Key::R => {
                                self.reset_colliding_link_colors();
                                gear::set_random_joint_angles(&mut self.planner).unwrap();
                                self.update_robot();
                            }
                            Key::C => {
                                self.colliding_link_names =
                                    self.planner.get_colliding_link_names(&self.target_objects);
                                for name in &self.colliding_link_names {
                                    println!("{}", name);
                                    self.viewer.set_temporal_color(name, 0.8, 0.8, 0.6);
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
    use std::env;
    env_logger::init().unwrap();
    let input_string = env::args().nth(1).unwrap_or("sample.urdf".to_owned());
    let input_end_link = env::args().nth(2).unwrap_or("l_wrist2".to_owned());
    let planner = gear::get_joint_path_planner_builder_from_urdf(&input_string, &input_end_link)
        .unwrap()
        .collision_check_margin(0.01)
        .finalize();
    let solver = gear::JacobianIKSolverBuilder::<f64>::new()
        .num_max_try(1000)
        .allowable_target_distance(0.01)
        .move_epsilon(0.00001)
        .jacobian_move_epsilon(0.001)
        .finalize();
    let solver = gear::RandomInitializeIKSolver::new(solver, 100);
    let planner = gear::JointPathPlannerWithIK::new(planner, solver);
    let mut app = CollisionAvoidApp::new(planner);
    app.run();
}
