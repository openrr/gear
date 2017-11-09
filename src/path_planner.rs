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
use k;
use ncollide::shape::Compound3;
use std;
use rrt;
use std::path::Path;
use urdf_rs;

use collision_checker::*;
use errors::*;
use funcs::*;

/// Collision Avoidance Path Planner
pub struct JointPathPlanner<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    /// Instance of `k::JointContainer` to set joint angles
    pub moving_arm: K,
    /// Instance of `k::LinkContainer` to check the collision
    pub collision_check_robot: R,
    /// Collision checker
    pub collision_checker: CollisionChecker<f64>,
    /// Unit length for searching
    ///
    /// If the value is large, the path become sparse.
    pub step_length: f64,
    /// Max num of RRT search loop
    pub max_try: usize,
    /// Num of path smoothing trials
    pub num_smoothing: usize,
    /// The robot instance which is used to create the robot model
    pub urdf_robot: Option<urdf_rs::Robot>,
}

impl<K, R> JointPathPlanner<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    /// Create `JointPathPlanner`
    pub fn new(
        moving_arm: K,
        collision_check_robot: R,
        collision_checker: CollisionChecker<f64>,
        step_length: f64,
        max_try: usize,
        num_smoothing: usize,
    ) -> Self {
        JointPathPlanner {
            moving_arm,
            collision_check_robot,
            collision_checker,
            step_length,
            max_try,
            num_smoothing,
            urdf_robot: None,
        }
    }
    /// Check if the joint_angles are OK
    pub fn is_feasible(&mut self, joint_angles: &[f64], objects: &Compound3<f64>) -> bool {
        self.set_joint_angles(joint_angles).unwrap();
        !self.has_any_colliding(objects)
    }
    /// Set the joint angles of `self.moving_arm`
    pub fn set_joint_angles(
        &mut self,
        joint_angles: &[f64],
    ) -> std::result::Result<(), k::JointError> {
        self.moving_arm.set_joint_angles(joint_angles)
    }
    /// Get the joint angles of `self.moving_arm`
    pub fn get_joint_angles(&self) -> Vec<f64> {
        self.moving_arm.get_joint_angles()
    }
    /// Check if there are any colliding links
    pub fn has_any_colliding(&self, objects: &Compound3<f64>) -> bool {
        for shape in objects.shapes() {
            if self.collision_checker.has_any_colliding(
                &self.collision_check_robot,
                &*shape.1,
                &shape.0,
            )
            {
                return true;
            }
        }
        false
    }
    /// Get the names of colliding links
    pub fn get_colliding_link_names(&self, objects: &Compound3<f64>) -> Vec<String> {
        let mut ret = Vec::new();
        for shape in objects.shapes() {
            let mut colliding_names = self.collision_checker.get_colliding_link_names(
                &self.collision_check_robot,
                &*shape.1,
                &shape.0,
            );
            ret.append(&mut colliding_names);
        }
        ret
    }

    /// Plan the sequence of joint angles of `self.moving_arm`
    ///
    /// # Arguments
    /// - goal_angles: goal joint angles of `self.moving_arm`.
    /// start is `self.moving_arm.get_joint_angles()`. set it by
    /// `set_joint_angles()`.
    pub fn plan(
        &mut self,
        start_angles: &[f64],
        goal_angles: &[f64],
        objects: &Compound3<f64>,
    ) -> Result<Vec<Vec<f64>>> {
        let limits = self.moving_arm.get_joint_limits();
        let step_length = self.step_length;
        let max_try = self.max_try;
        let current_angles = self.moving_arm.get_joint_angles();
        if !self.is_feasible(start_angles, objects) {
            self.moving_arm.set_joint_angles(&current_angles)?;
            return Err(Error::Collision("Initialis colliding".to_owned()));
        } else if !self.is_feasible(goal_angles, objects) {
            self.moving_arm.set_joint_angles(&current_angles)?;
            return Err(Error::Collision("Goal is colliding".to_owned()));
        }
        let mut path = match rrt::dual_rrt_connect(
            start_angles,
            goal_angles,
            |angles: &[f64]| self.is_feasible(angles, objects),
            || generate_random_joint_angles_from_limits(&limits),
            step_length,
            max_try,
        ) {
            Ok(p) => p,
            Err(error) => {
                self.moving_arm.set_joint_angles(&current_angles)?;
                return Err(Error::from(error));
            }
        };
        let num_smoothing = self.num_smoothing;
        rrt::smooth_path(
            &mut path,
            |angles: &[f64]| self.is_feasible(angles, objects),
            step_length,
            num_smoothing,
        );
        Ok(path)
    }
}

/// Builder pattern to create `JointPathPlanner`
pub struct JointPathPlannerBuilder<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    pub moving_arm: K,
    pub collision_check_robot: R,
    pub collision_checker: CollisionChecker<f64>,
    pub step_length: f64,
    pub max_try: usize,
    pub num_smoothing: usize,
    pub collision_check_margin: Option<f64>,
    pub urdf_robot: Option<urdf_rs::Robot>,
}

impl<K, R> JointPathPlannerBuilder<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    pub fn new(
        moving_arm: K,
        collision_check_robot: R,
        collision_checker: CollisionChecker<f64>,
    ) -> Self {
        JointPathPlannerBuilder {
            moving_arm: moving_arm,
            collision_check_robot: collision_check_robot,
            collision_checker: collision_checker,
            step_length: 0.1,
            max_try: 5000,
            num_smoothing: 100,
            collision_check_margin: None,
            urdf_robot: None,
        }
    }
    pub fn collision_check_margin(mut self, length: f64) -> Self {
        self.collision_check_margin = Some(length);
        self
    }
    pub fn step_length(mut self, step_length: f64) -> Self {
        self.step_length = step_length;
        self
    }
    pub fn max_try(mut self, max_try: usize) -> Self {
        self.max_try = max_try;
        self
    }
    pub fn num_smoothing(mut self, num_smoothing: usize) -> Self {
        self.num_smoothing = num_smoothing;
        self
    }
    pub fn finalize(mut self) -> JointPathPlanner<K, R> {
        let mut planner = JointPathPlanner::new(
            self.moving_arm,
            self.collision_check_robot,
            self.collision_checker,
            self.step_length,
            self.max_try,
            self.num_smoothing,
        );
        if let Some(margin) = self.collision_check_margin {
            self.collision_checker.prediction = margin;
        }
        planner.urdf_robot = self.urdf_robot;
        planner
    }
}


pub fn build_from_urdf_file_and_end_link_name<P>(
    file_path: P,
    end_link_name: &str,
) -> Result<JointPathPlannerBuilder<k::RcKinematicChain<f64>, k::LinkTree<f64>>>
where
    P: AsRef<Path>,
{
    let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(file_path.as_ref())?;
    const DEFAULT_MARGIN: f64 = 0.0;
    let collision_checker = CollisionChecker::<f64>::from_urdf_robot(&urdf_robot, DEFAULT_MARGIN);
    let collision_check_robot = k::urdf::create_tree::<f64>(&urdf_robot);
    let moving_arm = collision_check_robot
        .iter()
        .find(|&ljn_ref| ljn_ref.borrow().data.name == end_link_name)
        .and_then(|ljn| Some(k::RcKinematicChain::new(end_link_name, ljn)))
        .ok_or(Error::Other(format!("{} not found", end_link_name)))?;

    Ok(JointPathPlannerBuilder {
        moving_arm,
        collision_check_robot,
        collision_checker,
        step_length: 0.1,
        max_try: 5000,
        num_smoothing: 100,
        collision_check_margin: None,
        urdf_robot: Some(urdf_robot),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use urdf_rs;
    use na;
    use ncollide::shape::Cuboid;
    use na::{Isometry3, Vector3};
    use k::JointContainer;

    #[test]
    fn collision_check() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let checker = CollisionChecker::from_urdf_robot(&urdf_robot, 0.05);

        let target = Cuboid::new(Vector3::new(0.5, 1.0, 0.5));
        let target_pose = Isometry3::new(Vector3::new(0.9, 0.0, 0.0), na::zero());

        let mut robot = k::urdf::create_tree::<f32>(&urdf_robot);

        let names = checker.get_colliding_link_names(&robot, &target, &target_pose);
        assert_eq!(
            names,
            vec![
                "l_elbow1",
                "l_wrist1",
                "l_wrist2",
                "l_gripper1",
                "l_gripper2",
            ]
        );
        let angles = vec![-1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        robot.set_joint_angles(&angles).unwrap();
        let names = checker.get_colliding_link_names(&robot, &target, &target_pose);
        assert_eq!(
            names,
            vec!["l_wrist1", "l_wrist2", "l_gripper1", "l_gripper2"]
        );
        let target_pose = Isometry3::new(Vector3::new(0.7, 0.0, 0.0), na::zero());
        let names = checker.get_colliding_link_names(&robot, &target, &target_pose);
        assert_eq!(
            names,
            vec![
                "l_shoulder2",
                "l_shoulder3",
                "l_elbow1",
                "l_wrist1",
                "l_wrist2",
                "l_gripper1",
                "l_gripper2",
            ]
        );
    }
    #[test]
    fn from_urdf() {
        let _ = build_from_urdf_file_and_end_link_name("sample.urdf", "l_wrist2")
            .unwrap()
            .collision_check_margin(0.01)
            .finalize();
    }
}
