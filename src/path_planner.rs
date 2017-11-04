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
use na;
use ncollide::shape::Compound;
use std;
use rrt;

use collision_checker::*;
use funcs::*;

/// Collision Avoidance Path Planner
pub struct CollisionAvoidJointPathPlanner<K, R>
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
}

impl<K, R> CollisionAvoidJointPathPlanner<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    pub fn new(
        moving_arm: K,
        collision_check_robot: R,
        collision_checker: CollisionChecker<f64>,
        step_length: f64,
        max_try: usize,
        num_smoothing: usize,
    ) -> Self {
        CollisionAvoidJointPathPlanner {
            moving_arm,
            collision_check_robot,
            collision_checker,
            step_length,
            max_try,
            num_smoothing,
        }
    }
    /// Check if the joint_angles are OK
    pub fn is_feasible(
        &mut self,
        joint_angles: &[f64],
        objects: &Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ) -> bool {
        self.set_joint_angles(joint_angles).unwrap();
        !self.has_any_colliding(objects)
    }

    pub fn set_joint_angles(
        &mut self,
        joint_angles: &[f64],
    ) -> std::result::Result<(), k::JointError> {
        self.moving_arm.set_joint_angles(joint_angles)
    }

    pub fn get_joint_angles(&self) -> Vec<f64> {
        self.moving_arm.get_joint_angles()
    }

    pub fn has_any_colliding(
        &self,
        objects: &Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ) -> bool {
        for shape in objects.shapes() {
            if self.collision_checker.has_any_colliding(
                &self.collision_check_robot,
                &*shape.1,
                &shape.0,
            ) {
                return true;
            }
        }
        false
    }

    pub fn get_colliding_link_names(
        &self,
        objects: &Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ) -> Vec<String> {
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

    pub fn plan(
        &mut self,
        goal_angles: &[f64],
        objects: &Compound<na::Point3<f64>, na::Isometry3<f64>>,
    ) -> std::result::Result<Vec<Vec<f64>>, String> {
        let initial_angles = self.get_joint_angles();
        let limits = self.moving_arm.get_joint_limits();
        let step_length = self.step_length;
        let max_try = self.max_try;
        if !self.is_feasible(&initial_angles, objects) {
            return Err("Initialis colliding".to_owned());
        } else if !self.is_feasible(goal_angles, objects) {
            return Err("Goal is colliding".to_owned());
        }
        let mut path = try!(rrt::dual_rrt_connect(
            &initial_angles,
            goal_angles,
            |angles: &[f64]| self.is_feasible(angles, objects),
            || generate_random_joint_angles_from_limits(&limits),
            step_length,
            max_try,
        ));
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

pub struct CollisionAvoidJointPathPlannerBuilder<K, R>
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
}

impl<K, R> CollisionAvoidJointPathPlannerBuilder<K, R>
where
    K: k::JointContainer<f64>,
    R: k::LinkContainer<f64>,
{
    pub fn new(
        moving_arm: K,
        collision_check_robot: R,
        collision_checker: CollisionChecker<f64>,
    ) -> Self {
        CollisionAvoidJointPathPlannerBuilder {
            moving_arm: moving_arm,
            collision_check_robot: collision_check_robot,
            collision_checker: collision_checker,
            step_length: 0.1,
            max_try: 5000,
            num_smoothing: 100,
        }
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
    pub fn finalize(self) -> CollisionAvoidJointPathPlanner<K, R> {
        CollisionAvoidJointPathPlanner::new(
            self.moving_arm,
            self.collision_check_robot,
            self.collision_checker,
            self.step_length,
            self.max_try,
            self.num_smoothing,
        )
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use urdf_rs;
    use ncollide::shape::Cuboid;
    use na::{Isometry3, Vector3};

    #[test]
    fn it_works() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let checker = CollisionChecker::from_urdf_robot(&urdf_robot, None, 0.05);

        let target = Cuboid::new(Vector3::new(0.5, 0.5, 0.5));
        let target_pose = Isometry3::new(Vector3::new(0.0, 0.0, -0.5), na::zero());

        let robot = k::urdf::create_tree::<f32>(&urdf_robot);
        let names = checker.get_colliding_link_names(&robot, &target, &target_pose);
        assert_eq!(names, vec!["root", "l_elbow1", "l_wrist1", "l_wrist2"]);
    }
}
