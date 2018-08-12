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
use ncollide3d::shape::Compound;
use num_traits;
use urdf_rs;

use k::{EndTransform, HasJoints, HasLinks};

use errors::*;
use path_planner::DefaultJointPathPlanner;

/// Joint path planner which supports inverse kinematics
pub struct JointPathPlannerWithIK<T, I>
where
    I: k::InverseKinematicsSolver<T>,
    T: na::Real,
{
    /// Joint Path Planner to be used to find collision free path
    ///
    /// Currently, `JointPathPlanner<N, k::LinkTree<N>>` is used.
    pub path_planner: DefaultJointPathPlanner<T>,
    /// Inverse kinematics solver to find the goal joint angles
    pub ik_solver: I,
}

impl<T, I> JointPathPlannerWithIK<T, I>
where
    T: na::Real + num_traits::Float,
    I: k::InverseKinematicsSolver<T>,
{
    /// Create instance from `JointPathPlannerBuilder` and `InverseKinematicsSolver`
    ///
    /// # Example
    ///
    /// ```
    /// // Create path planner with loading urdf file and set end link name
    /// let planner = gear::JointPathPlannerBuilder::from_urdf_file("sample.urdf")
    ///     .unwrap()
    ///     .collision_check_margin(0.01)
    ///     .finalize();
    /// // Create inverse kinematics solver
    /// let solver = gear::JacobianIKSolverBuilder::<f64>::new()
    ///     .num_max_try(1000)
    ///     .allowable_target_distance(0.01)
    ///     .move_epsilon(0.00001)
    ///     .jacobian_move_epsilon(0.001)
    ///     .finalize();
    /// // Create path planner with IK solver
    /// let _planner = gear::JointPathPlannerWithIK::new(planner, solver);
    /// ```
    pub fn new(path_planner: DefaultJointPathPlanner<T>, ik_solver: I) -> Self {
        Self {
            path_planner,
            ik_solver,
        }
    }
    pub fn urdf_robot(&self) -> &Option<urdf_rs::Robot> {
        &self.path_planner.urdf_robot
    }
    pub fn create_arm(&self, end_link_name: &str) -> Result<k::Manipulator<T>> {
        let candidates = self.path_planner.collision_check_robot.link_names();
        k::Manipulator::from_link_tree(end_link_name, &self.path_planner.collision_check_robot)
            .ok_or(Error::Other {
                error: format!(
                    "end link `{}` not found: candidates = {:?}",
                    end_link_name, candidates
                ),
            })
    }
    pub fn solve_ik<K>(&mut self, arm: &mut K, target_pose: &na::Isometry3<T>) -> Result<T>
    where
        K: HasJoints<T> + EndTransform<T>,
    {
        Ok(self.ik_solver.solve(arm, target_pose)?)
    }
    pub fn colliding_link_names(&self, objects: &Compound<T>) -> Vec<String> {
        self.path_planner.colliding_link_names(objects)
    }
    pub fn plan_with_ik<K>(
        &mut self,
        arm: &mut K,
        target_pose: &na::Isometry3<T>,
        objects: &Compound<T>,
    ) -> Result<Vec<Vec<T>>>
    where
        K: HasJoints<T> + EndTransform<T>,
    {
        let initial = arm.joint_angles();
        let _ = self.ik_solver.solve(arm, target_pose)?;
        let goal = arm.joint_angles();
        self.path_planner.plan(arm, &initial, &goal, objects)
    }
    pub fn plan_joints<K>(
        &mut self,
        use_joints: &mut K,
        start_angles: &[T],
        goal_angles: &[T],
        objects: &Compound<T>,
    ) -> Result<Vec<Vec<T>>>
    where
        K: HasJoints<T> + EndTransform<T>,
    {
        self.path_planner
            .plan(use_joints, start_angles, goal_angles, objects)
    }
}

impl<T, I> HasLinks<T> for JointPathPlannerWithIK<T, I>
where
    T: na::Real,
    I: k::InverseKinematicsSolver<T>,
{
    /// Calculate the transforms of all of the links
    fn link_transforms(&self) -> Vec<na::Isometry3<T>> {
        self.path_planner.link_transforms()
    }

    /// Get the names of the links
    fn link_names(&self) -> Vec<String> {
        self.path_planner.link_names()
    }
}
