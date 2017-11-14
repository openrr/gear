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
use ncollide::shape::Compound3;
use urdf_rs;

use k::LinkContainer;
use k::CreateChain;

use errors::*;
use path_planner::DefaultJointPathPlanner;

/// Joint path planner which supports inverse kinematics
pub struct JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
{
    /// Joint Path Planner to be used to find collision free path
    ///
    /// Currently, `JointPathPlanner<k::LinkTree<f64>>` is used.
    pub path_planner: DefaultJointPathPlanner<f64>,
    /// Inverse kinematics solver to find the goal joint angles
    pub ik_solver: K,
}

impl<K> JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
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
    pub fn new(path_planner: DefaultJointPathPlanner<f64>, ik_solver: K) -> Self {
        Self {
            path_planner,
            ik_solver,
        }
    }
    pub fn urdf_robot(&self) -> &Option<urdf_rs::Robot> {
        &self.path_planner.urdf_robot
    }
    pub fn create_arm(&self, end_link_name: &str) -> Result<k::RcKinematicChain<f64>> {
        let candidates = self.path_planner.collision_check_robot.get_link_names();
        self.path_planner
            .collision_check_robot
            .chain_from_end_link_name(end_link_name)
            .ok_or(Error::Other(format!(
                "end link `{}` not found: candidates = {:?}",
                end_link_name,
                candidates
            )))
    }
    pub fn solve_ik<T>(&mut self, arm: &mut T, target_pose: &na::Isometry3<f64>) -> Result<f64>
    where
        T: k::KinematicChain<f64>,
    {
        Ok(self.ik_solver.solve(arm, target_pose)?)
    }
    pub fn get_colliding_link_names(&self, objects: &Compound3<f64>) -> Vec<String> {
        self.path_planner.get_colliding_link_names(objects)
    }
    pub fn plan_with_ik<T>(
        &mut self,
        arm: &mut T,
        target_pose: &na::Isometry3<f64>,
        objects: &Compound3<f64>,
    ) -> Result<Vec<Vec<f64>>>
    where
        T: k::KinematicChain<f64>,
    {
        let initial = arm.get_joint_angles();
        let _ = self.ik_solver.solve(arm, target_pose)?;
        let goal = arm.get_joint_angles();
        self.path_planner.plan(arm, &initial, &goal, objects)
    }
    pub fn plan_joints<T>(
        &mut self,
        use_joints: &mut T,
        start_angles: &[f64],
        goal_angles: &[f64],
        objects: &Compound3<f64>,
    ) -> Result<Vec<Vec<f64>>>
    where
        T: k::JointContainer<f64>,
    {
        self.path_planner.plan(
            use_joints,
            start_angles,
            goal_angles,
            objects,
        )
    }
}

impl<K> k::LinkContainer<f64> for JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
{
    /// Calculate the transforms of all of the links
    fn calc_link_transforms(&self) -> Vec<na::Isometry3<f64>> {
        self.path_planner.calc_link_transforms()
    }

    /// Get the names of the links
    fn get_link_names(&self) -> Vec<String> {
        self.path_planner.get_link_names()
    }
}
