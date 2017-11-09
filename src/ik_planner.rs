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
use std;

use ncollide::shape::Compound3;

use errors::*;
use path_planner::*;

pub struct JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
{
    pub path_planner: JointPathPlanner<k::RcKinematicChain<f64>, k::LinkTree<f64>>,
    pub ik_solver: K,
}

impl<K> JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
{
    pub fn new(
        path_planner: JointPathPlanner<k::RcKinematicChain<f64>, k::LinkTree<f64>>,
        ik_solver: K,
    ) -> Self {
        Self {
            path_planner,
            ik_solver,
        }
    }
    /// Set the joint angles of `self.path_planner`
    pub fn set_joint_angles(
        &mut self,
        joint_angles: &[f64],
    ) -> std::result::Result<(), k::JointError> {
        self.path_planner.set_joint_angles(joint_angles)
    }
    /// Get the joint angles of `self.path_planner`
    pub fn get_joint_angles(&self) -> Vec<f64> {
        self.path_planner.get_joint_angles()
    }
    pub fn solve_ik(&mut self, target_pose: &na::Isometry3<f64>) -> Result<f64> {
        Ok(self.ik_solver.solve(
            &mut self.path_planner.moving_arm,
            target_pose,
        )?)
    }
    pub fn plan_with_ik(
        &mut self,
        target_pose: &na::Isometry3<f64>,
        objects: &Compound3<f64>,
    ) -> Result<Vec<Vec<f64>>> {
        let initial = self.get_joint_angles();
        let _ = self.ik_solver.solve(
            &mut self.path_planner.moving_arm,
            target_pose,
        )?;
        let goal = self.get_joint_angles();
        self.path_planner.plan(&initial, &goal, objects)
    }
    pub fn plan_joints(
        &mut self,
        start_angles: &[f64],
        goal_angles: &[f64],
        objects: &Compound3<f64>,
    ) -> Result<Vec<Vec<f64>>> {
        self.path_planner.plan(start_angles, goal_angles, objects)
    }
}

/*
pub struct JointPathPlannerWithRandomIKBuilder {
    path_planner_builder: JointPathPlannerBuilder<k::RcKinematicChain<f64>, k::LinkTree<f64>>,
    pub step_length: f64,
    pub max_try: usize,
    pub num_smoothing: usize,
    pub collision_check_margin: Option<f64>,
}

impl JointPathPlannerWithRandomIKBuilder {
    pub fn new<P>(file_path: P, end_link_name: &str) -> Self
    where
        P: AsRef<std::path::Path>,
    {
        Self {
            // todo fix unwrap
            path_planner_builder: 
        })
    }
    pub fn finalize(mut self) -> JointPathPlannerWithRandomIK {
        let ik_solver = k::JacobianIKSolverBuilder::<f64>::new().finalize;
        build_from_urdf_file_and_end_link_name(file_path, end_link_name)?,

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
}
*/