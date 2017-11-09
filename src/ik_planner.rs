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
use urdf_rs;

use k::JointContainer;

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
    pub fn urdf_robot(&self) -> &Option<urdf_rs::Robot> {
        &self.path_planner.urdf_robot
    }
    pub fn solve_ik(&mut self, target_pose: &na::Isometry3<f64>) -> Result<f64> {
        Ok(self.ik_solver.solve(
            &mut self.path_planner.moving_arm,
            target_pose,
        )?)
    }
    pub fn get_colliding_link_names(&self, objects: &Compound3<f64>) -> Vec<String> {
        self.path_planner.get_colliding_link_names(objects)
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

impl<K> k::JointContainer<f64> for JointPathPlannerWithIK<K>
where
    K: k::InverseKinematicsSolver<f64>,
{
    fn set_joint_angles(&mut self, joint_angles: &[f64]) -> std::result::Result<(), k::JointError> {
        self.path_planner.set_joint_angles(joint_angles)
    }
    fn get_joint_angles(&self) -> Vec<f64> {
        self.path_planner.get_joint_angles()
    }
    fn get_joint_limits(&self) -> Vec<Option<k::Range<f64>>> {
        self.path_planner.get_joint_limits()
    }
    fn get_joint_names(&self) -> Vec<String> {
        self.path_planner.get_joint_names()
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