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

use k::{self, InverseKinematicsSolver};
use na::{self, Real};

use funcs::*;

/// Randomize initial joint angles before solving
#[derive(Debug)]
pub struct RandomInitializeIKSolver<T, I>
where
    I: InverseKinematicsSolver<T>,
    T: Real,
{
    /// The IK solver to be used after set random joint angles
    pub solver: I,
    /// The number to try to solve
    pub num_max_try: usize,
    phantom: ::std::marker::PhantomData<T>,
}

impl<T, I> RandomInitializeIKSolver<T, I>
where
    T: Real,
    I: InverseKinematicsSolver<T>,
{
    pub fn new(solver: I, num_max_try: usize) -> Self {
        RandomInitializeIKSolver {
            solver: solver,
            num_max_try: num_max_try,
            phantom: ::std::marker::PhantomData,
        }
    }
}

impl<T, I> InverseKinematicsSolver<T> for RandomInitializeIKSolver<T, I>
where
    T: Real,
    I: InverseKinematicsSolver<T>,
{
    fn solve(
        &self,
        arm: &k::SerialChain<T>,
        target_pose: &na::Isometry3<T>,
    ) -> ::std::result::Result<(), k::IKError> {
        let mut result = Err(k::IKError::NotConvergedError {
            error: "fail".to_owned(),
        });
        let limits = arm.iter_joints().map(|j|j.joint().limits.clone()).collect();
        let initial_angles = arm.joint_positions();

        for _ in 0..self.num_max_try {
            result = self.solver.solve(arm, target_pose);
            if result.is_ok() {
                return result;
            }
            let mut new_angles = generate_random_joint_positions_from_limits(&limits);
            modify_to_nearest_angle(&initial_angles, &mut new_angles, &limits);
            arm.set_joint_positions(&new_angles)?;
        }
        // failed
        arm.set_joint_positions(&initial_angles)?;
        result
    }
}
