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
use k::InverseKinematicsSolver;
use na::{self, Real};
use funcs::*;

pub struct RandomInitializeIKSolver<T, I>
where
    I: InverseKinematicsSolver<T>,
    T: Real,
{
    pub solver: I,
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
    fn solve<K>(
        &self,
        arm: &mut K,
        target_pose: &na::Isometry3<T>,
    ) -> ::std::result::Result<T, k::IKError>
    where
        K: k::KinematicChain<T>,
        T: Real,
    {
        let mut result = Err(k::IKError::NotConverged);
        for _ in 0..self.num_max_try {
            set_random_joint_angles(arm).unwrap();
            result = self.solver.solve(arm, target_pose);
            if result.is_ok() {
                return result;
            }
        }
        result
    }
}
