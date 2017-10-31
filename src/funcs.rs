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
use na::Real;
use errors::*;
use rand;


pub fn generate_clamped_joint_angles_from_limits<T>(
    angles: &[T],
    limits: &Vec<Option<k::Range<T>>>,
) -> Result<Vec<T>>
where
    T: Real,
{
    if angles.len() != limits.len() {
        return Err(Error::from("size mismatch of input angles and limits"));
    }
    Ok(
        limits
            .iter()
            .zip(angles.iter())
            .map(|(range, angle)| match *range {
                Some(ref range) => if *angle > range.max {
                    range.max
                } else {
                    if *angle < range.min {
                        range.min
                    } else {
                        *angle
                    }
                },
                None => *angle,
            })
            .collect(),
    )
}

pub fn generate_random_joint_angles_from_limits<T>(limits: &Vec<Option<k::Range<T>>>) -> Vec<T>
where
    T: Real,
{
    limits
        .iter()
        .map(|range| match *range {
            Some(ref range) => (range.max - range.min) * na::convert(rand::random()) + range.min,
            None => na::convert::<f64, T>(rand::random::<f64>() - 0.5) * na::convert(2.0 * 3.14),
        })
        .collect()
}


fn distance<T>(a: &[T], b: &[T]) -> T
where
    T: Real,
{
    debug_assert!(a.len() == b.len());
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (*x - *y) * (*x - *y))
        .fold(T::from_f64(0.0).unwrap(), ::std::ops::Add::add)
        .sqrt()
}

pub fn interpolate<T>(vec1: &[T], vec2: &[T], unit_length: T) -> Vec<Vec<T>>
where
    T: Real,
{
    //    let dist = distance(vec1, vec2);
    let mut ret: Vec<Vec<T>> = vec![];
    let dist = distance(vec1, vec2);
    let num: usize = (dist / unit_length).to_subset().unwrap() as usize;
    for i in 0..num {
        ret.push(
            vec1.iter()
                .zip(vec2.iter())
                .map(|(v1, v2)| {
                    *v1 + (*v2 - *v1) * T::from_f64((i as f64) / (num as f64)).unwrap()
                })
                .collect::<Vec<T>>(),
        );
    }
    ret
}

pub fn set_random_joint_angles<T, K>(robot: &mut K) -> ::std::result::Result<(), k::JointError>
where
    K: k::JointContainer<T>,
    T: Real,
{
    let limits = robot.get_joint_limits();
    robot.set_joint_angles(&generate_random_joint_angles_from_limits(&limits))
}
