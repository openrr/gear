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
use super::urdf::urdf_geometry_to_shape_handle;
use crate::errors::*;
use log::*;
use na::RealField;
use nalgebra as na;
use ncollide3d::{
    query,
    shape::{Compound, Shape, ShapeHandle},
};
use std::{collections::HashMap, path::Path};
type NameShapeMap<T> = HashMap<String, Vec<(ShapeHandle<T>, na::Isometry3<T>)>>;

#[derive(Clone)]
/// Collision checker for a robot
pub struct CollisionChecker<T>
where
    T: RealField,
{
    name_collision_model_map: NameShapeMap<T>,
    /// margin length for collision check
    pub prediction: T,
    pub self_collision_pairs: Vec<(String, String)>,
}

impl<T> CollisionChecker<T>
where
    T: RealField,
{
    /// Create CollisionChecker from HashMap
    pub fn new(name_collision_model_map: NameShapeMap<T>, prediction: T) -> Self {
        CollisionChecker {
            name_collision_model_map,
            prediction,
            self_collision_pairs: Vec::new(),
        }
    }

    /// Create CollisionChecker from urdf_rs::Robot
    pub fn from_urdf_robot(urdf_robot: &urdf_rs::Robot, prediction: T) -> Self {
        Self::from_urdf_robot_with_base_dir(urdf_robot, None, prediction)
    }
    /// Create CollisionChecker from urdf_rs::Robot with base_dir support
    ///
    /// base_dir: mesh files are loaded from this dir if the path does not start with "package://"
    pub fn from_urdf_robot_with_base_dir(
        urdf_robot: &urdf_rs::Robot,
        base_dir: Option<&Path>,
        prediction: T,
    ) -> Self {
        let mut name_collision_model_map = HashMap::new();
        let link_joint_map = k::urdf::link_to_joint_map(&urdf_robot);
        for l in &urdf_robot.links {
            let col_pose_vec = l
                .collision
                .iter()
                .filter_map(|collision| {
                    urdf_geometry_to_shape_handle(&collision.geometry, base_dir)
                        .map(|col| (col, k::urdf::isometry_from(&collision.origin)))
                })
                .collect::<Vec<_>>();
            debug!("name={}, ln={}", l.name, col_pose_vec.len());
            if !col_pose_vec.is_empty() {
                if let Some(joint_name) = link_joint_map.get(&l.name) {
                    name_collision_model_map.insert(joint_name.to_owned(), col_pose_vec);
                }
            }
        }
        CollisionChecker {
            name_collision_model_map,
            prediction,
            self_collision_pairs: Vec::new(),
        }
    }
    /// Check if there are any colliding links
    pub fn has_any_colliding(
        &self,
        robot: &k::Chain<T>,
        target_shape: &dyn Shape<T>,
        target_pose: &na::Isometry3<T>,
    ) -> bool {
        !self
            .colliding_link_names_with_first_return_flag(robot, target_shape, target_pose, true)
            .is_empty()
    }
    /// Returns the names which is colliding with the target shape/pose
    pub fn colliding_link_names(
        &self,
        robot: &k::Chain<T>,
        target_shape: &dyn Shape<T>,
        target_pose: &na::Isometry3<T>,
    ) -> Vec<String> {
        self.colliding_link_names_with_first_return_flag(robot, target_shape, target_pose, false)
    }

    /// Check collision and return the names of the link(joint) names
    ///
    /// robot: robot model
    /// target_shape: Check collision with this shape and the robot
    /// target_pose: Check collision with this shape in this pose and the robot
    /// first_return: if true the function returns immediately when it found a collision.
    /// This flag is to make it fast.
    pub fn colliding_link_names_with_first_return_flag(
        &self,
        robot: &k::Chain<T>,
        target_shape: &dyn Shape<T>,
        target_pose: &na::Isometry3<T>,
        first_return: bool,
    ) -> Vec<String> {
        let mut names = Vec::new();
        robot.update_transforms();
        for joint in robot.iter() {
            let trans = joint.world_transform().unwrap();
            let joint_name = &joint.joint().name;
            match self.name_collision_model_map.get(joint_name) {
                Some(obj_vec) => {
                    for obj in obj_vec {
                        // proximity and prediction does not work for meshes.
                        let dist =
                            query::distance(&(trans * obj.1), &*obj.0, target_pose, target_shape);
                        if dist < self.prediction {
                            debug!("name: {}, dist={}", joint_name, dist);
                            names.push(joint_name.to_owned());
                            if first_return {
                                return names;
                            } else {
                                break;
                            }
                        }
                    }
                }
                None => {
                    debug!("collision model {} not found", joint_name);
                }
            }
        }
        names
    }

    /// Check if there are any self colliding links
    pub fn has_self_collision(
        &self,
        collision_check_robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
    ) -> Result<bool> {
        Ok(!self
            .self_colliding_link_names_with_first_return_flag(
                collision_check_robot,
                self_collision_pairs,
                true,
            )?
            .is_empty())
    }
    /// Returns the names which is colliding with the target shape/pose
    pub fn self_colliding_link_names(
        &self,
        collision_check_robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
    ) -> Result<Vec<(String, String)>> {
        self.self_colliding_link_names_with_first_return_flag(
            collision_check_robot,
            self_collision_pairs,
            false,
        )
    }
    /// Check self collision and return the names of the link(joint) names
    ///
    /// robot: robot model
    /// self_collision_pairs: pairs of the names of the link(joint)
    /// first_return: if true the function returns immediately when it found a collision.
    /// This flag is to make it fast.
    pub fn self_colliding_link_names_with_first_return_flag(
        &self,
        collision_check_robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
        first_return: bool,
    ) -> Result<Vec<(String, String)>> {
        let mut names = Vec::new();
        collision_check_robot.update_transforms();
        for (j1, j2) in self_collision_pairs {
            if let Some(obj_vec1) = self.name_collision_model_map.get(j1) {
                if let Some(obj_vec2) = self.name_collision_model_map.get(j2) {
                    let node1_opt = collision_check_robot.find(j1);
                    let node2_opt = collision_check_robot.find(j2);
                    if node1_opt.is_none() {
                        return Err(format!("self_colliding: {} not found", j1).into());
                    }
                    if node2_opt.is_none() {
                        return Err(format!("self_colliding: {} not found", j2).into());
                    }
                    let node1 = node1_opt.unwrap();
                    let node2 = node2_opt.unwrap();
                    for obj1 in obj_vec1 {
                        for obj2 in obj_vec2 {
                            let trans1 = node1.world_transform().unwrap();
                            let trans2 = node2.world_transform().unwrap();
                            // proximity and predict does not work correctly for mesh
                            let dist = query::distance(
                                &(trans1 * obj1.1),
                                &*obj1.0,
                                &(trans2 * obj2.1),
                                &*obj2.0,
                            );
                            debug!("name: {}, name: {} dist={}", j1, j2, dist);
                            if dist < self.prediction {
                                names.push((j1.to_owned(), j2.to_owned()));
                                if first_return {
                                    return Ok(names);
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        Ok(names)
    }
}

pub trait FromUrdf {
    fn from_urdf_robot(robot: &urdf_rs::Robot) -> Self;
    fn from_urdf_file<P>(path: P) -> ::std::result::Result<Self, urdf_rs::UrdfError>
    where
        Self: ::std::marker::Sized,
        P: AsRef<Path>,
    {
        Ok(Self::from_urdf_robot(&urdf_rs::read_file(path)?))
    }
}

pub fn parse_colon_separated_pairs(pair_strs: &[String]) -> Result<Vec<(String, String)>> {
    let mut pairs = Vec::new();
    for pair_str in pair_strs {
        let mut sp = pair_str.split(':');
        if let Some(p1) = sp.next() {
            if let Some(p2) = sp.next() {
                pairs.push((p1.to_owned(), p2.to_owned()));
            } else {
                return Err(format!("failed to parse {}", pair_str).into());
            }
        } else {
            return Err(format!("failed to parse {}", pair_str).into());
        }
    }
    Ok(pairs)
}

#[cfg(test)]
mod test {
    use super::parse_colon_separated_pairs;
    #[test]
    fn test_parse_colon_separated_pairs() {
        let pairs =
            parse_colon_separated_pairs(&vec!["j0:j1".to_owned(), "j2:j0".to_owned()]).unwrap();
        assert_eq!(pairs.len(), 2);
        assert_eq!(pairs[0].0, "j0");
        assert_eq!(pairs[0].1, "j1");
        assert_eq!(pairs[1].0, "j2");
        assert_eq!(pairs[1].1, "j0");
    }
}

/// Create `ncollide::shape::Compound` from URDF file
///
/// The `<link>` elements are used as obstacles. set the origin/geometry of
/// `<visual>` and `<collision>`. You can skip `<inertia>`.
impl FromUrdf for Compound<f64> {
    fn from_urdf_robot(urdf_obstacle: &urdf_rs::Robot) -> Self {
        let compound_data = urdf_obstacle
            .links
            .iter()
            .flat_map(|l| {
                l.collision
                    .iter()
                    .map(|collision| {
                        match urdf_geometry_to_shape_handle(&collision.geometry, None) {
                            Some(col) => Some((k::urdf::isometry_from(&collision.origin), col)),
                            None => None,
                        }
                    })
                    .collect::<Vec<_>>()
            })
            .filter_map(|col_tuple| col_tuple)
            .collect::<Vec<_>>();
        Compound::new(compound_data)
    }
}
