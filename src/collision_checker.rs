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
use assimp;
use k;
use na::{self, RealField, Vector3};
use ncollide3d;
use ncollide3d::procedural::IndexBuffer::{Split, Unified};
use ncollide3d::query;
use ncollide3d::shape::{Ball, Compound, Cuboid, Cylinder, Shape, ShapeHandle, TriMesh};
use ncollide3d::transformation::ToTriMesh;
use std::collections::HashMap;
use std::path::Path;
use urdf_rs;

use errors::*;

fn load_mesh<P, T>(filename: P, scale: &[f64]) -> Result<TriMesh<T>>
where
    P: AsRef<Path>,
    T: RealField,
{
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename
        .as_ref()
        .to_str()
        .ok_or("faild to get string from path")?;
    Ok(assimp_scene_to_ncollide_mesh(
        importer.read_file(file_string)?,
        scale,
    ))
}

fn assimp_scene_to_ncollide_mesh<T>(scene: assimp::Scene, scale: &[f64]) -> TriMesh<T>
where
    T: RealField,
{
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut last_index: usize = 0;
    for mesh in scene.mesh_iter() {
        vertices.extend(mesh.vertex_iter().map(|v| {
            na::Point3::<T>::new(
                na::convert(v.x as f64 * scale[0]),
                na::convert(v.y as f64 * scale[1]),
                na::convert(v.z as f64 * scale[2]),
            )
        }));
        indices.extend(mesh.face_iter().filter_map(|f| {
            if f.num_indices == 3 {
                Some(na::Point3::<usize>::new(
                    f[0] as usize + last_index,
                    f[1] as usize + last_index,
                    f[2] as usize + last_index,
                ))
            } else {
                None
            }
        }));
        last_index = vertices.len() as usize;
    }
    TriMesh::new(vertices, indices, None)
}

fn urdf_geometry_to_shape_handle<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<ShapeHandle<T>>
where
    T: RealField,
{
    match *collision_geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let cube = Cuboid::new(Vector3::new(
                na::convert(size[0] * 0.5),
                na::convert(size[1] * 0.5),
                na::convert(size[2] * 0.5),
            ));
            Some(ShapeHandle::new(cube))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let y_cylinder = Cylinder::new(na::convert(length * 0.5), na::convert(radius));
            let tri_mesh = ncollide3d::transformation::convex_hull(
                &y_cylinder
                    .to_trimesh(30)
                    .coords
                    .iter()
                    .map(|point| point.xzy())
                    .collect::<Vec<_>>(),
            );
            let ind = match tri_mesh.indices {
                Unified(ind) => ind
                    .into_iter()
                    .map(|p| na::Point3::new(p[0] as usize, p[1] as usize, p[2] as usize))
                    .collect(),
                Split(_) => {
                    panic!("convex_hull implemenataion has been changed by ncollide3d update?");
                }
            };
            Some(ShapeHandle::new(TriMesh::new(
                tri_mesh.coords,
                ind,
                tri_mesh.uvs,
            )))
        }
        urdf_rs::Geometry::Sphere { radius } => {
            Some(ShapeHandle::new(Ball::new(na::convert(radius))))
        }
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                error!("{} not found", replaced_filename);
                return None;
            }
            match load_mesh(path, &scale) {
                Ok(mesh) => Some(ShapeHandle::new(mesh)),
                Err(err) => {
                    error!("load_mesh {:?} failed: {}", path, err);
                    None
                }
            }
        }
    }
}

/// Collision checker for a robot
pub struct CollisionChecker<T>
where
    T: RealField,
{
    name_collision_model_map: HashMap<String, Vec<(ShapeHandle<T>, na::Isometry3<T>)>>,
    /// margin length for collision check
    pub prediction: T,
    pub self_collision_pairs: Vec<(String, String)>,
}

impl<T> CollisionChecker<T>
where
    T: RealField,
{
    /// Create CollisionChecker from HashMap
    pub fn new(
        name_collision_model_map: HashMap<String, Vec<(ShapeHandle<T>, na::Isometry3<T>)>>,
        prediction: T,
    ) -> Self {
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

    fn colliding_link_names_with_first_return_flag(
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
                        // proximity and prediction does not works for meshes.
                        let dist =
                            query::distance(&(trans * obj.1), &*obj.0, target_pose, target_shape);
                        if dist < self.prediction {
                            println!("name: {}, dist={}", joint_name, dist);
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
        robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
    ) -> Result<bool> {
        Ok(!self
            .self_colliding_link_names_with_first_return_flag(robot, self_collision_pairs, true)?
            .is_empty())
    }
    /// Returns the names which is colliding with the target shape/pose
    pub fn self_colliding_link_names(
        &self,
        robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
    ) -> Result<Vec<(String, String)>> {
        self.self_colliding_link_names_with_first_return_flag(robot, self_collision_pairs, false)
    }
    // self
    fn self_colliding_link_names_with_first_return_flag(
        &self,
        robot: &k::Chain<T>,
        self_collision_pairs: &[(String, String)],
        first_return: bool,
    ) -> Result<Vec<(String, String)>> {
        let mut names = Vec::new();
        robot.update_transforms();
        for (j1, j2) in self_collision_pairs {
            if let Some(obj_vec1) = self.name_collision_model_map.get(j1) {
                if let Some(obj_vec2) = self.name_collision_model_map.get(j2) {
                    let node1_opt = robot.find(j1);
                    let node2_opt = robot.find(j2);
                    if node1_opt.is_none() {
                        return Err(format!("{} not found", j1).into());
                    }
                    if node2_opt.is_none() {
                        return Err(format!("{} not found", j2).into());
                    }
                    let node1 = node1_opt.unwrap();
                    let node2 = node2_opt.unwrap();
                    for obj1 in obj_vec1 {
                        for obj2 in obj_vec2 {
                            // proximity and predict
                            let trans1 = node1.world_transform().unwrap();
                            let trans2 = node2.world_transform().unwrap();
                            let dist = query::distance(
                                &(trans1 * obj1.1),
                                &*obj1.0,
                                &(trans2 * obj2.1),
                                &*obj2.0,
                            );
                            if dist < self.prediction {
                                println!("name: {}, name: {} dist={}", j1, j2, dist);
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
