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
use na::{self, Isometry3, Real, Translation3, UnitQuaternion, Vector3};
use ncollide::ncollide_geometry::query::Proximity;
use ncollide::query;
use ncollide::shape::{Ball, Cuboid3, Cylinder, Shape3, ShapeHandle3, TriMesh, Compound3};
use urdf_rs;
use std::collections::HashMap;
use std::path::Path;

use errors::*;

fn from_urdf_pose<T>(pose: &urdf_rs::Pose) -> Isometry3<T>
where
    T: Real,
{
    na::convert(Isometry3::from_parts(
        Translation3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]),
        UnitQuaternion::from_euler_angles(
            pose.rpy[0],
            pose.rpy[1],
            pose.rpy[2],
        ),
    ))
}

fn load_mesh<P, T>(filename: P, scale: &[f64]) -> Result<TriMesh<na::Point3<T>>>
where
    P: AsRef<Path>,
    T: Real,
{
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename.as_ref().to_str().ok_or(
        "faild to get string from path",
    )?;
    Ok(convert_assimp_scene_to_ncollide_mesh(
        importer.read_file(file_string)?,
        scale,
    ))
}

fn convert_assimp_scene_to_ncollide_mesh<T>(
    scene: assimp::Scene,
    scale: &[f64],
) -> TriMesh<na::Point3<T>>
where
    T: Real,
{
    use std;
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
        indices.extend(mesh.face_iter().filter_map(|f| if f.num_indices == 3 {
            Some(na::Point3::<usize>::new(
                f[0] as usize + last_index,
                f[1] as usize + last_index,
                f[2] as usize + last_index,
            ))
        } else {
            None
        }));
        last_index = vertices.len() as usize;
    }
    TriMesh::new(
        std::sync::Arc::new(vertices),
        std::sync::Arc::new(indices),
        None,
        None,
    )
}

fn create_collision_model<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<ShapeHandle3<T>>
where
    T: Real,
{
    match *collision_geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let cube = Cuboid3::new(Vector3::new(
                na::convert(size[0] * 0.5),
                na::convert(size[1] * 0.5),
                na::convert(size[2] * 0.5),
            ));
            Some(ShapeHandle3::new(cube))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let y_cylinder = Cylinder::new(na::convert(length * 0.5), na::convert(radius));
            Some(ShapeHandle3::new(Compound3::new(vec![
                (
                    na::convert(na::Isometry3::from_parts(
                        na::Translation3::new(0.0, 0.0, 0.0),
                        na::UnitQuaternion::from_euler_angles(1.57, 0.0, 0.0),
                    )),
                    ShapeHandle3::new(y_cylinder)
                ),
            ])))
        }
        urdf_rs::Geometry::Sphere { radius } => {
            Some(ShapeHandle3::new(Ball::new(na::convert(radius))))
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
                Ok(mesh) => Some(ShapeHandle3::new(mesh)),
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
    T: Real,
{
    name_collision_model_map: HashMap<String, Vec<(ShapeHandle3<T>, na::Isometry3<T>)>>,
    /// margin length for collision check
    pub prediction: T,
}

impl<T> CollisionChecker<T>
where
    T: Real,
{
    /// Create CollisionChecker from HashMap
    pub fn new(
        name_collision_model_map: HashMap<String, Vec<(ShapeHandle3<T>, na::Isometry3<T>)>>,
        prediction: T,
    ) -> Self {
        CollisionChecker {
            name_collision_model_map,
            prediction,
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
        for l in &urdf_robot.links {
            let col_pose_vec = l.collision
                .iter()
                .filter_map(|collision| {
                    create_collision_model(&collision.geometry, base_dir).map(
                        |col| (col, from_urdf_pose(&collision.origin)),
                    )
                })
                .collect::<Vec<_>>();
            debug!("name={}, ln={}", l.name, col_pose_vec.len());
            if !col_pose_vec.is_empty() {
                name_collision_model_map.insert(l.name.to_string(), col_pose_vec);
            }
        }
        CollisionChecker {
            name_collision_model_map,
            prediction,
        }
    }
    /// Check if there are any colliding links
    pub fn has_any_colliding<R>(
        &self,
        robot: &R,
        target_shape: &Shape3<T>,
        target_pose: &na::Isometry3<T>,
    ) -> bool
    where
        R: k::LinkContainer<T>,
    {
        !self.colliding_link_names_with_first_return_flag(robot, target_shape, target_pose, true)
            .is_empty()
    }
    /// Returns the names which is colliding with the target shape/pose
    pub fn colliding_link_names<R>(
        &self,
        robot: &R,
        target_shape: &Shape3<T>,
        target_pose: &na::Isometry3<T>,
    ) -> Vec<String>
    where
        R: k::LinkContainer<T>,
    {
        self.colliding_link_names_with_first_return_flag(robot, target_shape, target_pose, false)
    }

    fn colliding_link_names_with_first_return_flag<R>(
        &self,
        robot: &R,
        target_shape: &Shape3<T>,
        target_pose: &na::Isometry3<T>,
        first_return: bool,
    ) -> Vec<String>
    where
        R: k::LinkContainer<T>,
    {
        let mut names = Vec::new();
        for (trans, link_name) in robot.link_transforms().iter().zip(robot.link_names()) {
            match self.name_collision_model_map.get(&link_name) {
                Some(obj_vec) => {
                    for obj in obj_vec {
                        let ctct = query::proximity(
                            &(trans * obj.1),
                            &*obj.0,
                            target_pose,
                            target_shape,
                            self.prediction,
                        );
                        if ctct != Proximity::Disjoint {
                            names.push(link_name);
                            if first_return {
                                return names;
                            } else {
                                break;
                            }
                        }
                    }
                }
                None => {
                    debug!("collision model {} not found", link_name);
                }
            }
        }
        names
    }
}

/// Create `ncollide::shape::Compound3` from URDF file
///
/// The `<link>` elements are used as obstacles. set the origin/geometry of
/// `<visual>` and `<collision>`. You can skip `<inertia>`.
pub fn create_compound_from_urdf<P>(file: P) -> Result<Compound3<f64>>
where
    P: AsRef<Path>,
{
    let urdf_obstacles = urdf_rs::utils::read_urdf_or_xacro(file.as_ref())?;
    Ok(create_compound_from_urdf_robot(&urdf_obstacles))
}

/// Create `ncollide::shape::Compound3` from `urdf_rs::Robot`
pub fn create_compound_from_urdf_robot(urdf_obstacle: &urdf_rs::Robot) -> Compound3<f64> {
    let compound_data = urdf_obstacle
        .links
        .iter()
        .flat_map(|l| {
            l.collision
                .iter()
                .map(|collision| match create_collision_model(
                    &collision.geometry,
                    None,
                ) {
                    Some(col) => Some((from_urdf_pose(&collision.origin), col)),
                    None => None,
                })
                .collect::<Vec<_>>()
        })
        .filter_map(|col_tuple| col_tuple)
        .collect::<Vec<_>>();
    Compound3::new(compound_data)
}
