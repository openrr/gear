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
#[cfg(feature = "use-assimp")]
use assimp;
use k;
use na::{self, Isometry3, Real, Translation3, UnitQuaternion, Vector3};
use ncollide::ncollide_geometry::query::Proximity;
use ncollide::query;
use ncollide::shape::{Ball, Cuboid, Cylinder, Shape, ShapeHandle, TriMesh};
use urdf_rs;
use std::collections::HashMap;
use std::path::Path;

use errors::*;

fn from_urdf_pose<T>(pose: &urdf_rs::Pose) -> Isometry3<T>
where
    T: Real,
{
    Isometry3::from_parts(
        Translation3::new(
            na::convert(pose.xyz[0]),
            na::convert(pose.xyz[1]),
            na::convert(pose.xyz[2]),
        ),
        UnitQuaternion::from_euler_angles(
            na::convert(pose.rpy[0]),
            na::convert(pose.rpy[1]),
            na::convert(pose.rpy[2]),
        ),
    )
}

#[cfg(feature = "assimp")]
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

#[cfg(feature = "assimp")]
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


#[cfg(not(feature = "assimp"))]
fn load_mesh<P, T>(_filename: P, _scale: &[f64]) -> Result<TriMesh<na::Point3<T>>>
where
    P: AsRef<Path>,
    T: Real,
{
    Err(Error::from("mesh is not not supported"))
}

fn create_collision_model<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<ShapeHandle<na::Point3<T>, na::Isometry3<T>>>
where
    T: Real,
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
        urdf_rs::Geometry::Cylinder { radius, length } => Some(ShapeHandle::new(Cylinder::new(
            na::convert(length * 0.5),
            na::convert(radius),
        ))),
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
            if let Ok(mesh) = load_mesh(path, &scale) {
                Some(ShapeHandle::new(mesh))
            } else {
                None
            }
        }
    }
}

/// Collision checker for a robot
pub struct CollisionChecker<T>
where
    T: Real,
{
    name_collision_model_map:
        HashMap<String, (ShapeHandle<na::Point3<T>, na::Isometry3<T>>, na::Isometry3<T>)>,
    /// margin length for collision check
    pub prediction: T,
}

impl<T> CollisionChecker<T>
where
    T: Real,
{
    /// Create CollisionChecker from HashMap
    pub fn new(
        name_collision_model_map: HashMap<
            String,
            (ShapeHandle<na::Point3<T>, na::Isometry3<T>>,
             na::Isometry3<T>),
        >,
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
            if let Some(col) = create_collision_model(&l.collision.geometry, base_dir) {
                let pose = from_urdf_pose(&l.collision.origin);
                name_collision_model_map.insert(l.name.to_string(), (col, pose));
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
        target_shape: &Shape<na::Point3<T>, na::Isometry3<T>>,
        target_pose: &na::Isometry3<T>,
    ) -> bool
    where
        R: k::LinkContainer<T>,
    {
        !self.get_colliding_link_names_with_first_return_flag(
            robot,
            target_shape,
            target_pose,
            true,
        ).is_empty()
    }
    /// Returns the names which is colliding with the target shape/pose
    pub fn get_colliding_link_names<R>(
        &self,
        robot: &R,
        target_shape: &Shape<na::Point3<T>, na::Isometry3<T>>,
        target_pose: &na::Isometry3<T>,
    ) -> Vec<String>
    where
        R: k::LinkContainer<T>,
    {
        self.get_colliding_link_names_with_first_return_flag(
            robot,
            target_shape,
            target_pose,
            false,
        )
    }

    fn get_colliding_link_names_with_first_return_flag<R>(
        &self,
        robot: &R,
        target_shape: &Shape<na::Point3<T>, na::Isometry3<T>>,
        target_pose: &na::Isometry3<T>,
        first_return: bool,
    ) -> Vec<String>
    where
        R: k::LinkContainer<T>,
    {
        let mut names = Vec::new();
        for (trans, link_name) in
            robot.calc_link_transforms().iter().zip(
                robot.get_link_names(),
            )
        {
            match self.name_collision_model_map.get(&link_name) {
                Some(obj) => {
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
                        }
                    }
                }
                None => {
                    println!("collision model {} not found", link_name);
                }
            }
        }
        names
    }
}
