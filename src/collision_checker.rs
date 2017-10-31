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
use ncollide::shape::{Ball, Compound, Cuboid, Cylinder, Shape, ShapeHandle, TriMesh};
use urdf_rs;
use std::collections::HashMap;
use std::path::Path;
use std;

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
pub fn load_mesh<P, T>(filename: P, scale: &[f64]) -> Result<TriMesh<na::Point3<T>>>
where
    P: AsRef<Path>,
    T: Real,
{
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename
        .as_ref()
        .to_str()
        .ok_or("faild to get string from path")?;
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
pub fn load_mesh<P, T>(_filename: P, _scale: &[f64]) -> Result<TriMesh<na::Point3<T>>>
where
    P: AsRef<Path>,
    T: Real,
{
    Err(Error::from("mesh is not not supported"))
}

pub fn wrap_compound<T, S>(
    shape: S,
    origin: Isometry3<T>,
) -> Compound<na::Point3<T>, na::Isometry3<T>>
where
    T: Real,
    S: Shape<na::Point3<T>, na::Isometry3<T>>,
{
    let mut shapes = Vec::new();
    let handle = ShapeHandle::new(shape);
    shapes.push((origin, handle));
    Compound::new(shapes)
}

pub fn create_collision_model<T>(
    collision: &urdf_rs::Collision,
    base_dir: Option<&Path>,
) -> Option<Compound<na::Point3<T>, na::Isometry3<T>>>
where
    T: Real,
{
    let pose = from_urdf_pose(&collision.origin);
    match collision.geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let cube = Cuboid::new(Vector3::new(
                na::convert(size[0] * 0.5),
                na::convert(size[1] * 0.5),
                na::convert(size[2] * 0.5),
            ));
            Some(wrap_compound(cube, pose))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => Some(wrap_compound(
            Cylinder::new(na::convert(length * 0.5), na::convert(radius)),
            pose,
        )),
        urdf_rs::Geometry::Sphere { radius } => {
            Some(wrap_compound(Ball::new(na::convert(radius)), pose))
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
                Some(wrap_compound(mesh, pose))
            } else {
                None
            }
        }
    }
}


pub struct CollisionChecker<T>
where
    T: Real,
{
    name_collision_model_map: HashMap<String, Compound<na::Point3<T>, na::Isometry3<T>>>,
    pub prediction: T,
}

impl<T> CollisionChecker<T>
where
    T: Real,
{
    pub fn new(urdf_robot: &urdf_rs::Robot, base_dir: Option<&Path>, prediction: T) -> Self {
        let mut collisions = HashMap::new();
        for l in &urdf_robot.links {
            if let Some(col) = create_collision_model(&l.collision, base_dir) {
                collisions.insert(l.name.to_string(), col);
            }
        }
        CollisionChecker {
            name_collision_model_map: collisions,
            prediction: prediction,
        }
    }
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
        for (trans, link_name) in robot
            .calc_link_transforms()
            .iter()
            .zip(robot.get_link_names())
        {
            match self.name_collision_model_map.get(&link_name) {
                Some(obj) => {
                    // TODO: only first shape is supported
                    let ctct = query::proximity(
                        &(trans * obj.shapes()[0].0),
                        &*obj.shapes()[0].1,
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
