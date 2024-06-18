#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

#[macro_use]
#[cfg(test)]
extern crate approx;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use ros_pointcloud2::prelude::*;

pub struct Map {
    c_map: *mut std::os::raw::c_void,
}

impl From<nalgebra::Isometry3<f64>> for LioIsometry3d {
    fn from(iso: nalgebra::Isometry3<f64>) -> Self {
        LioIsometry3d {
            x: iso.translation.vector.x,
            y: iso.translation.vector.y,
            z: iso.translation.vector.z,
            q_i: iso.rotation.quaternion().i,
            q_j: iso.rotation.quaternion().j,
            q_k: iso.rotation.quaternion().k,
            q_w: iso.rotation.quaternion().w,
        }
    }
}

impl From<LioIsometry3d> for nalgebra::Isometry3<f64> {
    fn from(iso: LioIsometry3d) -> Self {
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(iso.x, iso.y, iso.z),
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                iso.q_w, iso.q_i, iso.q_j, iso.q_k,
            )),
        )
    }
}

#[derive(Debug, Clone)]
pub struct RegistrationResult {
    pub transform_target_to_source: nalgebra::Isometry3<f64>,
    pub converged: bool,
    pub num_iterations: i32,
    pub num_inliers: i32,
    pub fitness_score: f64,
    pub h: nalgebra::Matrix6<f64>,
    pub b: nalgebra::Vector6<f64>,
}

impl From<LioRegistrationResult> for RegistrationResult {
    fn from(res: LioRegistrationResult) -> Self {
        RegistrationResult {
            transform_target_to_source: res.transform_target_to_source.into(),
            converged: res.converged,
            num_iterations: res.num_iterations,
            num_inliers: res.num_inliers,
            fitness_score: res.fitness_score,
            h: nalgebra::Matrix6::from_iterator(res.h.iter().flatten().cloned()),
            b: nalgebra::Vector6::from_iterator(res.b.iter().cloned()),
        }
    }
}

impl Map {
    pub fn new(voxel_size: f64) -> Self {
        Map {
            c_map: unsafe { init_map(voxel_size) },
        }
    }

    pub fn reset(&mut self, voxel_size: f64) {
        unsafe {
            drop_map(self.c_map);
            self.c_map = init_map(voxel_size);
        }
    }

    /// Assuming PointCloud2 to be in XYZ format, so can copy directly to C++.
    /// Data gets copied on C++ side, so no need to worry about lifetimes.
    pub fn add(&mut self, cloud: &PointCloud2Msg, pose: Option<nalgebra::Isometry3<f64>>) {
        let pose = pose.unwrap_or(nalgebra::Isometry3::identity());

        unsafe {
            add_to_map(
                self.c_map,
                cloud.data.as_ptr() as *const LioPoint,
                (cloud.dimensions.width * cloud.dimensions.height) as i32,
                pose.into(),
            )
        };
    }

    pub fn register(
        &mut self,
        cloud: &PointCloud2Msg,
        init_guess: &nalgebra::Isometry3<f64>,
    ) -> RegistrationResult {
        let point_buffer = cloud.data.as_ptr() as *const LioPoint;

        unsafe {
            align(
                self.c_map,
                point_buffer,
                (cloud.dimensions.width * cloud.dimensions.height) as i32,
                (*init_guess).into(),
            )
        }
        .into()
    }
}

impl Drop for Map {
    fn drop(&mut self) {
        unsafe {
            drop_map(self.c_map);
        }
    }
}

impl Default for LioPoint {
    fn default() -> Self {
        LioPoint::new(0.0, 0.0, 0.0)
    }
}

impl LioPoint {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        LioPoint {
            x,
            y,
            z,
            padding: 0.0,
        }
    }
}

pub fn transform_cloud(cloud: PointCloud2Msg, pose: &nalgebra::Isometry3<f64>) -> PointCloud2Msg {
    let cloud: ros_pointcloud2::PointCloud2Msg = cloud.into();
    let cloud_header = cloud.header.clone();
    let it = cloud
        .try_into_vec()
        .unwrap()
        .into_iter()
        .map(|p: PointXYZ| {
            let t_p = pose * &nalgebra::Point3::new(p.x as f64, p.y as f64, p.z as f64);
            PointXYZ {
                x: t_p.x as f32,
                y: t_p.y as f32,
                z: t_p.z as f32,
            }
        })
        .collect();

    let mut out = PointCloud2Msg::try_from_vec(it).unwrap();
    out.header = cloud_header;
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::Rng;
    use std::fs::File;
    use std::io::BufRead;
    use std::io::BufReader;

    #[derive(Debug, Default, Clone)]
    pub struct PCDPoint {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    pub fn read_pcl_file(path: String) -> Vec<PCDPoint> {
        let mut data_start = false;
        let mut points = Vec::new();
        let file = File::open(path).unwrap();
        let reader = BufReader::new(file);
        for line in reader.lines() {
            let line = line.unwrap();
            if data_start {
                let mut split = line.split_whitespace();
                let x = split.next().unwrap().parse::<f32>().unwrap();
                let y = split.next().unwrap().parse::<f32>().unwrap();
                let z = split.next().unwrap().parse::<f32>().unwrap();
                points.push(PCDPoint { x, y, z });
            } else if line == "DATA ascii" {
                data_start = true;
            }
        }
        points
    }

    pub fn read_pcl(path: String) -> PointCloud2Msg {
        let points = read_pcl_file(path).into_iter().map(|p| PointXYZI {
            x: p.x,
            y: p.y,
            z: p.z,
            intensity: 0.0,
        });
        PointCloud2Msg::try_from_iter(points).unwrap()
    }

    pub fn generate_random_cloud(num_points: usize, min: f32, max: f32) -> Vec<PointXYZI> {
        let mut rng = rand::thread_rng();
        let mut pointcloud = Vec::with_capacity(num_points);
        for _ in 0..num_points {
            let point = PointXYZI {
                x: rng.gen_range(min..max),
                y: rng.gen_range(min..max),
                z: rng.gen_range(min..max),
                intensity: rng.gen_range(0.0..1.0),
            };
            pointcloud.push(point);
        }
        pointcloud
    }

    pub fn generate_random_pointcloud(num_points: usize, min: f32, max: f32) -> PointCloud2Msg {
        PointCloud2Msg::try_from_vec(generate_random_cloud(num_points, min, max)).unwrap()
    }

    #[test]
    fn degree_rotation() {
        let mut map = Map::new(0.15);

        let target_cloud = read_pcl("test_clouds/test_Q.pcd".to_string());
        let init_pose = nalgebra::Isometry3::identity();
        map.add(&target_cloud, Some(init_pose));

        let src_cloud = read_pcl("test_clouds/test_P.pcd".to_string());
        let registered = map.register(&src_cloud, &init_pose);

        println!(
            "{}",
            registered
                .transform_target_to_source
                .inverse()
                .to_homogeneous()
        );

        let rot_deg = 8.0_f64;
        let tolerance = 1.0_f64;
        let rot = nalgebra::Rotation3::from_euler_angles(0.0, 0.0, rot_deg.to_radians());
        let rot_pose = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 0.0, 0.0),
            nalgebra::UnitQuaternion::from_rotation_matrix(&rot),
        );

        let transformed_map = transform_cloud(target_cloud.clone(), &rot_pose);
        let registered = map.register(&transformed_map, &init_pose);

        assert_relative_eq!(
            registered
                .transform_target_to_source
                .inverse()
                .rotation
                .euler_angles()
                .2
                .to_degrees(),
            rot_deg,
            epsilon = tolerance
        );
    }
}
