#include "icp.hpp"

#include "small_gicp/include/small_gicp/registration/registration_helper.hpp"

#include <tbb/tbb.h>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/points/traits.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <iostream>

using namespace small_gicp;

typedef struct
{
    std::shared_ptr<GaussianVoxelMap> voxelmap;
    Eigen::Isometry3d global_transform;
} State;

void *init_map(double voxel_resolution)
{
    auto voxelmap = std::make_shared<GaussianVoxelMap>(voxel_resolution);
    State *state = new State();
    state->voxelmap = voxelmap;
    return state;
}

void add_to_map(void *map, const LioPoint *point_buffer, int num_points, LioIsometry3d global_transform)
{
    State *state = reinterpret_cast<State *>(map);
    auto voxelmap = state->voxelmap;

    int num_neighbors = 10;

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() << global_transform.x, global_transform.y, global_transform.z;
    Eigen::Quaterniond q(global_transform.q_w, global_transform.q_i, global_transform.q_j, global_transform.q_k);
    transform.rotate(q);

    PointCloud::Ptr cloud = std::make_shared<PointCloud>();
    cloud->resize(num_points);
    for (int i = 0; i < num_points; i++)
    {
        cloud->point(i) << point_buffer[i].x, point_buffer[i].y, point_buffer[i].z, 1.0;
    }

    state->voxelmap->insert(*cloud, transform);
}

// translation: xyz
// rotation: quaternion xyzw
LioRegistrationResult align(void *map, const LioPoint *point_buffer, int num_points, LioIsometry3d init_guess)
{
    State *state = reinterpret_cast<State *>(map);
    auto voxelmap = state->voxelmap;

    int num_neighbors = 20;

    PointCloud::Ptr cloud = std::make_shared<PointCloud>();
    cloud->resize(num_points);
    for (int i = 0; i < num_points; i++)
    {
        cloud->point(i) << point_buffer[i].x, point_buffer[i].y, point_buffer[i].z, 1.0;
    }

    estimate_covariances_tbb(*cloud, num_neighbors);

    Eigen::Isometry3d init_guess_eigen = Eigen::Isometry3d::Identity();
    init_guess_eigen.translation() << init_guess.x, init_guess.y, init_guess.z;
    Eigen::Quaterniond q(init_guess.q_w, init_guess.q_i, init_guess.q_j, init_guess.q_k);
    init_guess_eigen.rotate(q);

    Registration<GICPFactor, ParallelReductionTBB> registration;
    // align source to target cloud, so target is map, source is new cloud
    RegistrationResult result = registration.align(*state->voxelmap, *cloud, *state->voxelmap, init_guess_eigen);

    LioRegistrationResult out;

    out.transform_target_to_source.x = result.T_target_source.translation().x();
    out.transform_target_to_source.y = result.T_target_source.translation().y();
    out.transform_target_to_source.z = result.T_target_source.translation().z();

    Eigen::Quaterniond q2(result.T_target_source.rotation());
    out.transform_target_to_source.q_i = q2.x();
    out.transform_target_to_source.q_j = q2.y();
    out.transform_target_to_source.q_k = q2.z();
    out.transform_target_to_source.q_w = q2.w();

    out.converged = result.converged;
    out.num_iterations = result.iterations;
    out.num_inliers = result.num_inliers;
    out.fitness_score = result.error;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            out.h[i][j] = result.H(i, j);
        }
        out.b[i] = result.b(i, 0);
    }

    return out;
}

int get_map_size(void *map)
{
    State *state = reinterpret_cast<State *>(map);
    auto voxelmap = state->voxelmap;

    return traits::size(*voxelmap);
}

// assuming point_out_buffer has is big enough, use get_map_size before
void get_map(void *map, LioPoint *point_out_buffer)
{
    State *state = reinterpret_cast<State *>(map);
    auto voxelmap = state->voxelmap;

    int cloud_size = traits::size(*voxelmap);
    for (int i = 0; i < cloud_size; i++)
    {
        Eigen::Vector4d point = traits::point(*voxelmap, i);
        LioPoint p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        point_out_buffer[i] = p;
    }
}

void drop_map(void *map)
{
    State *state = reinterpret_cast<State *>(map);
    delete state;
    state = nullptr;
}
