#ifndef ICP_HPP
#define ICP_HPP

typedef struct
{
    double x;
    double y;
    double z;

    double q_i;
    double q_j;
    double q_k;
    double q_w;
} LioIsometry3d;

typedef struct
{
    float x, y, z, padding;
} LioPoint;

typedef struct
{
    LioIsometry3d transform_target_to_source;
    bool converged;
    int num_iterations;
    int num_inliers;
    double fitness_score;
    double h[6][6]; // information (hessian) matrix denoting the uncertainty of the transformation. High values -> high certainty
    double b[6];    // information vector
} LioRegistrationResult;

void *
init_map(double voxel_resolution);

void add_to_map(void *map, const LioPoint *point_buffer, int num_points, LioIsometry3d global_transform);

void get_map(void *map, LioPoint *point_out_buffer);

int get_map_size(void *map);

LioRegistrationResult align(void *map, const LioPoint *point_buffer, int num_points, LioIsometry3d init_guess);

void drop_map(void *map);
#endif