/*
    FILE: im2_mppi_cuda.h
    --------------------------------
    CUDA back-end for IM2-MPPI rollout + cost evaluation.

    Design:
      * Single device context allocated once at planner construction.
      * Per plan() call: flat host arrays are copied H->D, two kernels
        launched (rollout + cost), per-(rollout, joint-mode) cost matrix
        copied D->H. CPU does the weighted update + visualization.
      * If CUDA is unavailable at build time, isAvailable() returns false
        and the entire GPU code path is compiled out via IM2_MPPI_USE_CUDA.

    All math is FP32 on the device. Host code converts from Eigen::Vector3d
    (double) on transfer; control precision is unaffected.

    NOTE: map_->isInflatedOccupied() is not replicated inside CUDA kernels.
    The planner may download rollout states and add the voxel-map collision
    term on the CPU after the GPU cost pass.
*/

#ifndef IM2_MPPI_CUDA_H
#define IM2_MPPI_CUDA_H

#include <cstdint>

namespace im2mppi {
namespace cuda {

// Opaque device context — declared in im2_mppi_kernels.cu.
struct DeviceContext;

// Compile-time + runtime CUDA availability check.
//   build-time off  : always returns false (kernels not linked)
//   build-time on   : returns true iff cudaGetDeviceCount() >= 1 at runtime
bool isAvailable();

// Allocate persistent device buffers sized for the worst case.
//   N        : max rollouts
//   H        : horizon length
//   J_max    : max dynamic obstacles
//   K_max    : max intent modes per obstacle
//   M_max    : max joint modes
//   Ns_max   : max static boxes
//   P_max    : max reference path points
// Returns nullptr on failure (no CUDA device, allocation error, etc.).
DeviceContext* createContext(int N, int H,
                             int J_max, int K_max, int M_max,
                             int Ns_max, int P_max);

void destroyContext(DeviceContext* ctx);

// ─────────────────────────────────────────────────────────────────────────────
//  Main GPU entry point.
//
//  Inputs (all host-side, row-major flat):
//    x0[6]                         current state (px,py,pz,vx,vy,vz)
//    u_nominal[H*3]                warm-started nominal acceleration sequence
//    noise[N*H*3]                  pre-sampled Gaussian control noise
//    goal[3]                       MPPI terminal goal position
//    ref_targets[H*3]              precomputed reference path target per step
//    static_boxes[Ns*6]            (cx,cy,cz,sx,sy,sz) per static AABB
//    dyn_mus[J*K_per_obs*H*3]      per (obstacle, intent_mode, step) mean
//    dyn_sizes[J*3]                full extents per dynamic obstacle
//    joint_mode_idx[M*J]           intent mode used by each obstacle in mode m
//    Ns, J, K_per_obs, M           counts (use 0 to disable a term)
//
//  Outputs:
//    costs_out[N*M]                cost matrix, row-major [rollout][mode]
//    controls_out[N*H*3]           realized (post-clamp) controls per rollout
//
//  Returns false on any CUDA error.
// ─────────────────────────────────────────────────────────────────────────────
bool runRolloutAndCost(
    DeviceContext* ctx,

    const float* x0,
    const float* u_nominal,
    const float* noise,
    int N, int H,

    const float* goal,
    const float* ref_targets,

    const float* static_boxes, int Ns,

    const float* dyn_mus,  int J, int K_per_obs,
    const float* dyn_sizes,
    const int*   joint_mode_idx, int M,

    float dt, float a_max, float v_max, float d_safe,
    float w_goal, float w_path, float w_vel,
    float w_acc,  float w_jerk, float w_static, float w_dyn,
    int   brake_first_rollout, // 1 = force rollout 0 to brake to zero velocity
    int   skip_dyn_cost,   // 1 = omit deterministic dynamic-obstacle cost

    float* costs_out,
    float* controls_out,
    float* states_out_optional  // [N*(H+1)*6] or nullptr to skip
);

// ─────────────────────────────────────────────────────────────────────────────
//  Per-rollout CVaR over OBSTACLE prediction uncertainty (Phase-4 core).
//
//  For each (rollout i, joint mode m, obstacle j):
//      Sample R obstacle trajectories from N(μ_{m,j,k}, diag(σ²_{m,j,k}))
//      using cuRAND (seed derived from seed_base + thread id).
//      Compute hinge-squared loss per sample, take the mean of the worst
//      α-fraction → ρ[i,m,j].
//  delta_S[i,m] = λ_r · Σ_j ρ[i,m,j], in row-major [N,M] order.
//
//  Reuses the device buffers reserved by createContext (dyn_mus / dyn_sizes /
//  joint_mode_idx already uploaded by runRolloutAndCost). Uploads d_dyn_sigmas
//  per call. Reads ego states from the previously-stored d_states buffer.
//
//  Returns false on any CUDA error.
// ─────────────────────────────────────────────────────────────────────────────
bool runObstacleCVaR(
    DeviceContext* ctx,
    const float* dyn_sigmas,        // [J*K_per_obs*H*3]
    int J, int K_per_obs, int M,
    int N, int H, int R,
    float alpha, float d_safe, float lambda_r,
    unsigned int seed_base,
    float* delta_S_out              // [N*M] — to be ADDED to costs_out
);

bool runDRACollisionRisk(
    DeviceContext* ctx,
    const float* dyn_pis,
    const float* dyn_sigmas,
    int J, int K_per_obs, int M,
    int N, int H, int R,
    float d_safe, float robot_radius, float sigma_floor,
    float cp_lambda, float cp_threshold, float hard_penalty,
    int use_z_probability,
    unsigned int seed_base,
    float* delta_S_out);

} // namespace cuda
} // namespace im2mppi
#endif // IM2_MPPI_CUDA_H
