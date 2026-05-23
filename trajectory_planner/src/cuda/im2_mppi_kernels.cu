/*
    FILE: im2_mppi_kernels.cu
    --------------------------------
    Two CUDA kernels for IM2-MPPI:

      rolloutKernel : one thread per rollout i; integrates 3-D double-integrator
                      dynamics for H steps with control noise + acc/vel clamps.

      costKernel    : one thread per (rollout i, joint-mode m); evaluates
                      goal + path + smoothness + static-box + dynamic-box AABB
                      penalties against the rollout's state sequence.

    All math in FP32. AABB collision uses the same signed-distance formula as
    the CPU planner (aabbSDF in im2_mppi_planner.cpp) so results match within
    floating-point tolerance.
*/

#include <trajectory_planner/im2_mppi_cuda.h>

#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace im2mppi {
namespace cuda {

// ═══════════════════════════════════════════════════════════════════════════
//  Device context
// ═══════════════════════════════════════════════════════════════════════════

struct DeviceContext {
    // Capacity bounds (set at createContext)
    int N_cap, H_cap, J_cap, K_cap, M_cap, Ns_cap, P_cap;

    // Persistent device buffers (sized to the worst case)
    float* d_x0           = nullptr;   // [6]
    float* d_u_nominal    = nullptr;   // [H*3]
    float* d_noise        = nullptr;   // [N*H*3]
    float* d_states       = nullptr;   // [N*(H+1)*6]
    float* d_controls     = nullptr;   // [N*H*3]
    float* d_costs        = nullptr;   // [N*M]
    float* d_goal         = nullptr;   // [3]
    float* d_ref_targets  = nullptr;   // [H*3]
    float* d_static_boxes = nullptr;   // [Ns*6]
    float* d_dyn_mus      = nullptr;   // [J*K*H*3]
    float* d_dyn_sigmas   = nullptr;   // [J*K*H*3]  (Phase-4 CVaR)
    float* d_dyn_pis      = nullptr;   // [J*K]       (DRA-MPPI MoG weights)
    float* d_dyn_sizes    = nullptr;   // [J*3]
    int*   d_joint_mode_idx = nullptr; // [M*J]
    float* d_delta_S      = nullptr;   // [N*M]      (Phase-4 CVaR output)
};

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

#define CUDA_CHECK(call)                                                       \
    do {                                                                        \
        cudaError_t err = (call);                                               \
        if (err != cudaSuccess) {                                               \
            std::fprintf(stderr, "[IM2-MPPI/CUDA] %s:%d  %s -> %s\n",           \
                         __FILE__, __LINE__, #call, cudaGetErrorString(err));   \
            return false;                                                       \
        }                                                                        \
    } while (0)

static inline bool cudaAlloc(float** ptr, size_t n_floats) {
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(ptr), n_floats * sizeof(float)));
    return true;
}

static inline bool cudaAllocInt(int** ptr, size_t n_ints) {
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(ptr), n_ints * sizeof(int)));
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Rollout kernel
//      One thread per rollout. Integrates 3-D double-integrator dynamics:
//          p_{k+1} = p_k + v_k dt + 0.5 a_k dt^2
//          v_{k+1} = v_k + a_k dt
//      with ||a|| <= a_max and ||v|| <= v_max clamps. Writes the realized
//      (post-clamp) control back to d_controls.
// ═══════════════════════════════════════════════════════════════════════════

__global__ void rolloutKernel(
    const float* __restrict__ x0,
    const float* __restrict__ u_nominal,
    const float* __restrict__ noise,
    float*       __restrict__ states,
    float*       __restrict__ controls,
    int N, int H, float dt,
    float a_max, float v_max,
    int brake_first_rollout)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float px = x0[0], py = x0[1], pz = x0[2];
    float vx = x0[3], vy = x0[4], vz = x0[5];

    // Initial state
    const int s_stride = (H + 1) * 6;
    const int c_stride = H * 3;
    int sbase = i * s_stride;
    states[sbase + 0] = px; states[sbase + 1] = py; states[sbase + 2] = pz;
    states[sbase + 3] = vx; states[sbase + 4] = vy; states[sbase + 5] = vz;

    const float dt2 = 0.5f * dt * dt;

    for (int k = 0; k < H; ++k) {
        float ax, ay, az;
        if (brake_first_rollout && i == 0) {
            const float inv_dt = 1.0f / fmaxf(dt, 1e-6f);
            ax = -vx * inv_dt;
            ay = -vy * inv_dt;
            az = -vz * inv_dt;
        } else {
            // u = u_nominal[k] + noise[i, k]
            ax = u_nominal[k * 3 + 0] + noise[i * c_stride + k * 3 + 0];
            ay = u_nominal[k * 3 + 1] + noise[i * c_stride + k * 3 + 1];
            az = u_nominal[k * 3 + 2] + noise[i * c_stride + k * 3 + 2];
        }

        // Clamp ||a|| <= a_max
        float an = sqrtf(ax * ax + ay * ay + az * az);
        if (an > a_max) {
            float s = a_max / an;
            ax *= s; ay *= s; az *= s;
        }

        // Store realized control
        const int coff = i * c_stride + k * 3;
        controls[coff + 0] = ax;
        controls[coff + 1] = ay;
        controls[coff + 2] = az;

        // Propagate
        px += vx * dt + ax * dt2;
        py += vy * dt + ay * dt2;
        pz += vz * dt + az * dt2;
        vx += ax * dt;
        vy += ay * dt;
        vz += az * dt;

        // Clamp ||v|| <= v_max
        float vn = sqrtf(vx * vx + vy * vy + vz * vz);
        if (vn > v_max) {
            float s = v_max / vn;
            vx *= s; vy *= s; vz *= s;
        }

        // Store next state
        const int soff = i * s_stride + (k + 1) * 6;
        states[soff + 0] = px; states[soff + 1] = py; states[soff + 2] = pz;
        states[soff + 3] = vx; states[soff + 4] = vy; states[soff + 5] = vz;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  AABB signed-distance (device version)
//      Identical formula to aabbSDF() in im2_mppi_planner.cpp.
// ═══════════════════════════════════════════════════════════════════════════

__device__ inline float aabbSDFDev(
    float px, float py, float pz,
    float cx, float cy, float cz,
    float sx, float sy, float sz)
{
    const float hx = 0.5f * fmaxf(sx, 1e-6f);
    const float hy = 0.5f * fmaxf(sy, 1e-6f);
    const float hz = 0.5f * fmaxf(sz, 1e-6f);
    const float qx = fabsf(px - cx) - hx;
    const float qy = fabsf(py - cy) - hy;
    const float qz = fabsf(pz - cz) - hz;
    const float ox = fmaxf(qx, 0.0f);
    const float oy = fmaxf(qy, 0.0f);
    const float oz = fmaxf(qz, 0.0f);
    const float outside = sqrtf(ox * ox + oy * oy + oz * oz);
    const float inside  = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
    return outside + inside;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Cost kernel
//      One thread per (rollout i, joint-mode m). All cost terms accumulated
//      directly into the cost matrix entry costs[i*M + m].
// ═══════════════════════════════════════════════════════════════════════════

__global__ void costKernel(
    const float* __restrict__ states,        // [N*(H+1)*6]
    const float* __restrict__ controls,      // [N*H*3]
    const float* __restrict__ goal,          // [3]
    const float* __restrict__ ref_targets,   // [H*3]
    const float* __restrict__ static_boxes,  // [Ns*6]
    int Ns,
    const float* __restrict__ dyn_mus,       // [J*K*H*3]
    const float* __restrict__ dyn_sizes,     // [J*3]
    int J, int K_per_obs,
    const int*   __restrict__ joint_mode_idx,// [M*J]
    int M,
    float* __restrict__ costs,               // [N*M]
    int N, int H,
    float d_safe,
    float w_goal, float w_path, float w_vel,
    float w_acc,  float w_jerk, float w_static, float w_dyn,
    int   skip_dyn_cost)   // 1 = omit deterministic dynamic-obstacle term
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = N * M;
    if (idx >= total) return;

    const int i = idx / M;
    const int m = idx % M;

    const int s_stride = (H + 1) * 6;
    const int c_stride = H * 3;

    float c = 0.0f;

    // ── Terminal goal cost ────────────────────────────────────────────────
    {
        const int off = i * s_stride + H * 6;
        const float ex = states[off + 0] - goal[0];
        const float ey = states[off + 1] - goal[1];
        const float ez = states[off + 2] - goal[2];
        c += w_goal * (ex * ex + ey * ey + ez * ez);
    }

    // ── Per-step running costs ────────────────────────────────────────────
    float ax_prev = 0.0f, ay_prev = 0.0f, az_prev = 0.0f;

    for (int k = 0; k < H; ++k) {
        const int soff = i * s_stride + (k + 1) * 6;
        const float px = states[soff + 0];
        const float py = states[soff + 1];
        const float pz = states[soff + 2];
        const float vx = states[soff + 3];
        const float vy = states[soff + 4];
        const float vz = states[soff + 5];

        const int coff = i * c_stride + k * 3;
        const float ax = controls[coff + 0];
        const float ay = controls[coff + 1];
        const float az = controls[coff + 2];

        // Path tracking
        {
            const float ex = px - ref_targets[k * 3 + 0];
            const float ey = py - ref_targets[k * 3 + 1];
            const float ez = pz - ref_targets[k * 3 + 2];
            c += w_path * (ex * ex + ey * ey + ez * ez);
        }

        // Smoothness
        c += w_vel * (vx * vx + vy * vy + vz * vz);
        c += w_acc * (ax * ax + ay * ay + az * az);
        if (k > 0) {
            const float dax = ax - ax_prev;
            const float day = ay - ay_prev;
            const float daz = az - az_prev;
            c += w_jerk * (dax * dax + day * day + daz * daz);
        }
        ax_prev = ax; ay_prev = ay; az_prev = az;

        // Static AABB obstacles (soft hinge-squared)
        for (int s = 0; s < Ns; ++s) {
            const int boff = s * 6;
            const float clr = aabbSDFDev(
                px, py, pz,
                static_boxes[boff + 0], static_boxes[boff + 1], static_boxes[boff + 2],
                static_boxes[boff + 3], static_boxes[boff + 4], static_boxes[boff + 5]);
            if (clr < d_safe) {
                const float pen = d_safe - clr;
                c += w_static * pen * pen;
            }
        }

        // Dynamic AABB obstacles, per joint-mode m
        if (J > 0 && !skip_dyn_cost) {
            for (int j = 0; j < J; ++j) {
                const int mode_idx = joint_mode_idx[m * J + j];
                if (mode_idx < 0 || mode_idx >= K_per_obs) continue;

                // dyn_mus indexed as [j, mode_idx, k, xyz]
                const int mu_off = ((j * K_per_obs + mode_idx) * H + k) * 3;
                const float mx = dyn_mus[mu_off + 0];
                const float my = dyn_mus[mu_off + 1];
                const float mz = dyn_mus[mu_off + 2];

                const int sz_off = j * 3;
                const float clr = aabbSDFDev(
                    px, py, pz,
                    mx, my, mz,
                    dyn_sizes[sz_off + 0],
                    dyn_sizes[sz_off + 1],
                    dyn_sizes[sz_off + 2]);
                if (clr < d_safe) {
                    const float pen = d_safe - clr;
                    c += w_dyn * pen * pen;
                }
            }
        }
    }

    costs[idx] = c;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Per-rollout CVaR over OBSTACLE prediction uncertainty (Phase-4 core)
//      One thread per (rollout i, joint mode m). Each thread iterates over
//      J obstacles, draws R Monte-Carlo trajectory samples per obstacle from
//      N(μ, diag(σ²)), computes the worst-step hinge-squared loss per sample,
//      and reduces to the mean of the top ⌈α R⌉ losses (CVaR_α tail mean).
//      The sum over j (× λ_r) is written to delta_S[i, m].
//
//      R is capped at 32 to fit losses[] in registers / stack.
// ═══════════════════════════════════════════════════════════════════════════

#ifndef IM2_MPPI_CVAR_MAX_R
#define IM2_MPPI_CVAR_MAX_R 32
#endif

__global__ void obstacleCVaRKernel(
    const float* __restrict__ states,        // [N*(H+1)*6]
    const float* __restrict__ dyn_mus,       // [J*K*H*3]
    const float* __restrict__ dyn_sigmas,    // [J*K*H*3]
    const float* __restrict__ dyn_sizes,     // [J*3]
    const int*   __restrict__ joint_mode_idx,// [M*J]
    int N, int M, int J, int K, int H, int R,
    float alpha, float d_safe, float lambda_r,
    unsigned int seed_base,
    float* __restrict__ delta_S_out)         // [N*M]
{
    const int idx   = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = N * M;
    if (idx >= total) return;

    const int i = idx / M;
    const int m = idx % M;

    const int s_stride = (H + 1) * 6;
    const int k_tail   = max(1, (int)ceilf(alpha * (float)R));

    float losses[IM2_MPPI_CVAR_MAX_R];
    float sum_rho = 0.0f;

    for (int j = 0; j < J; ++j) {
        const int m_j = joint_mode_idx[m * J + j];
        if (m_j < 0 || m_j >= K) continue;

        // Per-obstacle box half-extents (Chebyshev SDF inputs)
        const int sz_off = j * 3;
        const float hx = 0.5f * fmaxf(dyn_sizes[sz_off + 0], 1e-6f);
        const float hy = 0.5f * fmaxf(dyn_sizes[sz_off + 1], 1e-6f);
        const float hz = 0.5f * fmaxf(dyn_sizes[sz_off + 2], 1e-6f);

        for (int r = 0; r < R; ++r) {
            // Deterministic per-thread RNG seed: depends on (i, m, j, r).
            curandStatePhilox4_32_10_t st;
            const unsigned int sub = (unsigned int)(((i * M + m) * J + j) * R + r);
            curand_init(seed_base, sub, 0, &st);

            float min_signed = INFINITY;

            for (int k = 0; k < H; ++k) {
                const int mu_off = ((j * K + m_j) * H + k) * 3;

                // Sample obstacle position from N(μ, diag(σ²)).
                // curand_normal4 returns 4 standard normals; we use 3.
                const float4 z = curand_normal4(&st);
                const float ox = dyn_mus[mu_off + 0] + dyn_sigmas[mu_off + 0] * z.x;
                const float oy = dyn_mus[mu_off + 1] + dyn_sigmas[mu_off + 1] * z.y;
                const float oz = dyn_mus[mu_off + 2] + dyn_sigmas[mu_off + 2] * z.z;

                // Ego position at step k+1 (skip the initial state).
                const int eo = i * s_stride + (k + 1) * 6;
                const float ex = states[eo + 0];
                const float ey = states[eo + 1];
                const float ez = states[eo + 2];

                // AABB signed clearance (matches aabbSDFDev).
                const float qx = fabsf(ex - ox) - hx;
                const float qy = fabsf(ey - oy) - hy;
                const float qz = fabsf(ez - oz) - hz;
                const float outside = sqrtf(fmaxf(qx, 0.0f) * fmaxf(qx, 0.0f)
                                          + fmaxf(qy, 0.0f) * fmaxf(qy, 0.0f)
                                          + fmaxf(qz, 0.0f) * fmaxf(qz, 0.0f));
                const float inside  = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
                const float clr     = outside + inside;
                if (clr < min_signed) min_signed = clr;
            }
            const float hinge = fmaxf(d_safe - min_signed, 0.0f);
            losses[r] = hinge * hinge;
        }

        // Partial selection sort: bring the k_tail largest values to the
        // beginning. R ≤ 32 so the O(k_tail · R) cost is negligible.
        for (int t = 0; t < k_tail; ++t) {
            int max_idx = t;
            float max_val = losses[t];
            for (int s = t + 1; s < R; ++s) {
                if (losses[s] > max_val) { max_val = losses[s]; max_idx = s; }
            }
            if (max_idx != t) {
                float tmp = losses[t];
                losses[t] = losses[max_idx];
                losses[max_idx] = tmp;
            }
        }

        float tail_sum = 0.0f;
        for (int t = 0; t < k_tail; ++t) tail_sum += losses[t];
        sum_rho += tail_sum / (float)k_tail;
    }

    delta_S_out[idx] = lambda_r * sum_rho;
}

__device__ inline float clamp01Dev(float x)
{
    return fminf(1.0f, fmaxf(0.0f, x));
}

__device__ inline float normalCdfDev(float z)
{
    return 0.5f * (1.0f + erff(z * 0.7071067811865475f));
}

__device__ inline float gaussianIntervalProbabilityDev(
    float center, float half_width, float mu, float sigma)
{
    sigma = fmaxf(1e-4f, sigma);
    const float lo = (center - half_width - mu) / sigma;
    const float hi = (center + half_width - mu) / sigma;
    return clamp01Dev(normalCdfDev(hi) - normalCdfDev(lo));
}

__device__ inline float gaussianDiskProbabilityApprox2DDev(
    float cx, float cy, float mx, float my, float sx, float sy, float radius)
{
    // Equal-area square approximation of a circular collision region.
    // This is the light DRA path: O(N*H*J*K), no per-rollout MC loop.
    const float half_width = 0.8862269254527580f * fmaxf(0.0f, radius);
    const float px = gaussianIntervalProbabilityDev(cx, half_width, mx, sx);
    const float py = gaussianIntervalProbabilityDev(cy, half_width, my, sy);
    return clamp01Dev(px * py);
}

__global__ void draCollisionRiskKernel(
    const float* __restrict__ states,
    const float* __restrict__ dyn_mus,
    const float* __restrict__ dyn_sigmas,
    const float* __restrict__ dyn_pis,
    const float* __restrict__ dyn_sizes,
    const int*   __restrict__ joint_mode_idx,
    int N, int M, int J, int K, int H, int R,
    float d_safe, float robot_radius, float sigma_floor,
    float cp_lambda, float cp_threshold, float hard_penalty,
    int use_z_probability,
    unsigned int seed_base,
    float* __restrict__ delta_S_out)
{
    (void)R;
    (void)seed_base;
    (void)joint_mode_idx;
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = N * M;
    if (idx >= total) return;

    const int i = idx / M;
    const int s_stride = (H + 1) * 6;

    float risk_cost = 0.0f;

    for (int k = 1; k <= H; ++k) {
        const int eo = i * s_stride + k * 6;
        const float ex = states[eo + 0];
        const float ey = states[eo + 1];
        const float ez = states[eo + 2];

        float no_collision_step = 1.0f;

        for (int j = 0; j < J; ++j) {
            float pi_sum = 0.0f;
            for (int mode_idx = 0; mode_idx < K; ++mode_idx) {
                pi_sum += fmaxf(0.0f, dyn_pis[j * K + mode_idx]);
            }
            if (pi_sum <= 1e-8f) continue;

            const int sz_off = j * 3;
            const float obs_x = fmaxf(0.0f, dyn_sizes[sz_off + 0]);
            const float obs_y = fmaxf(0.0f, dyn_sizes[sz_off + 1]);
            const float obs_radius = 0.5f * sqrtf(obs_x * obs_x + obs_y * obs_y);
            const float collision_radius = fmaxf(1e-4f, robot_radius + obs_radius + d_safe);

            float cp_j = 0.0f;
            for (int mode_idx = 0; mode_idx < K; ++mode_idx) {
                const float weight = fmaxf(0.0f, dyn_pis[j * K + mode_idx]) / pi_sum;
                if (weight <= 0.0f) continue;

                const int mu_off = ((j * K + mode_idx) * H + (k - 1)) * 3;
                const float mx = dyn_mus[mu_off + 0];
                const float my = dyn_mus[mu_off + 1];
                const float mz = dyn_mus[mu_off + 2];
                const float sx = fmaxf(sigma_floor, dyn_sigmas[mu_off + 0]);
                const float sy = fmaxf(sigma_floor, dyn_sigmas[mu_off + 1]);
                const float sz = fmaxf(sigma_floor, dyn_sigmas[mu_off + 2]);

                float p_mode = gaussianDiskProbabilityApprox2DDev(
                    ex, ey, mx, my, sx, sy, collision_radius);
                if (use_z_probability) {
                    const float z_half = 0.5f * fmaxf(0.0f, dyn_sizes[sz_off + 2]) + d_safe;
                    p_mode *= gaussianIntervalProbabilityDev(ez, z_half, mz, sz);
                }
                cp_j += weight * p_mode;
            }

            cp_j = clamp01Dev(cp_j);
            no_collision_step *= (1.0f - cp_j);
        }

        const float cp_step = clamp01Dev(1.0f - no_collision_step);
        risk_cost += cp_lambda * cp_step;
        if (cp_step > cp_threshold) risk_cost += hard_penalty;
    }

    delta_S_out[idx] = risk_cost;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════

bool isAvailable()
{
    int n = 0;
    cudaError_t err = cudaGetDeviceCount(&n);
    return (err == cudaSuccess && n >= 1);
}

DeviceContext* createContext(int N, int H,
                             int J_max, int K_max, int M_max,
                             int Ns_max, int P_max)
{
    if (!isAvailable()) {
        std::fprintf(stderr, "[IM2-MPPI/CUDA] No CUDA device available.\n");
        return nullptr;
    }
    DeviceContext* ctx = new DeviceContext();
    ctx->N_cap  = N;
    ctx->H_cap  = H;
    ctx->J_cap  = J_max;
    ctx->K_cap  = K_max;
    ctx->M_cap  = M_max;
    ctx->Ns_cap = Ns_max;
    ctx->P_cap  = P_max;

    auto fail = [&](const char* what) {
        std::fprintf(stderr, "[IM2-MPPI/CUDA] Alloc failed: %s\n", what);
        destroyContext(ctx);
        return nullptr;
    };

    if (!cudaAlloc(&ctx->d_x0,           6))                     return fail("x0");
    if (!cudaAlloc(&ctx->d_u_nominal,    static_cast<size_t>(H) * 3)) return fail("u_nominal");
    if (!cudaAlloc(&ctx->d_noise,        static_cast<size_t>(N) * H * 3)) return fail("noise");
    if (!cudaAlloc(&ctx->d_states,       static_cast<size_t>(N) * (H + 1) * 6)) return fail("states");
    if (!cudaAlloc(&ctx->d_controls,     static_cast<size_t>(N) * H * 3)) return fail("controls");
    if (!cudaAlloc(&ctx->d_costs,        static_cast<size_t>(N) * M_max)) return fail("costs");
    if (!cudaAlloc(&ctx->d_goal,         3))                     return fail("goal");
    if (!cudaAlloc(&ctx->d_ref_targets,  static_cast<size_t>(H) * 3)) return fail("ref_targets");
    if (Ns_max > 0 &&
        !cudaAlloc(&ctx->d_static_boxes, static_cast<size_t>(Ns_max) * 6)) return fail("static_boxes");
    if (J_max > 0) {
        if (!cudaAlloc(&ctx->d_dyn_mus,    static_cast<size_t>(J_max) * K_max * H * 3)) return fail("dyn_mus");
        if (!cudaAlloc(&ctx->d_dyn_sigmas, static_cast<size_t>(J_max) * K_max * H * 3)) return fail("dyn_sigmas");
        if (!cudaAlloc(&ctx->d_dyn_pis,    static_cast<size_t>(J_max) * K_max)) return fail("dyn_pis");
        if (!cudaAlloc(&ctx->d_dyn_sizes,  static_cast<size_t>(J_max) * 3)) return fail("dyn_sizes");
        if (!cudaAllocInt(&ctx->d_joint_mode_idx,
                          static_cast<size_t>(M_max) * J_max)) return fail("joint_mode_idx");
    }
    // delta_S buffer for CVaR output (always sized to N × M_max).
    if (!cudaAlloc(&ctx->d_delta_S, static_cast<size_t>(N) * M_max)) return fail("delta_S");
    return ctx;
}

void destroyContext(DeviceContext* ctx)
{
    if (!ctx) return;
    cudaFree(ctx->d_x0);
    cudaFree(ctx->d_u_nominal);
    cudaFree(ctx->d_noise);
    cudaFree(ctx->d_states);
    cudaFree(ctx->d_controls);
    cudaFree(ctx->d_costs);
    cudaFree(ctx->d_goal);
    cudaFree(ctx->d_ref_targets);
    cudaFree(ctx->d_static_boxes);
    cudaFree(ctx->d_dyn_mus);
    cudaFree(ctx->d_dyn_sigmas);
    cudaFree(ctx->d_dyn_pis);
    cudaFree(ctx->d_dyn_sizes);
    cudaFree(ctx->d_joint_mode_idx);
    cudaFree(ctx->d_delta_S);
    delete ctx;
}

bool runRolloutAndCost(
    DeviceContext* ctx,
    const float* x0, const float* u_nominal, const float* noise,
    int N, int H,
    const float* goal, const float* ref_targets,
    const float* static_boxes, int Ns,
    const float* dyn_mus, int J, int K_per_obs,
    const float* dyn_sizes,
    const int*   joint_mode_idx, int M,
    float dt, float a_max, float v_max, float d_safe,
    float w_goal, float w_path, float w_vel,
    float w_acc,  float w_jerk, float w_static, float w_dyn,
    int   brake_first_rollout,
    int   skip_dyn_cost,
    float* costs_out, float* controls_out, float* states_out_optional)
{
    if (!ctx) return false;
    if (N > ctx->N_cap || H > ctx->H_cap || J > ctx->J_cap ||
        M > ctx->M_cap || Ns > ctx->Ns_cap || K_per_obs > ctx->K_cap) {
        std::fprintf(stderr,
            "[IM2-MPPI/CUDA] Request exceeds context capacity: "
            "N=%d/%d H=%d/%d J=%d/%d M=%d/%d Ns=%d/%d K=%d/%d\n",
            N, ctx->N_cap, H, ctx->H_cap, J, ctx->J_cap,
            M, ctx->M_cap, Ns, ctx->Ns_cap, K_per_obs, ctx->K_cap);
        return false;
    }

    // ── H -> D copies ────────────────────────────────────────────────────
    CUDA_CHECK(cudaMemcpy(ctx->d_x0,          x0,        6 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx->d_u_nominal,   u_nominal, H * 3 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx->d_noise,       noise,     N * H * 3 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx->d_goal,        goal,      3 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx->d_ref_targets, ref_targets, H * 3 * sizeof(float), cudaMemcpyHostToDevice));
    if (Ns > 0) {
        CUDA_CHECK(cudaMemcpy(ctx->d_static_boxes, static_boxes,
                              Ns * 6 * sizeof(float), cudaMemcpyHostToDevice));
    }
    if (J > 0) {
        CUDA_CHECK(cudaMemcpy(ctx->d_dyn_mus, dyn_mus,
                              static_cast<size_t>(J) * K_per_obs * H * 3 * sizeof(float),
                              cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(ctx->d_dyn_sizes, dyn_sizes,
                              J * 3 * sizeof(float), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(ctx->d_joint_mode_idx, joint_mode_idx,
                              M * J * sizeof(int), cudaMemcpyHostToDevice));
    }

    // ── Launch rollout kernel ────────────────────────────────────────────
    {
        const int threads = 128;
        const int blocks  = (N + threads - 1) / threads;
        rolloutKernel<<<blocks, threads>>>(
            ctx->d_x0, ctx->d_u_nominal, ctx->d_noise,
            ctx->d_states, ctx->d_controls,
            N, H, dt, a_max, v_max, brake_first_rollout);
        CUDA_CHECK(cudaGetLastError());
    }

    // ── Launch cost kernel ───────────────────────────────────────────────
    {
        const int total = N * M;
        const int threads = 128;
        const int blocks  = (total + threads - 1) / threads;
        costKernel<<<blocks, threads>>>(
            ctx->d_states, ctx->d_controls,
            ctx->d_goal, ctx->d_ref_targets,
            ctx->d_static_boxes, Ns,
            ctx->d_dyn_mus, ctx->d_dyn_sizes,
            J, K_per_obs,
            ctx->d_joint_mode_idx, M,
            ctx->d_costs,
            N, H,
            d_safe,
            w_goal, w_path, w_vel, w_acc, w_jerk, w_static, w_dyn,
            skip_dyn_cost);
        CUDA_CHECK(cudaGetLastError());
    }

    // ── D -> H copies ────────────────────────────────────────────────────
    CUDA_CHECK(cudaMemcpy(costs_out,    ctx->d_costs,
                          N * M * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(controls_out, ctx->d_controls,
                          N * H * 3 * sizeof(float), cudaMemcpyDeviceToHost));
    if (states_out_optional) {
        CUDA_CHECK(cudaMemcpy(states_out_optional, ctx->d_states,
                              N * (H + 1) * 6 * sizeof(float), cudaMemcpyDeviceToHost));
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API: per-rollout CVaR over obstacle uncertainty.
// ─────────────────────────────────────────────────────────────────────────────

bool runObstacleCVaR(
    DeviceContext* ctx,
    const float* dyn_sigmas,
    int J, int K_per_obs, int M,
    int N, int H, int R,
    float alpha, float d_safe, float lambda_r,
    unsigned int seed_base,
    float* delta_S_out)
{
    if (!ctx)        return false;
    if (J == 0 || M == 0 || N == 0 || R == 0) {
        // Nothing to do — fill output with zeros so the caller can add safely.
        if (delta_S_out) {
            for (int i = 0; i < N * M; ++i) delta_S_out[i] = 0.0f;
        }
        return true;
    }
    if (R > 32) {
        std::fprintf(stderr, "[IM2-MPPI/CUDA] runObstacleCVaR: R=%d > 32 not supported.\n", R);
        return false;
    }
    if (N > ctx->N_cap || H > ctx->H_cap || J > ctx->J_cap ||
        M > ctx->M_cap || K_per_obs > ctx->K_cap) {
        std::fprintf(stderr, "[IM2-MPPI/CUDA] runObstacleCVaR: exceeds context capacity.\n");
        return false;
    }

    // Upload sigma (mu, sizes, joint_mode_idx were uploaded by runRolloutAndCost).
    CUDA_CHECK(cudaMemcpy(ctx->d_dyn_sigmas, dyn_sigmas,
                          static_cast<size_t>(J) * K_per_obs * H * 3 * sizeof(float),
                          cudaMemcpyHostToDevice));

    const int total   = N * M;
    const int threads = 128;
    const int blocks  = (total + threads - 1) / threads;
    obstacleCVaRKernel<<<blocks, threads>>>(
        ctx->d_states, ctx->d_dyn_mus, ctx->d_dyn_sigmas, ctx->d_dyn_sizes,
        ctx->d_joint_mode_idx,
        N, M, J, K_per_obs, H, R,
        alpha, d_safe, lambda_r,
        seed_base,
        ctx->d_delta_S);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaMemcpy(delta_S_out, ctx->d_delta_S,
                          N * M * sizeof(float), cudaMemcpyDeviceToHost));
    return true;
}

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
    float* delta_S_out)
{
    if (!ctx) return false;
    (void)R;
    // R is a legacy parameter from the old Monte Carlo implementation. The
    // current light DRA risk uses analytic interval probabilities.
    if (J == 0 || M == 0 || N == 0) {
        if (delta_S_out) {
            for (int i = 0; i < N * M; ++i) delta_S_out[i] = 0.0f;
        }
        return true;
    }
    if (N > ctx->N_cap || H > ctx->H_cap || J > ctx->J_cap ||
        M > ctx->M_cap || K_per_obs > ctx->K_cap) {
        std::fprintf(stderr, "[IM2-MPPI/CUDA] runDRACollisionRisk: exceeds context capacity.\n");
        return false;
    }

    CUDA_CHECK(cudaMemcpy(ctx->d_dyn_pis, dyn_pis,
                          static_cast<size_t>(J) * K_per_obs * sizeof(float),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx->d_dyn_sigmas, dyn_sigmas,
                          static_cast<size_t>(J) * K_per_obs * H * 3 * sizeof(float),
                          cudaMemcpyHostToDevice));

    const int total = N * M;
    const int threads = 128;
    const int blocks = (total + threads - 1) / threads;
    draCollisionRiskKernel<<<blocks, threads>>>(
        ctx->d_states, ctx->d_dyn_mus, ctx->d_dyn_sigmas, ctx->d_dyn_pis, ctx->d_dyn_sizes,
        ctx->d_joint_mode_idx,
        N, M, J, K_per_obs, H, R,
        d_safe, robot_radius, sigma_floor,
        cp_lambda, cp_threshold, hard_penalty,
        use_z_probability,
        seed_base,
        ctx->d_delta_S);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaMemcpy(delta_S_out, ctx->d_delta_S,
                          N * M * sizeof(float), cudaMemcpyDeviceToHost));
    return true;
}

} // namespace cuda
} // namespace im2mppi
