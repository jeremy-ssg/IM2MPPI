/*
    FILE: im2_mppi_params.h
    --------------------------------
    Parameter struct for the IM2-MPPI planner (Phases 1 – 4).

    Phase 4 adds CVaR (Conditional Value-at-Risk) aggregation:
        method_type = "cvar_mppi"
        cvar_alpha  ∈ (0, 1]   tail probability; smaller = more risk-averse
*/

#ifndef IM2_MPPI_PARAMS_H
#define IM2_MPPI_PARAMS_H

#include <string>
#include <ros/ros.h>
#include <algorithm>

namespace im2mppi {

struct IM2MPPIParams {
    // ── Dynamics / horizon ──────────────────────────────────────────────────
    double dt           = 0.05;  // simulation timestep [s]
    int    horizon_steps = 30;   // planning horizon length H

    // ── Sampling ─────────────────────────────────────────────────────────────
    int num_rollouts           = 512;   // N: parallel trajectory samples
    int num_modes_per_obstacle = 4;     // K: intent modes per obstacle
    int num_joint_modes_keep   = 4;     // Kbar: retained joint modes after pruning

    // ── MPPI temperature ─────────────────────────────────────────────────────
    double lambda = 1.0;  // cost-to-weight sharpness; lower → greedier

    // ── Safety geometry ──────────────────────────────────────────────────────
    double d_safe              = 0.5;  // soft-cost clearance threshold [m]
    double sigma_risk          = 1.0;  // scale for preliminary risk: exp(-d_min / sigma_risk)
    // Floor (per-axis) on the position-prediction std used by CVaR sampling.
    // The predictor exposes per-step empirical sigma; we clamp it from below
    // so that CVaR never collapses to deterministic hinge when the predictor
    // happens to report ~zero spread on a stationary obstacle.
    double sigma_min           = 0.05; // [m]
    // Hard-floor filter: any rollout whose minimum clearance against ANY
    // dynamic mode or static obstacle (over the full horizon) drops below
    // this value gets cost = +∞ before the MPPI weighted update. Effectively
    // imposes a hard safety floor on the policy without sacrificing MPPI's
    // soft gradients elsewhere. Set to 0.0 to disable.
    double hard_floor_clearance = 0.0;  // [m] 0 = disabled; recommended 0.15–0.25

    // ── Kinematic constraints ────────────────────────────────────────────────
    double v_max = 2.0;   // max velocity norm [m/s]
    double a_max = 3.0;   // max acceleration norm [m/s²]

    // ── Control noise std-dev (per axis) [m/s²] ──────────────────────────────
    double sigma_ax = 1.5;
    double sigma_ay = 1.5;
    double sigma_az = 0.5;

    // ── Cost weights ─────────────────────────────────────────────────────────
    double w_goal   = 10.0;   // terminal position error
    double w_path   = 1.0;    // tracking deviation from reference path
    double w_vel    = 0.1;    // velocity magnitude penalty
    double w_acc    = 0.05;   // acceleration magnitude penalty
    double w_jerk   = 0.05;   // jerk penalty
    double w_static = 30.0;   // static obstacle proximity penalty
    double w_dyn    = 50.0;   // dynamic obstacle proximity penalty

    // ── Ablation / method selection ──────────────────────────────────────────
    // Supported values:
    //   vanilla_mppi          — no predictions, basic MPPI baseline
    //   mean_prediction_mppi  — compress K modes into weighted-mean trajectory
    //   mode_aware_mppi       — multi-modal weighting (Cartesian product + prune)
    //   cvar_mppi             — multi-modal + CVaR tail aggregation (Phase 4)
    std::string method_type = "cvar_mppi";

    // Supported values: probability | risk_aware
    std::string mode_pruning_type = "risk_aware";

    // ── CVaR (Phase 4) ───────────────────────────────────────────────────────
    // Per-rollout CVaR aggregation over OBSTACLE prediction uncertainty
    // (NOT over joint modes — that would be cancelled by MPPI normalization).
    //
    // For each (ego rollout i, joint mode m, obstacle j):
    //   Sample R obstacle trajectories from N(μ_{m,j}, diag(σ_{m,j})²).
    //   Compute hinge-squared loss against the ego rollout per sample.
    //   ρ[i,m,j] = mean of the worst α fraction of those R losses.
    // Then S[i,m] = base_cost_with_dyn[i,m] + λ_r · Σ_j ρ[i,m,j].
    //
    // The current planner also adds a discrete-intent CVaR premium across
    // retained joint modes, so cvar_mppi protects branch-level tail risk too.
    // Only used when method_type == "cvar_mppi".
    double cvar_alpha                 = 0.20;   // tail fraction (α)
    int    cvar_num_obstacle_samples  = 16;     // R (≤ 32 recommended; GPU-bound at higher)
    double cvar_lambda_r              = 5.0;    // weight applied to Σ_j ρ in the cost

    // ── Mode fusion (Phase 4) ────────────────────────────────────────────────
    // Controls how joint-mode posteriors π_m are combined into the MPPI
    // free-energy update. Supported values:
    //   soft       : standard Proposition 1 (use π_m as-is)
    //   sharpened  : π_m^γ normalized;  γ = fusion_gamma (fixed)
    //   adaptive   : same as sharpened, but γ = 1 + κ·(log K − H(π))
    //                — high π entropy ⇒ no sharpening; low entropy ⇒ argmax-like
    //   argmax     : one-hot — pick argmax_m π_m·Σ_i exp(−S_{m,i}/λ)
    //
    // The adaptive mode is the IM2-MPPI default: it avoids dangerous
    // mode-averaging when modes disagree without committing prematurely.
    // After pruning, entropy is computed on the retained-mode conditional
    // posterior, and high tail-cost disagreement gates sharpening toward soft.
    std::string fusion_mode  = "adaptive";
    double      fusion_gamma = 2.0;     // sharpening exponent for "sharpened"
    double      fusion_kappa = 2.0;     // entropy-adaptive coefficient for "adaptive"

    // ── Visualization ────────────────────────────────────────────────────────
    int  viz_num_rollouts     = 60;    // how many rollouts to draw in RViz
    bool viz_color_by_weight  = true;  // true: gradient red→green; false: flat

    // ── GPU acceleration (CUDA) ──────────────────────────────────────────────
    // When true and the binary is built with IM2_MPPI_USE_CUDA, plan() runs
    // rollout + cost on the GPU. Falls back to CPU automatically if no CUDA
    // device is present at runtime. The CUDA kernels evaluate AABB costs; the
    // planner adds voxel-map collision cost on the CPU after downloading states.
    bool   use_gpu      = true;
    int    cuda_device  = 0;       // which GPU id to use (cudaSetDevice)

    // ── Misc ──────────────────────────────────────────────────────────────────
    int    random_seed         = 42;
    bool   use_yaw_postprocess = true;
    double v_yaw_min           = 0.1;  // min horizontal speed to update yaw
};

// ───────────────────────────────────────────────────────────────────────────
//  Load all parameters from the ROS parameter server.
//  Expects keys under <ns>/<param_name>, e.g. "im2_mppi/dt".
// ───────────────────────────────────────────────────────────────────────────
inline IM2MPPIParams loadParams(const ros::NodeHandle& nh,
                                 const std::string& ns = "im2_mppi")
{
    IM2MPPIParams p;

    nh.param(ns + "/dt",                     p.dt,                     p.dt);
    nh.param(ns + "/horizon_steps",          p.horizon_steps,          p.horizon_steps);
    nh.param(ns + "/num_rollouts",           p.num_rollouts,           p.num_rollouts);
    nh.param(ns + "/num_modes_per_obstacle", p.num_modes_per_obstacle, p.num_modes_per_obstacle);
    nh.param(ns + "/num_joint_modes_keep",   p.num_joint_modes_keep,   p.num_joint_modes_keep);

    nh.param(ns + "/lambda",      p.lambda,      p.lambda);
    nh.param(ns + "/d_safe",      p.d_safe,      p.d_safe);
    nh.param(ns + "/sigma_risk",  p.sigma_risk,  p.sigma_risk);
    nh.param(ns + "/sigma_min",   p.sigma_min,   p.sigma_min);
    nh.param(ns + "/hard_floor_clearance", p.hard_floor_clearance, p.hard_floor_clearance);

    nh.param(ns + "/v_max",  p.v_max,  p.v_max);
    nh.param(ns + "/a_max",  p.a_max,  p.a_max);

    nh.param(ns + "/sigma_ax", p.sigma_ax, p.sigma_ax);
    nh.param(ns + "/sigma_ay", p.sigma_ay, p.sigma_ay);
    nh.param(ns + "/sigma_az", p.sigma_az, p.sigma_az);

    nh.param(ns + "/w_goal",   p.w_goal,   p.w_goal);
    nh.param(ns + "/w_path",   p.w_path,   p.w_path);
    nh.param(ns + "/w_vel",    p.w_vel,    p.w_vel);
    nh.param(ns + "/w_acc",    p.w_acc,    p.w_acc);
    nh.param(ns + "/w_jerk",   p.w_jerk,   p.w_jerk);
    nh.param(ns + "/w_static", p.w_static, p.w_static);
    nh.param(ns + "/w_dyn",    p.w_dyn,    p.w_dyn);

    nh.param(ns + "/method_type",       p.method_type,       p.method_type);
    nh.param(ns + "/mode_pruning_type", p.mode_pruning_type, p.mode_pruning_type);
    nh.param(ns + "/cvar_alpha",                p.cvar_alpha,                p.cvar_alpha);
    nh.param(ns + "/cvar_num_obstacle_samples", p.cvar_num_obstacle_samples, p.cvar_num_obstacle_samples);
    nh.param(ns + "/cvar_lambda_r",             p.cvar_lambda_r,             p.cvar_lambda_r);

    nh.param(ns + "/fusion_mode",  p.fusion_mode,  p.fusion_mode);
    nh.param(ns + "/fusion_gamma", p.fusion_gamma, p.fusion_gamma);
    nh.param(ns + "/fusion_kappa", p.fusion_kappa, p.fusion_kappa);

    nh.param(ns + "/viz_num_rollouts",    p.viz_num_rollouts,    p.viz_num_rollouts);
    nh.param(ns + "/viz_color_by_weight", p.viz_color_by_weight, p.viz_color_by_weight);

    nh.param(ns + "/use_gpu",             p.use_gpu,             p.use_gpu);
    nh.param(ns + "/cuda_device",         p.cuda_device,         p.cuda_device);

    nh.param(ns + "/random_seed",         p.random_seed,         p.random_seed);
    nh.param(ns + "/use_yaw_postprocess", p.use_yaw_postprocess, p.use_yaw_postprocess);
    nh.param(ns + "/v_yaw_min",           p.v_yaw_min,           p.v_yaw_min);

    p.dt                     = std::max(1e-3, p.dt);
    p.horizon_steps          = std::max(1, p.horizon_steps);
    p.num_rollouts           = std::max(1, p.num_rollouts);
    p.num_modes_per_obstacle = std::max(1, p.num_modes_per_obstacle);
    p.num_joint_modes_keep   = std::max(1, p.num_joint_modes_keep);
    p.lambda                 = std::max(1e-6, p.lambda);
    p.d_safe                 = std::max(0.0, p.d_safe);
    p.sigma_min              = std::max(0.0, p.sigma_min);
    p.hard_floor_clearance   = std::max(0.0, p.hard_floor_clearance);
    p.sigma_risk             = std::max(1e-6, p.sigma_risk);
    p.v_max                  = std::max(1e-3, p.v_max);
    p.a_max                  = std::max(1e-3, p.a_max);
    p.sigma_ax               = std::max(0.0, p.sigma_ax);
    p.sigma_ay               = std::max(0.0, p.sigma_ay);
    p.sigma_az               = std::max(0.0, p.sigma_az);
    p.viz_num_rollouts       = std::max(0, p.viz_num_rollouts);
    p.v_yaw_min              = std::max(0.0, p.v_yaw_min);
    // CVaR α must be in (0, 1]; clamp to a tiny lower bound to avoid divide-by-zero.
    p.cvar_alpha                = std::max(1e-3, std::min(1.0, p.cvar_alpha));
    p.cvar_num_obstacle_samples = std::max(1, std::min(64, p.cvar_num_obstacle_samples));
    p.cvar_lambda_r             = std::max(0.0, p.cvar_lambda_r);
    p.fusion_gamma              = std::max(1.0, p.fusion_gamma);
    p.fusion_kappa              = std::max(0.0, p.fusion_kappa);

    return p;
}

} // namespace im2mppi
#endif // IM2_MPPI_PARAMS_H
