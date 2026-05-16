/*
    FILE: im2_mppi_params.h
    --------------------------------
    Parameter struct for IM2-MPPI planner.
    All fields have safe defaults; call loadParams() to override from rosparam.
*/

#ifndef IM2_MPPI_PARAMS_H
#define IM2_MPPI_PARAMS_H

#include <string>
#include <ros/ros.h>

namespace im2mppi {

struct IM2MPPIParams {
    // ── Dynamics / horizon ──────────────────────────────────────────────────
    double dt           = 0.05;  // simulation timestep [s]
    int horizon_steps   = 30;    // planning horizon length H

    // ── Sampling ─────────────────────────────────────────────────────────────
    int num_rollouts                  = 1024;  // N: parallel trajectory samples
    int num_modes_per_obstacle        = 4;     // K: intent modes per obstacle
    int num_joint_modes_keep          = 8;     // Kbar: retained joint modes after pruning
    int num_obstacle_samples_for_cvar = 16;    // R: per-rollout obstacle samples for CVaR

    // ── MPPI temperature ──────────────────────────────────────────────────────
    double lambda = 1.0;  // cost-to-weight sharpness; lower → greedier

    // ── CVaR (Phase 4) ────────────────────────────────────────────────────────
    double alpha_cvar = 0.95;  // CVaR confidence level; 0.95 → worst 5% tail

    // ── Safety geometry ───────────────────────────────────────────────────────
    double d_safe    = 0.5;  // minimum clearance distance [m]
    double sigma_risk = 1.0; // scale for preliminary risk: exp(-d_min / sigma_risk)

    // ── Kinematic constraints ─────────────────────────────────────────────────
    double v_max = 2.0;   // max velocity norm [m/s]
    double a_max = 3.0;   // max acceleration norm [m/s²]
    double j_max = 8.0;   // max jerk norm [m/s³] — used as cost context, not hard constraint

    // ── Control noise std-dev (per axis) [m/s²] ──────────────────────────────
    double sigma_ax = 1.0;
    double sigma_ay = 1.0;
    double sigma_az = 0.5;

    // ── Cost weights ─────────────────────────────────────────────────────────
    double w_goal   = 10.0;   // terminal position error ||p_H - goal||²
    double w_path   = 1.0;    // tracking deviation from reference path
    double w_vel    = 0.1;    // velocity magnitude penalty
    double w_acc    = 0.05;   // acceleration magnitude penalty
    double w_jerk   = 0.05;   // jerk penalty ||a_k - a_{k-1}||²
    double w_static = 20.0;   // static obstacle proximity penalty
    double w_dyn    = 30.0;   // dynamic obstacle proximity penalty (mean trajectory)
    double w_cvar   = 50.0;   // CVaR tail-risk penalty (Phase 4)

    // ── Ablation / method selection ───────────────────────────────────────────
    // Supported values:
    //   vanilla_mppi          — no predictions, basic MPPI
    //   mean_prediction_mppi  — compress modes into weighted-mean trajectory
    //   mode_aware_mppi       — multi-modal weighting, no CVaR
    //   mode_aware_mppi_cvar  — multi-modal weighting + CVaR
    //   im2_mppi_full         — multi-modal + CVaR + risk-aware pruning
    std::string method_type = "im2_mppi_full";

    // Supported values: probability | risk_aware
    std::string mode_pruning_type = "risk_aware";

    // ── Misc ──────────────────────────────────────────────────────────────────
    int  random_seed          = 42;
    bool use_yaw_postprocess  = true;
    double v_yaw_min          = 0.1;  // min horizontal speed to update yaw [m/s]
};

// Load all parameters from the ROS parameter server.
// Expects keys under <ns>/<param_name>, e.g. "im2_mppi/dt".
inline IM2MPPIParams loadParams(const ros::NodeHandle& nh,
                                 const std::string& ns = "im2_mppi")
{
    IM2MPPIParams p;

    nh.param(ns + "/dt",                            p.dt,                            p.dt);
    nh.param(ns + "/horizon_steps",                 p.horizon_steps,                 p.horizon_steps);
    nh.param(ns + "/num_rollouts",                  p.num_rollouts,                  p.num_rollouts);
    nh.param(ns + "/num_modes_per_obstacle",        p.num_modes_per_obstacle,        p.num_modes_per_obstacle);
    nh.param(ns + "/num_joint_modes_keep",          p.num_joint_modes_keep,          p.num_joint_modes_keep);
    nh.param(ns + "/num_obstacle_samples_for_cvar", p.num_obstacle_samples_for_cvar, p.num_obstacle_samples_for_cvar);

    nh.param(ns + "/lambda",      p.lambda,      p.lambda);
    nh.param(ns + "/alpha_cvar",  p.alpha_cvar,  p.alpha_cvar);
    nh.param(ns + "/d_safe",      p.d_safe,      p.d_safe);
    nh.param(ns + "/sigma_risk",  p.sigma_risk,  p.sigma_risk);

    nh.param(ns + "/v_max",  p.v_max,  p.v_max);
    nh.param(ns + "/a_max",  p.a_max,  p.a_max);
    nh.param(ns + "/j_max",  p.j_max,  p.j_max);

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
    nh.param(ns + "/w_cvar",   p.w_cvar,   p.w_cvar);

    nh.param(ns + "/method_type",       p.method_type,       p.method_type);
    nh.param(ns + "/mode_pruning_type", p.mode_pruning_type, p.mode_pruning_type);

    nh.param(ns + "/random_seed",         p.random_seed,         p.random_seed);
    nh.param(ns + "/use_yaw_postprocess", p.use_yaw_postprocess, p.use_yaw_postprocess);
    nh.param(ns + "/v_yaw_min",           p.v_yaw_min,           p.v_yaw_min);

    return p;
}

} // namespace im2mppi
#endif // IM2_MPPI_PARAMS_H
