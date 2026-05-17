/*
    FILE: im2_mppi_params.h
    --------------------------------
    Parameter struct for the IM2-MPPI planner (Phases 1 – 3).

    Phase 4 (CVaR) has been intentionally removed for this rewrite; the
    params struct will be re-extended in a later iteration.
*/

#ifndef IM2_MPPI_PARAMS_H
#define IM2_MPPI_PARAMS_H

#include <string>
#include <ros/ros.h>

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
    double d_safe     = 0.5;  // minimum clearance distance [m]
    double sigma_risk = 1.0;  // scale for preliminary risk: exp(-d_min / sigma_risk)

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
    // Supported values (Phase 3 scope):
    //   vanilla_mppi          — no predictions, basic MPPI baseline
    //   mean_prediction_mppi  — compress K modes into weighted-mean trajectory
    //   mode_aware_mppi       — multi-modal weighting (Cartesian product + prune)
    std::string method_type = "mode_aware_mppi";

    // Supported values: probability | risk_aware
    std::string mode_pruning_type = "risk_aware";

    // ── Visualization ────────────────────────────────────────────────────────
    int  viz_num_rollouts     = 60;    // how many rollouts to draw in RViz
    bool viz_color_by_weight  = true;  // true: gradient red→green; false: flat

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

    nh.param(ns + "/viz_num_rollouts",    p.viz_num_rollouts,    p.viz_num_rollouts);
    nh.param(ns + "/viz_color_by_weight", p.viz_color_by_weight, p.viz_color_by_weight);

    nh.param(ns + "/random_seed",         p.random_seed,         p.random_seed);
    nh.param(ns + "/use_yaw_postprocess", p.use_yaw_postprocess, p.use_yaw_postprocess);
    nh.param(ns + "/v_yaw_min",           p.v_yaw_min,           p.v_yaw_min);

    return p;
}

} // namespace im2mppi
#endif // IM2_MPPI_PARAMS_H
