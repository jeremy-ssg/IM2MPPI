/*
    FILE: im2_mppi_planner.cpp
    --------------------------------
    Implementation of IM2MPPIPlanner — Phase 2 baseline.

    What is implemented here (Phase 2):
      ✓ 3-D point-mass dynamics (propagate / clamp)
      ✓ Gaussian control noise sampling
      ✓ Parallel rollout execution (CPU, Eigen + STL)
      ✓ Cost function: goal + path + smoothness + static obstacles
      ✓ Numerically-stable MPPI weighted control update (with S_min subtraction)
      ✓ Warm-start via shiftControlSequence()
      ✓ Yaw post-processing from velocity direction
      ✓ method_type dispatch skeleton (vanilla_mppi active; others stubbed)
      ✓ buildJointModes / pruneJointModes interface (single trivial mode for now)

    Stubs for later phases:
      computeDynamicObstacleCost — returns 0 when dyn_predictions_ is empty (Phase 3)
      computeCVaRCost            — always returns 0 (Phase 4)
      buildJointModes (multi-modal) — Cartesian product skeleton ready (Phase 3/5)
*/

#include <trajectory_planner/im2_mppi_planner.h>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <stdexcept>

namespace im2mppi {

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

IM2MPPIPlanner::IM2MPPIPlanner(const ros::NodeHandle& nh)
    : nh_(nh)
{
    loadParams();
    rng_.seed(static_cast<unsigned>(params_.random_seed));
    initializeControlSequence();
    ROS_INFO("[IM2-MPPI] Planner ready. method=%s  N=%d  H=%d  dt=%.3f",
             params_.method_type.c_str(),
             params_.num_rollouts,
             params_.horizon_steps,
             params_.dt);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter loading
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::loadParams()
{
    params_ = im2mppi::loadParams(nh_, "im2_mppi");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public setters
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::setCurrentState(const Eigen::Vector3d& pos,
                                      const Eigen::Vector3d& vel)
{
    current_state_.p = pos;
    current_state_.v = vel;
    state_set_ = true;
}

void IM2MPPIPlanner::setGoal(const Eigen::Vector3d& goal)
{
    goal_     = goal;
    goal_set_ = true;
}

void IM2MPPIPlanner::setReferencePath(const std::vector<Eigen::Vector3d>& path)
{
    ref_path_ = path;
}

void IM2MPPIPlanner::setStaticObstacles(
    const std::vector<SphereObstacle>& obstacles)
{
    static_obstacles_ = obstacles;
}

void IM2MPPIPlanner::setDynamicObstaclePredictions(
    const std::vector<DynamicObstaclePrediction>& preds)
{
    dyn_predictions_ = preds;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Dynamics
// ─────────────────────────────────────────────────────────────────────────────

State IM2MPPIPlanner::propagate(const State& s, const Control& u) const
{
    const double dt = params_.dt;
    State next;
    next.p = s.p + s.v * dt + 0.5 * u.a * (dt * dt);
    next.v = s.v + u.a * dt;
    return next;
}

Control IM2MPPIPlanner::clampControl(const Control& u) const
{
    Control out = u;
    const double norm = u.a.norm();
    if (norm > params_.a_max + 1e-9) {
        out.a = u.a * (params_.a_max / norm);
    }
    return out;
}

State IM2MPPIPlanner::clampVelocity(const State& s) const
{
    State out = s;
    const double norm = s.v.norm();
    if (norm > params_.v_max + 1e-9) {
        out.v = s.v * (params_.v_max / norm);
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Control sequence management
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::initializeControlSequence()
{
    u_nominal_.assign(params_.horizon_steps, Control{});
}

void IM2MPPIPlanner::shiftControlSequence()
{
    const int H = params_.horizon_steps;
    for (int k = 0; k + 1 < H; ++k) {
        u_nominal_[k] = u_nominal_[k + 1];
    }
    u_nominal_.back() = Control{};  // zero-pad the last step
}

// ─────────────────────────────────────────────────────────────────────────────
//  Control noise sampling
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::sampleControlNoise(
    std::vector<std::vector<Control>>& noise_out)
{
    std::normal_distribution<double> ndx(0.0, params_.sigma_ax);
    std::normal_distribution<double> ndy(0.0, params_.sigma_ay);
    std::normal_distribution<double> ndz(0.0, params_.sigma_az);

    const int N = params_.num_rollouts;
    const int H = params_.horizon_steps;

    noise_out.resize(N);
    for (int i = 0; i < N; ++i) {
        noise_out[i].resize(H);
        for (int k = 0; k < H; ++k) {
            noise_out[i][k].a = Eigen::Vector3d(ndx(rng_), ndy(rng_), ndz(rng_));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Rollout
// ─────────────────────────────────────────────────────────────────────────────

std::vector<RolloutResult> IM2MPPIPlanner::rolloutDynamics(
    const std::vector<std::vector<Control>>& noise,
    const JointMode& /*jm*/) const
{
    // NOTE: dynamics do not depend on jm — the same trajectories are reused
    // across all joint modes; only the cost differs (see computeTrajectoryCost).
    const int N = params_.num_rollouts;
    const int H = params_.horizon_steps;

    std::vector<RolloutResult> results(N);

    for (int i = 0; i < N; ++i) {
        RolloutResult& r = results[i];
        r.states.resize(H + 1);
        r.controls.resize(H);
        r.states[0] = current_state_;

        for (int k = 0; k < H; ++k) {
            // Perturbed control: clamp after adding noise to stay within a_max
            Control u;
            u.a = u_nominal_[k].a + noise[i][k].a;
            u = clampControl(u);
            r.controls[k] = u;

            r.states[k + 1] = clampVelocity(propagate(r.states[k], u));
        }
    }
    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Reference path helper
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d IM2MPPIPlanner::getReferenceAtStep(int k) const
{
    // Straight-line from current position to goal when no path is supplied.
    if (ref_path_.empty()) {
        double alpha = static_cast<double>(k + 1) /
                       static_cast<double>(params_.horizon_steps);
        alpha = std::min(alpha, 1.0);
        return current_state_.p + alpha * (goal_ - current_state_.p);
    }

    // Arc-length interpolation along the provided path.
    double total_len = 0.0;
    for (int i = 1; i < static_cast<int>(ref_path_.size()); ++i) {
        total_len += (ref_path_[i] - ref_path_[i - 1]).norm();
    }
    if (total_len < 1e-9) return ref_path_.back();

    double target_dist = total_len *
        (static_cast<double>(k + 1) / static_cast<double>(params_.horizon_steps));
    target_dist = std::min(target_dist, total_len);

    double walked = 0.0;
    for (int i = 1; i < static_cast<int>(ref_path_.size()); ++i) {
        double seg = (ref_path_[i] - ref_path_[i - 1]).norm();
        if (walked + seg >= target_dist - 1e-9) {
            double t = (seg > 1e-9) ? (target_dist - walked) / seg : 0.0;
            t = std::max(0.0, std::min(t, 1.0));
            return ref_path_[i - 1] + t * (ref_path_[i] - ref_path_[i - 1]);
        }
        walked += seg;
    }
    return ref_path_.back();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Cost functions
// ─────────────────────────────────────────────────────────────────────────────

double IM2MPPIPlanner::computeGoalCost(const RolloutResult& r) const
{
    // Terminal position error
    return params_.w_goal * (r.states.back().p - goal_).squaredNorm();
}

double IM2MPPIPlanner::computePathCost(const RolloutResult& r) const
{
    double cost = 0.0;
    for (int k = 0; k < params_.horizon_steps; ++k) {
        const Eigen::Vector3d p_ref = getReferenceAtStep(k);
        cost += (r.states[k + 1].p - p_ref).squaredNorm();
    }
    return params_.w_path * cost;
}

double IM2MPPIPlanner::computeSmoothnessCost(const RolloutResult& r) const
{
    double vel_cost  = 0.0;
    double acc_cost  = 0.0;
    double jerk_cost = 0.0;

    for (int k = 0; k < params_.horizon_steps; ++k) {
        vel_cost += r.states[k + 1].v.squaredNorm();
        acc_cost += r.controls[k].a.squaredNorm();
        if (k > 0) {
            Eigen::Vector3d da = r.controls[k].a - r.controls[k - 1].a;
            jerk_cost += da.squaredNorm();
        }
    }
    return params_.w_vel  * vel_cost
         + params_.w_acc  * acc_cost
         + params_.w_jerk * jerk_cost;
}

double IM2MPPIPlanner::computeStaticObstacleCost(const RolloutResult& r) const
{
    if (static_obstacles_.empty()) return 0.0;

    double cost = 0.0;
    for (int k = 1; k <= params_.horizon_steps; ++k) {
        const Eigen::Vector3d& p = r.states[k].p;
        for (const auto& obs : static_obstacles_) {
            // signed clearance (negative = inside obstacle)
            double clearance = (p - obs.center).norm() - obs.radius;
            if (clearance < params_.d_safe) {
                double pen = params_.d_safe - clearance;
                cost += pen * pen;
            }
        }
    }
    return params_.w_static * cost;
}

double IM2MPPIPlanner::computeDynamicObstacleCost(const RolloutResult& r,
                                                   const JointMode&   jm) const
{
    // Phase 2: use the mean trajectory of the chosen mode for each obstacle.
    // Returns 0 when no predictions are available (vanilla_mppi case).
    if (dyn_predictions_.empty() || jm.obstacle_mode_indices.empty()) {
        return 0.0;
    }

    double cost = 0.0;
    const int H = params_.horizon_steps;

    for (int j = 0; j < static_cast<int>(dyn_predictions_.size()); ++j) {
        const auto& pred = dyn_predictions_[j];
        if (j >= static_cast<int>(jm.obstacle_mode_indices.size())) continue;

        const int mode_idx = jm.obstacle_mode_indices[j];
        if (mode_idx >= static_cast<int>(pred.modes.size())) continue;

        const ObstacleMode& mode = pred.modes[mode_idx];
        const int pred_H = static_cast<int>(mode.mu_seq.size());
        if (pred_H == 0) continue;

        for (int k = 1; k <= H; ++k) {
            // Clamp prediction index to available horizon
            const int pk = std::min(k - 1, pred_H - 1);
            const double clearance =
                (r.states[k].p - mode.mu_seq[pk]).norm() - pred.radius;
            if (clearance < params_.d_safe) {
                double pen = params_.d_safe - clearance;
                cost += pen * pen;
            }
        }
    }
    return params_.w_dyn * cost;
}

double IM2MPPIPlanner::computeCVaRCost(const RolloutResult& /*r*/,
                                        const JointMode&    /*jm*/) const
{
    // Phase 4 placeholder.
    // Will compute per-rollout CVaR_alpha over R sampled obstacle trajectories:
    //   o_{m,j,r}(k) ~ N(mu_{m,j}(k), Sigma_{m,j}(k))
    //   L_{i,m,j,r}  = max(0, d_safe - ||p_i(k) - o(k)||)^2
    //   CVaR_alpha    = mean of worst (1-alpha) fraction over r
    return 0.0;
}

double IM2MPPIPlanner::computeTrajectoryCost(const RolloutResult& r,
                                              const JointMode&    jm) const
{
    double cost = 0.0;
    cost += computeGoalCost(r);
    cost += computePathCost(r);
    cost += computeSmoothnessCost(r);
    cost += computeStaticObstacleCost(r);
    cost += computeDynamicObstacleCost(r, jm);
    cost += computeCVaRCost(r, jm);
    return cost;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Joint mode management
// ─────────────────────────────────────────────────────────────────────────────

double IM2MPPIPlanner::computePreliminaryRisk(const JointMode& jm) const
{
    // Evaluate risk on the *current nominal trajectory* (cheap, no rollout).
    // preliminary_risk = exp(-max(0, d_min) / sigma_risk)
    if (dyn_predictions_.empty() || jm.obstacle_mode_indices.empty()) {
        return 0.0;
    }

    double min_clearance = std::numeric_limits<double>::infinity();
    State s = current_state_;

    for (int k = 0; k < params_.horizon_steps; ++k) {
        s = clampVelocity(propagate(s, u_nominal_[k]));

        for (int j = 0; j < static_cast<int>(dyn_predictions_.size()); ++j) {
            if (j >= static_cast<int>(jm.obstacle_mode_indices.size())) continue;
            const int mode_idx = jm.obstacle_mode_indices[j];
            const auto& pred = dyn_predictions_[j];
            if (mode_idx >= static_cast<int>(pred.modes.size())) continue;

            const auto& mode = pred.modes[mode_idx];
            const int pk = std::min(k, static_cast<int>(mode.mu_seq.size()) - 1);
            if (pk < 0) continue;

            const double clearance =
                (s.p - mode.mu_seq[pk]).norm() - pred.radius;
            min_clearance = std::min(min_clearance, clearance);
        }
    }

    if (!std::isfinite(min_clearance)) return 0.0;
    return std::exp(-std::max(0.0, min_clearance) / params_.sigma_risk);
}

void IM2MPPIPlanner::pruneJointModes(std::vector<JointMode>& modes) const
{
    const int keep = params_.num_joint_modes_keep;
    if (static_cast<int>(modes.size()) <= keep) return;

    if (params_.mode_pruning_type == "risk_aware") {
        // Descending by score = pi * preliminary_risk
        std::partial_sort(modes.begin(),
                          modes.begin() + keep,
                          modes.end(),
                          [](const JointMode& a, const JointMode& b) {
                              return a.score > b.score;
                          });
    } else {
        // probability only
        std::partial_sort(modes.begin(),
                          modes.begin() + keep,
                          modes.end(),
                          [](const JointMode& a, const JointMode& b) {
                              return a.probability > b.probability;
                          });
    }
    modes.resize(keep);
}

void IM2MPPIPlanner::buildJointModes()
{
    joint_modes_.clear();

    // ── vanilla_mppi: single trivial mode, no dynamic obstacle accounting ──
    if (params_.method_type == "vanilla_mppi" || dyn_predictions_.empty()) {
        JointMode jm;
        jm.probability      = 1.0;
        jm.preliminary_risk = 0.0;
        jm.score            = 1.0;
        // obstacle_mode_indices is intentionally empty
        joint_modes_.push_back(jm);
        return;
    }

    // ── mean_prediction_mppi: Phase 3 stub ───────────────────────────────────
    // Will compress each obstacle's modes into a single probability-weighted
    // mean trajectory.  For now, fall back to vanilla with a one-time warning.
    if (params_.method_type == "mean_prediction_mppi") {
        ROS_WARN_ONCE("[IM2-MPPI] mean_prediction_mppi not yet implemented "
                      "(Phase 3). Falling back to vanilla_mppi.");
        JointMode jm;
        jm.probability = 1.0;
        jm.score       = 1.0;
        joint_modes_.push_back(jm);
        return;
    }

    // ── mode_aware_mppi / mode_aware_mppi_cvar / im2_mppi_full ──────────────
    // Enumerate Cartesian product of per-obstacle mode indices, then prune.
    const int M = static_cast<int>(dyn_predictions_.size());

    std::vector<JointMode> all_modes;
    {   // Seed with one empty mode
        JointMode seed;
        seed.probability = 1.0;
        all_modes.push_back(seed);
    }

    for (int j = 0; j < M; ++j) {
        const auto& pred = dyn_predictions_[j];
        const int K = static_cast<int>(pred.modes.size());
        if (K == 0) continue;

        std::vector<JointMode> expanded;
        expanded.reserve(all_modes.size() * K);
        for (const auto& existing : all_modes) {
            for (int m = 0; m < K; ++m) {
                JointMode nm = existing;
                nm.obstacle_mode_indices.push_back(m);
                nm.probability *= pred.modes[m].pi;
                expanded.push_back(std::move(nm));
            }
        }
        all_modes = std::move(expanded);
    }

    // Annotate with preliminary risk, then prune to top-Kbar
    for (auto& jm : all_modes) {
        jm.preliminary_risk = computePreliminaryRisk(jm);
        jm.score = jm.probability * jm.preliminary_risk;
    }

    pruneJointModes(all_modes);
    joint_modes_ = std::move(all_modes);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPPI control sequence update
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::updateControlSequence(
    const std::vector<JointMode>&                  modes,
    const std::vector<std::vector<RolloutResult>>& all_results)
{
    // w_mi = pi_m * exp(-(S_mi - S_min) / lambda)
    // u_new_k = Σ_{m,i} w_mi * u_{m,i,k} / Σ_{m,i} w_mi
    //
    // Numerically: subtract S_min before exponentiation to prevent underflow.

    const int K = params_.horizon_steps;
    const int N = params_.num_rollouts;

    // ── Global S_min ─────────────────────────────────────────────────────────
    double S_min = std::numeric_limits<double>::infinity();
    for (const auto& mode_results : all_results) {
        for (const auto& r : mode_results) {
            S_min = std::min(S_min, r.cost);
        }
    }
    if (!std::isfinite(S_min)) {
        ROS_WARN("[IM2-MPPI] All rollout costs are non-finite — skipping update.");
        return;
    }

    // ── Accumulate weighted controls ─────────────────────────────────────────
    std::vector<Eigen::Vector3d> weighted_a(K, Eigen::Vector3d::Zero());
    double total_weight = 0.0;

    for (size_t mi = 0; mi < modes.size(); ++mi) {
        const double pi_m = modes[mi].probability;
        for (int i = 0; i < N; ++i) {
            const double w =
                pi_m * std::exp(-(all_results[mi][i].cost - S_min) / params_.lambda);
            if (!std::isfinite(w)) continue;

            total_weight += w;
            for (int k = 0; k < K; ++k) {
                weighted_a[k] += w * all_results[mi][i].controls[k].a;
            }
        }
    }

    if (total_weight < 1e-12) {
        ROS_WARN("[IM2-MPPI] Total MPPI weight near zero — keeping previous nominal.");
        return;
    }

    for (int k = 0; k < K; ++k) {
        Control u;
        u.a = weighted_a[k] / total_weight;
        u_nominal_[k] = clampControl(u);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Yaw reference generation
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::generateYawReference(std::vector<TrajectoryPoint>& traj) const
{
    if (!params_.use_yaw_postprocess || traj.empty()) return;

    // Initialise from current velocity direction
    double prev_yaw = std::atan2(current_state_.v.y(), current_state_.v.x());

    for (auto& pt : traj) {
        const double vxy = std::hypot(pt.v.x(), pt.v.y());
        if (vxy >= params_.v_yaw_min) {
            prev_yaw = std::atan2(pt.v.y(), pt.v.x());
        }
        pt.yaw = prev_yaw;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main planning entry point
// ─────────────────────────────────────────────────────────────────────────────

bool IM2MPPIPlanner::plan()
{
    if (!state_set_) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] plan() called before setCurrentState().");
        return false;
    }
    if (!goal_set_) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] plan() called before setGoal().");
        return false;
    }

    // 1. Warm-start: shift nominal control left by one step
    shiftControlSequence();

    // 2. Build joint modes for this iteration
    buildJointModes();

    // 3. Sample control noise once — shared across all joint modes.
    //    This is correct because dynamics only depend on (u_nominal + noise),
    //    not on the mode.  The mode only affects the *cost* of each trajectory.
    std::vector<std::vector<Control>> noise;
    sampleControlNoise(noise);

    // 4. Roll out all N trajectories (one rollout set shared across modes)
    //    Pass joint_modes_[0] for signature compatibility; dynamics ignore jm.
    std::vector<RolloutResult> base_rollouts =
        rolloutDynamics(noise, joint_modes_[0]);

    // 5. For each joint mode, compute per-rollout costs
    //    (states/controls are identical; only cost weights differ)
    std::vector<std::vector<RolloutResult>> all_results;
    all_results.reserve(joint_modes_.size());

    for (const auto& jm : joint_modes_) {
        std::vector<RolloutResult> mode_results = base_rollouts;  // copy states
        for (auto& r : mode_results) {
            r.cost = computeTrajectoryCost(r, jm);
        }
        all_results.push_back(std::move(mode_results));
    }

    // 6. MPPI update
    updateControlSequence(joint_modes_, all_results);

    // 7. Build output trajectory from the updated nominal control sequence
    planned_traj_.resize(params_.horizon_steps + 1);
    State s = current_state_;
    planned_traj_[0].p = s.p;
    planned_traj_[0].v = s.v;
    planned_traj_[0].a = Eigen::Vector3d::Zero();

    for (int k = 0; k < params_.horizon_steps; ++k) {
        planned_traj_[k].a = u_nominal_[k].a;
        s = clampVelocity(propagate(s, u_nominal_[k]));
        planned_traj_[k + 1].p = s.p;
        planned_traj_[k + 1].v = s.v;
    }
    planned_traj_.back().a = Eigen::Vector3d::Zero();

    // 8. Yaw post-processing
    generateYawReference(planned_traj_);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory access
// ─────────────────────────────────────────────────────────────────────────────

std::vector<TrajectoryPoint> IM2MPPIPlanner::getPlannedTrajectory() const
{
    return planned_traj_;
}

Eigen::Vector3d IM2MPPIPlanner::getPos(double t) const
{
    if (planned_traj_.empty()) return current_state_.p;
    const int k = std::max(0,
        std::min(static_cast<int>(t / params_.dt),
                 static_cast<int>(planned_traj_.size()) - 1));
    return planned_traj_[k].p;
}

Eigen::Vector3d IM2MPPIPlanner::getVel(double t) const
{
    if (planned_traj_.empty()) return current_state_.v;
    const int k = std::max(0,
        std::min(static_cast<int>(t / params_.dt),
                 static_cast<int>(planned_traj_.size()) - 1));
    return planned_traj_[k].v;
}

Eigen::Vector3d IM2MPPIPlanner::getAcc(double t) const
{
    if (planned_traj_.empty()) return Eigen::Vector3d::Zero();
    const int k = std::max(0,
        std::min(static_cast<int>(t / params_.dt),
                 static_cast<int>(planned_traj_.size()) - 1));
    return planned_traj_[k].a;
}

} // namespace im2mppi
