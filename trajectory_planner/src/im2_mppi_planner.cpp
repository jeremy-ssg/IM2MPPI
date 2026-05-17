/*
    FILE: im2_mppi_planner.cpp
    --------------------------------
    IM2MPPIPlanner — Phases 1 – 3 implementation.

    Implemented:
      ✓ 3-D point-mass dynamics (propagate / clamp)
      ✓ Gaussian control noise sampling
      ✓ Parallel rollouts (CPU)
      ✓ Cost: goal + path + smoothness + static + dynamic-mean obstacles
      ✓ Numerically-stable MPPI update (S_min subtraction)
      ✓ Warm-start (shift control sequence)
      ✓ Yaw post-processing from velocity
      ✓ method_type dispatch:
          - vanilla_mppi         : no dynamic obstacle awareness
          - mean_prediction_mppi : nav layer pre-compresses K modes → 1 mean mode
          - mode_aware_mppi      : Cartesian product of per-obstacle modes + prune
      ✓ Visualization data caching (rollouts + weights) for RViz

    Phase 4 (CVaR) intentionally NOT included.
*/

#include <trajectory_planner/im2_mppi_planner.h>

#include <cmath>
#include <algorithm>
#include <numeric>

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

void IM2MPPIPlanner::loadParams()
{
    params_ = im2mppi::loadParams(nh_, "im2_mppi");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setters
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

void IM2MPPIPlanner::setMap(const std::shared_ptr<mapManager::dynamicMap>& map)
{
    map_ = map;
}

void IM2MPPIPlanner::setStaticObstacles(
    const std::vector<SphereObstacle>& obstacles)
{
    static_obstacles_ = obstacles;
}

void IM2MPPIPlanner::setDynamicObstaclePredictions(
    const std::vector<DynamicObstaclePrediction>& preds)
{
    dyn_predictions_.clear();
    dyn_predictions_.reserve(preds.size());

    for (const auto& pred : preds) {
        if (pred.modes.empty()) continue;

        DynamicObstaclePrediction clean;
        clean.id     = pred.id;
        clean.radius = std::max(0.0, pred.radius);

        std::vector<ObstacleMode> modes;
        modes.reserve(pred.modes.size());
        for (auto mode : pred.modes) {
            if (mode.mu_seq.empty()) continue;
            if (!std::isfinite(mode.pi) || mode.pi < 0.0) mode.pi = 0.0;
            modes.push_back(std::move(mode));
        }
        if (modes.empty()) continue;

        std::sort(modes.begin(), modes.end(),
            [](const ObstacleMode& a, const ObstacleMode& b) {
                return a.pi > b.pi;
            });

        const int keep = std::min(params_.num_modes_per_obstacle,
                                  static_cast<int>(modes.size()));
        clean.modes.assign(modes.begin(), modes.begin() + keep);

        double pi_sum = 0.0;
        for (const auto& mode : clean.modes) pi_sum += mode.pi;
        if (pi_sum > 1e-9) {
            for (auto& mode : clean.modes) mode.pi /= pi_sum;
        } else {
            const double uniform = 1.0 / static_cast<double>(clean.modes.size());
            for (auto& mode : clean.modes) mode.pi = uniform;
        }

        dyn_predictions_.push_back(std::move(clean));
    }
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
//  Control sequence
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
    u_nominal_.back() = Control{};
}

// ─────────────────────────────────────────────────────────────────────────────
//  Noise sampling
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::sampleControlNoise(
    std::vector<std::vector<Control>>& noise_out)
{
    std::normal_distribution<double> ndx(0.0, params_.sigma_ax);
    std::normal_distribution<double> ndy(0.0, params_.sigma_ay);
    std::normal_distribution<double> ndz(0.0, params_.sigma_az);

    const int N = params_.num_rollouts;
    const int H = params_.horizon_steps;

    noise_out.assign(N, std::vector<Control>(H));
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < H; ++k) {
            noise_out[i][k].a = Eigen::Vector3d(ndx(rng_), ndy(rng_), ndz(rng_));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Rollout
// ─────────────────────────────────────────────────────────────────────────────

std::vector<RolloutResult> IM2MPPIPlanner::rolloutDynamics(
    const std::vector<std::vector<Control>>& noise) const
{
    const int N = params_.num_rollouts;
    const int H = params_.horizon_steps;

    std::vector<RolloutResult> results(N);
    for (int i = 0; i < N; ++i) {
        RolloutResult& r = results[i];
        r.states.resize(H + 1);
        r.controls.resize(H);
        r.states[0] = current_state_;

        for (int k = 0; k < H; ++k) {
            Control u;
            u.a = u_nominal_[k].a + noise[i][k].a;
            u   = clampControl(u);
            r.controls[k]    = u;
            r.states[k + 1]  = clampVelocity(propagate(r.states[k], u));
        }
    }
    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Reference path helper
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d IM2MPPIPlanner::getReferenceAtStep(int k) const
{
    if (ref_path_.empty()) {
        double alpha = static_cast<double>(k + 1) /
                       static_cast<double>(params_.horizon_steps);
        alpha = std::min(alpha, 1.0);
        return current_state_.p + alpha * (goal_ - current_state_.p);
    }

    double total_len = 0.0;
    for (size_t i = 1; i < ref_path_.size(); ++i) {
        total_len += (ref_path_[i] - ref_path_[i - 1]).norm();
    }
    if (total_len < 1e-9) return ref_path_.back();

    double target_dist = total_len *
        (static_cast<double>(k + 1) / static_cast<double>(params_.horizon_steps));
    target_dist = std::min(target_dist, total_len);

    double walked = 0.0;
    for (size_t i = 1; i < ref_path_.size(); ++i) {
        const double seg = (ref_path_[i] - ref_path_[i - 1]).norm();
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
    double vel_c = 0.0, acc_c = 0.0, jerk_c = 0.0;
    for (int k = 0; k < params_.horizon_steps; ++k) {
        vel_c += r.states[k + 1].v.squaredNorm();
        acc_c += r.controls[k].a.squaredNorm();
        if (k > 0) {
            Eigen::Vector3d da = r.controls[k].a - r.controls[k - 1].a;
            jerk_c += da.squaredNorm();
        }
    }
    return params_.w_vel  * vel_c
         + params_.w_acc  * acc_c
         + params_.w_jerk * jerk_c;
}

double IM2MPPIPlanner::computeStaticObstacleCost(const RolloutResult& r) const
{
    if (static_obstacles_.empty()) return 0.0;
    double cost = 0.0;
    for (int k = 1; k <= params_.horizon_steps; ++k) {
        const Eigen::Vector3d& p = r.states[k].p;
        for (const auto& obs : static_obstacles_) {
            const double clearance = (p - obs.center).norm() - obs.radius;
            if (clearance < params_.d_safe) {
                const double pen = params_.d_safe - clearance;
                cost += pen * pen;
            }
        }
    }
    return params_.w_static * cost;
}

double IM2MPPIPlanner::computeMapObstacleCost(const RolloutResult& r) const
{
    if (!map_) return 0.0;

    double cost = 0.0;
    const double collision_penalty =
        std::max(1.0, params_.d_safe * params_.d_safe * 100.0);

    for (int k = 1; k <= params_.horizon_steps; ++k) {
        const Eigen::Vector3d& p_prev = r.states[k - 1].p;
        const Eigen::Vector3d& p      = r.states[k].p;

        if (map_->isInflatedOccupied(p)) {
            cost += collision_penalty;
        }
        if ((p - p_prev).squaredNorm() > 1e-10 &&
            map_->isInflatedOccupiedLine(p_prev, p)) {
            cost += collision_penalty;
        }
    }

    return params_.w_static * cost;
}

double IM2MPPIPlanner::computeDynamicObstacleCost(const RolloutResult& r,
                                                   const JointMode&   jm) const
{
    if (dyn_predictions_.empty() || jm.obstacle_mode_indices.empty()) {
        return 0.0;
    }

    double cost = 0.0;
    const int H = params_.horizon_steps;

    for (size_t j = 0; j < dyn_predictions_.size(); ++j) {
        if (j >= jm.obstacle_mode_indices.size()) continue;

        const auto& pred = dyn_predictions_[j];
        const int   mode_idx = jm.obstacle_mode_indices[j];
        if (mode_idx < 0 || mode_idx >= static_cast<int>(pred.modes.size())) continue;

        const ObstacleMode& mode = pred.modes[mode_idx];
        const int pred_H = static_cast<int>(mode.mu_seq.size());
        if (pred_H == 0) continue;

        for (int k = 1; k <= H; ++k) {
            const int pk = std::min(k - 1, pred_H - 1);
            const double clearance =
                (r.states[k].p - mode.mu_seq[pk]).norm() - pred.radius;
            if (clearance < params_.d_safe) {
                const double pen = params_.d_safe - clearance;
                cost += pen * pen;
            }
        }
    }
    return params_.w_dyn * cost;
}

double IM2MPPIPlanner::computeTrajectoryCost(const RolloutResult& r,
                                              const JointMode&    jm) const
{
    double c = 0.0;
    c += computeGoalCost(r);
    c += computePathCost(r);
    c += computeSmoothnessCost(r);
    c += computeStaticObstacleCost(r);
    c += computeMapObstacleCost(r);
    c += computeDynamicObstacleCost(r, jm);
    return c;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Joint mode management
// ─────────────────────────────────────────────────────────────────────────────

double IM2MPPIPlanner::computePreliminaryRisk(const JointMode& jm) const
{
    if (dyn_predictions_.empty() || jm.obstacle_mode_indices.empty()) return 0.0;

    double min_clearance = std::numeric_limits<double>::infinity();
    State  s = current_state_;

    for (int k = 0; k < params_.horizon_steps; ++k) {
        s = clampVelocity(propagate(s, u_nominal_[k]));

        for (size_t j = 0; j < dyn_predictions_.size(); ++j) {
            if (j >= jm.obstacle_mode_indices.size()) continue;
            const int mode_idx = jm.obstacle_mode_indices[j];
            const auto& pred = dyn_predictions_[j];
            if (mode_idx < 0 || mode_idx >= static_cast<int>(pred.modes.size())) continue;
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
        std::partial_sort(modes.begin(), modes.begin() + keep, modes.end(),
            [](const JointMode& a, const JointMode& b) { return a.score > b.score; });
    } else {
        std::partial_sort(modes.begin(), modes.begin() + keep, modes.end(),
            [](const JointMode& a, const JointMode& b) { return a.probability > b.probability; });
    }
    modes.resize(keep);
}

void IM2MPPIPlanner::buildJointModes()
{
    joint_modes_.clear();

    // vanilla_mppi OR no predictions → trivial single mode with no dyn cost
    if (params_.method_type == "vanilla_mppi" || dyn_predictions_.empty()) {
        JointMode jm;
        jm.probability      = 1.0;
        jm.preliminary_risk = 0.0;
        jm.score            = 1.0;
        joint_modes_.push_back(jm);
        return;
    }

    // mean_prediction_mppi: nav layer compressed K modes → 1 mean per obstacle
    // mode_aware_mppi:      full K modes per obstacle, Cartesian product + prune

    std::vector<JointMode> modes;
    JointMode seed;
    seed.probability = 1.0;
    modes.push_back(seed);

    for (const auto& pred : dyn_predictions_) {
        const int K = static_cast<int>(pred.modes.size());
        if (K == 0) continue;

        std::vector<JointMode> expanded;
        expanded.reserve(modes.size() * K);
        for (const auto& existing : modes) {
            for (int m = 0; m < K; ++m) {
                JointMode nm = existing;
                nm.obstacle_mode_indices.push_back(m);
                nm.probability *= pred.modes[m].pi;
                if (!std::isfinite(nm.probability)) nm.probability = 0.0;
                expanded.push_back(std::move(nm));
            }
        }

        for (auto& jm : expanded) {
            jm.preliminary_risk = computePreliminaryRisk(jm);
            jm.score            = jm.probability * (1.0 + jm.preliminary_risk);
        }
        pruneJointModes(expanded);
        modes = std::move(expanded);
    }

    for (auto& jm : modes) {
        jm.preliminary_risk = computePreliminaryRisk(jm);
        jm.score            = jm.probability * (1.0 + jm.preliminary_risk);
    }

    pruneJointModes(modes);
    joint_modes_ = std::move(modes);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPPI update
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::updateControlSequence(
    const std::vector<JointMode>&                  modes,
    const std::vector<std::vector<RolloutResult>>& all_results)
{
    const int H = params_.horizon_steps;
    const int N = params_.num_rollouts;

    double S_min = std::numeric_limits<double>::infinity();
    for (const auto& mode_results : all_results) {
        for (const auto& r : mode_results) {
            S_min = std::min(S_min, r.cost);
        }
    }
    if (!std::isfinite(S_min)) {
        ROS_WARN("[IM2-MPPI] All rollout costs non-finite — skipping update.");
        return;
    }

    std::vector<Eigen::Vector3d> weighted_a(H, Eigen::Vector3d::Zero());
    double total_weight = 0.0;

    for (size_t mi = 0; mi < modes.size(); ++mi) {
        const double pi_m = modes[mi].probability;
        for (int i = 0; i < N; ++i) {
            const double w =
                pi_m * std::exp(-(all_results[mi][i].cost - S_min) / params_.lambda);
            if (!std::isfinite(w)) continue;
            total_weight += w;
            for (int k = 0; k < H; ++k) {
                weighted_a[k] += w * all_results[mi][i].controls[k].a;
            }
        }
    }

    if (total_weight < 1e-12) {
        ROS_WARN("[IM2-MPPI] Total MPPI weight ~0 — keeping previous nominal.");
        return;
    }

    for (int k = 0; k < H; ++k) {
        Control u;
        u.a           = weighted_a[k] / total_weight;
        u_nominal_[k] = clampControl(u);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Yaw
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::generateYawReference(std::vector<TrajectoryPoint>& traj) const
{
    if (!params_.use_yaw_postprocess || traj.empty()) return;

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
//  Visualization data cache
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::cacheVisualizationData(
    const std::vector<RolloutResult>&              base_rollouts,
    const std::vector<std::vector<RolloutResult>>& all_results)
{
    const int N = static_cast<int>(base_rollouts.size());
    const int H = params_.horizon_steps;
    const int target = std::min(params_.viz_num_rollouts, N);

    viz_rollout_positions_.assign(target, {});
    viz_rollout_weights_.assign(target, 0.0);

    if (target == 0 || all_results.empty()) return;

    // Aggregate per-rollout cost = min cost across joint modes (most-favourable
    // hypothesis), giving a visually meaningful weight for the rollout.
    std::vector<double> rollout_min_cost(N, std::numeric_limits<double>::infinity());
    for (const auto& mode_results : all_results) {
        for (int i = 0; i < N; ++i) {
            rollout_min_cost[i] = std::min(rollout_min_cost[i], mode_results[i].cost);
        }
    }

    // Stride-based subsampling so we cover the rollout space evenly.
    const int stride = std::max(1, N / target);

    double S_min = *std::min_element(rollout_min_cost.begin(), rollout_min_cost.end());
    if (!std::isfinite(S_min)) return;

    double w_max = 0.0;

    int slot = 0;
    for (int i = 0; i < N && slot < target; i += stride, ++slot) {
        const auto& r = base_rollouts[i];
        auto& pos = viz_rollout_positions_[slot];
        pos.resize(H + 1);
        for (int k = 0; k <= H; ++k) pos[k] = r.states[k].p;

        double w = 0.0;
        if (std::isfinite(rollout_min_cost[i])) {
            w = std::exp(-(rollout_min_cost[i] - S_min) / params_.lambda);
        }
        viz_rollout_weights_[slot] = w;
        if (w > w_max) w_max = w;
    }

    // Normalize weights to [0, 1] for color mapping.
    if (w_max > 1e-12) {
        for (auto& w : viz_rollout_weights_) w /= w_max;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main planning entry point
// ─────────────────────────────────────────────────────────────────────────────

bool IM2MPPIPlanner::plan()
{
    if (!state_set_) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] plan() before setCurrentState().");
        return false;
    }
    if (!goal_set_) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] plan() before setGoal().");
        return false;
    }

    // 1. Warm-start
    shiftControlSequence();

    // 2. Joint modes (depends on method_type + current dyn_predictions_)
    buildJointModes();
    if (joint_modes_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] No joint modes — skipping plan().");
        return false;
    }

    // 3. Sample noise (shared across modes; dynamics don't depend on mode)
    std::vector<std::vector<Control>> noise;
    sampleControlNoise(noise);

    // 4. Roll out dynamics ONCE
    std::vector<RolloutResult> base_rollouts = rolloutDynamics(noise);

    // 5. Per-mode cost evaluation
    std::vector<std::vector<RolloutResult>> all_results;
    all_results.reserve(joint_modes_.size());
    for (const auto& jm : joint_modes_) {
        std::vector<RolloutResult> mode_results = base_rollouts;
        for (auto& r : mode_results) r.cost = computeTrajectoryCost(r, jm);
        all_results.push_back(std::move(mode_results));
    }

    // 6. MPPI update
    updateControlSequence(joint_modes_, all_results);

    // 7. Output trajectory from updated nominal controls
    planned_traj_.assign(params_.horizon_steps + 1, TrajectoryPoint{});
    State s = current_state_;
    planned_traj_[0].p = s.p;
    planned_traj_[0].v = s.v;
    for (int k = 0; k < params_.horizon_steps; ++k) {
        planned_traj_[k].a   = u_nominal_[k].a;
        s = clampVelocity(propagate(s, u_nominal_[k]));
        planned_traj_[k + 1].p = s.p;
        planned_traj_[k + 1].v = s.v;
    }
    generateYawReference(planned_traj_);

    // 8. Cache visualization data
    cacheVisualizationData(base_rollouts, all_results);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory access
// ─────────────────────────────────────────────────────────────────────────────

std::vector<TrajectoryPoint> IM2MPPIPlanner::getPlannedTrajectory() const
{
    return planned_traj_;
}

TrajectoryPoint IM2MPPIPlanner::sampleTrajectory(double t) const
{
    TrajectoryPoint out;
    if (planned_traj_.empty()) {
        out.p = current_state_.p;
        out.v = current_state_.v;
        return out;
    }
    if (planned_traj_.size() == 1 || t <= 0.0) {
        return planned_traj_.front();
    }

    const double horizon_time =
        static_cast<double>(planned_traj_.size() - 1) * params_.dt;
    if (t >= horizon_time) {
        return planned_traj_.back();
    }

    const double scaled = t / params_.dt;
    const int k = std::max(0, std::min(static_cast<int>(std::floor(scaled)),
                                       static_cast<int>(planned_traj_.size()) - 2));
    const double alpha = std::max(0.0, std::min(1.0, scaled - static_cast<double>(k)));

    const auto& a = planned_traj_[k];
    const auto& b = planned_traj_[k + 1];
    out.p = a.p + alpha * (b.p - a.p);
    out.v = a.v + alpha * (b.v - a.v);
    out.a = a.a + alpha * (b.a - a.a);
    out.yaw = (alpha < 0.5) ? a.yaw : b.yaw;
    return out;
}

Eigen::Vector3d IM2MPPIPlanner::getPos(double t) const
{
    return sampleTrajectory(t).p;
}

Eigen::Vector3d IM2MPPIPlanner::getVel(double t) const
{
    return sampleTrajectory(t).v;
}

Eigen::Vector3d IM2MPPIPlanner::getAcc(double t) const
{
    return sampleTrajectory(t).a;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Visualization getters
// ─────────────────────────────────────────────────────────────────────────────

const std::vector<std::vector<Eigen::Vector3d>>&
IM2MPPIPlanner::getRolloutPositions() const
{
    return viz_rollout_positions_;
}

const std::vector<double>& IM2MPPIPlanner::getRolloutWeights() const
{
    return viz_rollout_weights_;
}

const std::vector<DynamicObstaclePrediction>&
IM2MPPIPlanner::getDynamicObstaclePredictions() const
{
    return dyn_predictions_;
}

} // namespace im2mppi
