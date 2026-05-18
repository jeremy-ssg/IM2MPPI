/*
    FILE: im2_mppi_planner.cpp
    --------------------------------
    IM2MPPIPlanner — Phases 1 – 4 implementation.

    Implemented:
      ✓ 3-D point-mass dynamics (propagate / clamp)
      ✓ Gaussian control noise sampling
      ✓ Parallel rollouts (CPU)
      ✓ Cost: goal + path + smoothness + static + map + dynamic obstacles
      ✓ Numerically-stable MPPI update (S_min subtraction)
      ✓ Warm-start (shift control sequence)
      ✓ Yaw post-processing from velocity
      ✓ method_type dispatch:
          - vanilla_mppi         : no dynamic obstacle awareness
          - mean_prediction_mppi : nav layer pre-compresses K modes → 1 mean mode
          - mode_aware_mppi      : Cartesian product of per-obstacle modes + prune,
                                   π_m-weighted MPPI update
          - cvar_mppi            : same joint modes; per-rollout CVaR_α aggregation
                                   replaces π_m-weighted update (Phase 4)
      ✓ Visualization data caching (rollouts + weights) for RViz
*/

#include <trajectory_planner/im2_mppi_planner.h>

#include <cmath>
#include <algorithm>
#include <numeric>

#ifdef IM2_MPPI_USE_CUDA
#include <trajectory_planner/im2_mppi_cuda.h>
#endif

namespace im2mppi {

// ─────────────────────────────────────────────────────────────────────────────
//  AABB signed-distance function
//      Replaces the sphere clearance `‖p - c‖ - r` of earlier phases.
//      Positive outside the box (= Euclidean distance to nearest face),
//      zero on the surface, negative inside (= -distance to nearest face).
//      `size_full` is the (x, y, z) width — half-extents are size_full * 0.5.
// ─────────────────────────────────────────────────────────────────────────────
namespace {
inline double aabbSDF(const Eigen::Vector3d& p,
                      const Eigen::Vector3d& center,
                      const Eigen::Vector3d& size_full)
{
    const Eigen::Vector3d half = 0.5 * size_full.cwiseMax(1e-6);
    const Eigen::Vector3d q    = (p - center).cwiseAbs() - half;
    const double outside       = q.cwiseMax(0.0).norm();
    const double inside        = std::min(q.maxCoeff(), 0.0);
    return outside + inside;
}
} // anonymous

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

IM2MPPIPlanner::~IM2MPPIPlanner()
{
#ifdef IM2_MPPI_USE_CUDA
    if (cuda_ctx_) cuda::destroyContext(cuda_ctx_);
    cuda_ctx_ = nullptr;
#endif
}

void IM2MPPIPlanner::loadParams()
{
    params_ = im2mppi::loadParams(nh_, "im2_mppi");
}

// ─────────────────────────────────────────────────────────────────────────────
//  CUDA initialization (lazy on first plan() if use_gpu enabled)
// ─────────────────────────────────────────────────────────────────────────────

bool IM2MPPIPlanner::tryInitCuda()
{
    if (cuda_init_attempted_) return cuda_ctx_ != nullptr;
    cuda_init_attempted_ = true;

#ifdef IM2_MPPI_USE_CUDA
    if (!cuda::isAvailable()) {
        ROS_WARN("[IM2-MPPI] use_gpu=true but no CUDA device available; falling back to CPU.");
        return false;
    }
    // Allocate device buffers with generous capacity bounds (capped so we
    // don't have to re-allocate on every plan() call).
    const int N      = params_.num_rollouts;
    const int H      = params_.horizon_steps;
    const int K_max  = std::max(1, params_.num_modes_per_obstacle);
    const int M_max  = std::max(1, params_.num_joint_modes_keep);
    const int J_max  = 16;     // up to 16 dynamic obstacles
    const int Ns_max = 64;     // up to 64 static boxes
    const int P_max  = 256;    // ref path points

    cuda_ctx_ = cuda::createContext(N, H, J_max, K_max, M_max, Ns_max, P_max);
    if (!cuda_ctx_) {
        ROS_WARN("[IM2-MPPI] CUDA context creation failed; falling back to CPU.");
        return false;
    }
    ROS_INFO("[IM2-MPPI] CUDA back-end initialized "
             "(N=%d H=%d J_max=%d K_max=%d M_max=%d).",
             N, H, J_max, K_max, M_max);
    return true;
#else
    ROS_WARN_ONCE("[IM2-MPPI] use_gpu=true but binary was built without "
                  "IM2_MPPI_USE_CUDA; falling back to CPU.");
    return false;
#endif
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
    const std::vector<BoxObstacle>& obstacles)
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
        clean.id   = pred.id;
        clean.size = pred.size.cwiseMax(1e-3);   // guard against zero / negative widths

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
            const double clearance = aabbSDF(p, obs.center, obs.size);
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

    // Soft penalty (matches the GPU path). The old d_safe²×100 = 25 was an
    // NMPC-style hard-constraint penalty that dominated the cost landscape
    // and caused drift; 1.0 keeps map cost comparable to the per-step
    // deterministic dynamic obstacle cost.
    double cost = 0.0;
    const double collision_penalty = 1.0;
    const int    H                 = params_.horizon_steps;
    const int    MAP_STRIDE        = std::max(1, H / 6);   // ~6 checks / rollout

    int prev_check_k = 0;
    for (int k = MAP_STRIDE; k <= H; k += MAP_STRIDE) {
        const Eigen::Vector3d& p_prev = r.states[prev_check_k].p;
        const Eigen::Vector3d& p      = r.states[k].p;

        if (map_->isInflatedOccupied(p)) {
            cost += collision_penalty;
        } else if ((p - p_prev).squaredNorm() > 1e-10 &&
                   map_->isInflatedOccupiedLine(p_prev, p)) {
            cost += collision_penalty;
        }
        prev_check_k = k;
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
                aabbSDF(r.states[k].p, mode.mu_seq[pk], pred.size);
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
            const double clearance = aabbSDF(s.p, mode.mu_seq[pk], pred.size);
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

    if (modes.empty() || all_results.empty()) {
        ROS_WARN("[IM2-MPPI] updateControlSequence: no modes/results.");
        return;
    }

    std::vector<Eigen::Vector3d> weighted_a(H, Eigen::Vector3d::Zero());
    double total_weight = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    //  Mode-fused MPPI free-energy update:
    //      w_{m,i} = π_eff_m · exp(-(S_{m,i} - S_min) / λ)
    //  where π_eff_m comes from computeFusionWeights — soft/sharpened/
    //  adaptive/argmax determined by params_.fusion_mode.
    //
    //  For cvar_mppi, S_{m,i} already contains the CVaR risk term added in
    //  planCPU/planGPU before this call.
    // ─────────────────────────────────────────────────────────────────────────

    // Flatten cost matrix [M*N] for the fusion helper (argmax needs it).
    std::vector<double> costs_flat(static_cast<size_t>(modes.size()) * N, 0.0);
    for (size_t mi = 0; mi < modes.size(); ++mi)
        for (int i = 0; i < N; ++i)
            costs_flat[mi * N + i] = all_results[mi][i].cost;

    const std::vector<double> pi_eff = computeFusionWeights(modes, costs_flat, N);

    double S_min = std::numeric_limits<double>::infinity();
    for (const auto& mode_results : all_results) {
        for (const auto& r : mode_results) {
            if (std::isfinite(r.cost)) S_min = std::min(S_min, r.cost);
        }
    }
    if (!std::isfinite(S_min)) {
        ROS_WARN("[IM2-MPPI] All rollout costs non-finite — skipping update.");
        return;
    }

    for (size_t mi = 0; mi < modes.size(); ++mi) {
        const double pi_m = pi_eff[mi];
        if (pi_m <= 0.0) continue;        // argmax: skip non-selected modes
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
//  CVaR aggregation (Phase 4)
// ─────────────────────────────────────────────────────────────────────────────

double IM2MPPIPlanner::computeCVaR(const std::vector<double>& costs,
                                    const std::vector<double>& probs,
                                    double alpha) const
{
    const size_t M = costs.size();
    if (M == 0) return std::numeric_limits<double>::infinity();
    if (M != probs.size()) return std::numeric_limits<double>::infinity();
    alpha = std::max(1e-6, std::min(1.0, alpha));

    // Filter finite (cost, prob) pairs and normalize probabilities.
    std::vector<std::pair<double, double>> cp;
    cp.reserve(M);
    double psum = 0.0;
    for (size_t i = 0; i < M; ++i) {
        if (!std::isfinite(costs[i]) || probs[i] <= 0.0) continue;
        cp.emplace_back(costs[i], probs[i]);
        psum += probs[i];
    }
    if (cp.empty() || psum < 1e-12) return std::numeric_limits<double>::infinity();
    for (auto& kv : cp) kv.second /= psum;

    // Sort by cost DESCENDING (worst case first).
    std::sort(cp.begin(), cp.end(),
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            return a.first > b.first;
        });

    // Accumulate from worst-case tail until cumulative probability = alpha.
    //   CVaR_α = (1/α) · Σ_{m ∈ tail} π_m · S_m
    double accum_prob = 0.0;
    double accum_cost = 0.0;
    for (const auto& kv : cp) {
        const double s_m  = kv.first;
        const double pi_m = kv.second;
        const double need = alpha - accum_prob;
        if (need <= 0.0) break;

        if (pi_m <= need + 1e-12) {
            accum_cost += pi_m * s_m;
            accum_prob += pi_m;
        } else {
            // Partial inclusion of this mode (linear-interpolate to hit α exactly).
            accum_cost += need * s_m;
            accum_prob  = alpha;
            break;
        }
    }
    if (accum_prob < 1e-12) return cp.front().first;
    return accum_cost / accum_prob;   // = expected cost over the α-tail
}

std::vector<double> IM2MPPIPlanner::computeRolloutCVaR(
    const std::vector<JointMode>&                  modes,
    const std::vector<std::vector<RolloutResult>>& all_results) const
{
    const int N = params_.num_rollouts;
    const int M = static_cast<int>(modes.size());
    std::vector<double> cvar(N, std::numeric_limits<double>::infinity());
    if (M == 0 || all_results.empty()) return cvar;

    std::vector<double> costs(M);
    std::vector<double> probs(M);
    for (int m = 0; m < M; ++m) probs[m] = modes[m].probability;

    for (int i = 0; i < N; ++i) {
        for (int m = 0; m < M; ++m) costs[m] = all_results[m][i].cost;
        cvar[i] = computeCVaR(costs, probs, params_.cvar_alpha);
    }
    return cvar;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Adaptive Soft-to-Argmax Fusion (Phase-4)
//      Returns π_eff[m] used in the MPPI weighted update according to
//      params_.fusion_mode:
//        soft      → π_eff = π
//        sharpened → π_eff_m = π_m^γ / Σ π^γ        (γ = fusion_gamma)
//        adaptive  → γ = 1 + κ·(log K − H(π)), then sharpen
//        argmax    → one-hot at argmax_m π_m·Σ_i exp(−(S_{m,i} − S_min)/λ)
//
//      For argmax we numerically stabilize with S_min subtraction. For
//      soft/sharpened/adaptive we only need π (costs are unused).
// ─────────────────────────────────────────────────────────────────────────────

std::vector<double> IM2MPPIPlanner::computeFusionWeights(
    const std::vector<JointMode>& modes,
    const std::vector<double>&    costs_flat,
    int N) const
{
    const int M = static_cast<int>(modes.size());
    std::vector<double> pi_eff(M, 0.0);
    if (M == 0) return pi_eff;

    // Original posteriors, clamped to non-negative.
    std::vector<double> pi(M, 0.0);
    for (int m = 0; m < M; ++m) pi[m] = std::max(0.0, modes[m].probability);

    const std::string& mode = params_.fusion_mode;

    // ── argmax ────────────────────────────────────────────────────────────
    if (mode == "argmax") {
        if (static_cast<int>(costs_flat.size()) != M * N) {
            ROS_WARN_THROTTLE(5.0,
                "[IM2-MPPI/fusion] argmax: costs_flat size=%zu, expected %d. "
                "Falling back to soft.", costs_flat.size(), M * N);
        } else {
            // S_min for numerical stability of exp(-S/λ)
            double S_min = std::numeric_limits<double>::infinity();
            for (double c : costs_flat)
                if (std::isfinite(c)) S_min = std::min(S_min, c);
            if (std::isfinite(S_min)) {
                std::vector<double> score(M, 0.0);
                for (int m = 0; m < M; ++m) {
                    double s = 0.0;
                    for (int i = 0; i < N; ++i) {
                        double c = costs_flat[m * N + i];
                        if (!std::isfinite(c)) continue;
                        s += std::exp(-(c - S_min) / params_.lambda);
                    }
                    score[m] = pi[m] * s;
                }
                int best = 0;
                for (int m = 1; m < M; ++m)
                    if (score[m] > score[best]) best = m;
                pi_eff[best] = 1.0;
                return pi_eff;
            }
        }
        // Fallback: most-probable mode.
        int best = 0;
        for (int m = 1; m < M; ++m) if (pi[m] > pi[best]) best = m;
        pi_eff[best] = 1.0;
        return pi_eff;
    }

    // ── soft / sharpened / adaptive ──────────────────────────────────────
    double gamma = 1.0;
    if (mode == "sharpened") {
        gamma = std::max(1.0, params_.fusion_gamma);
    } else if (mode == "adaptive") {
        // H(π) using natural log; clamp π to (1e-12, 1] so log is finite.
        double H = 0.0;
        for (int m = 0; m < M; ++m) {
            if (pi[m] > 1e-12) H -= pi[m] * std::log(pi[m]);
        }
        const double H_max = std::log(static_cast<double>(M));
        const double slack = std::max(0.0, H_max - H);
        gamma = 1.0 + params_.fusion_kappa * slack;
    } else if (mode != "soft") {
        ROS_WARN_THROTTLE(5.0,
            "[IM2-MPPI/fusion] unknown fusion_mode '%s' — using soft.", mode.c_str());
    }

    double sum = 0.0;
    for (int m = 0; m < M; ++m) {
        pi_eff[m] = std::pow(pi[m], gamma);
        sum += pi_eff[m];
    }
    if (sum < 1e-12) {
        // Degenerate (all π near zero) → uniform fallback.
        const double u = 1.0 / static_cast<double>(M);
        for (int m = 0; m < M; ++m) pi_eff[m] = u;
    } else {
        for (int m = 0; m < M; ++m) pi_eff[m] /= sum;
    }
    return pi_eff;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Per-rollout CVaR over OBSTACLE prediction uncertainty (Phase-4 core)
//
//      For each (rollout i, joint mode m, obstacle j):
//          Sample R obstacle trajectories from N(μ_{m,j,k}, diag(σ²_{m,j,k}))
//          For each sample r, compute the minimum signed clearance over the
//          H prediction steps against the deterministic ego rollout, then the
//          hinge-squared loss  L_r = max(d_safe - clearance, 0)²
//          ρ[i,m,j] = mean of the top ⌈α R⌉ losses (worst α tail)
//      delta_S[m][i] = λ_r · Σ_j ρ[i,m,j]
//
//      Obstacle treated as an AABB with full extents pred.size centred on
//      the (sampled) μ. Signed clearance uses the same SDF as the
//      deterministic dynamic cost (aabbSDF in this file).
//
//      Reproducibility: this uses the planner's seeded RNG so two plan()
//      calls with the same inputs produce the same CVaR result.
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::computeObstacleCVaRCost(
    const std::vector<RolloutResult>&  base_rollouts,
    const std::vector<JointMode>&      modes,
    std::vector<std::vector<double>>&  delta_S)
{
    const int N  = static_cast<int>(base_rollouts.size());
    const int M  = static_cast<int>(modes.size());
    const int J  = static_cast<int>(dyn_predictions_.size());
    const int H  = params_.horizon_steps;
    const int R  = params_.cvar_num_obstacle_samples;
    const double alpha    = params_.cvar_alpha;
    const double d_safe   = params_.d_safe;
    const double lambda_r = params_.cvar_lambda_r;

    delta_S.assign(M, std::vector<double>(N, 0.0));
    if (J == 0 || M == 0 || N == 0 || R == 0) return;

    const int k_tail = std::max(1, static_cast<int>(std::ceil(alpha * R)));

    // Per-axis standard normals.
    std::normal_distribution<double> nd(0.0, 1.0);

    // Scratch buffers reused across the loop.
    std::vector<double> losses(R, 0.0);

    for (int m = 0; m < M; ++m) {
        const auto& jm = modes[m];
        for (int i = 0; i < N; ++i) {
            const auto& states = base_rollouts[i].states;   // [H+1]
            double sum_rho = 0.0;

            for (int j = 0; j < J; ++j) {
                const auto& pred = dyn_predictions_[j];
                const int   m_j  = (j < static_cast<int>(jm.obstacle_mode_indices.size()))
                                       ? jm.obstacle_mode_indices[j] : 0;
                if (m_j < 0 || m_j >= static_cast<int>(pred.modes.size())) continue;

                const auto& mode = pred.modes[m_j];
                const int   Hm   = std::min(H, static_cast<int>(mode.mu_seq.size()));
                if (Hm == 0) continue;

                // Generate R Monte-Carlo obstacle trajectories and compute loss.
                for (int r = 0; r < R; ++r) {
                    double min_clr = std::numeric_limits<double>::infinity();

                    for (int k = 0; k < Hm; ++k) {
                        const Eigen::Vector3d& mu     = mode.mu_seq[k];
                        const Eigen::Vector3d  sigma  = (k < static_cast<int>(mode.sigma_diag_seq.size()))
                                                         ? mode.sigma_diag_seq[k]
                                                         : Eigen::Vector3d(0.05, 0.05, 0.05);
                        const Eigen::Vector3d  noise(nd(rng_), nd(rng_), nd(rng_));
                        const Eigen::Vector3d  o_pos = mu + sigma.cwiseProduct(noise);

                        // Reuse the file-static aabbSDF helper.
                        const double clr = aabbSDF(states[k + 1].p, o_pos, pred.size);
                        if (clr < min_clr) min_clr = clr;
                    }
                    const double hinge = std::max(d_safe - min_clr, 0.0);
                    losses[r] = hinge * hinge;
                }

                // Mean of the worst α-fraction: partial_sort descending then average top k_tail.
                std::partial_sort(losses.begin(), losses.begin() + k_tail, losses.end(),
                                  std::greater<double>());
                double tail_sum = 0.0;
                for (int t = 0; t < k_tail; ++t) tail_sum += losses[t];
                sum_rho += tail_sum / static_cast<double>(k_tail);
            }

            delta_S[m][i] = lambda_r * sum_rho;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Yaw
// ─────────────────────────────────────────────────────────────────────────────

void IM2MPPIPlanner::generateYawReference(std::vector<TrajectoryPoint>& traj) const
{
    if (!params_.use_yaw_postprocess || traj.empty()) return;

    // Use the AVERAGED velocity over a short look-ahead window instead of
    // the per-step instantaneous velocity. Per-step v is contaminated by
    // MPPI sampling noise (σ_a up to 0.8 m/s² → vy may swing ±0.5 m/s) which
    // produces atan2 yaw jitter up to ±18° per step. Averaging over 5 steps
    // collapses that into a smooth heading reference.
    const int H    = static_cast<int>(traj.size());
    const int LOOK = std::min(5, H);

    double prev_yaw = std::atan2(current_state_.v.y(), current_state_.v.x());

    for (int k = 0; k < H; ++k) {
        Eigen::Vector3d avg_v = Eigen::Vector3d::Zero();
        int count = 0;
        for (int j = k; j < std::min(k + LOOK, H); ++j) {
            avg_v += traj[j].v;
            ++count;
        }
        if (count > 0) avg_v /= static_cast<double>(count);

        const double vxy = std::hypot(avg_v.x(), avg_v.y());
        if (vxy >= params_.v_yaw_min) {
            prev_yaw = std::atan2(avg_v.y(), avg_v.x());
        }
        traj[k].yaw = prev_yaw;
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

    // Aggregate per-rollout cost across joint modes (min = best-case viz).
    // For cvar_mppi, mode_results[i].cost ALREADY contains the CVaR risk
    // term added in planCPU(), so min-across-modes still reflects the right
    // objective for the colour mapping.
    std::vector<double> rollout_cost(N, std::numeric_limits<double>::infinity());
    for (const auto& mode_results : all_results) {
        for (int i = 0; i < N; ++i) {
            if (std::isfinite(mode_results[i].cost)) {
                rollout_cost[i] = std::min(rollout_cost[i], mode_results[i].cost);
            }
        }
    }

    // Stride-based subsampling so we cover the rollout space evenly.
    const int stride = std::max(1, N / target);

    double S_min = std::numeric_limits<double>::infinity();
    for (double c : rollout_cost) {
        if (std::isfinite(c)) S_min = std::min(S_min, c);
    }
    if (!std::isfinite(S_min)) return;

    double w_max = 0.0;

    int slot = 0;
    for (int i = 0; i < N && slot < target; i += stride, ++slot) {
        const auto& r = base_rollouts[i];
        auto& pos = viz_rollout_positions_[slot];
        pos.resize(H + 1);
        for (int k = 0; k <= H; ++k) pos[k] = r.states[k].p;

        double w = 0.0;
        if (std::isfinite(rollout_cost[i])) {
            w = std::exp(-(rollout_cost[i] - S_min) / params_.lambda);
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

    // Dispatch to GPU if enabled + available; otherwise CPU.
    if (params_.use_gpu) {
        if (tryInitCuda()) return planGPU();
    }
    return planCPU();
}

bool IM2MPPIPlanner::planCPU()
{

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

    // 5. Per-mode cost evaluation. In cvar_mppi the deterministic dynamic
    //    obstacle penalty remains active; CVaR is added as an extra risk term.
    const bool is_cvar = (params_.method_type == "cvar_mppi");

    std::vector<std::vector<RolloutResult>> all_results;
    all_results.reserve(joint_modes_.size());
    for (const auto& jm : joint_modes_) {
        std::vector<RolloutResult> mode_results = base_rollouts;
        for (auto& r : mode_results) {
            double c = 0.0;
            c += computeGoalCost(r);
            c += computePathCost(r);
            c += computeSmoothnessCost(r);
            c += computeStaticObstacleCost(r);
            c += computeMapObstacleCost(r);
            c += computeDynamicObstacleCost(r, jm);
            r.cost = c;
        }
        all_results.push_back(std::move(mode_results));
    }

    // 5b. Obstacle-uncertainty CVaR: enhance per-(rollout, joint-mode) cost
    //     with λ_r · Σ_j ρ[i,m,j]. ρ is computed from R Monte-Carlo samples
    //     of each obstacle's predicted (μ, σ).
    if (is_cvar) {
        std::vector<std::vector<double>> delta_S;
        computeObstacleCVaRCost(base_rollouts, joint_modes_, delta_S);
        for (size_t m = 0; m < all_results.size() && m < delta_S.size(); ++m) {
            for (size_t i = 0; i < all_results[m].size() && i < delta_S[m].size(); ++i) {
                all_results[m][i].cost += delta_S[m][i];
            }
        }
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
//  GPU planning path (CUDA)
//      Mirrors planCPU() but pushes rollout + cost evaluation onto the GPU.
//      The weighted-update step + viz cache stay on the CPU (small reductions).
//      Compiled out entirely when IM2_MPPI_USE_CUDA is not defined.
// ─────────────────────────────────────────────────────────────────────────────

bool IM2MPPIPlanner::planGPU()
{
#ifndef IM2_MPPI_USE_CUDA
    // Should never get here — tryInitCuda() would have returned false.
    return planCPU();
#else
    // 1. Warm-start
    shiftControlSequence();

    // 2. Joint modes (depends on method_type + current dyn_predictions_)
    buildJointModes();
    if (joint_modes_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI/GPU] No joint modes — skipping plan().");
        return false;
    }

    const int N = params_.num_rollouts;
    const int H = params_.horizon_steps;
    const int M = static_cast<int>(joint_modes_.size());
    const int J = static_cast<int>(dyn_predictions_.size());
    const int K = std::max(1, params_.num_modes_per_obstacle);
    const int Ns = static_cast<int>(static_obstacles_.size());

    // 3. Flatten host-side inputs to FP32 row-major buffers expected by the
    //    CUDA kernels. All temporaries live on the stack-allocated std::vector
    //    so they're freed at the end of this scope.
    std::vector<float> h_x0(6);
    h_x0[0] = static_cast<float>(current_state_.p.x());
    h_x0[1] = static_cast<float>(current_state_.p.y());
    h_x0[2] = static_cast<float>(current_state_.p.z());
    h_x0[3] = static_cast<float>(current_state_.v.x());
    h_x0[4] = static_cast<float>(current_state_.v.y());
    h_x0[5] = static_cast<float>(current_state_.v.z());

    std::vector<float> h_u_nominal(H * 3);
    for (int k = 0; k < H; ++k) {
        h_u_nominal[k * 3 + 0] = static_cast<float>(u_nominal_[k].a.x());
        h_u_nominal[k * 3 + 1] = static_cast<float>(u_nominal_[k].a.y());
        h_u_nominal[k * 3 + 2] = static_cast<float>(u_nominal_[k].a.z());
    }

    // Noise still sampled on CPU (cheap relative to rollout). curand could
    // replace this if we ever need to push it onto the GPU too.
    std::vector<float> h_noise(static_cast<size_t>(N) * H * 3);
    {
        std::normal_distribution<float> ndx(0.0f, static_cast<float>(params_.sigma_ax));
        std::normal_distribution<float> ndy(0.0f, static_cast<float>(params_.sigma_ay));
        std::normal_distribution<float> ndz(0.0f, static_cast<float>(params_.sigma_az));
        for (int i = 0; i < N; ++i) {
            for (int k = 0; k < H; ++k) {
                const int o = i * H * 3 + k * 3;
                h_noise[o + 0] = ndx(rng_);
                h_noise[o + 1] = ndy(rng_);
                h_noise[o + 2] = ndz(rng_);
            }
        }
    }

    std::vector<float> h_goal(3);
    h_goal[0] = static_cast<float>(goal_.x());
    h_goal[1] = static_cast<float>(goal_.y());
    h_goal[2] = static_cast<float>(goal_.z());

    // Precompute reference path target per step (matches getReferenceAtStep).
    std::vector<float> h_ref_targets(H * 3);
    for (int k = 0; k < H; ++k) {
        const Eigen::Vector3d r = getReferenceAtStep(k);
        h_ref_targets[k * 3 + 0] = static_cast<float>(r.x());
        h_ref_targets[k * 3 + 1] = static_cast<float>(r.y());
        h_ref_targets[k * 3 + 2] = static_cast<float>(r.z());
    }

    // Static AABBs flattened as (cx,cy,cz, sx,sy,sz).
    std::vector<float> h_static_boxes(static_cast<size_t>(Ns) * 6);
    for (int s = 0; s < Ns; ++s) {
        h_static_boxes[s * 6 + 0] = static_cast<float>(static_obstacles_[s].center.x());
        h_static_boxes[s * 6 + 1] = static_cast<float>(static_obstacles_[s].center.y());
        h_static_boxes[s * 6 + 2] = static_cast<float>(static_obstacles_[s].center.z());
        h_static_boxes[s * 6 + 3] = static_cast<float>(static_obstacles_[s].size.x());
        h_static_boxes[s * 6 + 4] = static_cast<float>(static_obstacles_[s].size.y());
        h_static_boxes[s * 6 + 5] = static_cast<float>(static_obstacles_[s].size.z());
    }

    // Dynamic predictions flattened as [j, k, step, xyz]. Pad missing modes
    // with zeros (the joint_mode_idx will still point to valid slots since
    // buildJointModes limits indices to existing modes per obstacle).
    std::vector<float> h_dyn_mus   (static_cast<size_t>(J) * K * H * 3, 0.0f);
    std::vector<float> h_dyn_sigmas(static_cast<size_t>(J) * K * H * 3, 0.05f);
    std::vector<float> h_dyn_sizes (static_cast<size_t>(J) * 3, 0.0f);
    for (int j = 0; j < J; ++j) {
        const auto& pred = dyn_predictions_[j];
        h_dyn_sizes[j * 3 + 0] = static_cast<float>(pred.size.x());
        h_dyn_sizes[j * 3 + 1] = static_cast<float>(pred.size.y());
        h_dyn_sizes[j * 3 + 2] = static_cast<float>(pred.size.z());
        const int Km = std::min(K, static_cast<int>(pred.modes.size()));
        for (int m = 0; m < Km; ++m) {
            const auto& mode = pred.modes[m];
            const int Hm = std::min(H, static_cast<int>(mode.mu_seq.size()));
            const int Hs = std::min(H, static_cast<int>(mode.sigma_diag_seq.size()));
            for (int k = 0; k < Hm; ++k) {
                const int o = ((j * K + m) * H + k) * 3;
                h_dyn_mus[o + 0] = static_cast<float>(mode.mu_seq[k].x());
                h_dyn_mus[o + 1] = static_cast<float>(mode.mu_seq[k].y());
                h_dyn_mus[o + 2] = static_cast<float>(mode.mu_seq[k].z());
            }
            for (int k = 0; k < Hs; ++k) {
                const int o = ((j * K + m) * H + k) * 3;
                h_dyn_sigmas[o + 0] = static_cast<float>(mode.sigma_diag_seq[k].x());
                h_dyn_sigmas[o + 1] = static_cast<float>(mode.sigma_diag_seq[k].y());
                h_dyn_sigmas[o + 2] = static_cast<float>(mode.sigma_diag_seq[k].z());
            }
        }
    }

    // Joint mode index table: M × J ints, telling each joint mode which
    // intent mode each obstacle takes.
    std::vector<int> h_joint_mode_idx(static_cast<size_t>(M) * std::max(1, J), 0);
    for (int m = 0; m < M; ++m) {
        const auto& jm = joint_modes_[m];
        for (int j = 0; j < J; ++j) {
            h_joint_mode_idx[m * J + j] =
                (j < static_cast<int>(jm.obstacle_mode_indices.size()))
                    ? jm.obstacle_mode_indices[j] : 0;
        }
    }

    const bool cvar_mode = (params_.method_type == "cvar_mppi");

    // 4a. Main rollout + cost kernel. CVaR mode keeps deterministic dynamic
    //     obstacle cost; the uncertainty CVaR term is added afterwards.
    std::vector<float> h_costs(static_cast<size_t>(N) * M);
    std::vector<float> h_controls(static_cast<size_t>(N) * H * 3);
    std::vector<float> h_states;
    float* h_states_out = nullptr;
    if (map_) {
        h_states.resize(static_cast<size_t>(N) * (H + 1) * 6);
        h_states_out = h_states.data();
    }
    const bool ok = cuda::runRolloutAndCost(
        cuda_ctx_,
        h_x0.data(), h_u_nominal.data(), h_noise.data(),
        N, H,
        h_goal.data(), h_ref_targets.data(),
        h_static_boxes.data(), Ns,
        h_dyn_mus.data(), J, K,
        h_dyn_sizes.data(),
        h_joint_mode_idx.data(), M,
        static_cast<float>(params_.dt),
        static_cast<float>(params_.a_max),
        static_cast<float>(params_.v_max),
        static_cast<float>(params_.d_safe),
        static_cast<float>(params_.w_goal),  static_cast<float>(params_.w_path),
        static_cast<float>(params_.w_vel),   static_cast<float>(params_.w_acc),
        static_cast<float>(params_.w_jerk),  static_cast<float>(params_.w_static),
        static_cast<float>(params_.w_dyn),
        0,
        h_costs.data(), h_controls.data(), h_states_out);

    if (!ok) {
        ROS_ERROR_THROTTLE(1.0, "[IM2-MPPI/GPU] kernel launch failed — falling back to CPU once.");
        return planCPU();
    }

    // CUDA evaluates AABB costs, while the voxel occupancy map remains on CPU.
    //
    // Two fixes from the drift-debug pass:
    //   (1) collision_penalty was d_safe² × 100 ≈ 25, which after × w_static=30
    //       gave 750 per voxel hit — ~15× the goal cost. That dominated the
    //       cost landscape and pushed the drone off goal whenever ANY voxel
    //       was nearby. Soft penalty 1.0 (× w_static ≈ 30) is comparable to
    //       the per-step deterministic dynamic-obstacle cost.
    //   (2) Checking every single step of every rollout was ~30 720 voxel
    //       lookups / plan() and saturated the CPU at ~460 ms / plan(),
    //       making the GPU's 5 ms rollout pointless. Stride MAP_STRIDE
    //       reduces this to ~6 checks / rollout — the line-collision check
    //       between sampled steps catches anything in between.
    if (map_ && !h_states.empty()) {
        const double collision_penalty = 1.0;
        const int    s_stride          = (H + 1) * 6;
        const int    MAP_STRIDE        = std::max(1, H / 6);   // ~6 checks per rollout

        for (int i = 0; i < N; ++i) {
            double map_cost      = 0.0;
            int    prev_check_k  = 0;

            for (int k = MAP_STRIDE; k <= H; k += MAP_STRIDE) {
                const int p0 = i * s_stride + prev_check_k * 6;
                const int p1 = i * s_stride + k * 6;
                const Eigen::Vector3d p_prev(h_states[p0 + 0],
                                             h_states[p0 + 1],
                                             h_states[p0 + 2]);
                const Eigen::Vector3d p(h_states[p1 + 0],
                                        h_states[p1 + 1],
                                        h_states[p1 + 2]);

                // Cheap endpoint check first; only raycast if endpoint is clear.
                if (map_->isInflatedOccupied(p)) {
                    map_cost += collision_penalty;
                } else if ((p - p_prev).squaredNorm() > 1e-10 &&
                           map_->isInflatedOccupiedLine(p_prev, p)) {
                    map_cost += collision_penalty;
                }
                prev_check_k = k;
            }

            const float weighted_map_cost =
                static_cast<float>(params_.w_static * map_cost);
            if (weighted_map_cost != 0.0f) {
                for (int m = 0; m < M; ++m) {
                    h_costs[i * M + m] += weighted_map_cost;
                }
            }
        }
    }

    // 4b. CVaR over obstacle uncertainty (Phase-4 core). Adds the risk term
    //     λ_r · Σ_j ρ[i,m,j] to each cost matrix entry.
    if (cvar_mode && J > 0) {
        std::vector<float> h_delta_S(static_cast<size_t>(N) * M, 0.0f);
        const unsigned int seed_base =
            static_cast<unsigned int>(params_.random_seed) ^
            static_cast<unsigned int>(ros::Time::now().toNSec() & 0xFFFFFFFFu);

        const bool cvar_ok = cuda::runObstacleCVaR(
            cuda_ctx_, h_dyn_sigmas.data(),
            J, K, M, N, H,
            params_.cvar_num_obstacle_samples,
            static_cast<float>(params_.cvar_alpha),
            static_cast<float>(params_.d_safe),
            static_cast<float>(params_.cvar_lambda_r),
            seed_base,
            h_delta_S.data());

        if (cvar_ok) {
            const int NM = N * M;
            for (int idx = 0; idx < NM; ++idx) h_costs[idx] += h_delta_S[idx];
        } else {
            ROS_WARN_THROTTLE(1.0, "[IM2-MPPI/GPU/CVaR] kernel failed; using base cost only this frame.");
        }
    }

    // 5. Mode-fused MPPI update. fusion_mode (soft/sharpened/adaptive/argmax)
    //    selects π_eff[m]; CVaR risk term already inside h_costs for cvar_mppi.
    auto cost_at = [&](int i, int m) { return h_costs[i * M + m]; };

    // Flatten to [M*N] in the layout computeFusionWeights expects (m*N + i).
    std::vector<double> costs_flat(static_cast<size_t>(M) * N, 0.0);
    for (int m = 0; m < M; ++m)
        for (int i = 0; i < N; ++i)
            costs_flat[m * N + i] = static_cast<double>(cost_at(i, m));

    const std::vector<double> pi_eff = computeFusionWeights(joint_modes_, costs_flat, N);

    double S_min = std::numeric_limits<double>::infinity();
    for (double c : costs_flat) if (std::isfinite(c)) S_min = std::min(S_min, c);
    if (!std::isfinite(S_min)) {
        ROS_WARN("[IM2-MPPI/GPU] All costs non-finite.");
        return false;
    }

    std::vector<Eigen::Vector3d> wa(H, Eigen::Vector3d::Zero());
    double tw = 0.0;
    for (int m = 0; m < M; ++m) {
        const double pi_m = pi_eff[m];
        if (pi_m <= 0.0) continue;        // argmax: skip non-selected modes
        for (int i = 0; i < N; ++i) {
            const double w = pi_m *
                std::exp(-(static_cast<double>(cost_at(i, m)) - S_min) / params_.lambda);
            if (!std::isfinite(w)) continue;
            tw += w;
            for (int k = 0; k < H; ++k) {
                const int o = i * H * 3 + k * 3;
                wa[k].x() += w * h_controls[o + 0];
                wa[k].y() += w * h_controls[o + 1];
                wa[k].z() += w * h_controls[o + 2];
            }
        }
    }
    if (tw < 1e-12) {
        ROS_WARN("[IM2-MPPI/GPU] Total weight ~0 — keeping previous nominal.");
    } else {
        for (int k = 0; k < H; ++k) {
            Control u;
            u.a = wa[k] / tw;
            u_nominal_[k] = clampControl(u);
        }
    }

    // 6. Build planned trajectory from updated nominal controls (CPU rollout
    //    of one trajectory — negligible cost).
    planned_traj_.assign(H + 1, TrajectoryPoint{});
    State s = current_state_;
    planned_traj_[0].p = s.p;
    planned_traj_[0].v = s.v;
    for (int k = 0; k < H; ++k) {
        planned_traj_[k].a   = u_nominal_[k].a;
        s = clampVelocity(propagate(s, u_nominal_[k]));
        planned_traj_[k + 1].p = s.p;
        planned_traj_[k + 1].v = s.v;
    }
    generateYawReference(planned_traj_);

    // 7. Visualization cache — subsample N rollouts and assign uniform weights.
    //    For accurate CVaR/mode-weighted viz we'd need to download states_out
    //    too; keep it cheap for now.
    const int target = std::min(params_.viz_num_rollouts, N);
    viz_rollout_positions_.assign(target, {});
    viz_rollout_weights_.assign(target, 0.0);
    if (target > 0) {
        const int stride = std::max(1, N / target);
        // Re-derive a per-rollout cost: min across modes (cheap approximation).
        std::vector<double> roll_cost(N, std::numeric_limits<double>::infinity());
        for (int i = 0; i < N; ++i) {
            double best = std::numeric_limits<double>::infinity();
            for (int m = 0; m < M; ++m)
                best = std::min(best, static_cast<double>(cost_at(i, m)));
            roll_cost[i] = best;
        }
        double rcmin = *std::min_element(roll_cost.begin(), roll_cost.end());
        double w_max = 0.0;
        int slot = 0;
        for (int i = 0; i < N && slot < target; i += stride, ++slot) {
            // Re-rollout this single trajectory to fill viz positions (cheap).
            auto& pos = viz_rollout_positions_[slot];
            pos.resize(H + 1);
            State ss = current_state_;
            pos[0] = ss.p;
            for (int k = 0; k < H; ++k) {
                Control u;
                u.a.x() = h_controls[i * H * 3 + k * 3 + 0];
                u.a.y() = h_controls[i * H * 3 + k * 3 + 1];
                u.a.z() = h_controls[i * H * 3 + k * 3 + 2];
                ss = clampVelocity(propagate(ss, u));
                pos[k + 1] = ss.p;
            }
            const double w = std::isfinite(roll_cost[i])
                ? std::exp(-(roll_cost[i] - rcmin) / params_.lambda) : 0.0;
            viz_rollout_weights_[slot] = w;
            if (w > w_max) w_max = w;
        }
        if (w_max > 1e-12)
            for (auto& w : viz_rollout_weights_) w /= w_max;
    }

    return true;
#endif // IM2_MPPI_USE_CUDA
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
