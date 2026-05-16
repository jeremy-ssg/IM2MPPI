/*
    FILE: im2_mppi_planner.h
    --------------------------------
    IM2-MPPI: Intent-Modal Risk-Aware MPPI planner for UAV dynamic avoidance.

    Models the UAV as a 3-D point mass:
        state   x = [px, py, pz, vx, vy, vz]
        control u = [ax, ay, az]

    Key design principles:
      - Phase 2: vanilla MPPI over the point-mass model.
      - Phase 3: plug in dynamic_predictor multi-modal predictions.
      - Phase 4: per-rollout CVaR dynamic risk (computeCVaRCost).
      - Phase 5: risk-aware joint-mode pruning (pruneJointModes).
      - All later phases require ZERO changes to this header's public API.
*/

#ifndef IM2_MPPI_PLANNER_H
#define IM2_MPPI_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <random>
#include <memory>
#include <limits>

#include <trajectory_planner/im2_mppi_params.h>

namespace im2mppi {

// ═══════════════════════════════════════════════════════════════════════════════
//  Primitive kinematics types
// ═══════════════════════════════════════════════════════════════════════════════

// 6-DOF point-mass state (no yaw — yaw is a post-process output only).
struct State {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  // position  [m]
    Eigen::Vector3d v = Eigen::Vector3d::Zero();  // velocity  [m/s]
};

// Control input: desired acceleration.
struct Control {
    Eigen::Vector3d a = Eigen::Vector3d::Zero();  // acceleration [m/s²]
};

// One point on the output trajectory with yaw attached.
struct TrajectoryPoint {
    Eigen::Vector3d p   = Eigen::Vector3d::Zero();
    Eigen::Vector3d v   = Eigen::Vector3d::Zero();
    Eigen::Vector3d a   = Eigen::Vector3d::Zero();
    double          yaw = 0.0;  // [rad], from generateYawReference()
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Obstacle types
// ═══════════════════════════════════════════════════════════════════════════════

// Static obstacle represented as an inflated sphere (center + effective radius).
struct SphereObstacle {
    Eigen::Vector3d center;
    double radius = 0.3;  // physical radius + UAV inflation margin [m]
};

// ── Dynamic obstacle prediction (fed in Phase 3 by dynamicPredictor) ─────────

// One intent mode for a single dynamic obstacle:
//   mu_seq[k]         — mean predicted position at rollout step k  [m]
//   sigma_diag_seq[k] — per-axis std-dev of position uncertainty [m]
//                       Used by CVaR sampling in Phase 4.
//                       Set to obstacle_size/2 as an approximation if
//                       the predictor does not expose covariance directly.
struct ObstacleMode {
    double              pi  = 0.0;          // intent probability (sums to 1 across modes)
    std::vector<Eigen::Vector3d> mu_seq;         // length horizon_steps (or pred horizon)
    std::vector<Eigen::Vector3d> sigma_diag_seq; // same length; filled from sizePred/2
};

// All modes for one tracked dynamic obstacle.
// Populated from dynamicPredictor::obstacle in Phase 3.
struct DynamicObstaclePrediction {
    int    id     = -1;   // obstacle tracker ID (for bookkeeping)
    double radius = 0.3;  // physical radius [m]
    std::vector<ObstacleMode> modes;  // length = num_modes_per_obstacle (K)
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Joint-mode structures (Phase 3 – 5)
// ═══════════════════════════════════════════════════════════════════════════════

// A joint mode assigns one intent index to every tracked obstacle.
// probability  = ∏_j  pi_{m_j}
// score        = probability × preliminary_risk  (used for risk-aware pruning)
struct JointMode {
    double probability       = 1.0;
    double preliminary_risk  = 0.0;  // exp(-d_min / sigma_risk) on nominal traj
    double score             = 1.0;  // probability * preliminary_risk

    // obstacle_mode_indices[j] = chosen mode index for obstacle j.
    // Empty vector = vanilla mode (no dynamic obstacles considered).
    std::vector<int> obstacle_mode_indices;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Rollout result
// ═══════════════════════════════════════════════════════════════════════════════

struct RolloutResult {
    std::vector<State>   states;    // [horizon_steps + 1] starting from current state
    std::vector<Control> controls;  // [horizon_steps] perturbed controls
    double cost   = 0.0;
    double weight = 0.0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  IM2MPPIPlanner
// ═══════════════════════════════════════════════════════════════════════════════

class IM2MPPIPlanner {
public:
    explicit IM2MPPIPlanner(const ros::NodeHandle& nh);

    // ── Configuration ─────────────────────────────────────────────────────────
    void loadParams();

    // ── State / goal / reference setters ─────────────────────────────────────
    void setCurrentState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
    void setGoal(const Eigen::Vector3d& goal);
    // Optional reference path; if empty, straight-line to goal is used.
    void setReferencePath(const std::vector<Eigen::Vector3d>& path);

    // ── Obstacle setters ──────────────────────────────────────────────────────
    // Phase 2: static obstacles as spheres (clear any previous list first).
    void setStaticObstacles(const std::vector<SphereObstacle>& obstacles);

    // Phase 3: multi-modal dynamic obstacle predictions from dynamicPredictor.
    // In Phase 2 this may be left empty; cost returns 0 when list is empty.
    void setDynamicObstaclePredictions(
        const std::vector<DynamicObstaclePrediction>& preds);

    // ── Main planning call ────────────────────────────────────────────────────
    // Returns true if a valid trajectory was produced.
    // Warm-starts the nominal control sequence between consecutive calls.
    bool plan();

    // ── Trajectory access (valid after a successful plan()) ───────────────────
    std::vector<TrajectoryPoint> getPlannedTrajectory() const;
    Eigen::Vector3d getPos(double t) const;
    Eigen::Vector3d getVel(double t) const;
    Eigen::Vector3d getAcc(double t) const;

    const IM2MPPIParams& getParams() const { return params_; }

private:
    // ── Dynamics ──────────────────────────────────────────────────────────────

    // One-step Euler integration:
    //   p_next = p + v*dt + 0.5*a*dt²
    //   v_next = v + a*dt
    State propagate(const State& s, const Control& u) const;

    // Project acceleration onto a_max ball.
    Control clampControl(const Control& u) const;

    // Project velocity onto v_max ball.
    State clampVelocity(const State& s) const;

    // ── Control sequence management ───────────────────────────────────────────

    void initializeControlSequence();

    // Warm-start: shift left by one step, pad tail with zero.
    void shiftControlSequence();

    // ── Sampling ──────────────────────────────────────────────────────────────

    // Sample i.i.d. Gaussian control noise for all rollouts × all steps.
    // Output shape: noise[rollout_i][step_k].a ~ N(0, diag(sigma_ax², ...))
    void sampleControlNoise(std::vector<std::vector<Control>>& noise_out);

    // ── Rollout ───────────────────────────────────────────────────────────────

    // Execute all N rollouts from current_state_ using (u_nominal + noise).
    // jm is passed for interface consistency (dynamics do not depend on it;
    // cost functions do — see computeTrajectoryCost).
    std::vector<RolloutResult> rolloutDynamics(
        const std::vector<std::vector<Control>>& noise,
        const JointMode& jm) const;

    // ── Cost functions ────────────────────────────────────────────────────────

    // Aggregate all enabled cost terms.
    double computeTrajectoryCost(const RolloutResult& r,
                                 const JointMode& jm) const;

    // w_goal  * ||p_H - goal||²
    double computeGoalCost(const RolloutResult& r) const;

    // w_path  * Σ_k ||p_k - p_ref_k||²
    double computePathCost(const RolloutResult& r) const;

    // w_vel * Σ||v||² + w_acc * Σ||a||² + w_jerk * Σ||a_k - a_{k-1}||²
    double computeSmoothnessCost(const RolloutResult& r) const;

    // w_static * Σ_k Σ_obs max(0, d_safe - dist)²
    double computeStaticObstacleCost(const RolloutResult& r) const;

    // w_dyn   * Σ_k Σ_obs max(0, d_safe - dist)²  using chosen mode mean traj.
    // Returns 0 when dyn_predictions_ is empty.
    double computeDynamicObstacleCost(const RolloutResult& r,
                                      const JointMode& jm) const;

    // Phase 4 placeholder — per-rollout CVaR over R obstacle trajectory samples.
    // Currently returns 0; will be filled in Phase 4.
    double computeCVaRCost(const RolloutResult& r,
                           const JointMode& jm) const;

    // ── Joint mode management ─────────────────────────────────────────────────

    // Build joint_modes_:
    //   vanilla_mppi          → single trivial mode (no dynamic obstacle info).
    //   mean_prediction_mppi  → Phase 3 stub (currently falls back to vanilla).
    //   mode_aware_*          → Cartesian product of per-obstacle modes, then prune.
    void buildJointModes();

    // Retain at most params_.num_joint_modes_keep modes.
    // Sort criterion: risk_aware → score DESC, probability → pi DESC.
    void pruneJointModes(std::vector<JointMode>& modes) const;

    // Evaluate preliminary risk for a joint mode using the current nominal traj.
    // preliminary_risk = exp(-max(0, d_min) / sigma_risk)
    double computePreliminaryRisk(const JointMode& jm) const;

    // ── Control update ────────────────────────────────────────────────────────

    // Weighted MPPI update across all retained modes and rollouts:
    //   w_mi = pi_m * exp(-(S_mi - S_min) / lambda)
    //   u_k  = Σ_{m,i} w_mi * u_{m,i,k} / Σ_{m,i} w_mi
    void updateControlSequence(
        const std::vector<JointMode>&                   modes,
        const std::vector<std::vector<RolloutResult>>&  all_results);

    // ── Yaw post-processing ───────────────────────────────────────────────────

    // Fill traj[k].yaw = atan2(vy, vx); hold last yaw if horizontal speed < v_yaw_min.
    void generateYawReference(std::vector<TrajectoryPoint>& traj) const;

    // ── Reference path helpers ────────────────────────────────────────────────

    // Return the reference position at rollout step k.
    // Falls back to straight-line interpolation from current pos to goal
    // when ref_path_ is empty.
    Eigen::Vector3d getReferenceAtStep(int k) const;

    // ── Members ───────────────────────────────────────────────────────────────

    ros::NodeHandle    nh_;
    IM2MPPIParams      params_;

    State              current_state_;
    Eigen::Vector3d    goal_     = Eigen::Vector3d::Zero();

    std::vector<Eigen::Vector3d> ref_path_;   // optional; empty = straight-line

    std::vector<Control> u_nominal_;          // [horizon_steps] warm-started nominal

    std::vector<SphereObstacle>           static_obstacles_;
    std::vector<DynamicObstaclePrediction> dyn_predictions_;

    std::vector<JointMode>     joint_modes_;  // built each call to plan()

    std::vector<TrajectoryPoint> planned_traj_;  // [horizon_steps + 1] output

    std::mt19937 rng_;

    bool state_set_ = false;
    bool goal_set_  = false;
};

} // namespace im2mppi
#endif // IM2_MPPI_PLANNER_H
