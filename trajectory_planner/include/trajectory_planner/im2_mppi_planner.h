/*
    FILE: im2_mppi_planner.h
    --------------------------------
    IM2-MPPI: Intent-Modal Risk-Aware MPPI planner for UAV dynamic avoidance.
    Phases 1 – 4: vanilla / mean_prediction / mode_aware / cvar.

    UAV model: 3-D point mass
        state   x = [px, py, pz, vx, vy, vz]
        control u = [ax, ay, az]

    Public API stable across phases:
      plan() returns true on success; visualization data is exposed via
      getRolloutPositions() / getRolloutWeights() for the RViz layer.
*/

#ifndef IM2_MPPI_PLANNER_H
#define IM2_MPPI_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <random>
#include <limits>
#include <memory>

#include <map_manager/dynamicMap.h>
#include <trajectory_planner/im2_mppi_params.h>

namespace im2mppi {

// ═══════════════════════════════════════════════════════════════════════════════
//  Primitive kinematics types
// ═══════════════════════════════════════════════════════════════════════════════

struct State {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
};

struct Control {
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

struct TrajectoryPoint {
    Eigen::Vector3d p   = Eigen::Vector3d::Zero();
    Eigen::Vector3d v   = Eigen::Vector3d::Zero();
    Eigen::Vector3d a   = Eigen::Vector3d::Zero();
    double          yaw = 0.0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Obstacle types
// ═══════════════════════════════════════════════════════════════════════════════

struct SphereObstacle {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double          radius = 0.3;
};

struct ObstacleMode {
    double                       pi = 0.0;       // intent probability
    std::vector<Eigen::Vector3d> mu_seq;         // mean position per step
    std::vector<Eigen::Vector3d> sigma_diag_seq; // per-axis std-dev per step
};

struct DynamicObstaclePrediction {
    int    id     = -1;
    double radius = 0.3;
    std::vector<ObstacleMode> modes;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Joint mode (Phase 3)
// ═══════════════════════════════════════════════════════════════════════════════

struct JointMode {
    double probability      = 1.0;
    double preliminary_risk = 0.0;
    double score            = 1.0;
    std::vector<int> obstacle_mode_indices;   // [num_obstacles]
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Rollout result
// ═══════════════════════════════════════════════════════════════════════════════

struct RolloutResult {
    std::vector<State>   states;     // [H + 1]
    std::vector<Control> controls;   // [H]
    double               cost   = 0.0;
    double               weight = 0.0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  IM2MPPIPlanner
// ═══════════════════════════════════════════════════════════════════════════════

class IM2MPPIPlanner {
public:
    explicit IM2MPPIPlanner(const ros::NodeHandle& nh);

    // Configuration
    void loadParams();

    // State / goal / reference setters
    void setCurrentState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
    void setGoal(const Eigen::Vector3d& goal);
    void setReferencePath(const std::vector<Eigen::Vector3d>& path);

    // Obstacle setters
    void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);
    void setStaticObstacles(const std::vector<SphereObstacle>& obstacles);
    void setDynamicObstaclePredictions(
        const std::vector<DynamicObstaclePrediction>& preds);

    // Main planning call — returns true on success.
    bool plan();

    // ── Trajectory output (valid after a successful plan()) ──────────────────
    std::vector<TrajectoryPoint> getPlannedTrajectory() const;
    Eigen::Vector3d              getPos(double t) const;
    Eigen::Vector3d              getVel(double t) const;
    Eigen::Vector3d              getAcc(double t) const;

    // ── Visualization data (filled by plan()) ────────────────────────────────
    //   getRolloutPositions()[i][k]  = position of rollout i at step k
    //   getRolloutWeights()[i]       = MPPI weight ∈ [0, 1], 1 = best
    const std::vector<std::vector<Eigen::Vector3d>>& getRolloutPositions() const;
    const std::vector<double>&                       getRolloutWeights()   const;

    // Predictions currently being used by the planner (post method-dispatch).
    const std::vector<DynamicObstaclePrediction>& getDynamicObstaclePredictions() const;

    const IM2MPPIParams& getParams() const { return params_; }

private:
    // ── Dynamics ─────────────────────────────────────────────────────────────
    State   propagate(const State& s, const Control& u) const;
    Control clampControl(const Control& u) const;
    State   clampVelocity(const State& s) const;

    // ── Control sequence management ──────────────────────────────────────────
    void initializeControlSequence();
    void shiftControlSequence();

    // ── Sampling ─────────────────────────────────────────────────────────────
    void sampleControlNoise(std::vector<std::vector<Control>>& noise_out);

    // ── Rollout (dynamics only; cost computed separately) ────────────────────
    std::vector<RolloutResult> rolloutDynamics(
        const std::vector<std::vector<Control>>& noise) const;

    // ── Cost functions ───────────────────────────────────────────────────────
    double computeTrajectoryCost(const RolloutResult& r,
                                 const JointMode&     jm) const;
    double computeGoalCost           (const RolloutResult& r) const;
    double computePathCost           (const RolloutResult& r) const;
    double computeSmoothnessCost     (const RolloutResult& r) const;
    double computeStaticObstacleCost (const RolloutResult& r) const;
    double computeMapObstacleCost    (const RolloutResult& r) const;
    double computeDynamicObstacleCost(const RolloutResult& r,
                                      const JointMode& jm) const;

    // ── Joint mode management ────────────────────────────────────────────────
    void   buildJointModes();
    void   pruneJointModes(std::vector<JointMode>& modes) const;
    double computePreliminaryRisk(const JointMode& jm) const;

    // ── Control update ───────────────────────────────────────────────────────
    void updateControlSequence(
        const std::vector<JointMode>&                  modes,
        const std::vector<std::vector<RolloutResult>>& all_results);

    // ── CVaR aggregation (Phase 4) ───────────────────────────────────────────
    // Given a per-mode cost / probability list for one rollout, return the
    // CVaR_α — the expected cost over the worst α tail of mode scenarios.
    //   α = 1.0  → ordinary expected value
    //   α → 0    → worst-case (max cost)
    double computeCVaR(const std::vector<double>& costs,
                       const std::vector<double>& probs,
                       double alpha) const;

    // Per-rollout CVaR vector used by both updateControlSequence and the
    // visualization cache when method_type == "cvar_mppi".
    std::vector<double> computeRolloutCVaR(
        const std::vector<JointMode>&                  modes,
        const std::vector<std::vector<RolloutResult>>& all_results) const;

    // ── Yaw post-processing ──────────────────────────────────────────────────
    void generateYawReference(std::vector<TrajectoryPoint>& traj) const;

    // ── Reference path helpers ───────────────────────────────────────────────
    Eigen::Vector3d getReferenceAtStep(int k) const;
    TrajectoryPoint sampleTrajectory(double t) const;

    // ── Visualization helpers ────────────────────────────────────────────────
    // Stash a subset of rollouts + normalized weights for the nav layer.
    void cacheVisualizationData(
        const std::vector<RolloutResult>&              base_rollouts,
        const std::vector<std::vector<RolloutResult>>& all_results);

    // ── Members ──────────────────────────────────────────────────────────────
    ros::NodeHandle nh_;
    IM2MPPIParams   params_;

    State           current_state_;
    Eigen::Vector3d goal_     = Eigen::Vector3d::Zero();
    bool            state_set_ = false;
    bool            goal_set_  = false;

    std::shared_ptr<mapManager::dynamicMap> map_;

    std::vector<Eigen::Vector3d> ref_path_;
    std::vector<Control>         u_nominal_;

    std::vector<SphereObstacle>             static_obstacles_;
    std::vector<DynamicObstaclePrediction>  dyn_predictions_;
    std::vector<JointMode>                  joint_modes_;

    std::vector<TrajectoryPoint> planned_traj_;

    // Visualization cache (filled by every plan())
    std::vector<std::vector<Eigen::Vector3d>> viz_rollout_positions_;
    std::vector<double>                       viz_rollout_weights_;

    std::mt19937 rng_;
};

} // namespace im2mppi
#endif // IM2_MPPI_PLANNER_H
