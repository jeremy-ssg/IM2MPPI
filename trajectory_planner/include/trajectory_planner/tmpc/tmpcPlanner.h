/*
    FILE: tmpcPlanner.h
    ----------------------------------------------------------------------------
    T-MPC++  (Topology-driven Model Predictive Control, de Groot et al.,
    IEEE T-RO vol.41 2025).  Benchmark method id: M6_tmpc.

    This class orchestrates one planning iteration of T-MPC++:

        1. setObstacles()  : detector obstacles -> constant-velocity predictions
        2. setReference()  : ref path + ego state -> static-aware local reference
        3. runGuidance()   : internal Visibility-PRM -> P topology-distinct (x,y,t) trajs,
                             lifted to 3D at z_lap
        4. optimizeBranches(): for each guidance traj i (plus 1 unguided in T-MPC++),
                             solve a local MPC tracking that traj and locked to its
                             homotopy class; record optimal cost J_i.
        5. decide()        : i* = argmin_i w_i J_i  with consistency weighting (Eq.12).

    DESIGN STATUS (see trajectory_planner/docs/TMPC_INTEGRATION.md):
      - The topology part is implemented locally: a Guard/Connector Visibility-PRM in
        (x,y,t), per-goal DFS path enumeration, and topology-signature filtering. This
        keeps the benchmark self-contained while following the official T-MPC++ pipeline.
      - The local planner is the in-package OSQP double-integrator MPC. Guided
        branches track their own topology path and add Eq.8/Eq.9 half-plane
        constraints; the unguided branch tracks the plain local reference.
      - Obstacle prediction = constant velocity (locked decision).
*/

#ifndef TMPC_PLANNER_H
#define TMPC_PLANNER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <limits>
#include <cstdint>
#include <utility>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <map_manager/occupancyMap.h>
#include <trajectory_planner/mpcPlanner.h>      // local MPC (ACADO + OSQP paths)
#include <trajectory_planner/utils.h>

namespace trajPlanner {

// One candidate produced per planning iteration (per guidance branch + unguided).
struct TMPCBranch {
    int                            classId = -1;     // topology class id
    bool                           guided  = true;   // false for the T-MPC++ unguided branch
    bool                           overTake = false; // true = vertical "fly-over" branch (3D)
    bool                           feasible = false;
    std::string                    status = "pending";
    double                         cost = std::numeric_limits<double>::infinity(); // J_i*
    std::vector<Eigen::Vector3d>   guidanceTraj;     // 3D, z=z_lap (warm start source)
    std::vector<Eigen::VectorXd>   statesSol;        // local-MPC optimized states
    std::vector<Eigen::VectorXd>   controlsSol;
};

class tmpcPlanner {
public:
    explicit tmpcPlanner(const ros::NodeHandle& nh);

    void initParam();                                          // read tmpc.yaml
    void setMap(const std::shared_ptr<mapManager::occMap>& map);
    void registerPub();                                        // /tmpc/* rviz topics

    // ---- per-iteration inputs ------------------------------------------------
    void updateCurrStates(const Eigen::Vector3d& pos,
                          const Eigen::Vector3d& vel,
                          double yaw);

    // Detector obstacles -> constant-velocity predictions over the horizon.
    // pos[j], vel[j], size[j] are current per-obstacle values.
    void setObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos,
                      const std::vector<Eigen::Vector3d>& obstaclesVel,
                      const std::vector<Eigen::Vector3d>& obstaclesSize);

    // Reference path (lap) -> builds the goal grid in Frenet coords around the
    // look-ahead point; also stores the local horizon reference for the local MPC.
    void setReference(const std::vector<Eigen::Vector3d>& refPath);

    // ---- main pipeline -------------------------------------------------------
    // Returns true if at least one branch produced a feasible trajectory.
    bool plan();

    // ---- outputs -------------------------------------------------------------
    bool   getBestTrajectory(nav_msgs::Path& traj) const;          // /tmpc/best_trajectory
    bool   getBestTrajectory(std::vector<Eigen::Vector3d>& traj) const;
    bool   getLocalReference(std::vector<Eigen::Vector3d>& ref) const;
    // Full best-branch state sequence: each entry is [x,y,z,vx,vy,vz].
    bool   getBestStates(std::vector<Eigen::VectorXd>& states) const;
    // Full best-branch control sequence: each entry is [ax,ay,az].
    bool   getBestControls(std::vector<Eigen::VectorXd>& controls) const;
    double getDt() const { return dt_; }
    int    getBestClassId() const { return bestClassId_; }
    double getPlanTimeMs()  const { return planTimeMs_; }
    std::string getLastPlanStatus() const { return lastPlanStatus_; }

    // visualization helpers (publish all P guidance + optimized branches)
    void publishGuidancePaths()       const;
    void publishOptimizedTrajectories() const;
    void publishObstaclePredictions() const;   // /tmpc/dynamic_obstacle_predictions
    void publishVisibleStaticObstacles() const; // /tmpc/visible_static_obstacles
    bool hasVisualizationSubscribers() const;

private:
    // ---- guidance --------------------------------------------------------------
    // Runs the internal Visibility-PRM topology search; fills branches_.
    bool runGuidance();

    // Build constant-velocity obstacle predictions for the internal guidance PRM
    // and the local MPC homotopy constraints.
    void buildConstantVelocityPredictions();

    // Place the goal grid along the reference path (Frenet: lateral spread + look-ahead).
    void buildGoalGrid();
    void buildGoalGridFromLocalRef();

    // Static-map guard: replace the local tracking reference with an A* path through
    // the inflated occupancy map when the direct local segment is blocked.
    void buildStaticAwareReference();

    // ---- local optimization ----------------------------------------------------
    // Solve ONE branch's local MPC, tracking guidanceTraj and locked to its
    // homotopy class via Eq.8 half-plane constraints (xy only). Sets feasible/cost/
    // statesSol/controlsSol on the branch.
    void solveBranch(TMPCBranch& branch);

    // Build the Eq.8 half-plane constraint (A_k, b_k) for step k against one obstacle.
    //   n   = (o_k - tau_k)/||o_k - tau_k||
    //   A_k = n ; b_k = n . (o_k - n*beta*(r_uav+r_obs))
    // (xy only; returns false if guidance point coincides with obstacle center.)
    bool homotopyHalfPlane(const Eigen::Vector2d& guidancePt,
                           const Eigen::Vector2d& obstaclePt,
                           double rSum,
                           Eigen::Vector2d& A_k, double& b_k) const;

    // ---- decision (Eq.12) ------------------------------------------------------
    // i* = argmin_i w_i J_i ; w_i = consistency_ci if branch i is the previously
    // executed homotopy class, else 1. Sets bestIdx_/bestClassId_.
    void decide();

    bool trajectoryHitsStaticMap(const std::vector<Eigen::VectorXd>& states) const;
    bool trajectoryHitsDynamicObstacles(const std::vector<Eigen::VectorXd>& states,
                                        bool allowVerticalOvertake) const;
    bool pointHitsStaticMapWithMargin(const Eigen::Vector3d& p, double margin) const;
    bool segmentHitsStaticMapWithMargin(const Eigen::Vector3d& a,
                                        const Eigen::Vector3d& b,
                                        double margin) const;
    void resetDiagnostics();
    void updateDiagnosticsAfterSolve();

    // ===========================================================================
    ros::NodeHandle nh_;
    std::shared_ptr<mapManager::occMap> map_;

    // publishers
    ros::Publisher guidancePathsPub_;       // /tmpc/guidance_paths
    ros::Publisher optimizedTrajPub_;       // /tmpc/optimized_trajectories
    ros::Publisher goalGridPub_;            // /tmpc/goal
    ros::Publisher dynObsPub_;              // /tmpc/dynamic_obstacle_predictions
    ros::Publisher visibleStaticPub_;       // /tmpc/visible_static_obstacles

    // --- parameters (from tmpc.yaml) -------------------------------------------
    double dt_              = 0.05;
    int    horizon_        = 50;
    double zLap_           = 1.0;
    double rUav_           = 0.30;
    int    numTrajP_       = 4;
    bool   addUnguided_    = true;      // T-MPC++
    int    prmSamplesN_    = 100;
    std::string homotopyMethod_ = "h_signature";
    double visibilityDt_   = 0.20;
    double smoothingRes_   = 0.05;
    int    goalGridLat_    = 5;
    int    goalGridLong_   = 3;
    double goalLatSpread_  = 2.0;
    double goalLongDist_   = 5.0;
    double betaRelax_      = 1.0;       // 1 = linearized disc avoidance (real clearance)
    double safetyMargin_   = 0.25;      // extra clearance [m] beyond r_uav + r_obs
    int    parallelThreads_ = 5;
    int    threadTimeoutMs_ = 50;
    bool   solveSequential_ = true;     // ACADO not thread-safe; start sequential
    double consistencyCi_  = 0.75;
    double vMax_           = 2.0;
    double aMax_           = 3.0;
    double vzMax_          = 1.0;       // vertical speed limit [m/s]
    double azMax_          = 2.0;       // vertical accel limit [m/s^2]
    double vRef_           = 2.0;
    bool   vertical_       = true;      // enable 3D vertical "fly-over" branch
    double vClearance_     = 0.4;       // vertical clearance above obstacle top [m]
    // local-MPC cost weights (cost_weights/* in tmpc.yaml)
    double wContour_       = 1.0;
    double wLag_           = 1.0;
    double wVel_           = 0.1;
    double wAcc_           = 0.05;
    std::string predictionSource_ = "constant_velocity";
    int    maxObstacles_   = 12;

    // Static-map avoidance (inflated occupancy map).
    bool   useStaticAstar_ = false;
    double staticAstarStep_ = 0.20;
    int    staticAstarPoolXY_ = 80;
    int    staticAstarPoolZ_ = 16;
    double staticHalfplaneSearchRadius_ = 0.8;
    double staticHalfplaneClearance_ = 0.25;
    int    staticHalfplaneRays_ = 16;
    double staticPostCheckClearance_ = 0.12;
    double staticFovRange_ = 7.0;      // [m] only consider static obstacles within this
                                       // range of the drone (FOV-consistent with dynamic)
    bool   publishGuidanceMarkers_ = true;
    bool   publishOptimizedMarkers_ = true;
    bool   publishObstaclePredictionMarkers_ = false;
    bool   publishVisibleStaticMarkers_ = true;
    int    visibleStaticMarkerStride_ = 2;
    int    visibleStaticMarkerMaxPoints_ = 6000;

    // --- per-iteration state ---------------------------------------------------
    struct GuidanceVizNode {
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        int type = 0;   // 0=guard, 1=connector, 2=goal
    };

    Eigen::Vector3d currPos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d currVel_ = Eigen::Vector3d::Zero();
    double          currYaw_ = 0.0;

    std::vector<Eigen::Vector3d> refPath_;                 // full lap reference
    std::vector<Eigen::Vector3d> localRef_;                // sliced horizon reference
    std::vector<Eigen::Vector3d> goalGrid_;                // candidate goals

    // obstacle current values + constant-velocity predictions [obstacle][step]
    std::vector<Eigen::Vector3d>              obsPos_, obsVel_, obsSize_;
    std::vector<std::vector<Eigen::Vector3d>> obsPredPos_;  // [j][k] predicted center (xy used)
    std::vector<double>                       obsRadius_;   // [j] horizontal disc radius
    std::vector<double>                       obsTop_;      // [j] obstacle top altitude [m]

    // candidates this iteration
    std::vector<TMPCBranch> branches_;
    int  bestIdx_      = -1;
    int  bestClassId_  = -1;
    int  prevClassId_  = -1;     // executed class last iteration (consistency)
    // Visibility-PRM graph propagation: guidance samples from the previous iteration,
    // re-seeded (time-decremented) so topology classes persist across cycles.
    std::vector<std::pair<Eigen::Vector2d, int>> prevGuidanceSeed_;
    std::vector<GuidanceVizNode> lastGuidanceVizNodes_;
    std::vector<std::pair<int, int>> lastGuidanceVizEdges_;
    uint32_t guidanceSampleCounter_ = 0;
    double planTimeMs_ = 0.0;
    std::string lastPlanStatus_ = "not_started";
    int lastGuidanceNodes_ = 0;
    int lastGuidanceGoals_ = 0;
    int lastGuidanceExpansions_ = 0;
    int lastGuidedBranches_ = 0;
    int lastTotalBranches_ = 0;
    int lastFeasibleBranches_ = 0;
    int lastStaticRejects_ = 0;
    int lastDynamicRejects_ = 0;
    int lastSolveRejects_ = 0;
    int lastSetupRejects_ = 0;
    int lastNumericRejects_ = 0;
    bool lastStaticDirectBlocked_ = false;
    bool lastStaticAstarActive_ = false;
    bool lastStaticAstarFailed_ = false;

    // pool of local MPC solvers (one per branch; ACADO state is per-instance).
    // NOTE: even with separate instances, ACADO-generated code may share a global
    // workspace -> see docs RISKS. solveSequential_ guards correctness.
    std::vector<std::shared_ptr<mpcPlanner>> localPlanners_;
};

} // namespace trajPlanner
#endif // TMPC_PLANNER_H
