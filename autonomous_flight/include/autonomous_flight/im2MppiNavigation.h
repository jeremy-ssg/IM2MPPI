/*
    FILE: im2MppiNavigation.h
    --------------------------------
    IM2-MPPI navigation orchestrator (Phases 1 – 3, no CVaR).

    Wires:
        dynamicPredictor::predictor  → intent-modal predictions
        im2mppi::IM2MPPIPlanner      → MPPI trajectory optimizer
        tracking_controller::Target  → downstream tracking controller

    Method-type dispatch:
        vanilla_mppi          → no predictions fed to planner
        mean_prediction_mppi  → compress K modes → 1 weighted-mean mode per obstacle
        mode_aware_mppi       → full K modes, Cartesian product, then prune

    Visualization (RViz topics, all under node namespace):
        ~im2mppi/best_trajectory          (nav_msgs/Path)         green
        ~im2mppi/sampled_rollouts         (visualization_msgs/MarkerArray)
        ~im2mppi/reference_path           (nav_msgs/Path)         yellow
        ~im2mppi/dynamic_obstacle_predictions (MarkerArray)
        ~im2mppi/goal                     (MarkerArray)
*/

#ifndef IM2_MPPI_NAVIGATION_H
#define IM2_MPPI_NAVIGATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <autonomous_flight/flightBase.h>

#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <dynamic_predictor/dynamicPredictor.h>

#include <trajectory_planner/im2_mppi_planner.h>

namespace AutoFlight {

class im2MppiNavigation : public flightBase {
public:
    explicit im2MppiNavigation(const ros::NodeHandle& nh);

    void initParam();
    void initModules();
    void registerPub();
    void registerCallback();

    void run();

private:
    // ── ROS timers ─────────────────────────────────────────────────────────
    ros::Timer mppiTimer_;     // planning loop  (~20 Hz)
    ros::Timer trajExeTimer_;  // target publishing (100 Hz)
    ros::Timer visTimer_;      // RViz visualization (~20 Hz)
    ros::Timer predTimer_;     // prediction loop (~5 Hz, decoupled from planning)

    // ── Publishers ─────────────────────────────────────────────────────────
    ros::Publisher bestTrajPub_;     // nav_msgs/Path  — current MPPI output
    ros::Publisher rolloutsPub_;     // MarkerArray    — sampled trajectory cloud
    ros::Publisher refPathPub_;      // nav_msgs/Path  — reference / straight-line
    ros::Publisher dynObsPredPub_;   // MarkerArray    — dynamic obstacle modes
    ros::Publisher goalPub_;         // MarkerArray    — current goal sphere

    // ── Component modules ──────────────────────────────────────────────────
    std::shared_ptr<mapManager::dynamicMap>          map_;
    std::shared_ptr<onboardDetector::fakeDetector>   detector_;
    std::shared_ptr<dynamicPredictor::predictor>     predictor_;
    std::shared_ptr<im2mppi::IM2MPPIPlanner>         mppi_;

    // ── Navigation parameters ──────────────────────────────────────────────
    bool        useFakeDetector_   = false;
    bool        usePredictor_      = false;
    bool        useYawControl_     = false;
    bool        usePredefinedGoal_ = false;
    double      desiredVel_        = 1.5;
    double      desiredAcc_        = 1.5;
    double      desiredAngularVel_ = 0.5;
    int         repeatPathNum_     = 1;
    std::string refTrajPath_;

    nav_msgs::Path predefinedGoal_;
    int            goalIdx_ = 0;

    // ── Planning state ─────────────────────────────────────────────────────
    bool          mppiReady_  = false;
    ros::Time     trajStartTime_;
    double        facingYaw_  = 0.0;

    std::vector<Eigen::Vector3d> lastReferencePath_;

    // ── Prediction cache (updated by predTimer_ at ~5 Hz) ──────────────────
    // Decoupled from mppiTimer_ so slow inference doesn't block planning.
    std::mutex                                      predMutex_;
    std::vector<im2mppi::DynamicObstaclePrediction> cachedDynPreds_;

    // Protects all access to mppi_ (read or write) across the
    // AsyncSpinner threads. predCB does NOT take this lock — it only
    // touches cachedDynPreds_ under predMutex_.
    mutable std::mutex planMutex_;

    // ── Callbacks ──────────────────────────────────────────────────────────
    void mppiCB    (const ros::TimerEvent&);
    void trajExeCB (const ros::TimerEvent&);
    void visCB     (const ros::TimerEvent&);
    void predCB    (const ros::TimerEvent&);  // async prediction update

    // ── Prediction conversion ──────────────────────────────────────────────
    // dynamicPredictor::obstacle → im2mppi format, with dt interpolation
    // (predictor dt 0.1s → MPPI dt 0.05s).
    std::vector<im2mppi::DynamicObstaclePrediction> convertPredictions(
        const std::vector<dynamicPredictor::obstacle>& predOb) const;

    // For mean_prediction_mppi: compress K modes → 1 weighted-mean mode.
    std::vector<im2mppi::DynamicObstaclePrediction> compressToMeanPrediction(
        const std::vector<im2mppi::DynamicObstaclePrediction>& preds) const;

    // Fallback static spheres when predictor disabled.
    void getDynamicSpheres(std::vector<im2mppi::SphereObstacle>& spheres) const;

    // Build reference path (predefined waypoints or straight-line).
    std::vector<Eigen::Vector3d> buildReferencePath() const;

    nav_msgs::Path loadRefTraj(const std::string& path) const;

    // ── Visualization publishers ───────────────────────────────────────────
    void publishBestTrajectory()       const;
    void publishSampledRollouts()      const;
    void publishReferencePath()        const;
    void publishDynamicObstaclePred()  const;
    void publishGoal()                 const;
};

} // namespace AutoFlight
#endif // IM2_MPPI_NAVIGATION_H
