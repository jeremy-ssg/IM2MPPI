/*
    FILE: im2MppiNavigation.h
    --------------------------------
    IM2-MPPI navigation orchestrator.

    Replaces mpcNavigation for the IM2-MPPI pipeline.
    Inherits flight primitives (takeoff, odom, goal) from flightBase.
    Wires together:
        dynamicPredictor::predictor  → intent-modal predictions
        im2mppi::IM2MPPIPlanner      → MPPI trajectory optimizer
        tracking_controller::Target  → downstream tracking controller

    Method-type dispatch:
        vanilla_mppi         → no predictions fed to planner
        mean_prediction_mppi → compress K modes → 1 mean mode per obstacle
        mode_aware_mppi      → full K modes, Cartesian product, no CVaR
        mode_aware_mppi_cvar → full K modes + CVaR (Phase 4)
        im2_mppi_full        → full K modes + CVaR + risk-aware pruning (Phase 5)
*/

#ifndef IM2_MPPI_NAVIGATION_H
#define IM2_MPPI_NAVIGATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

// Original AutoFlight base
#include <autonomous_flight/flightBase.h>

// Dynamic obstacle prediction
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <dynamic_predictor/dynamicPredictor.h>

// IM2-MPPI planner
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
    ros::Timer mppiTimer_;    // MPPI planning loop  (~20 Hz)
    ros::Timer trajExeTimer_; // Target publishing   (100 Hz)
    ros::Timer visTimer_;     // Visualization       (~30 Hz)

    // ── Publishers ─────────────────────────────────────────────────────────
    ros::Publisher mppiTrajPub_;   // nav_msgs::Path of MPPI output
    ros::Publisher goalPub_;       // MarkerArray visualising the current goal

    // ── Component modules ──────────────────────────────────────────────────
    std::shared_ptr<mapManager::dynamicMap>          map_;
    std::shared_ptr<onboardDetector::fakeDetector>   detector_;
    std::shared_ptr<dynamicPredictor::predictor>     predictor_;
    std::shared_ptr<im2mppi::IM2MPPIPlanner>         mppi_;

    // ── Navigation parameters ──────────────────────────────────────────────
    bool   useFakeDetector_   = false;
    bool   usePredictor_      = false;
    bool   useYawControl_     = false;
    bool   usePredefinedGoal_ = false;
    double desiredVel_        = 1.5;
    double desiredAcc_        = 1.5;
    double desiredAngularVel_ = 0.5;
    int    repeatPathNum_     = 1;
    std::string refTrajPath_;

    nav_msgs::Path predefinedGoal_;  // loaded from file
    int            goalIdx_ = 0;

    // ── Planning state ─────────────────────────────────────────────────────
    bool          mppiReady_    = false;   // planner has produced a valid traj
    ros::Time     trajStartTime_;          // time stamp of last successful plan
    double        facingYaw_    = 0.0;     // yaw direction towards goal

    nav_msgs::Path mppiTrajMsg_;           // last trajectory for visualisation

    // ── Internal helpers ───────────────────────────────────────────────────

    // Timer callbacks
    void mppiCB    (const ros::TimerEvent&);
    void trajExeCB (const ros::TimerEvent&);
    void visCB     (const ros::TimerEvent&);

    // Convert dynamicPredictor::obstacle vector → IM2-MPPI prediction format.
    // Handles time-step interpolation (pred dt=0.1 s → MPPI dt=0.05 s).
    std::vector<im2mppi::DynamicObstaclePrediction> convertPredictions(
        const std::vector<dynamicPredictor::obstacle>& predOb) const;

    // For mean_prediction_mppi: compress K modes into one weighted-mean mode.
    std::vector<im2mppi::DynamicObstaclePrediction> compressToMeanPrediction(
        const std::vector<im2mppi::DynamicObstaclePrediction>& preds) const;

    // Build im2mppi::SphereObstacle list from current dynamic obstacle state
    // (used when predictor is disabled).
    void getDynamicSpheres(std::vector<im2mppi::SphereObstacle>& spheres) const;

    // Load predefined waypoints from text file (dt x y z, one per line).
    nav_msgs::Path loadRefTraj(const std::string& path) const;

    // Build and publish a reference-path nav_msgs::Path for the MPPI planner.
    std::vector<Eigen::Vector3d> buildReferencePath() const;

    // Visualisation helpers
    void publishGoal() const;
};

} // namespace AutoFlight
#endif // IM2_MPPI_NAVIGATION_H
