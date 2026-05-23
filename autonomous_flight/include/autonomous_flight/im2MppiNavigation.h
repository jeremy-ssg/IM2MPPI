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

    Reference-path strategy:
        Local horizon path — slice predefined path from nearest waypoint
        forward by `desired_velocity × horizon_steps × dt` metres.
        The end of this slice is fed to MPPI as the terminal goal.
        Global goal (last predefined waypoint) is used only for yaw facing.

    Visualization (RViz topics, all under node namespace):
        ~im2mppi/best_trajectory          (nav_msgs/Path)         green
        ~im2mppi/sampled_rollouts         (visualization_msgs/MarkerArray)
        ~im2mppi/reference_path           (nav_msgs/Path)         yellow
        ~im2mppi/dynamic_obstacle_predictions (MarkerArray)
        ~im2mppi/goal                     (MarkerArray)           global goal
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
#include <algorithm>
#include <limits>

#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
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
    ros::Timer mppiTimer_;     // planning loop  (~10 Hz)
    ros::Timer trajExeTimer_;  // target publishing (100 Hz)
    ros::Timer visTimer_;      // RViz visualization (~5 Hz)
    ros::Timer predTimer_;     // prediction loop (~5 Hz, decoupled from planning)

    // ── Publishers ─────────────────────────────────────────────────────────
    ros::Publisher bestTrajPub_;     // nav_msgs/Path  — current MPPI output
    ros::Publisher rolloutsPub_;     // MarkerArray    — sampled trajectory cloud
    ros::Publisher refPathPub_;      // nav_msgs/Path  — local horizon reference
    ros::Publisher dynObsPredPub_;   // MarkerArray    — dynamic obstacle modes
    ros::Publisher goalPub_;         // MarkerArray    — global goal sphere
    ros::Publisher planTimePub_;     // std_msgs/Float64 — plan() duration in ms

    // ── Component modules ──────────────────────────────────────────────────
    std::shared_ptr<mapManager::dynamicMap>          map_;
    std::shared_ptr<onboardDetector::fakeDetector>   detector_;
    std::shared_ptr<dynamicPredictor::predictor>     predictor_;
    std::shared_ptr<im2mppi::IM2MPPIPlanner>         mppi_;

    // ── Navigation parameters ──────────────────────────────────────────────
    bool        useFakeDetector_         = false;
    bool        usePredictor_            = false;
    bool        useYawControl_           = false;
    bool        usePredefinedGoal_       = false;
    bool        closedLoopIntentEnabled_ = true;   // Phase-4: Bayesian π correction
    double      closedLoopMatchDistance_ = 1.0;    // m — obstacle id matching window
    double      desiredVel_              = 1.5;
    double      desiredAcc_              = 1.5;
    double      desiredAngularVel_       = 0.5;
    int         repeatPathNum_           = 1;
    std::string refTrajPath_;

    nav_msgs::Path predefinedGoal_;

    // ── Planning state ─────────────────────────────────────────────────────
    bool          mppiReady_  = false;
    ros::Time     trajStartTime_;
    double        facingYaw_  = 0.0;

    // Snapshot consumed by trajExeCB. It lets the 100-Hz target stream keep
    // following the last valid trajectory while plan() computes the next one.
    std::mutex                            trajMutex_;
    std::vector<im2mppi::TrajectoryPoint> activeTraj_;
    double                                activeTrajDt_ = 0.05;
    double                                activeFacingYaw_ = 0.0;
    bool                                  activeUseYawPostprocess_ = true;

    // Output yaw rate limiter state (trajExeCB). Caps how fast target.yaw
    // can change per 100-Hz tick so MPPI's potentially noisy yaw reference
    // doesn't translate into physical yaw flapping.
    double        lastTargetYaw_     = 0.0;
    ros::Time     lastTargetYawTime_;
    bool          targetYawInit_     = false;

    std::vector<Eigen::Vector3d> lastReferencePath_;

    // ── Prediction cache (updated by predTimer_ at ~5 Hz) ──────────────────
    // Decoupled from mppiTimer_ so slow inference doesn't block planning.
    std::mutex                                      predMutex_;
    std::vector<im2mppi::DynamicObstaclePrediction> cachedDynPreds_;

    // ── Closed-loop intent correction state (Phase 4 / innovation #5) ─────
    // We compare each new predictor output against the previous tick's
    // prediction at the time-step corresponding to "now" and reweight the
    // intent posterior by the per-mode Gaussian likelihood of the actual
    // observation. Held only inside predCB so no extra mutex needed.
    std::vector<dynamicPredictor::obstacle> lastPredOb_;
    ros::Time                               lastPredTime_;

    // Protects planner mutation/visualization reads. trajExeCB reads the
    // active trajectory snapshot under trajMutex_ instead.
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

    // For DRA-MPPI baseline: collapse explicit intent branches into one
    // marginal Gaussian per obstacle, preserving mixture variance.
    std::vector<im2mppi::DynamicObstaclePrediction> compressToMomentMatchedPrediction(
        const std::vector<im2mppi::DynamicObstaclePrediction>& preds) const;

    // Bayesian intent posterior update: π_corrected ∝ π_prior · likelihood,
    // where likelihood_m = N(observed_now; μ_m_predicted_for_now, σ_m).
    // Modifies newPred in place. Falls back to the predictor's prior when
    // no matching previous prediction is found for an obstacle.
    void applyClosedLoopIntentCorrection(
        std::vector<dynamicPredictor::obstacle>& newPred,
        double dt_since_last) const;

    // Fallback static AABBs when predictor disabled.
    void getDynamicBoxes(std::vector<im2mppi::BoxObstacle>& boxes) const;

    // Build local horizon reference path (sliced from predefined waypoints).
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
