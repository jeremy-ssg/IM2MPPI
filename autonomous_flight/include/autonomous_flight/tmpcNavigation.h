/*
    FILE: tmpcNavigation.h
    --------------------------------
    T-MPC++ navigation orchestrator (benchmark method M6_tmpc).

    Mirrors im2MppiNavigation but drives trajPlanner::tmpcPlanner:
        onboardDetector::fakeDetector  -> dynamic obstacle boxes + velocity
        trajPlanner::tmpcPlanner       -> topology-driven parallel MPC
        tracking_controller::Target    -> downstream tracking controller

    Obstacle prediction is constant-velocity (handled inside tmpcPlanner).
    No dynamic_predictor / intent modes are used for this baseline.

    Visualization (RViz topics, all under node namespace):
        ~tmpc/best_trajectory             (nav_msgs/Path)  executed winner
        ~tmpc/reference_path              (nav_msgs/Path)  local horizon reference
        ~tmpc/plan_time_ms               (std_msgs/Float64)
        ~tmpc/guidance_paths             (MarkerArray)    P guidance trajectories  [planner]
        ~tmpc/optimized_trajectories     (MarkerArray)    P+1 optimized branches   [planner]
        ~tmpc/goal                       (MarkerArray)    goal grid                [planner]
*/

#ifndef AUTOFLIGHT_TMPC_NAVIGATION_H
#define AUTOFLIGHT_TMPC_NAVIGATION_H

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

#include <trajectory_planner/tmpc/tmpcPlanner.h>

namespace AutoFlight {

class tmpcNavigation : public flightBase {
public:
    explicit tmpcNavigation(const ros::NodeHandle& nh);

    void initParam();
    void initModules();
    void registerPub();
    void registerCallback();
    void run();

private:
    // ── timers ───────────────────────────────────────────────────────────────
    ros::Timer planTimer_;     // ~10 Hz planning loop
    ros::Timer trajExeTimer_;  // 100 Hz target publishing
    ros::Timer visTimer_;      // ~5 Hz visualization

    // ── publishers ────────────────────────────────────────────────────────────
    ros::Publisher bestTrajPub_;     // nav_msgs/Path
    ros::Publisher refPathPub_;      // nav_msgs/Path
    ros::Publisher planTimePub_;     // std_msgs/Float64

    // ── modules ───────────────────────────────────────────────────────────────
    std::shared_ptr<mapManager::dynamicMap>        map_;
    std::shared_ptr<onboardDetector::fakeDetector> detector_;
    std::shared_ptr<trajPlanner::tmpcPlanner>      tmpc_;

    // ── parameters ────────────────────────────────────────────────────────────
    bool        useFakeDetector_   = false;
    bool        useYawControl_     = false;
    bool        usePredefinedGoal_ = false;
    double      desiredVel_        = 1.5;
    int         repeatPathNum_     = 1;
    double      visPeriod_         = 0.5;
    double      failHoldTime_      = 0.35;
    double      brakeTime_         = 0.45;
    std::string refTrajPath_;
    nav_msgs::Path predefinedGoal_;

    // ── execution snapshot (one trajectory point = position + velocity) ─────────
    struct ExecPoint { Eigen::Vector3d p = Eigen::Vector3d::Zero();
                       Eigen::Vector3d v = Eigen::Vector3d::Zero(); };

    std::mutex             planMutex_;
    std::mutex             trajMutex_;
    bool                   ready_ = false;
    std::vector<ExecPoint> activeTraj_;
    double                 activeTrajDt_ = 0.05;
    double                 activeFacingYaw_ = 0.0;
    ros::Time              trajStartTime_;
    double                 facingYaw_ = 0.0;
    int                    consecutivePlanFailures_ = 0;
    std::vector<Eigen::Vector3d> lastReferencePath_;

    // yaw rate limiter state
    bool      targetYawInit_ = false;
    double    lastTargetYaw_ = 0.0;
    ros::Time lastTargetYawTime_;

    // ── callbacks ───────────────────────────────────────────────────────────────
    void planCB(const ros::TimerEvent&);
    void trajExeCB(const ros::TimerEvent&);
    void visCB(const ros::TimerEvent&);

    // ── helpers ──────────────────────────────────────────────────────────────────
    nav_msgs::Path loadRefTraj(const std::string& path) const;
    std::vector<Eigen::Vector3d> refPathAsVector() const;
    void getObstacles(std::vector<Eigen::Vector3d>& pos,
                      std::vector<Eigen::Vector3d>& vel,
                      std::vector<Eigen::Vector3d>& size) const;
    ExecPoint sampleSnapshot(const std::vector<ExecPoint>& traj, double dt, double t) const;
    bool buildBrakeTrajectory(std::vector<ExecPoint>& traj, double dt, double duration) const;
    bool execTrajectoryHitsStaticMap(const std::vector<ExecPoint>& traj) const;
    void publishBestTrajectory() const;
    void publishReferencePath() const;
};

} // namespace AutoFlight
#endif // AUTOFLIGHT_TMPC_NAVIGATION_H
