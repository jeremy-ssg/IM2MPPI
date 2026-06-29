/*
    FILE: tmpcNavigation.cpp
    --------------------------------
    Implementation of tmpcNavigation (T-MPC++, benchmark method M6_tmpc).
    Mirrors im2MppiNavigation; drives trajPlanner::tmpcPlanner with constant-
    velocity obstacle predictions. See trajectory_planner/docs/TMPC_INTEGRATION.md
*/

#include <autonomous_flight/tmpcNavigation.h>

namespace AutoFlight {

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
tmpcNavigation::tmpcNavigation(const ros::NodeHandle& nh) : flightBase(nh) {
    this->initParam();
    this->initModules();
    this->registerPub();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameters
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::initParam() {
    if (!this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_))
        this->useFakeDetector_ = false;
    if (!this->nh_.getParam("autonomous_flight/use_yaw_control", this->useYawControl_))
        this->useYawControl_ = false;
    this->nh_.param("autonomous_flight/desired_velocity", this->desiredVel_, 1.5);
    this->nh_.param("autonomous_flight/use_predefined_goal", this->usePredefinedGoal_, false);

    if (this->usePredefinedGoal_) {
        if (!this->nh_.getParam("autonomous_flight/predefined_goal_directory", this->refTrajPath_)) {
            this->refTrajPath_ = "None";
            ROS_WARN("[T-MPC++ Nav] predefined_goal_directory not set.");
        } else {
            this->refTrajPath_ = ros::package::getPath("autonomous_flight") + this->refTrajPath_;
        }
        this->nh_.param("autonomous_flight/execute_path_times", this->repeatPathNum_, 1);
        this->predefinedGoal_ = this->loadRefTraj(this->refTrajPath_);
        if (!this->predefinedGoal_.poses.empty())
            this->goal_ = this->predefinedGoal_.poses.back();
    }
    ROS_INFO("[T-MPC++ Nav] use_fake_detector=%d use_predefined_goal=%d",
             (int)this->useFakeDetector_, (int)this->usePredefinedGoal_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modules
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::initModules() {
    if (this->useFakeDetector_) {
        this->detector_.reset(new onboardDetector::fakeDetector(this->nh_));
        this->map_.reset(new mapManager::dynamicMap(this->nh_, false));
    } else {
        this->map_.reset(new mapManager::dynamicMap(this->nh_));
    }

    this->tmpc_.reset(new trajPlanner::tmpcPlanner(this->nh_));
    this->tmpc_->initParam();
    this->tmpc_->setMap(this->map_);
    this->tmpc_->registerPub();

    // The full lap reference is static; store it once. tmpcPlanner re-derives the
    // local horizon reference + goal grid from the current state on each plan().
    if (this->usePredefinedGoal_)
        this->tmpc_->setReference(this->refPathAsVector());

    ROS_INFO("[T-MPC++ Nav] planner ready.");
}

void tmpcNavigation::registerPub() {
    this->bestTrajPub_ = this->nh_.advertise<nav_msgs::Path>("tmpc/best_trajectory", 10);
    this->refPathPub_  = this->nh_.advertise<nav_msgs::Path>("tmpc/reference_path", 10);
    this->planTimePub_ = this->nh_.advertise<std_msgs::Float64>("tmpc/plan_time_ms", 50);
}

void tmpcNavigation::registerCallback() {
    this->planTimer_    = this->nh_.createTimer(ros::Duration(0.1),  &tmpcNavigation::planCB,    this);
    this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &tmpcNavigation::trajExeCB, this);
    this->visTimer_     = this->nh_.createTimer(ros::Duration(0.2),  &tmpcNavigation::visCB,     this);
}

void tmpcNavigation::run() {
    this->takeoff();
    this->registerCallback();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Planning callback (~10 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::planCB(const ros::TimerEvent&) {
    if (!this->goalReceived_ && !this->usePredefinedGoal_) return;
    if (!this->odomReceived_) return;

    bool success = false;
    double plan_ms = 0.0;
    ros::Time planStart;
    double traj_dt = 0.05;
    double snapshot_facing_yaw = this->facingYaw_;
    std::vector<Eigen::VectorXd> states;

    {
        std::lock_guard<std::mutex> lk(this->planMutex_);

        // 1. current state (yaw from odom)
        const double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
        this->tmpc_->updateCurrStates(this->currPos_, this->currVel_, yaw);

        // 2. obstacles (constant-velocity predictions built inside the planner)
        std::vector<Eigen::Vector3d> oPos, oVel, oSize;
        this->getObstacles(oPos, oVel, oSize);
        this->tmpc_->setObstacles(oPos, oVel, oSize);

        // 3. reference path for the planner. Predefined lap if available, otherwise a
        //    straight line from the current position to the active goal. If neither is
        //    available the reference is empty and plan() skips (drone holds position) —
        //    this prevents the empty-reference garbage that flew the drone away.
        std::vector<Eigen::Vector3d> ref;
        if (this->usePredefinedGoal_ && !this->predefinedGoal_.poses.empty()) {
            ref = this->refPathAsVector();
        } else if (this->goalReceived_) {
            const Eigen::Vector3d g(this->goal_.pose.position.x,
                                    this->goal_.pose.position.y,
                                    this->goal_.pose.position.z);
            const int n = 20;
            for (int s = 0; s <= n; ++s)
                ref.push_back(this->currPos_ + (g - this->currPos_) * ((double)s / n));
        }
        if (ref.empty()) {
            ROS_WARN_THROTTLE(2.0, "[T-MPC++ Nav] no reference path "
                "(use_predefined_goal=%d, ref waypoints=%zu, goalReceived=%d); holding.",
                (int)this->usePredefinedGoal_, this->predefinedGoal_.poses.size(),
                (int)this->goalReceived_);
        }
        this->tmpc_->setReference(ref);
        this->lastReferencePath_ = ref;   // also drives the /tmpc/reference_path viz

        // 4. facing yaw toward the local horizon end
        std::vector<Eigen::Vector3d> bestPos;
        // (computed after plan; facing handled below)

        // 5. plan
        planStart = ros::Time::now();
        const ros::WallTime wallStart = ros::WallTime::now();
        success = this->tmpc_->plan();
        const ros::WallTime wallEnd = ros::WallTime::now();
        plan_ms = (wallEnd - wallStart).toSec() * 1000.0;

        if (success) {
            this->tmpc_->getBestStates(states);
            traj_dt = this->tmpc_->getDt();
            if (!states.empty()) {
                const Eigen::Vector3d endP(states.back()(0), states.back()(1), states.back()(2));
                Eigen::Vector3d gv = endP - this->currPos_;
                if (gv.head<2>().norm() > 0.3) this->facingYaw_ = std::atan2(gv.y(), gv.x());
                snapshot_facing_yaw = this->facingYaw_;
            }
        }
    }

    std_msgs::Float64 pt_msg; pt_msg.data = plan_ms;
    this->planTimePub_.publish(pt_msg);

    if (success && !states.empty()) {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        this->activeTraj_.clear();
        this->activeTraj_.reserve(states.size());
        for (const auto& s : states) {
            ExecPoint ep;
            ep.p = Eigen::Vector3d(s(0), s(1), s(2));
            ep.v = Eigen::Vector3d(s(3), s(4), s.size() > 5 ? s(5) : 0.0);
            this->activeTraj_.push_back(ep);
        }
        this->activeTrajDt_    = traj_dt;
        this->activeFacingYaw_ = snapshot_facing_yaw;
        this->trajStartTime_   = planStart;
        this->ready_           = true;
    } else {
        ROS_WARN_THROTTLE(1.0, "[T-MPC++ Nav] plan() failed.");
        {
            std::lock_guard<std::mutex> tk(this->trajMutex_);
            this->ready_ = false;
            this->activeTraj_.clear();
        }
        this->stop();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory execution (100 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::trajExeCB(const ros::TimerEvent&) {
    std::vector<ExecPoint> traj;
    ros::Time trajStartTime;
    double traj_dt = 0.05;
    double snapshot_facing_yaw = 0.0;

    {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        if (!this->ready_ || this->activeTraj_.empty()) return;
        traj = this->activeTraj_;
        trajStartTime = this->trajStartTime_;
        traj_dt = this->activeTrajDt_;
        snapshot_facing_yaw = this->activeFacingYaw_;
    }

    const double endTime  = std::max(0.0, (double)(traj.size() - 1) * traj_dt);
    const double realTime = std::max(0.0, (ros::Time::now() - trajStartTime).toSec());

    tracking_controller::Target target;
    double raw_yaw = snapshot_facing_yaw;

    if (realTime >= endTime) {
        const ExecPoint pt = this->sampleSnapshot(traj, traj_dt, endTime);
        target.position.x = pt.p.x(); target.position.y = pt.p.y(); target.position.z = pt.p.z();
        target.velocity.x = target.velocity.y = target.velocity.z = 0.0;
        target.acceleration.x = target.acceleration.y = target.acceleration.z = 0.0;
    } else {
        const ExecPoint pt = this->sampleSnapshot(traj, traj_dt, realTime);
        target.position.x = pt.p.x(); target.position.y = pt.p.y(); target.position.z = pt.p.z();
        target.velocity.x = pt.v.x(); target.velocity.y = pt.v.y(); target.velocity.z = pt.v.z();
        target.acceleration.x = target.acceleration.y = target.acceleration.z = 0.0;
    }

    // yaw rate limiter (same scheme as im2MppiNavigation)
    {
        const ros::Time now = ros::Time::now();
        if (!this->targetYawInit_) {
            this->lastTargetYaw_     = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
            this->lastTargetYawTime_ = now;
            this->targetYawInit_     = true;
        }
        const double dt_y = std::max(0.001, (now - this->lastTargetYawTime_).toSec());
        constexpr double YAW_RATE_MAX = M_PI;
        const double max_dy = YAW_RATE_MAX * dt_y;
        double dy = raw_yaw - this->lastTargetYaw_;
        while (dy >  M_PI) dy -= 2.0 * M_PI;
        while (dy < -M_PI) dy += 2.0 * M_PI;
        dy = std::max(-max_dy, std::min(max_dy, dy));
        const double limited = this->lastTargetYaw_ + dy;
        target.yaw               = static_cast<float>(limited);
        this->lastTargetYaw_     = limited;
        this->lastTargetYawTime_ = now;
    }

    this->updateTargetWithState(target);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Visualization (~5 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::visCB(const ros::TimerEvent&) {
    {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        if (!this->ready_) return;
    }
    std::lock_guard<std::mutex> lk(this->planMutex_);
    if (this->bestTrajPub_.getNumSubscribers() > 0) this->publishBestTrajectory();
    if (this->refPathPub_.getNumSubscribers()  > 0) this->publishReferencePath();
    // planner-owned markers (guidance / optimized / goal grid / obstacle predictions)
    this->tmpc_->publishGuidancePaths();
    this->tmpc_->publishOptimizedTrajectories();
    this->tmpc_->publishObstaclePredictions();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::getObstacles(std::vector<Eigen::Vector3d>& pos,
                                  std::vector<Eigen::Vector3d>& vel,
                                  std::vector<Eigen::Vector3d>& size) const {
    pos.clear(); vel.clear(); size.clear();
    if (!this->useFakeDetector_ || !this->detector_) return;

    Eigen::Vector3d robotSize(0.0, 0.0, 0.0);
    if (this->map_) this->map_->getRobotSize(robotSize);

    std::vector<onboardDetector::box3D> boxes;
    this->detector_->getObstaclesInSensorRange(2.0 * M_PI, boxes, robotSize);

    for (const auto& b : boxes) {
        pos.emplace_back(b.x, b.y, b.z);
        vel.emplace_back(b.Vx, b.Vy, 0.0);                 // constant-velocity (planar)
        size.emplace_back(b.x_width, b.y_width, b.z_width);
    }
}

std::vector<Eigen::Vector3d> tmpcNavigation::refPathAsVector() const {
    std::vector<Eigen::Vector3d> out;
    out.reserve(this->predefinedGoal_.poses.size());
    for (const auto& ps : this->predefinedGoal_.poses)
        out.emplace_back(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    return out;
}

tmpcNavigation::ExecPoint tmpcNavigation::sampleSnapshot(
    const std::vector<ExecPoint>& traj, double dt, double t) const {
    ExecPoint out;
    if (traj.empty()) return out;
    if (traj.size() == 1 || t <= 0.0 || dt <= 1e-6) return traj.front();
    const double horizon_time = (double)(traj.size() - 1) * dt;
    if (t >= horizon_time) return traj.back();
    const double scaled = t / dt;
    const int k = std::max(0, std::min((int)std::floor(scaled), (int)traj.size() - 2));
    const double a = std::max(0.0, std::min(1.0, scaled - (double)k));
    out.p = traj[k].p + a * (traj[k + 1].p - traj[k].p);
    out.v = traj[k].v + a * (traj[k + 1].v - traj[k].v);
    return out;
}

nav_msgs::Path tmpcNavigation::loadRefTraj(const std::string& path) const {
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("[T-MPC++ Nav] cannot open ref trajectory: %s", path.c_str());
        return msg;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double dt_val, x, y, z;
        if (!(iss >> dt_val >> x >> y >> z)) continue;
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    ROS_INFO("[T-MPC++ Nav] loaded %zu waypoints", msg.poses.size());
    return msg;
}

void tmpcNavigation::publishBestTrajectory() const {
    nav_msgs::Path msg;
    this->tmpc_->getBestTrajectory(msg);
    this->bestTrajPub_.publish(msg);
}

void tmpcNavigation::publishReferencePath() const {
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();
    for (const auto& p : this->lastReferencePath_) {
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    this->refPathPub_.publish(msg);
}

} // namespace AutoFlight
