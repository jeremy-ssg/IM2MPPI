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
    this->nh_.param("tmpc/visualization_period", this->visPeriod_, 0.5);
    this->nh_.param("tmpc/fail_hold_time", this->failHoldTime_, 0.35);
    this->nh_.param("tmpc/brake_time", this->brakeTime_, 0.45);
    this->nh_.param("tmpc/static_post_check_clearance", this->staticExecClearance_, 0.25);
    this->visPeriod_ = std::max(0.05, this->visPeriod_);
    this->failHoldTime_ = std::max(0.0, this->failHoldTime_);
    this->brakeTime_ = std::max(0.1, this->brakeTime_);
    this->staticExecClearance_ = std::max(0.0, this->staticExecClearance_);

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
    this->visTimer_     = this->nh_.createTimer(ros::Duration(this->visPeriod_), &tmpcNavigation::visCB, this);
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
    std::string plan_status = "not_run";

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
        plan_status = this->tmpc_->getLastPlanStatus();
        const ros::WallTime wallEnd = ros::WallTime::now();
        plan_ms = (wallEnd - wallStart).toSec() * 1000.0;

        if (success) {
            this->tmpc_->getBestStates(states);
            this->tmpc_->getLocalReference(this->lastReferencePath_);
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
        this->consecutivePlanFailures_ = 0;
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
        ++this->consecutivePlanFailures_;
        bool keepPrevious = false;
        {
            std::lock_guard<std::mutex> tk(this->trajMutex_);
            if (this->ready_ && !this->activeTraj_.empty() && this->activeTrajDt_ > 1e-6) {
                const double age = (ros::Time::now() - this->trajStartTime_).toSec();
                const double horizon = (double)(this->activeTraj_.size() - 1) * this->activeTrajDt_;
                keepPrevious = age < std::min(horizon, this->failHoldTime_) &&
                               !this->execTrajectoryHitsStaticMap(this->activeTraj_,
                                                                  this->activeTrajDt_);
            }
        }
        if (keepPrevious) {
            ROS_WARN_THROTTLE(1.0,
                "[T-MPC++ Nav] plan() failed (%s); holding previous trajectory briefly (%d failures).",
                plan_status.c_str(), this->consecutivePlanFailures_);
            return;
        }

        std::vector<ExecPoint> brakeTraj;
        if (this->buildBrakeTrajectory(brakeTraj, traj_dt, this->brakeTime_)) {
            std::lock_guard<std::mutex> tk(this->trajMutex_);
            this->activeTraj_ = brakeTraj;
            this->activeTrajDt_ = traj_dt;
            this->activeFacingYaw_ = snapshot_facing_yaw;
            this->trajStartTime_ = ros::Time::now();
            this->ready_ = true;
            ROS_WARN_THROTTLE(1.0,
                "[T-MPC++ Nav] plan() failed (%s); switching to smooth brake trajectory (%d failures).",
                plan_status.c_str(), this->consecutivePlanFailures_);
            return;
        }

        ROS_WARN_THROTTLE(1.0,
            "[T-MPC++ Nav] plan() failed (%s); brake trajectory unsafe, stopping.",
            plan_status.c_str());
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

    {
        std::vector<ExecPoint> immediate(2);
        immediate[0].p = this->currPos_;
        immediate[1].p = Eigen::Vector3d(target.position.x,
                                         target.position.y,
                                         target.position.z);
        if (this->execTrajectoryHitsStaticMap(immediate, 0.0)) {
            {
                std::lock_guard<std::mutex> tk(this->trajMutex_);
                this->ready_ = false;
                this->activeTraj_.clear();
            }
            ROS_ERROR_THROTTLE(0.5,
                "[T-MPC++ Nav] active setpoint intersects static map; stopping before publish.");
            this->stop();
            return;
        }
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
    const bool needBest = this->bestTrajPub_.getNumSubscribers() > 0;
    const bool needRef  = this->refPathPub_.getNumSubscribers() > 0;
    const bool needPlannerViz = this->tmpc_ && this->tmpc_->hasVisualizationSubscribers();
    if (!needBest && !needRef && !needPlannerViz) return;

    std::unique_lock<std::mutex> lk(this->planMutex_, std::try_to_lock);
    if (!lk.owns_lock()) return;
    if (needBest) this->publishBestTrajectory();
    if (needRef)  this->publishReferencePath();
    // Planner-owned markers. Dynamic obstacles are shown as detector bounding boxes
    // in RViz; T-MPC++ itself only consumes constant-velocity bbox states.
    if (needPlannerViz) {
        this->tmpc_->publishGuidancePaths();
        this->tmpc_->publishOptimizedTrajectories();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────
void tmpcNavigation::getObstacles(std::vector<Eigen::Vector3d>& pos,
                                  std::vector<Eigen::Vector3d>& vel,
                                  std::vector<Eigen::Vector3d>& size) const {
    pos.clear(); vel.clear(); size.clear();
    if (!this->useFakeDetector_) {
        if (this->map_) this->map_->getDynamicObstacles(pos, vel, size);
        return;
    }
    if (!this->detector_) return;

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

bool tmpcNavigation::buildBrakeTrajectory(std::vector<ExecPoint>& traj,
                                          double dt,
                                          double duration) const {
    traj.clear();
    dt = std::max(0.02, dt);
    duration = std::max(dt, duration);
    const int n = std::max(3, (int)std::ceil(duration / dt));
    const Eigen::Vector3d p0 = this->currPos_;
    const Eigen::Vector3d v0 = this->currVel_;

    traj.reserve(n + 1);
    for (int k = 0; k <= n; ++k) {
        const double t = std::min(duration, (double)k * dt);
        const double a = std::max(0.0, std::min(1.0, t / duration));
        ExecPoint ep;
        ep.p = p0 + v0 * (t - 0.5 * t * a);
        ep.v = v0 * (1.0 - a);
        traj.push_back(ep);
    }
    if (this->execTrajectoryHitsStaticMap(traj, dt)) {
        traj.clear();
        return false;
    }
    return true;
}

bool tmpcNavigation::execTrajectoryHitsStaticMap(const std::vector<ExecPoint>& traj,
                                                double dt) const {
    if (!this->map_) return false;
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();
    bool havePrev = false;
    const double step = std::max(0.05, this->map_->getRes());
    const double rampTime = 0.25;
    const double dtForRamp = std::max(0.0, dt);
    int k = 0;
    double prevMargin = 0.0;
    for (const auto& pt : traj) {
        const double margin = (dtForRamp <= 1e-6)
            ? 0.0
            : this->staticExecClearance_ *
              std::min(1.0, ((double)k * dtForRamp) / rampTime);
        if (this->execPointHitsStaticMapWithMargin(pt.p, margin)) return true;
        if (havePrev && (pt.p - prev).norm() > 1e-4) {
            if (this->map_->isInflatedOccupiedLine(prev, pt.p)) return true;
            const int samples = std::max(1, (int)std::ceil((pt.p - prev).norm() / step));
            for (int i = 1; i < samples; ++i) {
                const double u = (double)i / (double)samples;
                const double sampleMargin = prevMargin + u * (margin - prevMargin);
                if (this->execPointHitsStaticMapWithMargin(
                        prev + u * (pt.p - prev), sampleMargin)) {
                    return true;
                }
            }
        }
        prev = pt.p;
        prevMargin = margin;
        havePrev = true;
        ++k;
    }
    return false;
}

bool tmpcNavigation::execPointHitsStaticMapWithMargin(const Eigen::Vector3d& p,
                                                      double margin) const {
    if (!this->map_) return false;
    if (this->map_->isInflatedOccupied(p)) return true;
    if (margin <= 1e-6) return false;

    const double step = std::max(0.05, this->map_->getRes());
    const int radialSteps = std::max(1, (int)std::ceil(margin / step));
    const int dirs = 8;
    for (int r = 1; r <= radialSteps; ++r) {
        const double radius = std::min(margin, step * (double)r);
        Eigen::Vector3d qz = p;
        qz.z() += radius;
        if (this->map_->isInflatedOccupied(qz)) return true;
        qz = p;
        qz.z() -= radius;
        if (this->map_->isInflatedOccupied(qz)) return true;
        for (int d = 0; d < dirs; ++d) {
            const double th = 2.0 * M_PI * (double)d / (double)dirs;
            Eigen::Vector3d q = p;
            q.x() += radius * std::cos(th);
            q.y() += radius * std::sin(th);
            if (this->map_->isInflatedOccupied(q)) return true;
        }
    }
    return false;
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
