/*
    FILE: im2MppiNavigation.cpp
    --------------------------------
    Implementation of im2MppiNavigation (Phases 1 – 3 + RViz visualization).
*/

#include <autonomous_flight/im2MppiNavigation.h>

namespace AutoFlight {

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

im2MppiNavigation::im2MppiNavigation(const ros::NodeHandle& nh)
    : flightBase(nh)
{
    this->initParam();
    this->initModules();
    this->registerPub();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter loading
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::initParam()
{
    if (!this->nh_.getParam("autonomous_flight/use_fake_detector",
                            this->useFakeDetector_)) {
        this->useFakeDetector_ = false;
    }
    ROS_INFO("[IM2-MPPI Nav] use_fake_detector = %d", this->useFakeDetector_);

    if (!this->nh_.getParam("autonomous_flight/use_predictor",
                            this->usePredictor_)) {
        this->usePredictor_ = false;
    }
    ROS_INFO("[IM2-MPPI Nav] use_predictor      = %d", this->usePredictor_);

    if (!this->nh_.getParam("autonomous_flight/use_yaw_control",
                            this->useYawControl_)) {
        this->useYawControl_ = false;
    }

    this->nh_.param("autonomous_flight/desired_velocity",         this->desiredVel_,        1.5);
    this->nh_.param("autonomous_flight/desired_acceleration",     this->desiredAcc_,        1.5);
    this->nh_.param("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_, 0.5);

    this->nh_.param("autonomous_flight/use_predefined_goal", this->usePredefinedGoal_, false);

    if (this->usePredefinedGoal_) {
        if (!this->nh_.getParam("autonomous_flight/predefined_goal_directory",
                                this->refTrajPath_)) {
            this->refTrajPath_ = "None";
            ROS_WARN("[IM2-MPPI Nav] predefined_goal_directory not set.");
        } else {
            std::string pkgPath = ros::package::getPath("autonomous_flight");
            this->refTrajPath_  = pkgPath + this->refTrajPath_;
        }
        this->nh_.param("autonomous_flight/execute_path_times", this->repeatPathNum_, 1);
        this->predefinedGoal_ = this->loadRefTraj(this->refTrajPath_);
        if (!this->predefinedGoal_.poses.empty()) {
            // Sliding-waypoint mode: start from the first waypoint, not the last.
            this->goalIdx_ = 0;
            this->goal_    = this->predefinedGoal_.poses.front();
        }
    }

    // Waypoint-switch distance (used in mppiCB to advance goalIdx_).
    this->nh_.param("im2_mppi/waypoint_switch_dist", this->waypointSwitchDist_, 1.0);
    ROS_INFO("[IM2-MPPI Nav] waypoint_switch_dist = %.2f m", this->waypointSwitchDist_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Module init
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::initModules()
{
    if (this->useFakeDetector_) {
        this->detector_.reset(new onboardDetector::fakeDetector(this->nh_));
        this->map_.reset(new mapManager::dynamicMap(this->nh_, false));
    } else {
        this->map_.reset(new mapManager::dynamicMap(this->nh_));
    }

    if (this->usePredictor_) {
        this->predictor_.reset(new dynamicPredictor::predictor(this->nh_));
        this->predictor_->setMap(this->map_);
        if (this->useFakeDetector_) {
            this->predictor_->setDetector(this->detector_);
        }
    }

    this->mppi_.reset(new im2mppi::IM2MPPIPlanner(this->nh_));
    ROS_INFO("[IM2-MPPI Nav] Planner ready. method_type = %s",
             this->mppi_->getParams().method_type.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Publishers
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerPub()
{
    this->bestTrajPub_  = this->nh_.advertise<nav_msgs::Path>(
        "im2mppi/best_trajectory", 10);
    this->rolloutsPub_  = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/sampled_rollouts", 10);
    this->refPathPub_   = this->nh_.advertise<nav_msgs::Path>(
        "im2mppi/reference_path", 10);
    this->dynObsPredPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/dynamic_obstacle_predictions", 10);
    this->goalPub_      = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/goal", 10);
    this->waypointPub_  = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/waypoints", 10);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timers
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerCallback()
{
    this->mppiTimer_    = this->nh_.createTimer(ros::Duration(0.05),
                            &im2MppiNavigation::mppiCB, this);
    this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01),
                            &im2MppiNavigation::trajExeCB, this);
    this->visTimer_     = this->nh_.createTimer(ros::Duration(0.05),
                            &im2MppiNavigation::visCB, this);
    // Prediction at 5 Hz — decoupled so slow inference doesn't block planning
    if (this->usePredictor_) {
        this->predTimer_ = this->nh_.createTimer(ros::Duration(0.2),
                            &im2MppiNavigation::predCB, this);
    }
}

void im2MppiNavigation::run()
{
    this->takeoff();
    this->registerCallback();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Planning callback (20 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::mppiCB(const ros::TimerEvent&)
{
    if (!this->goalReceived_ && !this->usePredefinedGoal_) return;
    if (!this->odomReceived_) return;

    // Lock planner for the whole iteration — trajExeCB/visCB will wait.
    std::lock_guard<std::mutex> lk(this->planMutex_);

    // 1. Current state
    this->mppi_->setCurrentState(this->currPos_, this->currVel_);

    // 2. Sliding-waypoint: advance goalIdx_ when within switch distance
    if (this->usePredefinedGoal_ && !this->predefinedGoal_.poses.empty()) {
        const int totalWPs = static_cast<int>(this->predefinedGoal_.poses.size());
        // Advance as long as we are close enough AND there are more waypoints
        while (this->goalIdx_ < totalWPs - 1) {
            const auto& wp = this->predefinedGoal_.poses[this->goalIdx_];
            const Eigen::Vector3d wpPos(wp.pose.position.x,
                                        wp.pose.position.y,
                                        wp.pose.position.z);
            if ((this->currPos_ - wpPos).norm() < this->waypointSwitchDist_) {
                this->goalIdx_++;
                ROS_INFO("[IM2-MPPI Nav] → waypoint %d / %d",
                         this->goalIdx_, totalWPs - 1);
            } else {
                break;
            }
        }
        // Always sync goal_ to current sliding target
        this->goal_ = this->predefinedGoal_.poses[this->goalIdx_];
    }

    const Eigen::Vector3d goalEigen(this->goal_.pose.position.x,
                                    this->goal_.pose.position.y,
                                    this->goal_.pose.position.z);
    this->mppi_->setGoal(goalEigen);

    // Update facing yaw for yaw control
    Eigen::Vector3d gv = goalEigen - this->currPos_;
    if (gv.head<2>().norm() > 0.1) {
        this->facingYaw_ = std::atan2(gv.y(), gv.x());
    }

    // 3. Reference path
    this->lastReferencePath_ = this->buildReferencePath();
    this->mppi_->setReferencePath(this->lastReferencePath_);

    // 4. Method-type dispatch — predictions come from async cache (predCB)
    const std::string& method = this->mppi_->getParams().method_type;

    if (this->usePredictor_ && method != "vanilla_mppi") {
        // Read from cache (filled by predCB at 5 Hz); never blocks planning
        std::vector<im2mppi::DynamicObstaclePrediction> dynPreds;
        {
            std::lock_guard<std::mutex> lk(this->predMutex_);
            dynPreds = this->cachedDynPreds_;
        }
        this->mppi_->setDynamicObstaclePredictions(dynPreds);
        this->mppi_->setStaticObstacles({});
    } else {
        // vanilla_mppi or predictor disabled: treat current obstacles as static
        std::vector<im2mppi::SphereObstacle> spheres;
        this->getDynamicSpheres(spheres);
        this->mppi_->setStaticObstacles(spheres);
        this->mppi_->setDynamicObstaclePredictions({});
    }

    // 5. Plan
    const ros::Time planStart = ros::Time::now();
    const bool success = this->mppi_->plan();

    if (success) {
        this->trajStartTime_ = planStart;
        this->mppiReady_     = true;
    } else {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI Nav] plan() failed.");
        this->mppiReady_ = false;
        this->stop();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory execution callback (100 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::trajExeCB(const ros::TimerEvent&)
{
    if (!this->mppiReady_) return;

    // Brief lock to read planner state safely (typical < 0.1 ms).
    std::lock_guard<std::mutex> lk(this->planMutex_);

    const auto&  params  = this->mppi_->getParams();
    const double endTime = static_cast<double>(params.horizon_steps) * params.dt;
    const double realTime = (ros::Time::now() - this->trajStartTime_).toSec();

    tracking_controller::Target target;

    if (realTime >= endTime) {
        const Eigen::Vector3d p = this->mppi_->getPos(endTime);
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        target.velocity.x = target.velocity.y = target.velocity.z = 0.0;
        target.acceleration.x = target.acceleration.y = target.acceleration.z = 0.0;
        target.yaw = AutoFlight::rpy_from_quaternion(
            this->odom_.pose.pose.orientation);
    } else {
        const Eigen::Vector3d p   = this->mppi_->getPos(realTime);
        const Eigen::Vector3d v   = this->mppi_->getVel(realTime);
        const Eigen::Vector3d acc = this->mppi_->getAcc(realTime);

        target.position.x     = p.x();
        target.position.y     = p.y();
        target.position.z     = p.z();
        target.velocity.x     = v.x();
        target.velocity.y     = v.y();
        target.velocity.z     = v.z();
        target.acceleration.x = acc.x();
        target.acceleration.y = acc.y();
        target.acceleration.z = acc.z();

        if (this->useYawControl_ && params.use_yaw_postprocess) {
            const auto& traj = this->mppi_->getPlannedTrajectory();
            int k = static_cast<int>(realTime / params.dt);
            k = std::max(0, std::min(k, static_cast<int>(traj.size()) - 1));
            target.yaw = static_cast<float>(traj[k].yaw);
        } else {
            target.yaw = static_cast<float>(this->facingYaw_);
        }
    }

    this->updateTargetWithState(target);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Prediction callback (~5 Hz) — runs inference asynchronously
//  Result is cached in cachedDynPreds_ behind predMutex_.
//  mppiCB reads the cache without blocking.
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::predCB(const ros::TimerEvent&)
{
    if (!this->predictor_) return;

    const std::string& method = this->mppi_->getParams().method_type;
    if (method == "vanilla_mppi") return;

    std::vector<dynamicPredictor::obstacle> predOb;
    this->predictor_->getPrediction(predOb);   // may take 100–200 ms; OK here

    // If predictor returns empty (detector momentarily lost tracks), keep the
    // previous cache so visualization & MPPI don't lose all obstacle info.
    if (predOb.empty()) return;

    std::vector<im2mppi::DynamicObstaclePrediction> dynPreds =
        this->convertPredictions(predOb);
    if (method == "mean_prediction_mppi") {
        dynPreds = this->compressToMeanPrediction(dynPreds);
    }

    {
        std::lock_guard<std::mutex> lk(this->predMutex_);
        this->cachedDynPreds_ = std::move(dynPreds);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Visualization callback (~20 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::visCB(const ros::TimerEvent&)
{
    // Single lock covers all visualization reads:
    //   goal_, goalIdx_, predefinedGoal_ are written by mppiCB under planMutex_
    //   mppi_ state is also protected by planMutex_
    std::lock_guard<std::mutex> lk(this->planMutex_);
    this->publishGoal();
    this->publishWaypoints();
    if (!this->mppiReady_) return;
    this->publishBestTrajectory();
    this->publishSampledRollouts();
    this->publishReferencePath();
    this->publishDynamicObstaclePred();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Prediction conversion
// ═════════════════════════════════════════════════════════════════════════════

std::vector<im2mppi::DynamicObstaclePrediction>
im2MppiNavigation::convertPredictions(
    const std::vector<dynamicPredictor::obstacle>& predOb) const
{
    const double dt_pred  = 0.1;
    const auto&  params   = this->mppi_->getParams();
    const double dt_mppi  = params.dt;
    const int    H        = params.horizon_steps;

    std::vector<im2mppi::DynamicObstaclePrediction> result;
    result.reserve(predOb.size());

    for (size_t j = 0; j < predOb.size(); ++j) {
        const auto& ob = predOb[j];

        im2mppi::DynamicObstaclePrediction pred;
        pred.id = static_cast<int>(j);

        if (!ob.sizePred.empty() &&
            !ob.sizePred[0].empty() &&
            ob.sizePred[0][0].x() > 0.0) {
            const Eigen::Vector3d& sz0 = ob.sizePred[0][0];
            pred.radius = std::max(sz0.x(), sz0.y()) * 0.5;
        } else {
            pred.radius = 0.3;
        }

        const int K = static_cast<int>(ob.intentProb.size());
        pred.modes.reserve(K);

        for (int m = 0; m < K; ++m) {
            im2mppi::ObstacleMode mode;
            mode.pi = ob.intentProb[m];

            if (m >= static_cast<int>(ob.posPred.size())) {
                mode.pi = 0.0;
                mode.mu_seq.assign(H, Eigen::Vector3d::Zero());
                mode.sigma_diag_seq.assign(H, Eigen::Vector3d(0.3, 0.3, 0.3));
                pred.modes.push_back(mode);
                continue;
            }

            const auto& pos_seq  = ob.posPred[m];
            const auto& size_seq = (m < static_cast<int>(ob.sizePred.size()))
                                       ? ob.sizePred[m]
                                       : std::vector<Eigen::Vector3d>{};
            const int   N_pred   = static_cast<int>(pos_seq.size());

            mode.mu_seq.resize(H);
            mode.sigma_diag_seq.resize(H);

            for (int k = 0; k < H; ++k) {
                const double pred_f = static_cast<double>(k) * dt_mppi / dt_pred;
                const int    idx_lo = static_cast<int>(std::floor(pred_f));
                const int    idx_hi = idx_lo + 1;
                const double alpha  = pred_f - static_cast<double>(idx_lo);

                const int lo = std::min(std::max(idx_lo, 0), std::max(N_pred - 1, 0));
                const int hi = std::min(std::max(idx_hi, 0), std::max(N_pred - 1, 0));

                if (N_pred == 0) {
                    mode.mu_seq[k]         = Eigen::Vector3d::Zero();
                    mode.sigma_diag_seq[k] = Eigen::Vector3d(0.3, 0.3, 0.3);
                    continue;
                }

                mode.mu_seq[k] = pos_seq[lo] + alpha * (pos_seq[hi] - pos_seq[lo]);

                if (!size_seq.empty()) {
                    const int slo = std::min(lo, static_cast<int>(size_seq.size()) - 1);
                    const int shi = std::min(hi, static_cast<int>(size_seq.size()) - 1);
                    const Eigen::Vector3d sz =
                        size_seq[slo] + alpha * (size_seq[shi] - size_seq[slo]);
                    mode.sigma_diag_seq[k] = (sz * 0.5).cwiseMax(0.1);
                } else {
                    mode.sigma_diag_seq[k] = Eigen::Vector3d(0.3, 0.3, 0.3);
                }
            }
            pred.modes.push_back(mode);
        }

        result.push_back(std::move(pred));
    }

    return result;
}

std::vector<im2mppi::DynamicObstaclePrediction>
im2MppiNavigation::compressToMeanPrediction(
    const std::vector<im2mppi::DynamicObstaclePrediction>& preds) const
{
    std::vector<im2mppi::DynamicObstaclePrediction> compressed;
    compressed.reserve(preds.size());

    for (const auto& pred : preds) {
        im2mppi::DynamicObstaclePrediction cp;
        cp.id     = pred.id;
        cp.radius = pred.radius;

        if (pred.modes.empty()) {
            compressed.push_back(cp);
            continue;
        }

        const int H = static_cast<int>(pred.modes[0].mu_seq.size());

        im2mppi::ObstacleMode mean_mode;
        mean_mode.pi = 1.0;
        mean_mode.mu_seq.assign(H, Eigen::Vector3d::Zero());
        mean_mode.sigma_diag_seq.assign(H, Eigen::Vector3d::Zero());

        double pi_sum = 0.0;
        for (const auto& mode : pred.modes) pi_sum += mode.pi;
        if (pi_sum < 1e-9) pi_sum = 1.0;

        for (const auto& mode : pred.modes) {
            const double w = mode.pi / pi_sum;
            for (int k = 0; k < H; ++k) {
                mean_mode.mu_seq[k]         += w * mode.mu_seq[k];
                mean_mode.sigma_diag_seq[k] += w * mode.sigma_diag_seq[k];
            }
        }

        cp.modes.push_back(mean_mode);
        compressed.push_back(std::move(cp));
    }
    return compressed;
}

void im2MppiNavigation::getDynamicSpheres(
    std::vector<im2mppi::SphereObstacle>& spheres) const
{
    spheres.clear();
    if (!this->useFakeDetector_ || !this->detector_) return;

    Eigen::Vector3d robotSize(0.0, 0.0, 0.0);
    if (this->map_) this->map_->getRobotSize(robotSize);
    const double inflation =
        std::max({robotSize.x(), robotSize.y(), robotSize.z()}) * 0.5;

    std::vector<onboardDetector::box3D> boxes;
    this->detector_->getObstaclesInSensorRange(2.0 * M_PI, boxes, robotSize);

    spheres.reserve(boxes.size());
    for (const auto& b : boxes) {
        im2mppi::SphereObstacle s;
        s.center = Eigen::Vector3d(b.x, b.y, b.z);
        s.radius = std::max({b.x_width, b.y_width, b.z_width}) * 0.5 + inflation;
        spheres.push_back(s);
    }
}

std::vector<Eigen::Vector3d> im2MppiNavigation::buildReferencePath() const
{
    if (this->usePredefinedGoal_ && !this->predefinedGoal_.poses.empty()) {
        // Only return the sub-path from the current waypoint onward.
        // This keeps the reference path local so w_path pulls forward, not sideways.
        const int N = static_cast<int>(this->predefinedGoal_.poses.size());
        const int start = std::min(this->goalIdx_, N - 1);
        std::vector<Eigen::Vector3d> path;
        path.reserve(N - start);
        for (int i = start; i < N; ++i) {
            const auto& ps = this->predefinedGoal_.poses[i];
            path.emplace_back(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
        }
        return path;
    }
    return {};
}

nav_msgs::Path im2MppiNavigation::loadRefTraj(const std::string& path) const
{
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();

    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("[IM2-MPPI Nav] cannot open ref trajectory: %s", path.c_str());
        return msg;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double dt_val, x, y, z;
        if (!(iss >> dt_val >> x >> y >> z)) continue;
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    ROS_INFO("[IM2-MPPI Nav] loaded %zu waypoints", msg.poses.size());
    return msg;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Visualization
// ═════════════════════════════════════════════════════════════════════════════

void im2MppiNavigation::publishBestTrajectory() const
{
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();
    for (const auto& pt : this->mppi_->getPlannedTrajectory()) {
        geometry_msgs::PoseStamped ps;
        ps.header           = msg.header;
        ps.pose.position.x  = pt.p.x();
        ps.pose.position.y  = pt.p.y();
        ps.pose.position.z  = pt.p.z();
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    this->bestTrajPub_.publish(msg);
}

void im2MppiNavigation::publishSampledRollouts() const
{
    const auto& positions = this->mppi_->getRolloutPositions();
    const auto& weights   = this->mppi_->getRolloutWeights();
    const auto& params    = this->mppi_->getParams();

    visualization_msgs::MarkerArray arr;

    // Always issue a DELETEALL first so stale lines vanish.
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.header.frame_id = "map";
    del.header.stamp    = ros::Time::now();
    del.ns = "im2mppi_rollouts";
    arr.markers.push_back(del);

    for (size_t i = 0; i < positions.size(); ++i) {
        if (positions[i].size() < 2) continue;

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns              = "im2mppi_rollouts";
        m.id              = static_cast<int>(i);
        m.type            = visualization_msgs::Marker::LINE_STRIP;
        m.action          = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x         = 0.015;   // line width [m]
        m.lifetime        = ros::Duration(0.5);

        // Color: gradient red(0) → green(1) by normalized weight
        if (params.viz_color_by_weight && i < weights.size()) {
            const float w = static_cast<float>(weights[i]);
            m.color.r = 1.0f - w;
            m.color.g = w;
            m.color.b = 0.1f;
            m.color.a = 0.25f + 0.55f * w;   // best rollouts more opaque
        } else {
            m.color.r = 0.4f;
            m.color.g = 0.4f;
            m.color.b = 0.9f;
            m.color.a = 0.4f;
        }

        m.points.reserve(positions[i].size());
        for (const auto& p : positions[i]) {
            geometry_msgs::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = p.z();
            m.points.push_back(pt);
        }
        arr.markers.push_back(m);
    }
    this->rolloutsPub_.publish(arr);
}

void im2MppiNavigation::publishReferencePath() const
{
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();

    if (!this->lastReferencePath_.empty()) {
        for (const auto& p : this->lastReferencePath_) {
            geometry_msgs::PoseStamped ps;
            ps.header           = msg.header;
            ps.pose.position.x  = p.x();
            ps.pose.position.y  = p.y();
            ps.pose.position.z  = p.z();
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
    } else {
        // Straight-line from current pos to goal
        geometry_msgs::PoseStamped a, b;
        a.header = b.header = msg.header;
        a.pose.position.x = this->currPos_.x();
        a.pose.position.y = this->currPos_.y();
        a.pose.position.z = this->currPos_.z();
        a.pose.orientation.w = 1.0;
        b.pose.position   = this->goal_.pose.position;
        b.pose.orientation.w = 1.0;
        msg.poses.push_back(a);
        msg.poses.push_back(b);
    }
    this->refPathPub_.publish(msg);
}

void im2MppiNavigation::publishDynamicObstaclePred() const
{
    const auto& preds = this->mppi_->getDynamicObstaclePredictions();

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.header.frame_id = "map";
    del.header.stamp    = ros::Time::now();
    del.ns = "im2mppi_dyn_pred";
    arr.markers.push_back(del);

    // Mode color palette (HSV-ish, fixed)
    static const float palette[6][3] = {
        {1.0f, 0.35f, 0.35f},
        {0.35f, 0.7f,  1.0f},
        {1.0f,  0.8f,  0.2f},
        {0.65f, 0.3f,  0.9f},
        {0.2f,  0.9f,  0.5f},
        {1.0f,  0.5f,  0.9f},
    };

    int id = 0;
    for (size_t j = 0; j < preds.size(); ++j) {
        const auto& pred = preds[j];
        for (size_t m = 0; m < pred.modes.size(); ++m) {
            const auto& mode = pred.modes[m];
            if (mode.mu_seq.empty()) continue;

            const float* c = palette[m % 6];

            // Line strip through predicted means
            visualization_msgs::Marker line;
            line.header.frame_id = "map";
            line.header.stamp    = ros::Time::now();
            line.ns              = "im2mppi_dyn_pred";
            line.id              = id++;
            line.type            = visualization_msgs::Marker::LINE_STRIP;
            line.action          = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x         = 0.04;
            line.lifetime        = ros::Duration(0.5);
            line.color.r = c[0];
            line.color.g = c[1];
            line.color.b = c[2];
            line.color.a = 0.4f + 0.5f * static_cast<float>(mode.pi);
            for (const auto& p : mode.mu_seq) {
                geometry_msgs::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            arr.markers.push_back(line);

            // Sphere at the first prediction step (=now) showing radius
            visualization_msgs::Marker sphere;
            sphere.header   = line.header;
            sphere.ns       = "im2mppi_dyn_pred";
            sphere.id       = id++;
            sphere.type     = visualization_msgs::Marker::SPHERE;
            sphere.action   = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = mode.mu_seq.front().x();
            sphere.pose.position.y = mode.mu_seq.front().y();
            sphere.pose.position.z = mode.mu_seq.front().z();
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * pred.radius;
            sphere.color.r = c[0];
            sphere.color.g = c[1];
            sphere.color.b = c[2];
            sphere.color.a = 0.25f;
            sphere.lifetime = ros::Duration(0.5);
            arr.markers.push_back(sphere);

            // Sphere at horizon end
            if (mode.mu_seq.size() > 1) {
                visualization_msgs::Marker sphere_end = sphere;
                sphere_end.id  = id++;
                sphere_end.pose.position.x = mode.mu_seq.back().x();
                sphere_end.pose.position.y = mode.mu_seq.back().y();
                sphere_end.pose.position.z = mode.mu_seq.back().z();
                sphere_end.color.a = 0.15f;
                arr.markers.push_back(sphere_end);
            }
        }
    }

    this->dynObsPredPub_.publish(arr);
}

void im2MppiNavigation::publishGoal() const
{
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = ros::Time::now();
    m.ns              = "im2mppi_goal";
    m.id              = 0;
    m.type            = visualization_msgs::Marker::SPHERE;
    m.action          = visualization_msgs::Marker::ADD;
    m.pose.position.x = this->goal_.pose.position.x;
    m.pose.position.y = this->goal_.pose.position.y;
    m.pose.position.z = this->goal_.pose.position.z;
    m.pose.orientation.w = 1.0;
    m.lifetime        = ros::Duration(0.5);
    m.scale.x = m.scale.y = m.scale.z = 0.4;
    m.color.r = 0.0f;
    m.color.g = 0.85f;
    m.color.b = 0.2f;
    m.color.a = 1.0f;
    arr.markers.push_back(m);
    this->goalPub_.publish(arr);
}

void im2MppiNavigation::publishWaypoints() const
{
    if (!this->usePredefinedGoal_ || this->predefinedGoal_.poses.empty()) return;

    visualization_msgs::MarkerArray arr;

    // ── DELETEALL to clear stale markers ────────────────────────────────────
    visualization_msgs::Marker del;
    del.action          = visualization_msgs::Marker::DELETEALL;
    del.header.frame_id = "map";
    del.header.stamp    = ros::Time::now();
    del.ns              = "im2mppi_waypoints";
    arr.markers.push_back(del);

    const int N   = static_cast<int>(this->predefinedGoal_.poses.size());
    const int cur = this->goalIdx_;

    // ── Sphere per waypoint ──────────────────────────────────────────────────
    for (int i = 0; i < N; ++i) {
        const auto& ps = this->predefinedGoal_.poses[i];

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns              = "im2mppi_waypoints";
        m.id              = i;
        m.type            = visualization_msgs::Marker::SPHERE;
        m.action          = visualization_msgs::Marker::ADD;
        m.pose             = ps.pose;
        m.pose.orientation.w = 1.0;
        m.lifetime        = ros::Duration(0.5);

        if (i < cur) {
            // Completed waypoints — small, dim gray
            m.scale.x = m.scale.y = m.scale.z = 0.20;
            m.color.r = 0.55f; m.color.g = 0.55f; m.color.b = 0.55f;
            m.color.a = 0.45f;
        } else if (i == cur) {
            // Current target — larger, bright cyan
            m.scale.x = m.scale.y = m.scale.z = 0.45;
            m.color.r = 0.0f; m.color.g = 0.95f; m.color.b = 1.0f;
            m.color.a = 1.0f;
        } else {
            // Future waypoints — medium, light blue
            m.scale.x = m.scale.y = m.scale.z = 0.28;
            m.color.r = 0.3f; m.color.g = 0.6f; m.color.b = 1.0f;
            m.color.a = 0.75f;
        }
        arr.markers.push_back(m);
    }

    // ── LINE_STRIP connecting all waypoints ──────────────────────────────────
    {
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
        line.header.stamp    = ros::Time::now();
        line.ns              = "im2mppi_waypoints";
        line.id              = N;          // after the N sphere ids
        line.type            = visualization_msgs::Marker::LINE_STRIP;
        line.action          = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x         = 0.04;
        line.lifetime        = ros::Duration(0.5);
        // Dashed look via alpha; completed portion dim, future bright
        line.color.r = 0.3f; line.color.g = 0.75f; line.color.b = 1.0f;
        line.color.a = 0.55f;
        line.points.reserve(N);
        for (const auto& p : this->predefinedGoal_.poses) {
            geometry_msgs::Point pt;
            pt.x = p.pose.position.x;
            pt.y = p.pose.position.y;
            pt.z = p.pose.position.z;
            line.points.push_back(pt);
        }
        arr.markers.push_back(line);
    }

    this->waypointPub_.publish(arr);
}

} // namespace AutoFlight
