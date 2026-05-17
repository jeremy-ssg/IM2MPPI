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
            // Global goal is the FINAL waypoint; used for yaw facing.
            // MPPI's per-step goal is the end of the local horizon (computed in mppiCB).
            this->goal_ = this->predefinedGoal_.poses.back();
        }
    }
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
    this->mppi_->setMap(this->map_);
    ROS_INFO("[IM2-MPPI Nav] Planner ready. method_type = %s",
             this->mppi_->getParams().method_type.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Publishers
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerPub()
{
    this->bestTrajPub_   = this->nh_.advertise<nav_msgs::Path>(
        "im2mppi/best_trajectory", 10);
    this->rolloutsPub_   = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/sampled_rollouts", 10);
    this->refPathPub_    = this->nh_.advertise<nav_msgs::Path>(
        "im2mppi/reference_path", 10);
    this->dynObsPredPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/dynamic_obstacle_predictions", 10);
    this->goalPub_       = this->nh_.advertise<visualization_msgs::MarkerArray>(
        "im2mppi/goal", 10);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timers
//      mppiTimer_    : 10 Hz planning loop
//      trajExeTimer_ : 100 Hz target publishing
//      visTimer_     : 5 Hz RViz visualization
//      predTimer_    : 5 Hz async prediction (separate spinner thread)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerCallback()
{
    this->mppiTimer_    = this->nh_.createTimer(ros::Duration(0.1),
                            &im2MppiNavigation::mppiCB, this);
    this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01),
                            &im2MppiNavigation::trajExeCB, this);
    this->visTimer_     = this->nh_.createTimer(ros::Duration(0.2),
                            &im2MppiNavigation::visCB, this);
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
//  Planning callback (10 Hz)
//      globalGoal  = last predefined waypoint (yaw target)
//      plannerGoal = end of local horizon reference (MPPI terminal goal)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::mppiCB(const ros::TimerEvent&)
{
    if (!this->goalReceived_ && !this->usePredefinedGoal_) return;
    if (!this->odomReceived_) return;

    // Lock for the whole iteration — trajExeCB/visCB will briefly wait.
    std::lock_guard<std::mutex> lk(this->planMutex_);

    // 1. Current state
    this->mppi_->setCurrentState(this->currPos_, this->currVel_);

    // 2. Global goal (used for yaw facing only)
    const Eigen::Vector3d globalGoal(this->goal_.pose.position.x,
                                     this->goal_.pose.position.y,
                                     this->goal_.pose.position.z);

    Eigen::Vector3d gv = globalGoal - this->currPos_;
    if (gv.head<2>().norm() > 0.1) {
        this->facingYaw_ = std::atan2(gv.y(), gv.x());
    }

    // 3. Local horizon reference path (sliced from predefined waypoints)
    this->lastReferencePath_ = this->buildReferencePath();
    this->mppi_->setReferencePath(this->lastReferencePath_);

    // 4. MPPI terminal goal = end of local horizon
    //    Falls back to globalGoal if the reference is empty.
    const Eigen::Vector3d plannerGoal =
        this->lastReferencePath_.empty() ? globalGoal : this->lastReferencePath_.back();
    this->mppi_->setGoal(plannerGoal);

    // 5. Method-type dispatch — predictions come from async cache (predCB)
    const std::string& method = this->mppi_->getParams().method_type;

    if (this->usePredictor_ && method != "vanilla_mppi") {
        std::vector<im2mppi::DynamicObstaclePrediction> dynPreds;
        {
            std::lock_guard<std::mutex> pk(this->predMutex_);
            dynPreds = this->cachedDynPreds_;
        }
        this->mppi_->setDynamicObstaclePredictions(dynPreds);
        this->mppi_->setStaticObstacles({});
    } else {
        std::vector<im2mppi::SphereObstacle> spheres;
        this->getDynamicSpheres(spheres);
        this->mppi_->setStaticObstacles(spheres);
        this->mppi_->setDynamicObstaclePredictions({});
    }

    // 6. Plan
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

    // params_ is set once in the constructor and never modified at runtime —
    // reading method_type here is safe without planMutex_ (no write race).
    const std::string method = this->mppi_->getParams().method_type;
    if (method == "vanilla_mppi") return;

    std::vector<dynamicPredictor::obstacle> predOb;
    this->predictor_->getPrediction(predOb);   // may take 100–200 ms

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
//  Visualization callback (~5 Hz)
//  Only publishes to topics that have at least one subscriber.
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::visCB(const ros::TimerEvent&)
{
    // publishGoal reads goal_ only (plain struct, benign torn read).
    if (this->goalPub_.getNumSubscribers() > 0) {
        this->publishGoal();
    }

    if (!this->mppiReady_) return;

    // Lock only for mppi_ internal vector reads.
    std::lock_guard<std::mutex> lk(this->planMutex_);

    if (this->bestTrajPub_.getNumSubscribers() > 0) {
        this->publishBestTrajectory();
    }
    if (this->rolloutsPub_.getNumSubscribers() > 0) {
        this->publishSampledRollouts();
    }
    if (this->refPathPub_.getNumSubscribers() > 0) {
        this->publishReferencePath();
    }
    if (this->dynObsPredPub_.getNumSubscribers() > 0) {
        this->publishDynamicObstaclePred();
    }
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

// ─────────────────────────────────────────────────────────────────────────────
//  Local horizon reference path
//      1. Find waypoint closest to current position.
//      2. Walk forward along the predefined path until accumulated arc-length
//         equals `desired_velocity × horizon_steps × dt`.
//      3. Output sequence (currPos, wp[closest], …, end_at_horizon).
// ─────────────────────────────────────────────────────────────────────────────

std::vector<Eigen::Vector3d> im2MppiNavigation::buildReferencePath() const
{
    if (!this->usePredefinedGoal_ || this->predefinedGoal_.poses.empty()) return {};

    const auto& params = this->mppi_->getParams();
    const double local_len =
        std::max(0.3, this->desiredVel_) *
        static_cast<double>(params.horizon_steps) * params.dt;

    // 1. Closest predefined waypoint to current position
    size_t closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < this->predefinedGoal_.poses.size(); ++i) {
        const auto& ps = this->predefinedGoal_.poses[i];
        const Eigen::Vector3d p(ps.pose.position.x,
                                ps.pose.position.y,
                                ps.pose.position.z);
        const double d = (p - this->currPos_).squaredNorm();
        if (d < closest_dist) {
            closest_dist = d;
            closest_idx  = i;
        }
    }

    // 2. Walk forward, accumulating arc-length until local_len
    std::vector<Eigen::Vector3d> path;
    path.reserve(static_cast<size_t>(params.horizon_steps) + 2);
    path.push_back(this->currPos_);

    Eigen::Vector3d last = this->currPos_;
    double walked = 0.0;
    for (size_t i = closest_idx; i < this->predefinedGoal_.poses.size(); ++i) {
        const auto& ps = this->predefinedGoal_.poses[i];
        const Eigen::Vector3d p(ps.pose.position.x,
                                ps.pose.position.y,
                                ps.pose.position.z);
        const double seg = (p - last).norm();
        if (seg < 1e-6) continue;

        if (walked + seg >= local_len) {
            const double alpha = std::max(0.0, std::min(1.0,
                (local_len - walked) / seg));
            path.push_back(last + alpha * (p - last));
            return path;
        }

        path.push_back(p);
        walked += seg;
        last    = p;
    }

    // 3. Fallback: if we didn't walk far enough, append final waypoint
    if (path.size() < 2) {
        const auto& ps = this->predefinedGoal_.poses.back();
        path.emplace_back(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    }
    return path;
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
        ps.header             = msg.header;
        ps.pose.position.x    = pt.p.x();
        ps.pose.position.y    = pt.p.y();
        ps.pose.position.z    = pt.p.z();
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

    for (size_t i = 0; i < positions.size(); ++i) {
        if (positions[i].size() < 2) continue;

        visualization_msgs::Marker m;
        m.header.frame_id    = "map";
        m.header.stamp       = ros::Time::now();
        m.ns                 = "im2mppi_rollouts";
        m.id                 = static_cast<int>(i);
        m.type               = visualization_msgs::Marker::LINE_STRIP;
        m.action             = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x            = 0.015;
        m.lifetime           = ros::Duration(0.5);

        if (params.viz_color_by_weight && i < weights.size()) {
            const float w = static_cast<float>(weights[i]);
            m.color.r = 1.0f - w;
            m.color.g = w;
            m.color.b = 0.1f;
            m.color.a = 0.25f + 0.55f * w;
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
            ps.header             = msg.header;
            ps.pose.position.x    = p.x();
            ps.pose.position.y    = p.y();
            ps.pose.position.z    = p.z();
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
    } else {
        geometry_msgs::PoseStamped a, b;
        a.header = b.header = msg.header;
        a.pose.position.x = this->currPos_.x();
        a.pose.position.y = this->currPos_.y();
        a.pose.position.z = this->currPos_.z();
        a.pose.orientation.w = 1.0;
        b.pose.position      = this->goal_.pose.position;
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

    static const float palette[6][3] = {
        {1.0f,  0.35f, 0.35f},
        {0.35f, 0.7f,  1.0f },
        {1.0f,  0.8f,  0.2f },
        {0.65f, 0.3f,  0.9f },
        {0.2f,  0.9f,  0.5f },
        {1.0f,  0.5f,  0.9f },
    };

    int id = 0;
    for (size_t j = 0; j < preds.size(); ++j) {
        const auto& pred = preds[j];
        for (size_t m = 0; m < pred.modes.size(); ++m) {
            const auto& mode = pred.modes[m];
            if (mode.mu_seq.empty()) continue;

            const float* c = palette[m % 6];

            // Line through predicted means
            visualization_msgs::Marker line;
            line.header.frame_id    = "map";
            line.header.stamp       = ros::Time::now();
            line.ns                 = "im2mppi_dyn_pred";
            line.id                 = id++;
            line.type               = visualization_msgs::Marker::LINE_STRIP;
            line.action             = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x            = 0.04;
            line.lifetime           = ros::Duration(0.5);
            line.color.r = c[0]; line.color.g = c[1]; line.color.b = c[2];
            line.color.a = 0.4f + 0.5f * static_cast<float>(mode.pi);
            for (const auto& p : mode.mu_seq) {
                geometry_msgs::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            arr.markers.push_back(line);

            // Sphere at first step
            visualization_msgs::Marker sphere;
            sphere.header           = line.header;
            sphere.ns               = "im2mppi_dyn_pred";
            sphere.id               = id++;
            sphere.type             = visualization_msgs::Marker::SPHERE;
            sphere.action           = visualization_msgs::Marker::ADD;
            sphere.pose.position.x  = mode.mu_seq.front().x();
            sphere.pose.position.y  = mode.mu_seq.front().y();
            sphere.pose.position.z  = mode.mu_seq.front().z();
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * pred.radius;
            sphere.color.r = c[0]; sphere.color.g = c[1]; sphere.color.b = c[2];
            sphere.color.a = 0.25f;
            sphere.lifetime = ros::Duration(0.5);
            arr.markers.push_back(sphere);

            // Sphere at horizon end
            if (mode.mu_seq.size() > 1) {
                visualization_msgs::Marker sphere_end = sphere;
                sphere_end.id              = id++;
                sphere_end.pose.position.x = mode.mu_seq.back().x();
                sphere_end.pose.position.y = mode.mu_seq.back().y();
                sphere_end.pose.position.z = mode.mu_seq.back().z();
                sphere_end.color.a         = 0.15f;
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
    m.header.frame_id    = "map";
    m.header.stamp       = ros::Time::now();
    m.ns                 = "im2mppi_goal";
    m.id                 = 0;
    m.type               = visualization_msgs::Marker::SPHERE;
    m.action             = visualization_msgs::Marker::ADD;
    m.pose.position.x    = this->goal_.pose.position.x;
    m.pose.position.y    = this->goal_.pose.position.y;
    m.pose.position.z    = this->goal_.pose.position.z;
    m.pose.orientation.w = 1.0;
    m.lifetime           = ros::Duration(0.5);
    m.scale.x = m.scale.y = m.scale.z = 0.4;
    m.color.r = 0.0f; m.color.g = 0.85f; m.color.b = 0.2f; m.color.a = 1.0f;
    arr.markers.push_back(m);
    this->goalPub_.publish(arr);
}

} // namespace AutoFlight
