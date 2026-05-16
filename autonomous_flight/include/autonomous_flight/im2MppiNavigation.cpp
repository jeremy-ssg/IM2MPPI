/*
    FILE: im2MppiNavigation.cpp
    --------------------------------
    IM2-MPPI navigation orchestrator — implementation.

    Phase 3 additions over Phase 2:
      ✓ convertPredictions()      — dynamicPredictor::obstacle → IM2-MPPI format
                                    with dt interpolation (pred 0.1 s → MPPI 0.05 s)
      ✓ compressToMeanPrediction() — for mean_prediction_mppi ablation
      ✓ mppiCB()                  — full planning loop with predictor integration
      ✓ trajExeCB()               — publishes tracking_controller::Target at 100 Hz
      ✓ method_type dispatch      — vanilla / mean_prediction / mode_aware variants

    Phase 4 / 5 changes will only require enabling CVaR in im2_mppi_planner.cpp;
    this file needs no modification.
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
//  Parameter loading  (mirrors mpcNavigation::initParam)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::initParam()
{
    // use_fake_detector
    if (!this->nh_.getParam("autonomous_flight/use_fake_detector",
                            this->useFakeDetector_)) {
        this->useFakeDetector_ = false;
        ROS_INFO("[IM2-MPPI Nav] use_fake_detector not found, default: false");
    } else {
        ROS_INFO("[IM2-MPPI Nav] use_fake_detector = %d", this->useFakeDetector_);
    }

    // use_predictor
    if (!this->nh_.getParam("autonomous_flight/use_predictor",
                            this->usePredictor_)) {
        this->usePredictor_ = false;
        ROS_INFO("[IM2-MPPI Nav] use_predictor not found, default: false");
    } else {
        ROS_INFO("[IM2-MPPI Nav] use_predictor = %d", this->usePredictor_);
    }

    // use_yaw_control
    if (!this->nh_.getParam("autonomous_flight/use_yaw_control",
                            this->useYawControl_)) {
        this->useYawControl_ = false;
    }

    // desired_velocity / desired_acceleration / desired_angular_velocity
    if (!this->nh_.getParam("autonomous_flight/desired_velocity",
                            this->desiredVel_)) {
        this->desiredVel_ = 1.5;
    }
    if (!this->nh_.getParam("autonomous_flight/desired_acceleration",
                            this->desiredAcc_)) {
        this->desiredAcc_ = 1.5;
    }
    if (!this->nh_.getParam("autonomous_flight/desired_angular_velocity",
                            this->desiredAngularVel_)) {
        this->desiredAngularVel_ = 0.5;
    }

    // use_predefined_goal / directory / repeat times
    if (!this->nh_.getParam("autonomous_flight/use_predefined_goal",
                            this->usePredefinedGoal_)) {
        this->usePredefinedGoal_ = false;
    }

    if (this->usePredefinedGoal_) {
        if (!this->nh_.getParam("autonomous_flight/predefined_goal_directory",
                                this->refTrajPath_)) {
            this->refTrajPath_ = "None";
            ROS_WARN("[IM2-MPPI Nav] predefined_goal_directory not set.");
        } else {
            std::string pkgPath = ros::package::getPath("autonomous_flight");
            this->refTrajPath_ = pkgPath + this->refTrajPath_;
            ROS_INFO("[IM2-MPPI Nav] ref traj path: %s",
                     this->refTrajPath_.c_str());
        }

        if (!this->nh_.getParam("autonomous_flight/execute_path_times",
                                this->repeatPathNum_)) {
            this->repeatPathNum_ = 1;
        }

        this->predefinedGoal_ = this->loadRefTraj(this->refTrajPath_);
        if (!this->predefinedGoal_.poses.empty()) {
            this->goal_ = this->predefinedGoal_.poses.back();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Module initialisation
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::initModules()
{
    // Map
    if (this->useFakeDetector_) {
        this->detector_.reset(new onboardDetector::fakeDetector(this->nh_));
        this->map_.reset(new mapManager::dynamicMap(this->nh_, false));
    } else {
        this->map_.reset(new mapManager::dynamicMap(this->nh_));
    }

    // Predictor
    if (this->usePredictor_) {
        this->predictor_.reset(new dynamicPredictor::predictor(this->nh_));
        this->predictor_->setMap(this->map_);
        if (this->useFakeDetector_) {
            this->predictor_->setDetector(this->detector_);
        }
    }

    // IM2-MPPI planner
    this->mppi_.reset(new im2mppi::IM2MPPIPlanner(this->nh_));
    ROS_INFO("[IM2-MPPI Nav] Planner initialised. method_type = %s",
             this->mppi_->getParams().method_type.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Publisher registration
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerPub()
{
    this->mppiTrajPub_ =
        this->nh_.advertise<nav_msgs::Path>("im2mppi/trajectory", 10);
    this->goalPub_ =
        this->nh_.advertise<visualization_msgs::MarkerArray>("im2mppi/goal", 10);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback registration
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::registerCallback()
{
    // MPPI planning loop — 20 Hz (every 50 ms = 1 × dt_mppi)
    this->mppiTimer_ = this->nh_.createTimer(
        ros::Duration(0.05),
        &im2MppiNavigation::mppiCB, this);

    // Trajectory execution — 100 Hz
    this->trajExeTimer_ = this->nh_.createTimer(
        ros::Duration(0.01),
        &im2MppiNavigation::trajExeCB, this);

    // Visualisation — 30 Hz
    this->visTimer_ = this->nh_.createTimer(
        ros::Duration(0.033),
        &im2MppiNavigation::visCB, this);
}

// ─────────────────────────────────────────────────────────────────────────────
//  run()
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::run()
{
    this->takeoff();
    this->registerCallback();
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPPI planning callback  (20 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::mppiCB(const ros::TimerEvent&)
{
    // Wait until we have a valid goal
    if (!this->goalReceived_ && !this->usePredefinedGoal_) return;
    if (!this->odomReceived_) return;

    // ── 1. Feed current state ─────────────────────────────────────────────
    this->mppi_->setCurrentState(this->currPos_, this->currVel_);

    // ── 2. Feed goal ──────────────────────────────────────────────────────
    Eigen::Vector3d goalEigen(
        this->goal_.pose.position.x,
        this->goal_.pose.position.y,
        this->goal_.pose.position.z);
    this->mppi_->setGoal(goalEigen);

    // ── 3. Reference path ─────────────────────────────────────────────────
    std::vector<Eigen::Vector3d> refPath = this->buildReferencePath();
    this->mppi_->setReferencePath(refPath);

    // ── 4. Dynamic obstacle predictions ──────────────────────────────────
    const std::string& method = this->mppi_->getParams().method_type;

    if (this->usePredictor_ && method != "vanilla_mppi") {
        // Get multi-modal predictions from predictor
        std::vector<dynamicPredictor::obstacle> predOb;
        this->predictor_->getPrediction(predOb);

        if (!predOb.empty()) {
            // Convert to IM2-MPPI format (with time-step interpolation)
            auto dynPreds = this->convertPredictions(predOb);

            // For mean_prediction_mppi: compress K modes to 1 weighted mean
            if (method == "mean_prediction_mppi") {
                dynPreds = this->compressToMeanPrediction(dynPreds);
            }

            this->mppi_->setDynamicObstaclePredictions(dynPreds);
        } else {
            // No obstacles detected — clear predictions
            this->mppi_->setDynamicObstaclePredictions({});
        }
    } else if (!this->usePredictor_) {
        // Predictor disabled: feed current obstacle positions as static spheres
        // so the planner can still avoid them.
        std::vector<im2mppi::SphereObstacle> spheres;
        this->getDynamicSpheres(spheres);
        this->mppi_->setStaticObstacles(spheres);
        this->mppi_->setDynamicObstaclePredictions({});
    }

    // ── 5. Plan ──────────────────────────────────────────────────────────
    ros::Time planStart = ros::Time::now();
    bool success = this->mppi_->plan();

    if (success) {
        this->trajStartTime_ = planStart;
        this->mppiReady_ = true;

        // Build nav_msgs::Path for visualisation
        nav_msgs::Path pathMsg;
        pathMsg.header.frame_id = "map";
        pathMsg.header.stamp = planStart;
        for (const auto& pt : this->mppi_->getPlannedTrajectory()) {
            geometry_msgs::PoseStamped ps;
            ps.header = pathMsg.header;
            ps.pose.position.x = pt.p.x();
            ps.pose.position.y = pt.p.y();
            ps.pose.position.z = pt.p.z();
            ps.pose.orientation.w = 1.0;
            pathMsg.poses.push_back(ps);
        }
        this->mppiTrajMsg_ = pathMsg;
    } else {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI Nav] plan() failed — stopping.");
        this->mppiReady_ = false;
        this->stop();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory execution callback  (100 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::trajExeCB(const ros::TimerEvent&)
{
    if (!this->mppiReady_) return;

    const auto& params = this->mppi_->getParams();
    const double endTime =
        static_cast<double>(params.horizon_steps) * params.dt;

    ros::Time currTime = ros::Time::now();
    double realTime = (currTime - this->trajStartTime_).toSec();

    // Build Target message
    tracking_controller::Target target;

    if (realTime >= endTime) {
        // Horizon exhausted: hold final position, zero vel/acc
        Eigen::Vector3d p = this->mppi_->getPos(endTime);
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        target.velocity.x = 0.0;
        target.velocity.y = 0.0;
        target.velocity.z = 0.0;
        target.acceleration.x = 0.0;
        target.acceleration.y = 0.0;
        target.acceleration.z = 0.0;
        target.yaw = AutoFlight::rpy_from_quaternion(
            this->odom_.pose.pose.orientation);
    } else {
        Eigen::Vector3d p   = this->mppi_->getPos(realTime);
        Eigen::Vector3d v   = this->mppi_->getVel(realTime);
        Eigen::Vector3d acc = this->mppi_->getAcc(realTime);

        target.position.x     = p.x();
        target.position.y     = p.y();
        target.position.z     = p.z();
        target.velocity.x     = v.x();
        target.velocity.y     = v.y();
        target.velocity.z     = v.z();
        target.acceleration.x = acc.x();
        target.acceleration.y = acc.y();
        target.acceleration.z = acc.z();

        // Yaw
        if (this->useYawControl_ && params.use_yaw_postprocess) {
            // Use yaw from MPPI trajectory (generated by generateYawReference)
            const auto& traj = this->mppi_->getPlannedTrajectory();
            int k = static_cast<int>(realTime / params.dt);
            k = std::max(0, std::min(k,
                static_cast<int>(traj.size()) - 1));
            target.yaw = static_cast<float>(traj[k].yaw);
        } else {
            target.yaw = this->facingYaw_;
        }
    }

    this->updateTargetWithState(target);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Visualisation callback  (30 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::visCB(const ros::TimerEvent&)
{
    if (!this->mppiTrajMsg_.poses.empty()) {
        this->mppiTrajPub_.publish(this->mppiTrajMsg_);
    }
    this->publishGoal();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Prediction conversion: dynamicPredictor::obstacle → IM2-MPPI format
// ─────────────────────────────────────────────────────────────────────────────

std::vector<im2mppi::DynamicObstaclePrediction>
im2MppiNavigation::convertPredictions(
    const std::vector<dynamicPredictor::obstacle>& predOb) const
{
    // Time-step alignment:
    //   predictor:   dt_pred = 0.1 s,  N_pred = 30 steps  → 3.0 s horizon
    //   MPPI planner: dt_mppi = 0.05 s, H = 30 steps       → 1.5 s horizon
    //
    // Each MPPI step k (time t = k * dt_mppi) maps to a predictor
    // time index:  pred_f = t / dt_pred = k * dt_mppi / dt_pred
    //
    // Example (dt_mppi=0.05, dt_pred=0.1):
    //   k=0 → pred_f=0.0  → pred[0]
    //   k=1 → pred_f=0.5  → lerp(pred[0], pred[1], 0.5)
    //   k=2 → pred_f=1.0  → pred[1]   ...

    const double dt_pred = 0.1;   // predictor timestep [s]
    const auto&  params  = this->mppi_->getParams();
    const double dt_mppi = params.dt;
    const int    H       = params.horizon_steps;

    std::vector<im2mppi::DynamicObstaclePrediction> result;
    result.reserve(predOb.size());

    for (int j = 0; j < static_cast<int>(predOb.size()); ++j) {
        const dynamicPredictor::obstacle& ob = predOb[j];

        im2mppi::DynamicObstaclePrediction pred;
        pred.id = j;

        // Estimate radius from first available size prediction
        // sizePred[intent][step] = (width_x, width_y, width_z)
        if (!ob.sizePred.empty() &&
            !ob.sizePred[0].empty() &&
            ob.sizePred[0][0].x() > 0.0) {
            const Eigen::Vector3d& sz0 = ob.sizePred[0][0];
            pred.radius = std::max(sz0.x(), sz0.y()) * 0.5;
        } else {
            pred.radius = 0.3;  // safe default
        }

        // Convert each intent mode
        const int K = static_cast<int>(ob.intentProb.size());
        pred.modes.reserve(K);

        for (int m = 0; m < K; ++m) {
            im2mppi::ObstacleMode mode;
            mode.pi = ob.intentProb[m];

            // posPred[m] = vector<Vector3d> of length N_pred (predictor steps)
            if (m >= static_cast<int>(ob.posPred.size())) {
                mode.pi = 0.0;
                mode.mu_seq.assign(H, Eigen::Vector3d::Zero());
                mode.sigma_diag_seq.assign(H, Eigen::Vector3d(0.3, 0.3, 0.3));
                pred.modes.push_back(mode);
                continue;
            }

            const auto& pos_seq  = ob.posPred[m];   // [N_pred] positions
            const auto& size_seq = ob.sizePred[m];  // [N_pred] sizes
            const int   N_pred   = static_cast<int>(pos_seq.size());

            mode.mu_seq.resize(H);
            mode.sigma_diag_seq.resize(H);

            for (int k = 0; k < H; ++k) {
                // Fractional predictor index for MPPI step k
                const double pred_f   = static_cast<double>(k) * dt_mppi / dt_pred;
                const int    idx_lo   = static_cast<int>(std::floor(pred_f));
                const int    idx_hi   = idx_lo + 1;
                const double alpha    = pred_f - static_cast<double>(idx_lo);

                // Clamp to valid range
                const int lo = std::min(idx_lo, N_pred - 1);
                const int hi = std::min(idx_hi, N_pred - 1);

                // Linear interpolation of mean position
                mode.mu_seq[k] = pos_seq[lo] + alpha * (pos_seq[hi] - pos_seq[lo]);

                // Linear interpolation of sigma (half obstacle size)
                if (!size_seq.empty()) {
                    const int slo = std::min(lo, static_cast<int>(size_seq.size()) - 1);
                    const int shi = std::min(hi, static_cast<int>(size_seq.size()) - 1);
                    Eigen::Vector3d sz =
                        size_seq[slo] + alpha * (size_seq[shi] - size_seq[slo]);
                    // sigma = half-width; ensure positive
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

// ─────────────────────────────────────────────────────────────────────────────
//  mean_prediction_mppi: compress K modes → 1 weighted-mean mode per obstacle
// ─────────────────────────────────────────────────────────────────────────────

std::vector<im2mppi::DynamicObstaclePrediction>
im2MppiNavigation::compressToMeanPrediction(
    const std::vector<im2mppi::DynamicObstaclePrediction>& preds) const
{
    std::vector<im2mppi::DynamicObstaclePrediction> compressed;
    compressed.reserve(preds.size());

    for (const auto& pred : preds) {
        im2mppi::DynamicObstaclePrediction cpred;
        cpred.id     = pred.id;
        cpred.radius = pred.radius;

        if (pred.modes.empty()) {
            compressed.push_back(cpred);
            continue;
        }

        const int H = static_cast<int>(pred.modes[0].mu_seq.size());

        im2mppi::ObstacleMode mean_mode;
        mean_mode.pi = 1.0;   // single mode → weight = 1
        mean_mode.mu_seq.assign(H, Eigen::Vector3d::Zero());
        mean_mode.sigma_diag_seq.assign(H, Eigen::Vector3d::Zero());

        // Normalise intent probabilities (predictor should sum to 1 but guard)
        double pi_sum = 0.0;
        for (const auto& mode : pred.modes) pi_sum += mode.pi;
        if (pi_sum < 1e-9) pi_sum = 1.0;

        // Weighted sum of per-step mean and sigma
        for (const auto& mode : pred.modes) {
            const double w = mode.pi / pi_sum;
            for (int k = 0; k < H; ++k) {
                mean_mode.mu_seq[k] +=
                    w * mode.mu_seq[k];
                mean_mode.sigma_diag_seq[k] +=
                    w * mode.sigma_diag_seq[k];
            }
        }

        cpred.modes.push_back(mean_mode);
        compressed.push_back(std::move(cpred));
    }

    return compressed;
}

// ─────────────────────────────────────────────────────────────────────────────
//  getDynamicSpheres — fallback when predictor is disabled
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::getDynamicSpheres(
    std::vector<im2mppi::SphereObstacle>& spheres) const
{
    spheres.clear();
    if (!this->useFakeDetector_ || !this->detector_) return;

    // Inflate by robot size
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
        // radius = half diagonal of bounding box + robot inflation
        s.radius = std::max({b.x_width, b.y_width, b.z_width}) * 0.5
                   + inflation;
        spheres.push_back(s);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build reference path for MPPI from predefined waypoints or goal direction
// ─────────────────────────────────────────────────────────────────────────────

std::vector<Eigen::Vector3d> im2MppiNavigation::buildReferencePath() const
{
    if (this->usePredefinedGoal_ &&
        !this->predefinedGoal_.poses.empty()) {
        std::vector<Eigen::Vector3d> path;
        path.reserve(this->predefinedGoal_.poses.size());
        for (const auto& ps : this->predefinedGoal_.poses) {
            path.emplace_back(ps.pose.position.x,
                              ps.pose.position.y,
                              ps.pose.position.z);
        }
        return path;
    }
    // Empty → planner uses straight-line from current pos to goal
    return {};
}

// ─────────────────────────────────────────────────────────────────────────────
//  Load reference trajectory from text file  (dt x y z per line)
// ─────────────────────────────────────────────────────────────────────────────

nav_msgs::Path im2MppiNavigation::loadRefTraj(const std::string& path) const
{
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();

    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("[IM2-MPPI Nav] Cannot open ref trajectory file: %s",
                  path.c_str());
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
    file.close();
    ROS_INFO("[IM2-MPPI Nav] Loaded %zu waypoints from %s",
             msg.poses.size(), path.c_str());
    return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  publishGoal — MarkerArray visualisation (mirrors mpcNavigation::publishGoal)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::publishGoal() const
{
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker pt;
    pt.header.frame_id = "map";
    pt.header.stamp    = ros::Time::now();
    pt.ns              = "im2mppi_goal";
    pt.id              = 0;
    pt.type            = visualization_msgs::Marker::SPHERE;
    pt.action          = visualization_msgs::Marker::ADD;
    pt.pose.position.x = this->goal_.pose.position.x;
    pt.pose.position.y = this->goal_.pose.position.y;
    pt.pose.position.z = this->goal_.pose.position.z;
    pt.pose.orientation.w = 1.0;
    pt.lifetime        = ros::Duration(0.5);
    pt.scale.x = pt.scale.y = pt.scale.z = 0.35;
    pt.color.a = 1.0;
    pt.color.r = 0.0;
    pt.color.g = 0.8;
    pt.color.b = 0.2;
    msg.markers.push_back(pt);
    this->goalPub_.publish(msg);
}

} // namespace AutoFlight
