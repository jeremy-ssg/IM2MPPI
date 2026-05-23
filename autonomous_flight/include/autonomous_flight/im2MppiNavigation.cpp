/*
    FILE: im2MppiNavigation.cpp
    --------------------------------
    Implementation of im2MppiNavigation (Phases 1 – 3 + RViz visualization).
*/

#include <autonomous_flight/im2MppiNavigation.h>

namespace AutoFlight {

namespace {

im2mppi::TrajectoryPoint sampleTrajectorySnapshot(
    const std::vector<im2mppi::TrajectoryPoint>& traj,
    double dt,
    double t)
{
    im2mppi::TrajectoryPoint out;
    if (traj.empty()) return out;
    if (traj.size() == 1 || t <= 0.0 || dt <= 1e-6) return traj.front();

    const double horizon_time =
        static_cast<double>(traj.size() - 1) * dt;
    if (t >= horizon_time) return traj.back();

    const double scaled = t / dt;
    const int k = std::max(0, std::min(
        static_cast<int>(std::floor(scaled)),
        static_cast<int>(traj.size()) - 2));
    const double alpha = std::max(0.0, std::min(
        1.0, scaled - static_cast<double>(k)));

    const auto& a = traj[k];
    const auto& b = traj[k + 1];
    out.p = a.p + alpha * (b.p - a.p);
    out.v = a.v + alpha * (b.v - a.v);
    out.a = a.a + alpha * (b.a - a.a);
    out.yaw = (alpha < 0.5) ? a.yaw : b.yaw;
    return out;
}

} // namespace

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

    this->nh_.param("autonomous_flight/closed_loop_intent_enabled",
                    this->closedLoopIntentEnabled_, true);
    this->nh_.param("autonomous_flight/closed_loop_match_distance",
                    this->closedLoopMatchDistance_, 1.0);
    ROS_INFO("[IM2-MPPI Nav] closed_loop_intent = %d (match_dist = %.2f m)",
             this->closedLoopIntentEnabled_, this->closedLoopMatchDistance_);

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
    this->planTimePub_   = this->nh_.advertise<std_msgs::Float64>(
        "im2mppi/plan_time_ms", 50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timers
//      mppiTimer_    : 10 Hz planning loop
//      trajExeTimer_ : 100 Hz target publishing
//      visTimer_     : 5 Hz RViz visualization
//      predTimer_    : 10 Hz async prediction (separate spinner thread)
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
        this->predTimer_ = this->nh_.createTimer(ros::Duration(0.1),
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

    bool success = false;
    double plan_ms = 0.0;
    ros::Time planStart;
    double traj_dt = 0.05;
    double snapshot_facing_yaw = this->facingYaw_;
    bool use_yaw_postprocess = true;
    std::vector<im2mppi::TrajectoryPoint> planned_traj;

    {

    // Planner internals are locked while plan() mutates them. trajExeCB reads
    // the previous published snapshot, so target streaming is not blocked here.
    std::lock_guard<std::mutex> lk(this->planMutex_);

    // 1. Current state
    this->mppi_->setCurrentState(this->currPos_, this->currVel_);

    // 2. Local horizon reference path (built FIRST so we can derive both the
    //    MPPI terminal goal and the facing-yaw target from the same point).
    this->lastReferencePath_ = this->buildReferencePath();
    this->mppi_->setReferencePath(this->lastReferencePath_);

    // 3. Goals
    //    globalGoal  = final predefined waypoint (only used if local path empty)
    //    plannerGoal = end of local horizon  → MPPI terminal cost target
    //                                        → also facing-yaw target (smooth)
    const Eigen::Vector3d globalGoal(this->goal_.pose.position.x,
                                     this->goal_.pose.position.y,
                                     this->goal_.pose.position.z);
    const Eigen::Vector3d plannerGoal =
        this->lastReferencePath_.empty() ? globalGoal : this->lastReferencePath_.back();
    this->mppi_->setGoal(plannerGoal);

    // 4. Facing yaw — head toward the LOCAL horizon end, not the far-away
    //    global goal. Two advantages:
    //      (a) Yaw smoothly follows the path curve (no more lurching).
    //      (b) Larger 0.3-m deadband (vs old 0.1 m) avoids the divide-by-tiny
    //          atan2 jitter when the drone is near a waypoint.
    Eigen::Vector3d gv = plannerGoal - this->currPos_;
    if (gv.head<2>().norm() > 0.3) {
        this->facingYaw_ = std::atan2(gv.y(), gv.x());
    }

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
        std::vector<im2mppi::BoxObstacle> boxes;
        this->getDynamicBoxes(boxes);
        this->mppi_->setStaticObstacles(boxes);
        this->mppi_->setDynamicObstaclePredictions({});
    }

    // 6. Plan. Latency uses wall-clock time; the trajectory start stays at
    // the ROS time when the state was sampled, so execution compensates for
    // planning delay instead of replaying a stale t=0 target.
    planStart = ros::Time::now();
    const ros::WallTime wallStart = ros::WallTime::now();
    success = this->mppi_->plan();
    const ros::WallTime wallEnd = ros::WallTime::now();
    plan_ms = (wallEnd - wallStart).toSec() * 1000.0;

    if (success) {
        planned_traj = this->mppi_->getPlannedTrajectory();
        const auto& params = this->mppi_->getParams();
        traj_dt = params.dt;
        use_yaw_postprocess = params.use_yaw_postprocess;
        snapshot_facing_yaw = this->facingYaw_;
    }
    }

    std_msgs::Float64 pt_msg;
    pt_msg.data = plan_ms;
    this->planTimePub_.publish(pt_msg);

    if (success && !planned_traj.empty()) {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        this->activeTraj_              = std::move(planned_traj);
        this->activeTrajDt_            = traj_dt;
        this->activeFacingYaw_         = snapshot_facing_yaw;
        this->activeUseYawPostprocess_ = use_yaw_postprocess;
        this->trajStartTime_           = planStart;
        this->mppiReady_               = true;
    } else {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI Nav] plan() failed.");
        {
            std::lock_guard<std::mutex> tk(this->trajMutex_);
            this->mppiReady_ = false;
            this->activeTraj_.clear();
        }
        this->stop();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory execution callback (100 Hz)
// ─────────────────────────────────────────────────────────────────────────────

void im2MppiNavigation::trajExeCB(const ros::TimerEvent&)
{
    std::vector<im2mppi::TrajectoryPoint> traj;
    ros::Time trajStartTime;
    double traj_dt = 0.05;
    double snapshot_facing_yaw = 0.0;
    bool use_yaw_postprocess = true;

    {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        if (!this->mppiReady_ || this->activeTraj_.empty()) return;
        traj = this->activeTraj_;
        trajStartTime = this->trajStartTime_;
        traj_dt = this->activeTrajDt_;
        snapshot_facing_yaw = this->activeFacingYaw_;
        use_yaw_postprocess = this->activeUseYawPostprocess_;
    }

    const double endTime =
        std::max(0.0, static_cast<double>(traj.size() - 1) * traj_dt);
    const double realTime =
        std::max(0.0, (ros::Time::now() - trajStartTime).toSec());

    tracking_controller::Target target;

    // ── Compute raw position / velocity / accel from the plan ────────────
    double raw_yaw = 0.0;
    if (realTime >= endTime) {
        const auto pt = sampleTrajectorySnapshot(traj, traj_dt, endTime);
        const Eigen::Vector3d p = pt.p;
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        target.velocity.x = target.velocity.y = target.velocity.z = 0.0;
        target.acceleration.x = target.acceleration.y = target.acceleration.z = 0.0;

        // Hold the LAST PLANNED yaw instead of snapping to current odom yaw
        // (which used to inject a one-shot step every time a plan expired).
        if (this->useYawControl_ && use_yaw_postprocess) {
            raw_yaw = traj.back().yaw;
        } else {
            raw_yaw = snapshot_facing_yaw;
        }
    } else {
        const auto pt = sampleTrajectorySnapshot(traj, traj_dt, realTime);
        const Eigen::Vector3d p   = pt.p;
        const Eigen::Vector3d v   = pt.v;
        const Eigen::Vector3d acc = pt.a;

        target.position.x     = p.x();
        target.position.y     = p.y();
        target.position.z     = p.z();
        target.velocity.x     = v.x();
        target.velocity.y     = v.y();
        target.velocity.z     = v.z();
        target.acceleration.x = acc.x();
        target.acceleration.y = acc.y();
        target.acceleration.z = acc.z();

        if (this->useYawControl_ && use_yaw_postprocess) {
            raw_yaw = pt.yaw;
        } else {
            raw_yaw = snapshot_facing_yaw;
        }
    }

    // ── Yaw rate limiter ─────────────────────────────────────────────────
    //   Caps |Δyaw| at YAW_RATE_MAX × Δt so the 100-Hz setpoint stream stays
    //   physically follow-able by the attitude controller. Initial yaw is
    //   seeded from the actual odom orientation so the limiter doesn't drag
    //   the drone toward 0 at start-up.
    {
        const ros::Time now = ros::Time::now();
        if (!this->targetYawInit_) {
            this->lastTargetYaw_     = AutoFlight::rpy_from_quaternion(
                                          this->odom_.pose.pose.orientation);
            this->lastTargetYawTime_ = now;
            this->targetYawInit_     = true;
        }
        const double dt_y = std::max(0.001,
                                     (now - this->lastTargetYawTime_).toSec());
        constexpr double YAW_RATE_MAX = M_PI;          // 180 °/s
        const double max_dy = YAW_RATE_MAX * dt_y;

        double dy = raw_yaw - this->lastTargetYaw_;
        while (dy >  M_PI) dy -= 2.0 * M_PI;
        while (dy < -M_PI) dy += 2.0 * M_PI;
        dy = std::max(-max_dy, std::min(max_dy, dy));

        const double limited = this->lastTargetYaw_ + dy;
        target.yaw                  = static_cast<float>(limited);
        this->lastTargetYaw_        = limited;
        this->lastTargetYawTime_    = now;
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

    // ── Closed-loop intent correction (Phase 4 / innovation #5) ──────────
    // Reweight the predictor's posterior using the per-mode Gaussian
    // likelihood of the actual observation against the previous prediction.
    const ros::Time now = ros::Time::now();
    if (this->closedLoopIntentEnabled_ && !this->lastPredOb_.empty()
        && !this->lastPredTime_.isZero()) {
        const double dt = (now - this->lastPredTime_).toSec();
        if (dt > 0.01 && dt < 2.0) {     // sanity window
            this->applyClosedLoopIntentCorrection(predOb, dt);
        }
    }
    this->lastPredOb_   = predOb;
    this->lastPredTime_ = now;

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

    {
        std::lock_guard<std::mutex> tk(this->trajMutex_);
        if (!this->mppiReady_) return;
    }

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
    // sigma_min is a per-axis floor on the predictor's empirical sigma, so
    // that CVaR sampling never fully degenerates to deterministic hinge.
    const Eigen::Vector3d sigma_floor(params.sigma_min,
                                      params.sigma_min,
                                      params.sigma_min);

    std::vector<im2mppi::DynamicObstaclePrediction> result;
    result.reserve(predOb.size());

    for (size_t j = 0; j < predOb.size(); ++j) {
        const auto& ob = predOb[j];

        im2mppi::DynamicObstaclePrediction pred;
        pred.id = static_cast<int>(j);

        // Preserve full 3-axis dimensions (matches Intent-MPC AABB representation).
        // sizePred[intent][step] gives the (x, y, z) widths in metres.
        if (!ob.sizePred.empty() &&
            !ob.sizePred[0].empty() &&
            ob.sizePred[0][0].x() > 0.0) {
            pred.size = ob.sizePred[0][0].cwiseMax(0.05);
        } else {
            pred.size = Eigen::Vector3d(0.6, 0.6, 1.8);
        }

        const int K = static_cast<int>(ob.intentProb.size());
        pred.modes.reserve(K);

        for (int m = 0; m < K; ++m) {
            im2mppi::ObstacleMode mode;
            mode.pi = ob.intentProb[m];

            if (m >= static_cast<int>(ob.posPred.size())) {
                mode.pi = 0.0;
                mode.mu_seq.assign(H, Eigen::Vector3d::Zero());
                mode.sigma_diag_seq.assign(H, sigma_floor);
                pred.modes.push_back(mode);
                continue;
            }

            const auto& pos_seq  = ob.posPred[m];
            const int   N_pred   = static_cast<int>(pos_seq.size());

            // Predictor's per-step empirical sigma for THIS intent mode.
            // Same indexing as pos_seq when present; otherwise we fall back
            // to the sigma_floor below.
            const std::vector<Eigen::Vector3d>* sigma_seq = nullptr;
            if (m < static_cast<int>(ob.sigmaPred.size()) &&
                !ob.sigmaPred[m].empty()) {
                sigma_seq = &ob.sigmaPred[m];
            }

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
                    mode.sigma_diag_seq[k] = sigma_floor;
                    continue;
                }

                mode.mu_seq[k] = pos_seq[lo] + alpha * (pos_seq[hi] - pos_seq[lo]);

                // Sigma: linear-interpolate the predictor's empirical std at
                // the same lo/hi, then floor each axis at sigma_min so CVaR
                // sampling cannot collapse to zero when the predictor reports
                // a perfectly confident step (e.g., a stationary obstacle).
                Eigen::Vector3d sigma_k = Eigen::Vector3d::Zero();
                if (sigma_seq && !sigma_seq->empty()) {
                    const int s_N = static_cast<int>(sigma_seq->size());
                    const int slo = std::min(lo, s_N - 1);
                    const int shi = std::min(hi, s_N - 1);
                    sigma_k = (*sigma_seq)[slo] +
                              alpha * ((*sigma_seq)[shi] - (*sigma_seq)[slo]);
                }
                mode.sigma_diag_seq[k] = sigma_k.cwiseMax(sigma_floor);
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
        cp.id   = pred.id;
        cp.size = pred.size;

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

// Closed-loop intent correction: reweight mode probabilities by how well the
// previous prediction explained the newly observed obstacle position.
void im2MppiNavigation::applyClosedLoopIntentCorrection(
    std::vector<dynamicPredictor::obstacle>& newPred,
    double dt_since_last) const
{
    constexpr double dt_pred = 0.1;    // predictor step
    const int offset = std::max(1, static_cast<int>(std::round(dt_since_last / dt_pred)));

    int n_corrected = 0;

    for (auto& obs : newPred) {
        // 1. Observed position "now" — the new prediction's step-0 (any mode
        //    has the same step-0, they all start at the current observation).
        if (obs.posPred.empty() || obs.posPred[0].empty()) continue;
        const Eigen::Vector3d observed = obs.posPred[0][0];

        // 2. Match this obstacle to one in lastPredOb_ by current-position
        //    proximity. (No persistent IDs across detector ticks, so we use
        //    nearest-neighbour with a sanity gate.)
        int    best_idx = -1;
        double best_d   = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < this->lastPredOb_.size(); ++j) {
            const auto& last = this->lastPredOb_[j];
            if (last.posPred.empty() || last.posPred[0].empty()) continue;
            const Eigen::Vector3d last_now = last.posPred[0][0];
            const double d = (observed - last_now).norm();
            if (d < best_d) { best_d = d; best_idx = static_cast<int>(j); }
        }
        if (best_idx < 0 || best_d > this->closedLoopMatchDistance_) continue;

        const auto& last = this->lastPredOb_[best_idx];

        const int K = static_cast<int>(obs.intentProb.size());
        if (K <= 1 || K != static_cast<int>(last.intentProb.size())) continue;

        // 3. Per-mode log-likelihood under the previous prediction.
        Eigen::VectorXd ll(K);
        bool any_valid = false;
        for (int m = 0; m < K; ++m) {
            // Predicted μ_m at step `offset` (clamped to available horizon)
            if (m >= static_cast<int>(last.posPred.size())
                || last.posPred[m].empty()) {
                ll(m) = -std::numeric_limits<double>::infinity();
                continue;
            }
            const int off = std::min(offset,
                static_cast<int>(last.posPred[m].size()) - 1);
            const Eigen::Vector3d mu = last.posPred[m][off];

            // Fixed observation-noise proxy. Do not derive this from
            // sizePred; that would mix physical obstacle size with prediction
            // uncertainty and inflate the displayed/planned obstacle geometry.
            const Eigen::Vector3d sigma(0.15, 0.15, 0.10);

            // Log-Gaussian (drop normalization constants — they cancel in softmax)
            const Eigen::Vector3d d = observed - mu;
            const double log_l = -0.5 * (
                d.x() * d.x() / (sigma.x() * sigma.x()) +
                d.y() * d.y() / (sigma.y() * sigma.y()) +
                d.z() * d.z() / (sigma.z() * sigma.z()));
            ll(m) = log_l;
            any_valid = true;
        }
        if (!any_valid) continue;

        // 4. Convert log-likelihoods to normalized likelihoods (numerically
        //    stable softmax with -inf entries set to 0 weight).
        double max_ll = -std::numeric_limits<double>::infinity();
        for (int m = 0; m < K; ++m) if (std::isfinite(ll(m)) && ll(m) > max_ll) max_ll = ll(m);
        if (!std::isfinite(max_ll)) continue;

        Eigen::VectorXd lh(K);
        double lh_sum = 0.0;
        for (int m = 0; m < K; ++m) {
            lh(m) = std::isfinite(ll(m)) ? std::exp(ll(m) - max_ll) : 0.0;
            lh_sum += lh(m);
        }
        if (lh_sum < 1e-12) continue;
        lh /= lh_sum;

        // 5. Bayesian update: posterior ∝ prior · likelihood, renormalized.
        Eigen::VectorXd post(K);
        double post_sum = 0.0;
        for (int m = 0; m < K; ++m) {
            const double prior_m = std::max(0.0,
                static_cast<double>(obs.intentProb[m]));
            post(m) = prior_m * lh(m);
            post_sum += post(m);
        }
        if (post_sum < 1e-12) continue;

        for (int m = 0; m < K; ++m) {
            obs.intentProb[m] = post(m) / post_sum;
        }
        ++n_corrected;
    }

    if (n_corrected > 0) {
        ROS_DEBUG_THROTTLE(1.0,
            "[IM2-MPPI Nav/CL-Intent] corrected %d / %zu obstacles "
            "(dt=%.3fs, offset=%d steps).",
            n_corrected, newPred.size(), dt_since_last, offset);
    }
}

void im2MppiNavigation::getDynamicBoxes(
    std::vector<im2mppi::BoxObstacle>& boxes_out) const
{
    boxes_out.clear();
    if (!this->useFakeDetector_ || !this->detector_) return;

    Eigen::Vector3d robotSize(0.0, 0.0, 0.0);
    if (this->map_) this->map_->getRobotSize(robotSize);

    // The detector's getObstaclesInSensorRange ALREADY inflates b.x/y/z_width
    // by robotSize internally (fakeDetector.cpp). Adding it again here was a
    // double-inflation bug that made vanilla MPPI see obstacles ~0.5 m wider
    // than M4 sees them, artificially boosting M1's MinClr in comparison.
    std::vector<onboardDetector::box3D> boxes;
    this->detector_->getObstaclesInSensorRange(2.0 * M_PI, boxes, robotSize);

    boxes_out.reserve(boxes.size());
    for (const auto& b : boxes) {
        im2mppi::BoxObstacle bo;
        bo.center = Eigen::Vector3d(b.x, b.y, b.z);
        bo.size   = Eigen::Vector3d(b.x_width, b.y_width, b.z_width);
        boxes_out.push_back(bo);
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

            // Axis-aligned bounding box at the first prediction step
            visualization_msgs::Marker box;
            box.header              = line.header;
            box.ns                  = "im2mppi_dyn_pred";
            box.id                  = id++;
            box.type                = visualization_msgs::Marker::CUBE;
            box.action              = visualization_msgs::Marker::ADD;
            box.pose.position.x     = mode.mu_seq.front().x();
            box.pose.position.y     = mode.mu_seq.front().y();
            box.pose.position.z     = mode.mu_seq.front().z();
            box.pose.orientation.w  = 1.0;
            box.scale.x             = std::max(0.05, pred.size.x());
            box.scale.y             = std::max(0.05, pred.size.y());
            box.scale.z             = std::max(0.05, pred.size.z());
            box.color.r = c[0]; box.color.g = c[1]; box.color.b = c[2];
            box.color.a             = 0.30f;
            box.lifetime            = ros::Duration(0.5);
            arr.markers.push_back(box);

            // Box at horizon end (faded)
            if (mode.mu_seq.size() > 1) {
                visualization_msgs::Marker box_end = box;
                box_end.id              = id++;
                box_end.pose.position.x = mode.mu_seq.back().x();
                box_end.pose.position.y = mode.mu_seq.back().y();
                box_end.pose.position.z = mode.mu_seq.back().z();
                box_end.color.a         = 0.15f;
                arr.markers.push_back(box_end);
            }

            // Optional: thin wire-frame outlines at intermediate prediction steps
            // (kept off by default to reduce marker count; uncomment if desired).
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
