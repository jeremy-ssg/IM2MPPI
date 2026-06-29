/*
    FILE: tmpcPlanner.cpp
    ----------------------------------------------------------------------------
    Implementation of T-MPC++ (de Groot et al., IEEE T-RO 2025) for the IM2-MPPI
    UAV benchmark (method id M6_tmpc).

    Pipeline per plan():
        buildConstantVelocityPredictions()  (locked decision: const-vel)
        buildGoalGrid()                      (Frenet grid along the lap ref path)
        runGuidance()                        (vendored guidance_planner -> P branches)
        solveBranch() x (P + unguided)       (self-contained OSQP linear MPC)
        decide()                             (Eq.12 consistency-weighted min cost)

    LOCAL PLANNER (the "fork" resolved, see docs §8): a self-contained linear MPC
    on a 2-D double integrator (z fixed at z_lap), solved with the vendored
    OsqpEigen. ALL branches share the SAME tracking cost (track the real reference
    path) so their optimal costs are directly comparable (paper Eq.11). The branches
    differ ONLY in their linear half-plane constraints (Eq.8): guided branches derive
    them from their distinct guidance trajectory; the unguided (T-MPC++) branch
    derives them from the plain reference path. This keeps the problem convex while
    preserving the topology-distinct behavior.

    NOTE: written without on-machine compilation (dev box is Windows; build on the
    Linux ROS workspace). Mirrors existing OsqpEigen usage in polyTrajSolver.cpp.
*/

#include <trajectory_planner/tmpc/tmpcPlanner.h>
#include <trajectory_planner/path_search/astarOcc.h>
#include <trajectory_planner/third_party/OsqpEigen/OsqpEigen.h>
#if TMPC_HAVE_GUIDANCE_PLANNER
  #include <guidance_planner/config.h>
  #include <ros_tools/spline.h>
#endif

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>

namespace trajPlanner {

// State/control dimensions of the local MPC (3-D double integrator).
static constexpr int NS = 6;   // [x, y, z, vx, vy, vz]
static constexpr int NU = 3;   // [ax, ay, az]

static std::vector<Eigen::Vector3d> resamplePolyline(
    const std::vector<Eigen::Vector3d>& path, int count, double zFallback) {
    std::vector<Eigen::Vector3d> out;
    if (path.empty() || count <= 0) return out;
    if (path.size() == 1 || count == 1) {
        out.assign(std::max(1, count), path.front());
        for (auto& p : out) p.z() = zFallback;
        return out;
    }

    std::vector<double> s(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i)
        s[i] = s[i - 1] + (path[i].head<2>() - path[i - 1].head<2>()).norm();
    const double total = s.back();
    if (total < 1e-6) {
        out.assign(count, path.front());
        for (auto& p : out) p.z() = zFallback;
        return out;
    }

    out.reserve(count);
    size_t seg = 0;
    for (int k = 0; k < count; ++k) {
        const double target = total * (double)k / (double)std::max(1, count - 1);
        while (seg + 1 < s.size() && s[seg + 1] < target) ++seg;
        if (seg + 1 >= path.size()) {
            out.push_back(path.back());
        } else {
            const double denom = std::max(1e-9, s[seg + 1] - s[seg]);
            const double a = std::max(0.0, std::min(1.0, (target - s[seg]) / denom));
            out.push_back(path[seg] + a * (path[seg + 1] - path[seg]));
        }
        out.back().z() = zFallback;
    }
    return out;
}

static Eigen::Vector2d localTangent(const std::vector<Eigen::Vector3d>& path, int k) {
    if (path.size() < 2) return Eigen::Vector2d(1.0, 0.0);
    const int n = (int)path.size();
    const int a = std::max(0, k - 1);
    const int b = std::min(n - 1, k + 1);
    Eigen::Vector2d t = path[b].head<2>() - path[a].head<2>();
    if (t.norm() < 1e-6) t = path[std::min(n - 1, k + 1)].head<2>() - path[k].head<2>();
    if (t.norm() < 1e-6) return Eigen::Vector2d(1.0, 0.0);
    return t.normalized();
}

#if TMPC_HAVE_GUIDANCE_PLANNER
static std::string guidanceTopologyMethodName(std::string method) {
    std::transform(method.begin(), method.end(), method.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    if (method == "winding" || method == "winding_angle") return "Winding";
    if (method == "uvd") return "UVD";
    return "Homology"; // h_signature / homology: official homology implementation.
}

static std::shared_ptr<RosTools::Spline2D> makeGuidanceReferenceSpline(
    const std::vector<Eigen::Vector3d>& ref) {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    x.reserve(ref.size());
    y.reserve(ref.size());
    s.reserve(ref.size());

    double acc = 0.0;
    Eigen::Vector2d prev = Eigen::Vector2d::Zero();
    bool havePrev = false;
    for (const auto& p3 : ref) {
        Eigen::Vector2d p = p3.head<2>();
        if (havePrev) {
            const double ds = (p - prev).norm();
            if (ds < 1e-4) continue;
            acc += ds;
        }
        x.push_back(p.x());
        y.push_back(p.y());
        s.push_back(acc);
        prev = p;
        havePrev = true;
    }

    if (x.size() < 2 || s.back() <= 1e-4) return nullptr;
    return std::make_shared<RosTools::Spline2D>(x, y, s);
}
#endif

tmpcPlanner::tmpcPlanner(const ros::NodeHandle& nh) : nh_(nh) {}

void tmpcPlanner::initParam() {
    nh_.param("tmpc/dt",                  dt_,              0.05);
    nh_.param("tmpc/horizon_steps",       horizon_,         38);
    nh_.param("tmpc/z_lap",               zLap_,            1.0);
    nh_.param("tmpc/r_uav",               rUav_,            0.30);
    nh_.param("tmpc/num_trajectories_P",  numTrajP_,        4);
    nh_.param("tmpc/add_unguided_planner",addUnguided_,     true);
    nh_.param("tmpc/prm_samples_n",       prmSamplesN_,     100);
    nh_.param<std::string>("tmpc/homotopy_method", homotopyMethod_, "h_signature");
    nh_.param("tmpc/visibility_dt",       visibilityDt_,    0.20);
    nh_.param("tmpc/smoothing_resolution",smoothingRes_,    0.05);
    nh_.param("tmpc/goal_grid_lat",       goalGridLat_,     5);
    nh_.param("tmpc/goal_grid_long",      goalGridLong_,    3);
    nh_.param("tmpc/goal_lat_spread",     goalLatSpread_,   2.0);
    nh_.param("tmpc/goal_long_distance",  goalLongDist_,    4.0);
    nh_.param("tmpc/beta_relax",          betaRelax_,       0.05);
    nh_.param("tmpc/parallel_threads",    parallelThreads_, 5);
    nh_.param("tmpc/thread_timeout_ms",   threadTimeoutMs_, 50);
    nh_.param("tmpc/solve_sequential",    solveSequential_, true);
    nh_.param("tmpc/consistency_ci",      consistencyCi_,   0.75);
    nh_.param("tmpc/v_max",               vMax_,            2.0);
    nh_.param("tmpc/a_max",               aMax_,            3.0);
    nh_.param("tmpc/vz_max",              vzMax_,           1.0);
    nh_.param("tmpc/az_max",              azMax_,           2.0);
    nh_.param("tmpc/v_ref",               vRef_,            1.5);
    nh_.param("tmpc/enable_vertical_avoidance", vertical_,  false);
    nh_.param("tmpc/vertical_clearance",  vClearance_,      0.4);
    nh_.param<std::string>("tmpc/prediction_source", predictionSource_, "constant_velocity");
    nh_.param("tmpc/max_obstacles",       maxObstacles_,    12);
    nh_.param("tmpc/use_static_astar",    useStaticAstar_,  true);
    nh_.param("tmpc/static_astar_step",   staticAstarStep_, 0.20);
    nh_.param("tmpc/static_astar_pool_xy",staticAstarPoolXY_, 80);
    nh_.param("tmpc/static_astar_pool_z", staticAstarPoolZ_, 16);
    nh_.param("tmpc/static_halfplane_search_radius", staticHalfplaneSearchRadius_, 0.8);
    nh_.param("tmpc/static_halfplane_clearance",     staticHalfplaneClearance_, 0.25);
    nh_.param("tmpc/static_halfplane_rays",          staticHalfplaneRays_, 16);
    // cost weights
    nh_.param("tmpc/cost_weights/w_contour", wContour_, 1.0);
    nh_.param("tmpc/cost_weights/w_lag",     wLag_,     1.0);
    nh_.param("tmpc/cost_weights/w_vel",     wVel_,     0.1);
    nh_.param("tmpc/cost_weights/w_acc",     wAcc_,     0.05);

    parallelThreads_ = std::max(1, parallelThreads_);
    threadTimeoutMs_ = std::max(1, threadTimeoutMs_);
    staticAstarStep_ = std::max(0.05, staticAstarStep_);
    staticAstarPoolXY_ = std::max(20, staticAstarPoolXY_);
    staticAstarPoolZ_ = std::max(3, staticAstarPoolZ_);
    staticHalfplaneSearchRadius_ = std::max(0.0, staticHalfplaneSearchRadius_);
    staticHalfplaneClearance_ = std::max(0.02, staticHalfplaneClearance_);
    staticHalfplaneRays_ = std::max(4, staticHalfplaneRays_);

    ROS_INFO("[tmpcPlanner] init: P=%d unguided=%d horizon=%d dt=%.3f z_lap=%.2f pred=%s",
             numTrajP_, (int)addUnguided_, horizon_, dt_, zLap_, predictionSource_.c_str());
}

void tmpcPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map) { map_ = map; }

void tmpcPlanner::registerPub() {
    guidancePathsPub_   = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/guidance_paths", 1);
    optimizedTrajPub_   = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/optimized_trajectories", 1);
    goalGridPub_        = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/goal", 1);
    dynObsPub_          = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/dynamic_obstacle_predictions", 1);
}

void tmpcPlanner::updateCurrStates(const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& vel,
                                   double yaw) {
    currPos_ = pos; currVel_ = vel; currYaw_ = yaw;
}

void tmpcPlanner::setObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos,
                               const std::vector<Eigen::Vector3d>& obstaclesVel,
                               const std::vector<Eigen::Vector3d>& obstaclesSize) {
    obsPos_  = obstaclesPos;
    obsVel_  = obstaclesVel;
    obsSize_ = obstaclesSize;
}

void tmpcPlanner::setReference(const std::vector<Eigen::Vector3d>& refPath) {
    refPath_ = refPath;
    // UAV adaptation: operate at the reference lap's actual altitude rather than a
    // hard-coded value. Guidance/homotopy stay planar (x,y); z just tracks the lap.
    // Falls back to the yaml z_lap if the reference is empty.
    if (!refPath_.empty()) {
        double zsum = 0.0;
        for (const auto& p : refPath_) zsum += p.z();
        zLap_ = zsum / (double)refPath_.size();
    }
}

// ---------------------------------------------------------------------------
// Constant-velocity obstacle predictions (locked decision #3). Each obstacle's
// center is propagated linearly: o_k = o_0 + v * (k*dt). Radius from bbox.
// ---------------------------------------------------------------------------
void tmpcPlanner::buildConstantVelocityPredictions() {
    obsPredPos_.clear();
    obsRadius_.clear();
    obsTop_.clear();

    // Keep only the nearest maxObstacles_ obstacles.
    std::vector<int> idx(obsPos_.size());
    for (size_t i = 0; i < obsPos_.size(); ++i) idx[i] = (int)i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        return (obsPos_[a].head<2>() - currPos_.head<2>()).squaredNorm()
             < (obsPos_[b].head<2>() - currPos_.head<2>()).squaredNorm();
    });
    int nKeep = std::min<int>((int)idx.size(), maxObstacles_);

    for (int n = 0; n < nKeep; ++n) {
        int j = idx[n];
        std::vector<Eigen::Vector3d> pred(horizon_ + 1);
        for (int k = 0; k <= horizon_; ++k) {
            pred[k] = obsPos_[j] + obsVel_[j] * (k * dt_);
            pred[k].z() = zLap_;
        }
        obsPredPos_.push_back(std::move(pred));
        // horizontal disc radius = half of the larger horizontal bbox extent
        double r = 0.5 * std::max(obsSize_[j].x(), obsSize_[j].y());
        obsRadius_.push_back(std::max(r, 0.1));
        // obstacle top altitude (for the 3-D vertical "fly-over" branch)
        obsTop_.push_back(obsPos_[j].z() + 0.5 * obsSize_[j].z());
    }
}

// ---------------------------------------------------------------------------
// Goal grid in Frenet frame: find the nearest point on the reference path, look
// ahead goal_long_distance, then lay a lateral x longitudinal grid around it.
// ---------------------------------------------------------------------------
void tmpcPlanner::buildGoalGrid() {
    goalGrid_.clear();
    localRef_.clear();
    if (refPath_.size() < 2) return;

    // nearest reference index to the current position (xy)
    int nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < refPath_.size(); ++i) {
        double d = (refPath_[i].head<2>() - currPos_.head<2>()).squaredNorm();
        if (d < best) { best = d; nearest = (int)i; }
    }

    // Local horizon reference: resample refPath forward at v_ref*dt arc-length per
    // step. We track segConsumed (distance already used inside the current segment)
    // across steps so the cursor truly advances — without it, when the path's point
    // spacing exceeds v_ref*dt the reference collapses onto a single point and the
    // drone barely moves (the "very slow / very short trajectory" bug).
    {
        const double step = vRef_ * dt_;
        const int    last = (int)refPath_.size() - 1;
        int    i = nearest;          // current segment [i, i+1]
        double segConsumed = 0.0;    // distance already consumed within segment i

        Eigen::Vector3d p0 = refPath_[nearest]; p0.z() = zLap_;
        localRef_.push_back(p0);

        for (int k = 1; k <= horizon_; ++k) {
            double need = step;
            while (i < last && need > 0.0) {
                Eigen::Vector3d seg = refPath_[i + 1] - refPath_[i];
                double segLen = seg.head<2>().norm();
                double segRemain = segLen - segConsumed;
                if (segRemain <= 1e-9) { ++i; segConsumed = 0.0; continue; }
                if (need < segRemain) { segConsumed += need; need = 0.0; }
                else                  { need -= segRemain; ++i; segConsumed = 0.0; }
            }
            Eigen::Vector3d p;
            if (i >= last) {
                p = refPath_[last];                    // reached the end of the path
            } else {
                Eigen::Vector3d seg = refPath_[i + 1] - refPath_[i];
                double segLen = seg.head<2>().norm();
                double frac = (segLen > 1e-9) ? (segConsumed / segLen) : 0.0;
                p = refPath_[i] + seg * frac;
            }
            p.z() = zLap_;
            localRef_.push_back(p);
        }
    }

    buildGoalGridFromLocalRef();
}

void tmpcPlanner::buildGoalGridFromLocalRef() {
    goalGrid_.clear();
    if (localRef_.size() < 2) return;

    // Goal grid centered on the look-ahead point of the final local reference.
    int lookIdx = 0;
    double remaining = goalLongDist_;
    while (lookIdx < (int)localRef_.size() - 1 && remaining > 0.0) {
        double segLen = (localRef_[lookIdx + 1].head<2>() - localRef_[lookIdx].head<2>()).norm();
        remaining -= segLen;
        ++lookIdx;
    }
    Eigen::Vector2d center = localRef_[std::min(lookIdx, (int)localRef_.size() - 1)].head<2>();
    Eigen::Vector2d tang   = localTangent(localRef_, lookIdx);
    Eigen::Vector2d normal(-tang.y(), tang.x());

    for (int lo = 0; lo < goalGridLong_; ++lo) {
        double along = lo * (vRef_ * dt_ * horizon_) / std::max(1, goalGridLong_);
        for (int la = 0; la < goalGridLat_; ++la) {
            double frac = (goalGridLat_ == 1) ? 0.0
                        : (double)la / (goalGridLat_ - 1) - 0.5;       // -0.5..0.5
            double lat = frac * goalLatSpread_;
            Eigen::Vector2d g = center + tang * along + normal * lat;
            goalGrid_.emplace_back(g.x(), g.y(), zLap_);
        }
    }
}

void tmpcPlanner::buildStaticAwareReference() {
    if (!useStaticAstar_ || !map_ || localRef_.size() < 2) return;

    Eigen::Vector3d start = currPos_;
    Eigen::Vector3d goal = localRef_.back();
    start.z() = zLap_;
    goal.z() = zLap_;
    if ((goal.head<2>() - start.head<2>()).norm() < 0.25) return;

    bool directBlocked = false;
    if (map_->isInflatedOccupied(start) || map_->isInflatedOccupied(goal)) {
        directBlocked = true;
    } else if (map_->isInflatedOccupiedLine(start, goal)) {
        directBlocked = true;
    } else {
        for (const auto& p0 : localRef_) {
            Eigen::Vector3d p = p0;
            p.z() = zLap_;
            if (map_->isInflatedOccupied(p)) { directBlocked = true; break; }
        }
    }
    if (!directBlocked) return;

    const double dist = std::max(1.0, (goal.head<2>() - start.head<2>()).norm());
    const int xyPool = std::max(staticAstarPoolXY_,
        (int)std::ceil(dist / staticAstarStep_) + 24);
    const double minH = std::max(0.05, zLap_ - 0.35);
    const double maxH = zLap_ + (vertical_ ? std::max(0.8, vClearance_ + 0.8) : 0.35);

    AStar astar;
    astar.initGridMap(map_, Eigen::Vector3i(xyPool, xyPool, staticAstarPoolZ_),
                      minH, maxH);
    if (!astar.AstarSearch(staticAstarStep_, start, goal)) {
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] static A* failed; keeping reference and relying on static constraints.");
        return;
    }

    std::vector<Eigen::Vector3d> astarPath = astar.getPath();
    if (astarPath.size() < 2) return;
    std::vector<Eigen::Vector3d> ref = resamplePolyline(astarPath, horizon_ + 1, zLap_);
    if ((int)ref.size() == horizon_ + 1) {
        ref.front() = start;
        ref.back() = goal;
        localRef_ = ref;
        ROS_INFO_THROTTLE(1.0,
            "[tmpcPlanner] static A* reference active: %zu raw points -> %zu horizon points",
            astarPath.size(), localRef_.size());
    }
}

// ---------------------------------------------------------------------------
// Guidance: call the vendored guidance_planner to get P topology-distinct
// trajectories. If the official package is absent or returns no path, do not
// invent a non-paper fallback topology.
// ---------------------------------------------------------------------------
bool tmpcPlanner::runGuidance() {
    branches_.clear();

#if TMPC_HAVE_GUIDANCE_PLANNER
    const int guidanceVerticalGoals = (goalGridLat_ % 2 == 1)
                                    ? std::max(1, goalGridLat_)
                                    : std::max(1, goalGridLat_ + 1);
    if (!guidance_) {
        nh_.setParam("/guidance_planner/N", horizon_);
        nh_.setParam("/guidance_planner/T", dt_ * (double)horizon_);
        nh_.setParam("/guidance_planner/sampling/n_samples", std::max(1, prmSamplesN_));
        nh_.setParam("/guidance_planner/homotopy/n_paths", std::max(1, numTrajP_));
        nh_.setParam("/guidance_planner/homotopy/comparison_function",
                     guidanceTopologyMethodName(homotopyMethod_));
        nh_.setParam("/guidance_planner/goals/longitudinal", std::max(2, goalGridLong_));
        nh_.setParam("/guidance_planner/goals/vertical", guidanceVerticalGoals);
        nh_.setParam("/guidance_planner/max_velocity", vMax_);
        nh_.setParam("/guidance_planner/max_acceleration", aMax_);
        nh_.setParam("/guidance_planner/predictions_are_constant_velocity", true);
        nh_.setParam("/clock_frequency", 1.0 / std::max(1e-3, dt_));
        guidance_.reset(new GuidancePlanner::GlobalGuidance());
    }
    GuidancePlanner::Config* gCfg = guidance_->GetConfig();
    GuidancePlanner::Config::N = horizon_;
    GuidancePlanner::Config::DT = dt_;
    GuidancePlanner::Config::CONTROL_DT = dt_;
    gCfg->T_ = dt_ * (double)horizon_;
    gCfg->n_paths_ = std::max(1, numTrajP_);
    gCfg->n_samples_ = std::max(1, prmSamplesN_);
    gCfg->longitudinal_goals_ = std::max(2, goalGridLong_);
    gCfg->vertical_goals_ = guidanceVerticalGoals;
    gCfg->topology_comparison_function_ = guidanceTopologyMethodName(homotopyMethod_);
    gCfg->selection_weight_consistency_ = std::max(1e-3, consistencyCi_);
    gCfg->max_velocity_ = vMax_;
    gCfg->max_acceleration_ = aMax_;
    gCfg->assume_constant_velocity_ = true;

    // start
    guidance_->SetStart(currPos_.head<2>(), currYaw_, currVel_.head<2>().norm());
    guidance_->SetReferenceVelocity(vRef_);
    guidance_->SetPlanningFrequency(1.0 / std::max(1e-3, dt_));

    // obstacles via constant-velocity constructor (id, start, vel, DT, N, radius)
    std::vector<GuidancePlanner::Obstacle> gObs;
    for (size_t j = 0; j < obsPredPos_.size(); ++j) {
        std::vector<Eigen::Vector2d> pos2d(obsPredPos_[j].size());
        for (size_t k = 0; k < obsPredPos_[j].size(); ++k)
            pos2d[k] = obsPredPos_[j][k].head<2>();
        gObs.emplace_back((int)j, pos2d, obsRadius_[j]);
    }
    std::vector<GuidancePlanner::Halfspace> staticObs;   // (static map handled by local MPC)
    guidance_->LoadObstacles(gObs, staticObs);

    auto refSpline = makeGuidanceReferenceSpline(localRef_);
    if (!refSpline) {
        ROS_WARN_THROTTLE(1.0, "[tmpcPlanner] cannot build guidance reference spline.");
        return false;
    }

    // Official guidance_planner path: LoadReferencePath samples the Visibility-PRM
    // along the reference and constructs the goal grid internally, matching the
    // open-source T-MPC++ stack more closely than manually injecting goals.
    guidance_->LoadReferencePath(0.0, refSpline, std::max(0.5, goalLatSpread_));

    if (!guidance_->Update() || guidance_->NumberOfGuidanceTrajectories() == 0) {
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] official guidance_planner produced no topology paths; skipping this plan.");
        return false;
    }

    int nTraj = std::min(numTrajP_, guidance_->NumberOfGuidanceTrajectories());
    for (int i = 0; i < nTraj; ++i) {
        auto& out = guidance_->GetGuidanceTrajectory(i);
        auto& traj2d = out.spline.GetTrajectory();
        TMPCBranch b;
        b.guided  = true;
        b.classId = out.topology_class;
        b.guidanceTraj.resize(horizon_ + 1);
        for (int k = 0; k <= horizon_; ++k) {
            Eigen::Vector2d p = traj2d.getPoint(k * dt_);
            b.guidanceTraj[k] = Eigen::Vector3d(p.x(), p.y(), zLap_);
        }
        branches_.push_back(std::move(b));
    }
#else
    ROS_ERROR_THROTTLE(2.0,
        "[tmpcPlanner] T-MPC++ topology requires the official tud-amr/guidance_planner package; "
        "install it in the catkin workspace instead of using a non-paper fallback.");
    return false;
#endif

    if (branches_.empty()) return false;

    // T-MPC++ adds one non-guided local planner in parallel to the guided topology
    // branches; this is the official "++" behavior, not a replacement for topology.
    if (addUnguided_) {
        TMPCBranch b;
        b.guided  = false;
        b.classId = -1;
        b.guidanceTraj = localRef_;   // plain reference path as the "guidance"
        branches_.push_back(std::move(b));
    }

    // 3-D vertical "fly-over" branch: an extra topology option that passes OVER the
    // obstacles instead of around them. No horizontal half-planes; instead a vertical
    // clearance floor where the path passes near an obstacle (built in solveBranch).
    if (vertical_ && !obsPredPos_.empty()) {
        TMPCBranch b;
        b.guided   = false;
        b.overTake = true;
        b.classId  = -2;
        b.guidanceTraj = localRef_;
        branches_.push_back(std::move(b));
    }
    return !branches_.empty();
}

// ---------------------------------------------------------------------------
// Eq.8 half-plane for one (guidance point, obstacle point) pair (xy only).
//   n   = (o - tau)/||o - tau|| ;  A = n ;  b = n . (o - n*beta*(r_uav+r_obs))
// Keeps the ego on the guidance side of the obstacle: A . p <= b.
// ---------------------------------------------------------------------------
bool tmpcPlanner::homotopyHalfPlane(const Eigen::Vector2d& guidancePt,
                                    const Eigen::Vector2d& obstaclePt,
                                    double rSum,
                                    Eigen::Vector2d& A_k, double& b_k) const {
    Eigen::Vector2d diff = obstaclePt - guidancePt;
    double dn = diff.norm();
    if (dn < 1e-6) return false;          // guidance point sits on the obstacle center
    A_k = diff / dn;
    b_k = A_k.dot(obstaclePt - A_k * (betaRelax_ * rSum));
    return true;
}

// ---------------------------------------------------------------------------
// Local MPC for one branch: self-contained OSQP QP. Cost tracks localRef_ for ALL
// branches (comparable J); constraints add the branch's homotopy half-planes.
//   z = [X ; U],  X = [x_0..x_N] (NS each),  U = [u_0..u_{N-1}] (NU each)
// ---------------------------------------------------------------------------
void tmpcPlanner::solveBranch(TMPCBranch& branch) {
    // No valid reference -> never solve (reading an empty localRef_ would be UB and
    // produce garbage setpoints that fly the drone away).
    if ((int)localRef_.size() < horizon_ + 1) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        return;
    }
    const int N   = horizon_;
    const int nX  = NS * (N + 1);                       // NS=6: [x,y,z,vx,vy,vz]
    const int nU  = NU * N;                             // NU=3: [ax,ay,az]
    const int nVar = nX + nU;

    auto xi = [&](int k){ return k * NS; };             // index of state block k
    auto ui = [&](int k){ return nX + k * NU; };        // index of control block k

    // ---- Hessian P (sparse, diagonal) and gradient q ----
    Eigen::SparseMatrix<double> P(nVar, nVar);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nVar);
    std::vector<Eigen::Triplet<double>> Ptr;

    // position tracking weight (isotropic approx of contour+lag; see docs)
    const double wPos = 0.5 * (wContour_ + wLag_);
    for (int k = 0; k <= N; ++k) {
        const double wp = (k == N) ? 2.0 * wPos : wPos; // small terminal emphasis
        const Eigen::Vector3d ref = (k < (int)localRef_.size()) ? localRef_[k] : localRef_.back();
        for (int d = 0; d < 3; ++d) {                   // 3-D position tracking
            Ptr.emplace_back(xi(k)+d, xi(k)+d, 2.0*wp);
            q(xi(k)+d) = -2.0*wp*ref(d);
        }
        // velocity cost toward v_ref along the 3-D path tangent
        Eigen::Vector3d tang(1, 0, 0);
        if (k < (int)localRef_.size() - 1) {
            Eigen::Vector3d t = localRef_[k+1] - localRef_[k];
            if (t.norm() > 1e-6) tang = t.normalized();
        }
        const Eigen::Vector3d vdes = vRef_ * tang;
        for (int d = 0; d < 3; ++d) {
            Ptr.emplace_back(xi(k)+3+d, xi(k)+3+d, 2.0*wVel_);
            q(xi(k)+3+d) = -2.0*wVel_*vdes(d);
        }
    }
    for (int k = 0; k < N; ++k)
        for (int d = 0; d < 3; ++d)
            Ptr.emplace_back(ui(k)+d, ui(k)+d, 2.0*wAcc_);
    P.setFromTriplets(Ptr.begin(), Ptr.end());

    // ---- Constraints A z in [l, u] ----
    std::vector<Eigen::Triplet<double>> Atr;
    std::vector<double> lo, up;
    int row = 0;
    const double INF = OsqpEigen::INFTY;
    auto addBound = [&](double l, double u){ lo.push_back(l); up.push_back(u); };

    // initial state equality x_0 = [currPos, currVel]
    {
        const double x0v[NS] = { currPos_.x(), currPos_.y(), currPos_.z(),
                                 currVel_.x(), currVel_.y(), currVel_.z() };
        for (int d = 0; d < NS; ++d) {
            Atr.emplace_back(row, xi(0)+d, 1.0); addBound(x0v[d], x0v[d]); ++row;
        }
    }
    // dynamics: 3-D double integrator, per axis d in {x,y,z}
    //   p_{k+1,d} = p_{k,d} + dt*v_{k,d} + 0.5dt^2*u_{k,d}
    //   v_{k+1,d} = v_{k,d} + dt*u_{k,d}
    const double dt = dt_, hdt2 = 0.5 * dt_ * dt_;
    for (int k = 0; k < N; ++k) {
        for (int d = 0; d < 3; ++d) {                   // position rows
            Atr.emplace_back(row, xi(k+1)+d,   1.0);
            Atr.emplace_back(row, xi(k)+d,    -1.0);
            Atr.emplace_back(row, xi(k)+3+d,  -dt);
            Atr.emplace_back(row, ui(k)+d,    -hdt2);
            addBound(0.0, 0.0); ++row;
        }
        for (int d = 0; d < 3; ++d) {                   // velocity rows
            Atr.emplace_back(row, xi(k+1)+3+d, 1.0);
            Atr.emplace_back(row, xi(k)+3+d,  -1.0);
            Atr.emplace_back(row, ui(k)+d,    -dt);
            addBound(0.0, 0.0); ++row;
        }
    }
    // velocity box (horizontal vMax_, vertical vzMax_)
    for (int k = 0; k <= N; ++k) {
        Atr.emplace_back(row, xi(k)+3, 1.0); addBound(-vMax_,  vMax_);  ++row; // vx
        Atr.emplace_back(row, xi(k)+4, 1.0); addBound(-vMax_,  vMax_);  ++row; // vy
        Atr.emplace_back(row, xi(k)+5, 1.0); addBound(-vzMax_, vzMax_); ++row; // vz
    }
    // acceleration box (horizontal aMax_, vertical azMax_)
    for (int k = 0; k < N; ++k) {
        Atr.emplace_back(row, ui(k)+0, 1.0); addBound(-aMax_,  aMax_);  ++row;
        Atr.emplace_back(row, ui(k)+1, 1.0); addBound(-aMax_,  aMax_);  ++row;
        Atr.emplace_back(row, ui(k)+2, 1.0); addBound(-azMax_, azMax_); ++row;
    }

    if (!branch.overTake) {
        // Horizontal homotopy half-planes (Eq.8): A_k . p_{xy,k} <= b_k per step/obstacle.
        for (int k = 0; k <= N && k < (int)branch.guidanceTraj.size(); ++k) {
            const Eigen::Vector2d gp = branch.guidanceTraj[k].head<2>();
            for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                if (k >= (int)obsPredPos_[j].size()) continue;
                Eigen::Vector2d A_k; double b_k;
                const double rSum = rUav_ + obsRadius_[j];
                if (!homotopyHalfPlane(gp, obsPredPos_[j][k].head<2>(), rSum, A_k, b_k)) continue;
                Atr.emplace_back(row, xi(k)+0, A_k.x());
                Atr.emplace_back(row, xi(k)+1, A_k.y());
                addBound(-INF, b_k); ++row;
            }
        }
    } else {
        // 3-D "fly-over" branch: where the reference passes horizontally near an
        // obstacle, force z_k >= obstacle_top + clearance. The nearness test uses the
        // (fixed) reference trajectory, so the resulting constraint stays linear in z_k.
        for (int k = 0; k <= N && k < (int)branch.guidanceTraj.size(); ++k) {
            const Eigen::Vector2d gp = branch.guidanceTraj[k].head<2>();
            double zFloor = -INF;
            for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                if (k >= (int)obsPredPos_[j].size()) continue;
                const double horizDist = (gp - obsPredPos_[j][k].head<2>()).norm();
                if (horizDist < rUav_ + obsRadius_[j] + 0.5)   // footprint + margin
                    zFloor = std::max(zFloor, obsTop_[j] + vClearance_);
            }
            if (zFloor > -INF) {
                Atr.emplace_back(row, xi(k)+2, 1.0); addBound(zFloor, INF); ++row;  // z_k >= zFloor
            }
        }
    }

    if (map_ && staticHalfplaneSearchRadius_ > 1e-3) {
        const int rays = std::max(4, staticHalfplaneRays_);
        const double step = std::max(0.05, map_->getRes());
        const int radialSteps = std::max(1, (int)std::ceil(staticHalfplaneSearchRadius_ / step));
        const double clearance = std::max(0.02, staticHalfplaneClearance_);

        for (int k = 0; k <= N && k < (int)branch.guidanceTraj.size(); ++k) {
            Eigen::Vector3d gp3 = branch.guidanceTraj[k];
            gp3.z() = zLap_;
            for (int r = 0; r < rays; ++r) {
                const double th = 2.0 * M_PI * (double)r / (double)rays;
                const Eigen::Vector2d dir(std::cos(th), std::sin(th));
                Eigen::Vector3d occ = gp3;
                bool found = false;
                for (int s = 1; s <= radialSteps; ++s) {
                    Eigen::Vector3d q = gp3;
                    q.x() += dir.x() * step * (double)s;
                    q.y() += dir.y() * step * (double)s;
                    if (map_->isInflatedOccupied(q)) {
                        occ = q;
                        found = true;
                        break;
                    }
                }
                if (!found) continue;
                const Eigen::Vector2d diff = occ.head<2>() - gp3.head<2>();
                const double dn = diff.norm();
                if (dn < clearance + 1e-4) continue;
                const Eigen::Vector2d A = diff / dn;
                const double b = A.dot(occ.head<2>() - A * clearance);
                Atr.emplace_back(row, xi(k)+0, A.x());
                Atr.emplace_back(row, xi(k)+1, A.y());
                addBound(-INF, b);
                ++row;
            }
        }
    }

    const int nCon = row;
    Eigen::SparseMatrix<double> Ac(nCon, nVar);
    Ac.setFromTriplets(Atr.begin(), Atr.end());
    Eigen::VectorXd l(nCon), u(nCon);
    for (int i = 0; i < nCon; ++i) { l(i) = lo[i]; u(i) = up[i]; }

    // ---- solve ----
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    // Speed: this QP is re-solved every cycle for every branch, so cap the work.
    // Loose tolerances + no polish + a hard iteration/time cap keep each solve in
    // the low-ms range instead of grinding to the 4000-iter default on tight/poorly
    // conditioned branches (the usual cause of T-MPC lag).
    solver.settings()->setMaxIteration(800);
    solver.settings()->setAbsoluteTolerance(2e-3);
    solver.settings()->setRelativeTolerance(2e-3);
    solver.settings()->setPolish(false);
    solver.settings()->setAdaptiveRho(true);
    solver.settings()->setTimeLimit(0.001 * (double)threadTimeoutMs_);
    solver.data()->setNumberOfVariables(nVar);
    solver.data()->setNumberOfConstraints(nCon);
    if (!solver.data()->setHessianMatrix(P))            { branch.feasible = false; return; }
    if (!solver.data()->setGradient(q))                 { branch.feasible = false; return; }
    if (!solver.data()->setLinearConstraintsMatrix(Ac)) { branch.feasible = false; return; }
    if (!solver.data()->setLowerBound(l))               { branch.feasible = false; return; }
    if (!solver.data()->setUpperBound(u))               { branch.feasible = false; return; }
    if (!solver.initSolver())                           { branch.feasible = false; return; }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        return;
    }
    const Eigen::VectorXd sol = solver.getSolution();

    // Reject garbage solutions (non-finite or absurdly large). Without this a failed
    // / diverged solve could be streamed to the controller as a setpoint and fly the
    // drone off into space. World is ~+-15 m, so 1e4 only catches true garbage.
    if (!sol.allFinite() || sol.cwiseAbs().maxCoeff() > 1e4) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        return;
    }

    // optimal cost J = 0.5 z'Pz + q'z  (constant ref term dropped; equal for all branches)
    branch.cost = 0.5 * sol.dot(P * sol) + q.dot(sol);
    branch.feasible = true;

    // extract full 3-D states [x,y,z,vx,vy,vz]
    branch.statesSol.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        Eigen::VectorXd s(6);
        s << sol(xi(k)+0), sol(xi(k)+1), sol(xi(k)+2),
             sol(xi(k)+3), sol(xi(k)+4), sol(xi(k)+5);
        branch.statesSol[k] = s;
    }

    if (trajectoryHitsStaticMap(branch.statesSol)) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.statesSol.clear();
        branch.controlsSol.clear();
    }
}

// ---------------------------------------------------------------------------
// Decision (Eq.12): i* = argmin_i w_i J_i ; w_i = consistency_ci if branch i is the
// previously executed class, else 1. Infeasible branches are skipped.
// ---------------------------------------------------------------------------
void tmpcPlanner::decide() {
    bestIdx_ = -1;
    double bestVal = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < branches_.size(); ++i) {
        if (!branches_[i].feasible) continue;
        double w = (branches_[i].classId == prevClassId_ && prevClassId_ >= 0)
                 ? consistencyCi_ : 1.0;
        double val = w * branches_[i].cost;
        if (val < bestVal) { bestVal = val; bestIdx_ = (int)i; }
    }
    if (bestIdx_ >= 0) {
        bestClassId_ = branches_[bestIdx_].classId;
        prevClassId_ = bestClassId_;
#if TMPC_HAVE_GUIDANCE_PLANNER
        if (guidance_) {
            if (bestClassId_ >= 0) guidance_->OverrideSelectedTrajectory(bestClassId_);
            else                  guidance_->OverrideSelectedTrajectory(0, true);
        }
#endif
    }
}

bool tmpcPlanner::plan() {
    auto t0 = std::chrono::steady_clock::now();

    buildConstantVelocityPredictions();
    buildGoalGrid();
    buildStaticAwareReference();
    buildGoalGridFromLocalRef();
    if ((int)localRef_.size() < horizon_ + 1) {   // no valid reference path was set
        ROS_WARN_THROTTLE(2.0, "[tmpcPlanner] empty/short local reference "
                               "(reference path not set?); skipping plan - drone holds.");
        planTimeMs_ = 0.0;
        bestIdx_ = -1;
        return false;
    }
    if (!runGuidance()) { planTimeMs_ = 0.0; return false; }

    // Solve each branch's local MPC. Each OsqpEigen solver is constructed locally
    // inside solveBranch (independent state), so the branches can be solved in
    // parallel — this mirrors the paper's P+1 parallel local planners. Set
    // solve_sequential:true for deterministic single-thread timing.
#ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) num_threads(parallelThreads_) if(!solveSequential_)
#endif
    for (int i = 0; i < (int)branches_.size(); ++i) solveBranch(branches_[i]);

    decide();

    auto t1 = std::chrono::steady_clock::now();
    planTimeMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ROS_INFO_THROTTLE(1.0,
        "[tmpcPlanner] obs=%zu branches=%zu best=%d class=%d plan=%.1fms zLap=%.2f",
        obsPredPos_.size(), branches_.size(), bestIdx_, bestClassId_, planTimeMs_, zLap_);
    return bestIdx_ >= 0;
}

bool tmpcPlanner::getBestTrajectory(std::vector<Eigen::Vector3d>& traj) const {
    traj.clear();
    if (bestIdx_ < 0) return false;
    for (const auto& s : branches_[bestIdx_].statesSol)
        traj.emplace_back(s(0), s(1), s(2));
    return true;
}

bool tmpcPlanner::getBestStates(std::vector<Eigen::VectorXd>& states) const {
    states.clear();
    if (bestIdx_ < 0) return false;
    states = branches_[bestIdx_].statesSol;
    return true;
}

bool tmpcPlanner::getLocalReference(std::vector<Eigen::Vector3d>& ref) const {
    ref = localRef_;
    return !ref.empty();
}

bool tmpcPlanner::trajectoryHitsStaticMap(const std::vector<Eigen::VectorXd>& states) const {
    if (!map_) return false;
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();
    bool havePrev = false;
    for (const auto& s : states) {
        if (s.size() < 3 || !s.allFinite()) return true;
        Eigen::Vector3d p(s(0), s(1), s(2));
        if (map_->isInflatedOccupied(p)) return true;
        if (havePrev && (p - prev).norm() > 1e-4 && map_->isInflatedOccupiedLine(prev, p))
            return true;
        prev = p;
        havePrev = true;
    }
    return false;
}

bool tmpcPlanner::getBestTrajectory(nav_msgs::Path& traj) const {
    traj.poses.clear();
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "map";
    if (bestIdx_ < 0) return false;
    for (const auto& s : branches_[bestIdx_].statesSol) {
        geometry_msgs::PoseStamped ps;
        ps.header = traj.header;
        ps.pose.position.x = s(0);
        ps.pose.position.y = s(1);
        ps.pose.position.z = s(2);
        ps.pose.orientation.w = 1.0;
        traj.poses.push_back(ps);
    }
    return true;
}

// ---- visualization --------------------------------------------------------
static visualization_msgs::Marker lineMarker(int id, double r, double g, double b,
                                             double width, const std::string& ns) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = ns; m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = width;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    m.lifetime = ros::Duration(0.5);
    return m;
}

void tmpcPlanner::publishGuidancePaths() const {
    visualization_msgs::MarkerArray arr;
    for (size_t i = 0; i < branches_.size(); ++i) {
        auto m = lineMarker((int)i, 0.7, 0.3, 0.85, 0.05, "tmpc_guidance");
        for (const auto& p : branches_[i].guidanceTraj) {
            geometry_msgs::Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            m.points.push_back(pt);
        }
        arr.markers.push_back(m);
    }
    guidancePathsPub_.publish(arr);
}

// Color a branch by its type so the chosen passing behavior is obvious in RViz:
//   cyan  = vertical "fly-over"      (overTake)
//   gray  = unguided / free MPC      (!guided && !overTake)
//   warm palette by class id         = horizontal left/right homotopy branches
static void tmpcBranchColor(const TMPCBranch& b, double rgb[3]) {
    if (b.overTake)      { rgb[0] = 0.20; rgb[1] = 0.85; rgb[2] = 1.00; return; }
    if (!b.guided)       { rgb[0] = 0.60; rgb[1] = 0.60; rgb[2] = 0.60; return; }
    static const double pal[4][3] = {
        {0.95, 0.45, 0.15}, {0.30, 0.55, 0.95}, {0.60, 0.30, 0.85}, {0.95, 0.75, 0.15}};
    const int idx = ((b.classId % 4) + 4) % 4;
    rgb[0] = pal[idx][0]; rgb[1] = pal[idx][1]; rgb[2] = pal[idx][2];
}

void tmpcPlanner::publishOptimizedTrajectories() const {
    visualization_msgs::MarkerArray arr;

    // clear stale markers from the previous iteration
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "tmpc_optimized";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int mid = 0;
    for (size_t i = 0; i < branches_.size(); ++i) {
        if (!branches_[i].feasible) continue;
        const bool best = ((int)i == bestIdx_);
        double rgb[3]; tmpcBranchColor(branches_[i], rgb);
        auto m = lineMarker(mid++, rgb[0], rgb[1], rgb[2],
                            best ? 0.14 : 0.05, "tmpc_optimized");
        m.color.a = best ? 1.0 : 0.45;   // chosen branch is bright + thick
        for (const auto& s : branches_[i].statesSol) {
            geometry_msgs::Point pt; pt.x = s(0); pt.y = s(1); pt.z = s(2);
            m.points.push_back(pt);
        }
        arr.markers.push_back(m);
    }

    // floating text label over the chosen branch's start: OVER / FREE / L/R <class>
    if (bestIdx_ >= 0 && !branches_[bestIdx_].statesSol.empty()) {
        const auto& b = branches_[bestIdx_];
        const auto& s0 = b.statesSol.front();
        visualization_msgs::Marker txt;
        txt.header.frame_id = "map";
        txt.header.stamp = ros::Time::now();
        txt.ns = "tmpc_optimized";
        txt.id = mid++;
        txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::Marker::ADD;
        txt.scale.z = 0.4;
        txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0;
        txt.lifetime = ros::Duration(0.5);
        txt.pose.orientation.w = 1.0;
        txt.pose.position.x = s0(0);
        txt.pose.position.y = s0(1);
        txt.pose.position.z = s0(2) + 0.6;
        if (b.overTake)     txt.text = "OVER";
        else if (!b.guided) txt.text = "FREE";
        else                txt.text = "L/R " + std::to_string(b.classId);
        arr.markers.push_back(txt);
    }

    optimizedTrajPub_.publish(arr);
}

void tmpcPlanner::publishObstaclePredictions() const {
    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "tmpc_obstacles";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int mid = 0;
    for (size_t j = 0; j < obsPredPos_.size(); ++j) {
        // predicted-motion line (constant velocity)
        auto line = lineMarker(mid++, 1.0, 0.35, 0.2, 0.05, "tmpc_obstacles");
        for (const auto& p : obsPredPos_[j]) {
            geometry_msgs::Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = zLap_;
            line.points.push_back(pt);
        }
        arr.markers.push_back(line);

        // disc at the current position (radius = horizontal obstacle radius)
        visualization_msgs::Marker disc;
        disc.header.frame_id = "map";
        disc.header.stamp = ros::Time::now();
        disc.ns = "tmpc_obstacles";
        disc.id = mid++;
        disc.type = visualization_msgs::Marker::CYLINDER;
        disc.action = visualization_msgs::Marker::ADD;
        disc.pose.position.x = obsPredPos_[j].front().x();
        disc.pose.position.y = obsPredPos_[j].front().y();
        disc.pose.position.z = zLap_;
        disc.pose.orientation.w = 1.0;
        disc.scale.x = disc.scale.y = 2.0 * obsRadius_[j];
        disc.scale.z = 0.1;
        disc.color.r = 1.0; disc.color.g = 0.35; disc.color.b = 0.2; disc.color.a = 0.5;
        disc.lifetime = ros::Duration(0.5);
        arr.markers.push_back(disc);
    }
    dynObsPub_.publish(arr);
}

} // namespace trajPlanner
