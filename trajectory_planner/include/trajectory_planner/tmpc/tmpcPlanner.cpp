/*
    FILE: tmpcPlanner.cpp
    ----------------------------------------------------------------------------
    Implementation of T-MPC++ (de Groot et al., IEEE T-RO 2025) for the IM2-MPPI
    UAV benchmark (method id M6_tmpc).

    Pipeline per plan():
        buildConstantVelocityPredictions()  (locked decision: const-vel)
        buildGoalGrid()                      (Frenet grid along the lap ref path)
        runGuidance()                        (internal Visibility-PRM -> P branches)
        solveBranch() x (P + unguided)       (self-contained OSQP linear MPC)
        decide()                             (Eq.12 consistency-weighted min cost)

    LOCAL PLANNER (the "fork" resolved, see docs section 8): a self-contained linear MPC
    on a 3-D double integrator, solved with the bundled OsqpEigen. All branches
    keep the same original reference-tracking objective from Eq. (9a); topology
    branches differ through their collision/homotopy constraints, matching the
    paper's split between optimization objective and topology guidance.

    NOTE: written without on-machine compilation (dev box is Windows; build on the
    Linux ROS workspace). Mirrors existing OsqpEigen usage in polyTrajSolver.cpp.
*/

#include <trajectory_planner/tmpc/tmpcPlanner.h>
#include <trajectory_planner/path_search/astarOcc.h>
#include <trajectory_planner/third_party/OsqpEigen/OsqpEigen.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <random>
#include <sstream>

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

static double cross2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// Smooth a guidance polyline with a few moving-average passes (endpoints fixed).
// The raw guidance path is piecewise-linear between sparse PRM nodes; tracking that
// jagged path made the MPC output jerky. Smoothing (the paper fits cubic splines)
// gives the local MPC a smooth reference -> smoother optimized trajectory.
static std::vector<Eigen::Vector3d> smoothPolyline(std::vector<Eigen::Vector3d> path,
                                                   int passes) {
    if (path.size() < 3) return path;
    for (int it = 0; it < passes; ++it) {
        std::vector<Eigen::Vector3d> out = path;
        for (size_t i = 1; i + 1 < path.size(); ++i)
            out[i] = 0.25 * path[i - 1] + 0.5 * path[i] + 0.25 * path[i + 1];
        path.swap(out);
    }
    return path;
}

struct GuidanceNode {
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    int k = 0;
    bool goal = false;
    double goalCost = 0.0;
};

struct GuidanceCandidate {
    double cost = 0.0;
    std::vector<int> nodes;
    std::vector<Eigen::Vector3d> traj;
    std::string signature;
};

static bool sameGuidanceSample(const GuidanceNode& a, const GuidanceNode& b) {
    return a.k == b.k && (a.p - b.p).norm() < 0.18;
}

static int stableClassId(const std::string& sig) {
    uint32_t h = 2166136261u;
    for (char c : sig) {
        h ^= (uint8_t)c;
        h *= 16777619u;
    }
    return (int)(h & 0x3fffffff);
}

static Eigen::Vector2d interpolateGuidancePath(
    const std::vector<GuidanceNode>& nodes,
    const std::vector<int>& path,
    int queryK) {
    if (path.empty()) return Eigen::Vector2d::Zero();
    if (queryK <= nodes[path.front()].k) return nodes[path.front()].p;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const GuidanceNode& a = nodes[path[i]];
        const GuidanceNode& b = nodes[path[i + 1]];
        if (queryK >= a.k && queryK <= b.k) {
            double den = std::max(1, b.k - a.k);
            double u = (double)(queryK - a.k) / den;
            return a.p + u * (b.p - a.p);
        }
    }
    return nodes[path.back()].p;
}

tmpcPlanner::tmpcPlanner(const ros::NodeHandle& nh) : nh_(nh) {}

void tmpcPlanner::initParam() {
    nh_.param("tmpc/dt",                  dt_,              0.05);
    nh_.param("tmpc/horizon_steps",       horizon_,         50);
    nh_.param("tmpc/z_lap",               zLap_,            1.0);
    nh_.param("tmpc/r_uav",               rUav_,            0.30);
    nh_.param("tmpc/num_trajectories_P",  numTrajP_,        4);
    nh_.param("tmpc/add_unguided_planner",addUnguided_,     true);
    nh_.param("tmpc/prm_samples_n",       prmSamplesN_,     100);
    nh_.param("tmpc/prm_max_edges_per_node", prmMaxEdgesPerNode_, 18);
    nh_.param("tmpc/prm_max_edge_checks_per_node", prmMaxEdgeChecksPerNode_, 64);
    nh_.param<std::string>("tmpc/homotopy_method", homotopyMethod_, "h_signature");
    nh_.param("tmpc/visibility_dt",       visibilityDt_,    0.20);
    nh_.param("tmpc/smoothing_resolution",smoothingRes_,    0.05);
    nh_.param("tmpc/goal_grid_lat",       goalGridLat_,     5);
    nh_.param("tmpc/goal_grid_long",      goalGridLong_,    3);
    nh_.param("tmpc/goal_lat_spread",     goalLatSpread_,   2.0);
    nh_.param("tmpc/goal_long_distance",  goalLongDist_,    5.0);
    nh_.param("tmpc/beta_relax",          betaRelax_,       1.0);
    nh_.param("tmpc/safety_margin",       safetyMargin_,    0.25);
    nh_.param("tmpc/parallel_threads",    parallelThreads_, 5);
    nh_.param("tmpc/thread_timeout_ms",   threadTimeoutMs_, 50);
    nh_.param("tmpc/solve_sequential",    solveSequential_, true);
    nh_.param("tmpc/consistency_ci",      consistencyCi_,   0.75);
    nh_.param("tmpc/v_max",               vMax_,            2.0);
    nh_.param("tmpc/a_max",               aMax_,            3.0);
    nh_.param("tmpc/vz_max",              vzMax_,           1.0);
    nh_.param("tmpc/az_max",              azMax_,           2.0);
    nh_.param("tmpc/v_ref",               vRef_,            2.0);
    nh_.param("tmpc/enable_vertical_avoidance", vertical_,  false);
    nh_.param("tmpc/vertical_clearance",  vClearance_,      0.4);
    nh_.param<std::string>("tmpc/prediction_source", predictionSource_, "constant_velocity");
    nh_.param("tmpc/max_obstacles",       maxObstacles_,    12);
    nh_.param("tmpc/use_static_astar",    useStaticAstar_,  false);
    nh_.param("tmpc/static_astar_step",   staticAstarStep_, 0.20);
    nh_.param("tmpc/static_astar_pool_xy",staticAstarPoolXY_, 80);
    nh_.param("tmpc/static_astar_pool_z", staticAstarPoolZ_, 16);
    nh_.param("tmpc/static_halfplane_search_radius", staticHalfplaneSearchRadius_, 0.8);
    nh_.param("tmpc/static_halfplane_clearance",     staticHalfplaneClearance_, 0.25);
    nh_.param("tmpc/static_halfplane_rays",          staticHalfplaneRays_, 16);
    nh_.param("tmpc/static_post_check_clearance",    staticPostCheckClearance_, 0.25);
    nh_.param("tmpc/static_fov_range",               staticFovRange_, 7.0);
    nh_.param("tmpc/publish_guidance_markers", publishGuidanceMarkers_, true);
    nh_.param("tmpc/publish_optimized_markers", publishOptimizedMarkers_, false);
    nh_.param("tmpc/publish_obstacle_prediction_markers", publishObstaclePredictionMarkers_, false);
    nh_.param("tmpc/publish_visible_static_markers", publishVisibleStaticMarkers_, true);
    nh_.param("tmpc/visible_static_marker_stride", visibleStaticMarkerStride_, 2);
    nh_.param("tmpc/visible_static_marker_max_points", visibleStaticMarkerMaxPoints_, 6000);
    // cost weights
    nh_.param("tmpc/cost_weights/w_contour", wContour_, 1.0);
    nh_.param("tmpc/cost_weights/w_lag",     wLag_,     1.0);
    nh_.param("tmpc/cost_weights/w_vel",     wVel_,     0.1);
    nh_.param("tmpc/cost_weights/w_acc",     wAcc_,     0.05);

    parallelThreads_ = std::max(1, parallelThreads_);
    threadTimeoutMs_ = std::max(1, threadTimeoutMs_);
    prmMaxEdgesPerNode_ = std::max(1, prmMaxEdgesPerNode_);
    prmMaxEdgeChecksPerNode_ = std::max(prmMaxEdgesPerNode_, prmMaxEdgeChecksPerNode_);
    visibilityDt_ = std::max(dt_, visibilityDt_);
    staticAstarStep_ = std::max(0.05, staticAstarStep_);
    staticAstarPoolXY_ = std::max(20, staticAstarPoolXY_);
    staticAstarPoolZ_ = std::max(3, staticAstarPoolZ_);
    staticHalfplaneSearchRadius_ = std::max(0.0, staticHalfplaneSearchRadius_);
    staticHalfplaneClearance_ = std::max(0.02, staticHalfplaneClearance_);
    staticHalfplaneRays_ = std::max(4, staticHalfplaneRays_);
    staticPostCheckClearance_ = std::max(0.0, staticPostCheckClearance_);
    staticFovRange_ = std::max(1.0, staticFovRange_);
    visibleStaticMarkerStride_ = std::max(1, visibleStaticMarkerStride_);
    visibleStaticMarkerMaxPoints_ = std::max(100, visibleStaticMarkerMaxPoints_);

    ROS_INFO("[tmpcPlanner] init: P=%d unguided=%d horizon=%d dt=%.3f z_lap=%.2f pred=%s",
             numTrajP_, (int)addUnguided_, horizon_, dt_, zLap_, predictionSource_.c_str());
}

void tmpcPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map) { map_ = map; }

void tmpcPlanner::registerPub() {
    guidancePathsPub_   = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/guidance_paths", 1);
    optimizedTrajPub_   = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/optimized_trajectories", 1);
    goalGridPub_        = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/goal", 1);
    dynObsPub_          = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/dynamic_obstacle_predictions", 1);
    visibleStaticPub_   = nh_.advertise<visualization_msgs::MarkerArray>("tmpc/visible_static_obstacles", 1);
}

void tmpcPlanner::resetDiagnostics() {
    lastPlanStatus_ = "running";
    lastGuidanceNodes_ = 0;
    lastGuidanceGoals_ = 0;
    lastGuidanceExpansions_ = 0;
    lastGuidedBranches_ = 0;
    lastTotalBranches_ = 0;
    lastFeasibleBranches_ = 0;
    lastStaticRejects_ = 0;
    lastDynamicRejects_ = 0;
    lastSolveRejects_ = 0;
    lastSetupRejects_ = 0;
    lastNumericRejects_ = 0;
    lastStaticDirectBlocked_ = false;
    lastStaticAstarActive_ = false;
    lastStaticAstarFailed_ = false;
}

void tmpcPlanner::updateDiagnosticsAfterSolve() {
    lastTotalBranches_ = (int)branches_.size();
    for (const auto& b : branches_) {
        if (b.guided) ++lastGuidedBranches_;
        if (b.feasible) {
            ++lastFeasibleBranches_;
        } else if (b.status == "static_collision") {
            ++lastStaticRejects_;
        } else if (b.status == "dynamic_collision") {
            ++lastDynamicRejects_;
        } else if (b.status == "solve_error" || b.status == "init_solver" ||
                   b.status.find("solve_") == 0) {
            ++lastSolveRejects_;
        } else if (b.status.find("setup_") == 0 || b.status == "short_ref") {
            ++lastSetupRejects_;
        } else if (b.status == "numeric") {
            ++lastNumericRejects_;
        }
    }
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
// Goal grid in Frenet frame: project the current position onto the reference
// path, march forward by arc length, then lay a lateral x longitudinal grid.
// ---------------------------------------------------------------------------
void tmpcPlanner::buildGoalGrid() {
    goalGrid_.clear();
    localRef_.clear();
    if (refPath_.size() < 2) return;

    // Closest point on a reference segment, not just closest waypoint. Using a
    // waypoint index can jump backward/forward on coarse paths and makes the
    // first local reference segment nearly zero or even point the wrong way.
    int projSeg = 0;
    double projAlpha = 0.0;
    double best = std::numeric_limits<double>::infinity();
    const int last = (int)refPath_.size() - 1;
    const Eigen::Vector2d cxy = currPos_.head<2>();
    for (int i = 0; i < last; ++i) {
        const Eigen::Vector2d a = refPath_[i].head<2>();
        const Eigen::Vector2d b = refPath_[i + 1].head<2>();
        const Eigen::Vector2d ab = b - a;
        const double len2 = ab.squaredNorm();
        const double u = (len2 > 1e-9)
            ? std::max(0.0, std::min(1.0, (cxy - a).dot(ab) / len2))
            : 0.0;
        const double d2 = (a + u * ab - cxy).squaredNorm();
        if (d2 < best) {
            best = d2;
            projSeg = i;
            projAlpha = u;
        }
    }

    // Local horizon reference: resample refPath forward at v_ref*dt arc-length per
    // step from the projected arc-length position. localRef_[0] remains the true
    // UAV position for a smooth initial condition, but k>=1 is always ahead on the
    // reference path. The far points derived from this reference are planner goals;
    // the controller only tracks the optimized MPC trajectory.
    {
        const double step = vRef_ * dt_;
        int    i = projSeg;          // current segment [i, i+1]
        double segLen0 = (refPath_[i + 1].head<2>() - refPath_[i].head<2>()).norm();
        double segConsumed = projAlpha * segLen0;

        Eigen::Vector3d p0 = currPos_;
        p0.z() = zLap_;
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

    // A series of local goal points ahead on the reference path. These are PRM/MPC
    // goals, not controller targets: the downstream controller tracks the selected
    // MPC trajectory in time. Longitudinal goals end at goalLongDist_ and include a
    // few nearer-but-still-forward alternatives so the PRM has reachable gates.
    const double refStep = std::max(0.05, vRef_ * dt_);
    const double maxGoalDist = std::min(goalLongDist_, refStep * (double)(localRef_.size() - 1));
    const double minGoalDist = std::max(refStep, 0.65 * maxGoalDist);

    for (int lo = 0; lo < goalGridLong_; ++lo) {
        const double alpha = (goalGridLong_ <= 1) ? 1.0
                           : (double)lo / (double)(goalGridLong_ - 1);
        const double goalDist = minGoalDist + alpha * (maxGoalDist - minGoalDist);
        const int lookIdx = std::max(1, std::min((int)localRef_.size() - 1,
            (int)std::ceil(goalDist / refStep)));
        Eigen::Vector2d center = localRef_[lookIdx].head<2>();
        Eigen::Vector2d tang   = localTangent(localRef_, lookIdx);
        Eigen::Vector2d normal(-tang.y(), tang.x());
        for (int la = 0; la < goalGridLat_; ++la) {
            double frac = (goalGridLat_ == 1) ? 0.0
                        : (double)la / (goalGridLat_ - 1) - 0.5;       // -0.5..0.5
            double lat = frac * goalLatSpread_;
            Eigen::Vector2d g = center + normal * lat;
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
    if (pointHitsStaticMapWithMargin(start, staticPostCheckClearance_) ||
        pointHitsStaticMapWithMargin(goal, staticPostCheckClearance_)) {
        directBlocked = true;
    } else if (segmentHitsStaticMapWithMargin(start, goal, staticPostCheckClearance_)) {
        directBlocked = true;
    } else {
        for (const auto& p0 : localRef_) {
            Eigen::Vector3d p = p0;
            p.z() = zLap_;
            if (pointHitsStaticMapWithMargin(p, staticPostCheckClearance_)) {
                directBlocked = true;
                break;
            }
        }
    }
    if (!directBlocked) return;
    lastStaticDirectBlocked_ = true;

    if (pointHitsStaticMapWithMargin(start, staticPostCheckClearance_)) {
        lastStaticAstarFailed_ = true;
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] static A* fallback skipped because start violates static clearance.");
        return;
    }

    if (pointHitsStaticMapWithMargin(goal, staticPostCheckClearance_)) {
        bool foundGoal = false;
        Eigen::Vector3d bestGoal = goal;
        double bestScore = std::numeric_limits<double>::infinity();

        auto considerGoal = [&](const Eigen::Vector3d& raw) {
            Eigen::Vector3d c = raw;
            c.z() = zLap_;
            if ((c.head<2>() - start.head<2>()).norm() < 0.5) return;
            if (pointHitsStaticMapWithMargin(c, staticPostCheckClearance_)) return;
            const double score = (c.head<2>() - goal.head<2>()).norm();
            if (score < bestScore) {
                bestScore = score;
                bestGoal = c;
                foundGoal = true;
            }
        };

        for (const auto& g : goalGrid_) considerGoal(g);
        for (int i = (int)localRef_.size() - 1; i >= 1; --i) considerGoal(localRef_[i]);

        if (!foundGoal) {
            lastStaticAstarFailed_ = true;
            ROS_WARN_THROTTLE(1.0,
                "[tmpcPlanner] static A* fallback failed before search: local goal is occupied and no free fallback goal exists.");
            return;
        }
        goal = bestGoal;
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] static A* fallback retargeted occupied local goal to free point (%.2f, %.2f, %.2f).",
            goal.x(), goal.y(), goal.z());
    }

    const double dist = std::max(1.0, (goal.head<2>() - start.head<2>()).norm());
    const int xyPool = std::max(staticAstarPoolXY_,
        (int)std::ceil(dist / staticAstarStep_) + 24);
    const double minH = std::max(0.05, zLap_ - 0.35);
    const double maxH = zLap_ + (vertical_ ? std::max(0.8, vClearance_ + 0.8) : 0.35);

    AStar astar;
    astar.initGridMap(map_, Eigen::Vector3i(xyPool, xyPool, staticAstarPoolZ_),
                      minH, maxH);
    if (!astar.AstarSearch(staticAstarStep_, start, goal)) {
        lastStaticAstarFailed_ = true;
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] static A* fallback failed; keeping topology-PRM reference and static constraints.");
        return;
    }

    std::vector<Eigen::Vector3d> astarPath = astar.getPath();
    if (astarPath.size() < 2) return;
    std::vector<Eigen::Vector3d> ref = resamplePolyline(astarPath, horizon_ + 1, zLap_);
    if ((int)ref.size() == horizon_ + 1) {
        ref.front() = start;
        ref.back() = goal;
        localRef_ = ref;
        lastStaticAstarActive_ = true;
        ROS_INFO_THROTTLE(1.0,
            "[tmpcPlanner] static A* fallback reference active: %zu raw points -> %zu horizon points",
            astarPath.size(), localRef_.size());
    }
}

// ---------------------------------------------------------------------------
// Guidance: lightweight in-package Visibility-PRM in (x,y,t).
// Nodes are sampled along the current reference corridor and around dynamic
// obstacles. Edges are forward-in-time visibility checks against dynamic obstacle
// tubes and the inflated static map. A uniform-cost graph search enumerates low
// cost paths; topology signatures keep only homotopy-distinct branches.
// ---------------------------------------------------------------------------
bool tmpcPlanner::runGuidance() {
    branches_.clear();
    if ((int)localRef_.size() < horizon_ + 1) return false;

    const int N = horizon_;
    const double halfWidth = std::max(0.5, 0.5 * goalLatSpread_);
    const double sampleSpread = std::max(halfWidth, 0.5 * goalLatSpread_ + rUav_ + 0.5);
    const double minBlockedRefDeviation = std::max(0.50, rUav_ + staticPostCheckClearance_ + 0.20);
    const double speedLimit = std::max(4.0, std::max(vMax_ * 2.5, vRef_ * 3.0));
    std::vector<GuidanceNode> nodes;
    std::vector<int> goalIds;
    struct StaticTopoAnchor {
        int k = 0;
        Eigen::Vector2d p = Eigen::Vector2d::Zero();
        Eigen::Vector2d tangent = Eigen::Vector2d::UnitX();
    };
    std::vector<StaticTopoAnchor> staticAnchors;

    auto staticMarginAtK = [&](int k) -> double {
        // Full static clearance from k>=1. Only the start sample (k=0, where the drone
        // already is) may use reduced margin so planning is not blocked when the drone
        // starts close to a wall. The old 0.25 s ramp left the first ~5 executed steps
        // almost un-cleared -> the near-start trajectory clipped static obstacles (the
        // "hits static obstacles" bug). k>=1 now always uses the full clearance.
        return (k <= 0) ? 0.0 : staticPostCheckClearance_;
    };

    auto staticFree = [&](const Eigen::Vector2d& p, int k) -> bool {
        if (!map_) return true;
        return !pointHitsStaticMapWithMargin(
            Eigen::Vector3d(p.x(), p.y(), zLap_), staticMarginAtK(k));
    };

    auto addNode = [&](const Eigen::Vector2d& p, int k, bool goal, double goalCost) -> int {
        k = std::max(0, std::min(N, k));
        if (!staticFree(p, k)) return -1;
        GuidanceNode candidate;
        candidate.p = p;
        candidate.k = k;
        candidate.goal = goal;
        candidate.goalCost = goalCost;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (sameGuidanceSample(candidate, nodes[i])) {
                if (goal) {
                    nodes[i].goal = true;
                    nodes[i].goalCost = std::min(nodes[i].goalCost, goalCost);
                    goalIds.push_back((int)i);
                }
                return (int)i;
            }
        }
        nodes.push_back(candidate);
        int id = (int)nodes.size() - 1;
        if (goal) goalIds.push_back(id);
        return id;
    };

    const int startId = addNode(currPos_.head<2>(), 0, false, 0.0);
    if (startId < 0) {
        lastGuidanceNodes_ = (int)nodes.size();
        lastGuidanceGoals_ = (int)goalIds.size();
        ROS_WARN_THROTTLE(1.0, "[tmpcPlanner] guidance start is in collision/outside map.");
        return false;
    }

    // Static-obstacle anchors (for the topology signature's static left/right).
    bool directRefStaticBlocked = false;
    if (map_) {
        const int minAnchorGap = std::max(2, N / 8);
        int lastAnchorK = -1000;
        Eigen::Vector3d prev = localRef_[0]; prev.z() = zLap_;
        for (int k = 1; k < N; ++k) {
            Eigen::Vector3d p = localRef_[k]; p.z() = zLap_;
            const bool blocked = segmentHitsStaticMapWithMargin(prev, p, staticPostCheckClearance_);
            prev = p;
            if (!blocked) continue;
            directRefStaticBlocked = true;
            if (k - lastAnchorK < minAnchorGap || (int)staticAnchors.size() >= 6) continue;
            Eigen::Vector2d t = localTangent(localRef_, k);
            if (t.norm() < 1e-6) t = Eigen::Vector2d::UnitX();
            t.normalize();
            StaticTopoAnchor anchor; anchor.k = k; anchor.p = localRef_[k].head<2>(); anchor.tangent = t;
            staticAnchors.push_back(anchor);
            lastStaticDirectBlocked_ = true; lastAnchorK = k;
        }
    }

    bool directRefDynamicBlocked = false;
    for (int k = 1; k <= N && !directRefDynamicBlocked; ++k) {
        const Eigen::Vector2d p = localRef_[k].head<2>();
        for (size_t j = 0; j < obsPredPos_.size(); ++j) {
            if (k >= (int)obsPredPos_[j].size()) continue;
            const double d = (p - obsPredPos_[j][k].head<2>()).norm();
            if (d < rUav_ + obsRadius_[j] + safetyMargin_) {
                directRefDynamicBlocked = true;
                break;
            }
        }
    }
    const bool directReferenceBlocked = directRefStaticBlocked || directRefDynamicBlocked;

    // Goal nodes (guards at k=N), added before random sampling so connectors can
    // see them. If the direct reference is already blocked, do NOT add the reference
    // endpoint as a zero-cost goal; otherwise the shortest graph path collapses back
    // onto the unsafe centerline instead of committing to a left/right topology.
    if (!directReferenceBlocked) addNode(localRef_.back().head<2>(), N, true, 0.0);
    for (const auto& g3 : goalGrid_) {
        const double centerDist = (g3.head<2>() - localRef_.back().head<2>()).norm();
        if (directReferenceBlocked && centerDist < minBlockedRefDeviation) continue;
        addNode(g3.head<2>(), N, true, centerDist);
    }

    // Space-time visibility (Visibility-PRM): forward-time, speed-feasible, clear of
    // dynamic tubes (real clearance r_uav+r_obs+margin, per step) and the inflated
    // static map. Used to classify samples as Guard/Connector.
    auto isVisibleST = [&](const Eigen::Vector2d& pa, int ka,
                           const Eigen::Vector2d& pb, int kb) -> bool {
        Eigen::Vector2d A = pa, B = pb; int kA = ka, kB = kb;
        if (kA == kB) return false;
        if (kA > kB) { std::swap(A, B); std::swap(kA, kB); }
        const double dtSpan = std::max(1e-3, (double)(kB - kA) * dt_);
        if ((B - A).norm() / dtSpan > speedLimit) return false;
        const int visStep = std::max(1, (int)std::round(visibilityDt_ / dt_));
        Eigen::Vector3d prevStatic(A.x(), A.y(), zLap_);
        for (int k = kA; k <= kB; ++k) {
            const double u = (double)(k - kA) / (double)std::max(1, kB - kA);
            const Eigen::Vector2d p2 = A + u * (B - A);
            for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                if (k >= (int)obsPredPos_[j].size()) continue;
                if ((p2 - obsPredPos_[j][k].head<2>()).norm() < rUav_ + obsRadius_[j] + safetyMargin_)
                    return false;
            }
            if (map_ && ((k - kA) % visStep == 0 || k == kB)) {
                const Eigen::Vector3d p3(p2.x(), p2.y(), zLap_);
                if (segmentHitsStaticMapWithMargin(prevStatic, p3, staticMarginAtK(k))) return false;
                prevStatic = p3;
            }
        }
        return true;
    };

    // Visibility-PRM Guard/Connector admission rule for one sample.
    auto classifyAndAdd = [&](const Eigen::Vector2d& p, int k, bool keepProgress) {
        if (!staticFree(p, k)) return;
        int nGuards = 0, nGoals = 0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!isVisibleST(p, k, nodes[i].p, nodes[i].k)) continue;
            if (nodes[i].goal) ++nGoals; else ++nGuards;
            if (nGuards > 2) break;               // >2 visible guards -> redundant
        }
        //   sees nothing               -> GUARD (explore a new region)
        //   sees 2 guards, or guard+goal -> CONNECTOR (bridge two regions)
        //   sees exactly 1 guard        -> redundant, discard
        // For a directed space-time graph, deterministic corridor/side samples also
        // need "progress" milestones: a point that sees only one earlier guard may
        // still be the only time-feasible step toward a later goal.
        const bool asGuard     = (nGuards == 0 && nGoals == 0);
        const bool asConnector = (nGuards == 2) || (nGuards >= 1 && nGoals >= 1);
        const bool asProgress  = keepProgress && (nGuards == 1 && nGoals == 0);
        if (asGuard || asConnector || asProgress) addNode(p, k, false, 0.0);
    };

    // Graph propagation: re-seed guards from the previous iteration's guidance
    // samples, time-decremented by one control step (receding horizon), so topology
    // classes persist and re-form quickly across cycles.
    for (const auto& seed : prevGuidanceSeed_) {
        const int k = seed.second - 1;
        if (k >= 1 && k <= N - 1 && staticFree(seed.first, k)) addNode(seed.first, k, false, 0.0);
    }

    // Deterministic connector candidates around static blocks. Random samples alone
    // often miss the narrow left/right gates around a wall or pillar, so each blocked
    // reference segment contributes samples on both sides and at nearby time layers.
    if (!staticAnchors.empty()) {
        const double staticSep = std::max(sampleSpread,
            staticHalfplaneSearchRadius_ + staticPostCheckClearance_ + rUav_ + 0.7);
        const double offsets[] = {-staticSep, -0.65 * staticSep, 0.65 * staticSep, staticSep};
        const int timeOffsets[] = {-4, -2, 0, 2, 4};
        for (const auto& anchor : staticAnchors) {
            for (int dk : timeOffsets) {
                const int k = std::max(1, std::min(N - 1, anchor.k + dk));
                const Eigen::Vector2d c = localRef_[k].head<2>();
                Eigen::Vector2d t = localTangent(localRef_, k);
                if (t.norm() < 1e-6) t = anchor.tangent;
                t.normalize();
                const Eigen::Vector2d nrm(-t.y(), t.x());
                for (double off : offsets) classifyAndAdd(c + off * nrm, k, true);
            }
        }
    }

    // Random Guard/Connector sampling along the reference corridor.
    const uint32_t seed =
        20260517u ^
        (uint32_t)(++guidanceSampleCounter_ * 2654435761u) ^
        (uint32_t)(obsPredPos_.size() * 131u) ^
        (uint32_t)(std::llround((currPos_.x() + 50.0) * 10.0) * 73856093u) ^
        (uint32_t)(std::llround((currPos_.y() + 50.0) * 10.0) * 19349663u);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> kDist(1, std::max(1, N - 1));
    std::uniform_real_distribution<double> latDist(-sampleSpread, sampleSpread);
    const int sampleBudget = std::max(20, prmSamplesN_);
    for (int iter = 0; iter < sampleBudget; ++iter) {
        const int k = kDist(rng);
        const Eigen::Vector2d c = localRef_[k].head<2>();
        const Eigen::Vector2d t = localTangent(localRef_, k);
        const Eigen::Vector2d nrm(-t.y(), t.x());
        classifyAndAdd(c + latDist(rng) * nrm, k, false);
    }

    // Low-discrepancy deterministic corridor samples keep the graph connected when
    // the random guard pass happens to miss the centerline progress nodes.
    {
        int latCount = std::max(3, goalGridLat_);
        if (latCount % 2 == 0) ++latCount;
        const int halfLat = latCount / 2;
        const int longSamples = std::max(3, std::min(N - 1, std::max(1, prmSamplesN_) / latCount));
        for (int s = 1; s <= longSamples; ++s) {
            const int k = std::max(1, std::min(N - 1,
                (int)std::round((double)s * N / (double)(longSamples + 1))));
            const Eigen::Vector2d c = localRef_[k].head<2>();
            const Eigen::Vector2d t = localTangent(localRef_, k);
            const Eigen::Vector2d nrm(-t.y(), t.x());
            for (int li = -halfLat; li <= halfLat; ++li) {
                const double off = (halfLat > 0) ? sampleSpread * (double)li / (double)halfLat : 0.0;
                classifyAndAdd(c + off * nrm, k, true);
            }
        }
    }

    // Deterministic seeds on both sides of each dynamic obstacle's closest approach,
    // so the sampler reliably discovers the left/right classes around obstacles.
    for (size_t j = 0; j < obsPredPos_.size(); ++j) {
        int bestK = 0; double bestClear = std::numeric_limits<double>::infinity();
        for (int k = 0; k <= N && k < (int)obsPredPos_[j].size(); ++k) {
            const double clear = (localRef_[k].head<2>() - obsPredPos_[j][k].head<2>()).norm()
                               - (rUav_ + obsRadius_[j]);
            if (clear < bestClear) { bestClear = clear; bestK = k; }
        }
        if (bestClear > sampleSpread + rUav_ + obsRadius_[j] + 1.0) continue;
        const double sep = rUav_ + obsRadius_[j] + std::max(0.55, 0.35 * goalLatSpread_);
        for (int dk : {-4, -2, 0, 2, 4}) {
            const int k = std::max(1, std::min(N - 1, bestK + dk));
            if (k >= (int)obsPredPos_[j].size()) continue;
            const Eigen::Vector2d t = localTangent(localRef_, k);
            const Eigen::Vector2d nrm(-t.y(), t.x());
            const Eigen::Vector2d o = obsPredPos_[j][k].head<2>();
            classifyAndAdd(o + sep * nrm, k, true);
            classifyAndAdd(o - sep * nrm, k, true);
        }
    }

    if (goalIds.empty()) {
        lastGuidanceNodes_ = (int)nodes.size();
        lastGuidanceGoals_ = 0;
        ROS_WARN_THROTTLE(1.0, "[tmpcPlanner] guidance has no collision-free goals.");
        return false;
    }

    auto edgeVisible = [&](int aId, int bId) -> bool {
        const GuidanceNode& a = nodes[aId];
        const GuidanceNode& b = nodes[bId];
        if (b.k <= a.k) return false;
        const double dtSpan = std::max(1e-3, (double)(b.k - a.k) * dt_);
        if ((b.p - a.p).norm() / dtSpan > speedLimit) return false;

        const int visStep = std::max(1, (int)std::round(visibilityDt_ / dt_));
        Eigen::Vector3d prevStatic(a.p.x(), a.p.y(), zLap_);
        for (int k = a.k; k <= b.k; ++k) {
            const double u = (double)(k - a.k) / (double)std::max(1, b.k - a.k);
            const Eigen::Vector2d p2 = a.p + u * (b.p - a.p);
            // Dynamic obstacles: check EVERY step, with the SAME clearance the local
            // planner + post-check enforce (r_uav + r_obs + safety_margin). Sampling
            // only every visStep let a fast crossing obstacle slip through the gap so
            // the guidance path passed straight through it; 0.10 m also grazed too close.
            for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                if (k >= (int)obsPredPos_[j].size()) continue;
                const double d = (p2 - obsPredPos_[j][k].head<2>()).norm();
                if (d < rUav_ + obsRadius_[j] + safetyMargin_) return false;
            }
            // Static map: obstacles don't move, so the coarser visStep grid suffices.
            if (map_ && ((k - a.k) % visStep == 0 || k == b.k)) {
                const Eigen::Vector3d p3(p2.x(), p2.y(), zLap_);
                if (segmentHitsStaticMapWithMargin(prevStatic, p3, staticMarginAtK(k))) return false;
                prevStatic = p3;
            }
        }
        return true;
    };

    std::vector<std::vector<std::pair<int, double>>> adj(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        struct EdgeCandidate {
            int id = -1;
            double key = 0.0;
        };
        std::vector<EdgeCandidate> edgeCandidates;
        edgeCandidates.reserve(nodes.size());
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (nodes[j].k <= nodes[i].k) continue;
            const double spatial = (nodes[j].p - nodes[i].p).norm();
            const double temporal = std::abs(nodes[j].k - nodes[i].k);
            // Order candidates by geometric length only (+ tiny temporal regularizer).
            // No distance-to-reference bias: it would pull the graph onto the centerline
            // and suppress genuinely distinct topology classes.
            edgeCandidates.push_back({(int)j, spatial + 0.08 * temporal});
        }
        std::sort(edgeCandidates.begin(), edgeCandidates.end(),
                  [](const EdgeCandidate& a, const EdgeCandidate& b){
                      return a.key < b.key;
                  });

        int checked = 0;
        for (const auto& cand : edgeCandidates) {
            if (checked++ >= prmMaxEdgeChecksPerNode_) break;
            if ((int)adj[i].size() >= prmMaxEdgesPerNode_) break;
            const int j = cand.id;
            if (!edgeVisible((int)i, j)) continue;
            double spatial = (nodes[j].p - nodes[i].p).norm();
            double temporal = 0.02 * (double)(nodes[j].k - nodes[i].k);
            double goalCost = nodes[j].goal ? nodes[j].goalCost : 0.0;
            // Edge cost = geometric path length only (+ tiny temporal regularizer) + goal
            // preference. No centerline distance term (paper-faithful): the graph search
            // finds shortest paths per topology class, and the homotopy filter keeps the
            // distinct classes; biasing toward the reference collapses them.
            adj[i].push_back(std::make_pair((int)j, spatial + temporal + goalCost));
        }
        std::sort(adj[i].begin(), adj[i].end(),
                  [](const std::pair<int,double>& a, const std::pair<int,double>& b){
                      return a.second < b.second;
                  });
    }

    auto guidanceTrajectorySafe = [&](const std::vector<Eigen::Vector3d>& traj) -> bool {
        if ((int)traj.size() < N + 1) return false;
        Eigen::Vector3d prev = traj.front();
        for (int k = 0; k <= N; ++k) {
            const Eigen::Vector3d& p3 = traj[k];
            if (map_) {
                if (pointHitsStaticMapWithMargin(p3, staticMarginAtK(k))) return false;
                if (k > 0 && segmentHitsStaticMapWithMargin(
                        prev, p3, std::max(staticMarginAtK(k - 1), staticMarginAtK(k)))) {
                    return false;
                }
            }
            if (k > 0) {
                for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                    if (k >= (int)obsPredPos_[j].size()) continue;
                    const double d = (p3.head<2>() - obsPredPos_[j][k].head<2>()).norm();
                    if (d < rUav_ + obsRadius_[j] + safetyMargin_) return false;
                }
            }
            prev = p3;
        }
        return true;
    };

    auto makeTrajectory = [&](const std::vector<int>& path) {
        std::vector<Eigen::Vector3d> traj(N + 1);
        for (int k = 0; k <= N; ++k) {
            Eigen::Vector2d p = interpolateGuidancePath(nodes, path, k);
            traj[k] = Eigen::Vector3d(p.x(), p.y(), zLap_);
        }
        // Smooth the piecewise-linear guidance so the local MPC tracks a smooth
        // reference (the paper fits cubic splines) -> smoother optimized trajectory.
        std::vector<Eigen::Vector3d> smoothed = smoothPolyline(traj, 2);
        return guidanceTrajectorySafe(smoothed) ? smoothed : traj;
    };

    auto topologySignature = [&](const std::vector<Eigen::Vector3d>& traj) {
        std::ostringstream oss;
        int relevant = 0;
        for (size_t j = 0; j < obsPredPos_.size(); ++j) {
            // Relevance: does the trajectory come near this obstacle at any time?
            double minClear = std::numeric_limits<double>::infinity();
            for (int k = 0; k <= N && k < (int)obsPredPos_[j].size(); ++k) {
                const double clear =
                    (traj[k].head<2>() - obsPredPos_[j][k].head<2>()).norm()
                    - (rUav_ + obsRadius_[j]);
                minClear = std::min(minClear, clear);
            }
            if (minClear >= halfWidth + rUav_ + obsRadius_[j] + 0.8) continue;

            // H-signature via the winding number of the RELATIVE trajectory
            // (ego - obstacle) around the origin, accumulated over the horizon. This
            // captures HOW the ego passes the moving obstacle (side + number of wraps),
            // the paper's 2-D dynamic homotopy invariant; far more robust than a single
            // closest-approach side sign (which flips with tiny geometry changes and
            // made distinct topologies collapse into one class).
            double wind = 0.0;
            for (int k = 1; k <= N && k < (int)obsPredPos_[j].size(); ++k) {
                Eigen::Vector2d a = traj[k - 1].head<2>() - obsPredPos_[j][k - 1].head<2>();
                Eigen::Vector2d b = traj[k].head<2>()     - obsPredPos_[j][k].head<2>();
                if (a.norm() < 1e-6 || b.norm() < 1e-6) continue;
                wind += std::atan2(cross2d(a, b), a.dot(b));
            }
            const int windClass = (int)std::llround(wind / M_PI);   // signed half-turns
            oss << j << ":" << windClass << ";";
            ++relevant;
        }
        for (const auto& a : staticAnchors) {
            const int k = std::max(0, std::min(N, a.k));
            const Eigen::Vector2d rel = traj[k].head<2>() - a.p;
            const double signedSide = cross2d(a.tangent, rel);
            if (std::abs(signedSide) > 0.05) {
                oss << "S" << a.k << (signedSide >= 0.0 ? "L" : "R") << ";";
                ++relevant;
            }
        }
        if (relevant == 0) oss << "direct";
        return oss.str();
    };

    struct SearchItem {
        double cost = 0.0;
        int node = 0;
        std::vector<int> path;
    };
    struct SearchCompare {
        bool operator()(const SearchItem& a, const SearchItem& b) const {
            return a.cost > b.cost;
        }
    };

    struct DistItem {
        double cost = 0.0;
        int node = 0;
    };
    struct DistCompare {
        bool operator()(const DistItem& a, const DistItem& b) const {
            return a.cost > b.cost;
        }
    };

    std::vector<GuidanceCandidate> candidates;
    std::vector<std::string> seenSignatures;

    auto maxDeviationFromLocalRef = [&](const std::vector<Eigen::Vector3d>& traj) -> double {
        double maxDev = 0.0;
        const int endK = std::min(N, (int)traj.size() - 1);
        for (int k = 1; k <= endK && k < (int)localRef_.size(); ++k) {
            maxDev = std::max(maxDev, (traj[k].head<2>() - localRef_[k].head<2>()).norm());
        }
        return maxDev;
    };

    auto addCandidateIfUsable = [&](GuidanceCandidate&& cand) -> bool {
        if (!guidanceTrajectorySafe(cand.traj)) return false;
        cand.signature = topologySignature(cand.traj);
        if (directReferenceBlocked) {
            if (cand.signature == "direct") return false;
            if (maxDeviationFromLocalRef(cand.traj) < minBlockedRefDeviation) return false;
        }
        if (std::find(seenSignatures.begin(), seenSignatures.end(), cand.signature) != seenSignatures.end())
            return false;
        seenSignatures.push_back(cand.signature);
        candidates.push_back(std::move(cand));
        return true;
    };

    // First run ordinary Dijkstra to guarantee we get one reachable topology if
    // the Visibility-PRM graph is connected. The later queue enumerator is only
    // for additional distinct classes.
    {
        std::vector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
        std::vector<int> parent(nodes.size(), -1);
        std::priority_queue<DistItem, std::vector<DistItem>, DistCompare> pq;
        dist[startId] = 0.0;
        DistItem rootDist;
        rootDist.cost = 0.0;
        rootDist.node = startId;
        pq.push(rootDist);

        int bestGoal = -1;
        while (!pq.empty()) {
            DistItem cur = pq.top();
            pq.pop();
            if (cur.cost > dist[cur.node] + 1e-9) continue;
            if (nodes[cur.node].goal) { bestGoal = cur.node; break; }
            for (const auto& e : adj[cur.node]) {
                const int nb = e.first;
                const double nextCost = cur.cost + e.second;
                if (nextCost + 1e-9 < dist[nb]) {
                    dist[nb] = nextCost;
                    parent[nb] = cur.node;
                    DistItem item;
                    item.cost = nextCost;
                    item.node = nb;
                    pq.push(item);
                }
            }
        }

        if (bestGoal >= 0) {
            std::vector<int> path;
            for (int v = bestGoal; v >= 0; v = parent[v]) {
                path.push_back(v);
                if (v == startId) break;
            }
            if (!path.empty() && path.back() == startId) {
                std::reverse(path.begin(), path.end());
                GuidanceCandidate cand;
                cand.cost = dist[bestGoal];
                cand.nodes = path;
                cand.traj = makeTrajectory(path);
                addCandidateIfUsable(std::move(cand));
            }
        }
    }

    std::priority_queue<SearchItem, std::vector<SearchItem>, SearchCompare> open;
    SearchItem root;
    root.cost = 0.0;
    root.node = startId;
    root.path.push_back(startId);
    open.push(root);
    std::vector<int> poppedPerNode(nodes.size(), 0);
    int expansions = 0;
    const int maxExpansions = candidates.empty() ? 8000 : 1600;
    const int maxPopsPerNode = 8;

    while (!open.empty() && (int)candidates.size() < std::max(1, numTrajP_) && expansions < maxExpansions) {
        SearchItem cur = open.top();
        open.pop();
        if (poppedPerNode[cur.node]++ >= maxPopsPerNode) continue;
        ++expansions;

        if (nodes[cur.node].goal) {
            GuidanceCandidate cand;
            cand.cost = cur.cost;
            cand.nodes = cur.path;
            cand.traj = makeTrajectory(cur.path);
            addCandidateIfUsable(std::move(cand));
            continue;
        }

        for (const auto& e : adj[cur.node]) {
            SearchItem next;
            next.cost = cur.cost + e.second;
            next.node = e.first;
            next.path = cur.path;
            next.path.push_back(e.first);
            open.push(std::move(next));
        }
    }

    for (const auto& cand : candidates) {
        TMPCBranch b;
        b.guided = true;
        b.classId = stableClassId(cand.signature);
        b.guidanceTraj = cand.traj;
        branches_.push_back(std::move(b));
    }

    if (branches_.empty()) {
        lastGuidanceNodes_ = (int)nodes.size();
        lastGuidanceGoals_ = (int)goalIds.size();
        lastGuidanceExpansions_ = expansions;
        ROS_WARN_THROTTLE(1.0,
            "[tmpcPlanner] internal Visibility-PRM found no guided topology path "
            "(nodes=%zu goals=%zu expansions=%d ref_blocked=%d static=%d dynamic=%d); "
            "continuing with fallback branches only if allowed.",
            nodes.size(), goalIds.size(), expansions, (int)directReferenceBlocked,
            (int)directRefStaticBlocked, (int)directRefDynamicBlocked);
        if (!addUnguided_ && !(vertical_ && !obsPredPos_.empty())) return false;
    }

    // T-MPC++ adds one non-guided local planner in parallel to the guided topology
    // branches; this is the official "++" behavior, not a replacement for topology.
    if (addUnguided_ && !directRefStaticBlocked) {
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
    // Save this iteration's accepted PRM graph samples for next-iteration graph
    // propagation (re-seeded time-decremented at the top of the next runGuidance).
    prevGuidanceSeed_.clear();
    const size_t maxPrevSeeds = 160;
    for (size_t i = 0; i < nodes.size() && prevGuidanceSeed_.size() < maxPrevSeeds; ++i) {
        if (nodes[i].goal || nodes[i].k <= 1 || nodes[i].k >= N) continue;
        prevGuidanceSeed_.emplace_back(nodes[i].p, nodes[i].k);
    }
    for (const auto& b : branches_) {
        if (!b.guided || prevGuidanceSeed_.size() >= maxPrevSeeds) continue;
        for (int k = 2; k <= N && prevGuidanceSeed_.size() < maxPrevSeeds; k += 4) {
            if (k < (int)b.guidanceTraj.size()) {
                prevGuidanceSeed_.emplace_back(b.guidanceTraj[k].head<2>(), k);
            }
        }
    }

    lastGuidanceNodes_ = (int)nodes.size();
    lastGuidanceGoals_ = (int)goalIds.size();
    lastGuidanceExpansions_ = expansions;
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
    // Eq. (9e) topology lock: keep the ego on the guidance side of the obstacle.
    // Actual collision clearance is enforced separately by the linearized Eq. (9d)
    // constraints in solveBranch().
    b_k = A_k.dot(obstaclePt) - (betaRelax_ * rSum + safetyMargin_);
    return true;
}

// ---------------------------------------------------------------------------
// Local MPC for one branch: self-contained OSQP QP. All branches use the same
// localRef_ objective (paper Eq. 9a). Guided branches add the branch-specific
// collision and homotopy half-planes (Eq. 9d/9e).
//   z = [X ; U],  X = [x_0..x_N] (NS each),  U = [u_0..u_{N-1}] (NU each)
// ---------------------------------------------------------------------------
void tmpcPlanner::solveBranch(TMPCBranch& branch) {
    branch.status = "running";
    // No valid reference -> never solve (reading an empty localRef_ would be UB and
    // produce garbage setpoints that fly the drone away).
    if ((int)localRef_.size() < horizon_ + 1) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.status = "short_ref";
        return;
    }
    const int N   = horizon_;
    const int nX  = NS * (N + 1);                       // NS=6: [x,y,z,vx,vy,vz]
    const int nU  = NU * N;                             // NU=3: [ax,ay,az]
    const int nBaseVar = nX + nU;
    int nVar = nBaseVar;                                // grows when soft constraints add slack

    auto xi = [&](int k){ return k * NS; };             // index of state block k
    auto ui = [&](int k){ return nX + k * NU; };        // index of control block k

    // ---- Hessian P (sparse, diagonal) and gradient q ----
    std::vector<Eigen::Triplet<double>> Ptr;
    std::vector<double> qVals(nBaseVar, 0.0);
    auto addDiagCost = [&](int idx, double weight) {
        if (idx >= (int)qVals.size()) qVals.resize(idx + 1, 0.0);
        Ptr.emplace_back(idx, idx, 2.0 * weight);
    };
    auto addSlackVar = [&](double weight) {
        const int idx = nVar++;
        qVals.resize(nVar, 0.0);
        addDiagCost(idx, weight);
        return idx;
    };

    // position tracking weight (isotropic approx of contour+lag; see docs)
    const double wPos = 0.5 * (wContour_ + wLag_);
    // Each guided branch optimizes around its own Visibility-PRM path. The circular
    // lap reference still defines progress and the goal grid, but the local MPC now
    // tracks the topology path it was given instead of being pulled back onto the
    // blocked centerline. The unguided/free branch keeps localRef_ as before.
    const std::vector<Eigen::Vector3d>& guidancePath =
        ((int)branch.guidanceTraj.size() >= N + 1) ? branch.guidanceTraj : localRef_;
    const std::vector<Eigen::Vector3d>& objectivePath =
        (branch.guided && (int)branch.guidanceTraj.size() >= N + 1) ? guidancePath : localRef_;
    double objectiveConstant = 0.0;
    for (int k = 0; k <= N; ++k) {
        const double wp = (k == N) ? 2.0 * wPos : wPos; // small terminal emphasis
        const Eigen::Vector3d ref = (k < (int)objectivePath.size()) ? objectivePath[k] : objectivePath.back();
        objectiveConstant += wp * ref.squaredNorm();
        for (int d = 0; d < 3; ++d) {                   // 3-D position tracking
            addDiagCost(xi(k)+d, wp);
            qVals[xi(k)+d] = -2.0*wp*ref(d);
        }
        // velocity cost toward v_ref along the objective-path tangent
        Eigen::Vector3d tang(1, 0, 0);
        if (k < (int)objectivePath.size() - 1) {
            Eigen::Vector3d t = objectivePath[k+1] - objectivePath[k];
            if (t.norm() > 1e-6) tang = t.normalized();
        }
        const Eigen::Vector3d vdes = vRef_ * tang;
        objectiveConstant += wVel_ * vdes.squaredNorm();
        for (int d = 0; d < 3; ++d) {
            addDiagCost(xi(k)+3+d, wVel_);
            qVals[xi(k)+3+d] = -2.0*wVel_*vdes(d);
        }
    }
    for (int k = 0; k < N; ++k)
        for (int d = 0; d < 3; ++d)
            addDiagCost(ui(k)+d, wAcc_);

    // ---- Constraints A z in [l, u] ----
    std::vector<Eigen::Triplet<double>> Atr;
    std::vector<double> lo, up;
    int row = 0;
    const double INF = OsqpEigen::INFTY;
    auto addBound = [&](double l, double u){ lo.push_back(l); up.push_back(u); };
    const double wCollisionSlack = 2500.0;
    const double wHomotopySlack  = 900.0;
    const double wStaticSlack    = 2500.0;
    auto addSlackNonnegative = [&](int slackIdx) {
        Atr.emplace_back(row, slackIdx, 1.0);
        addBound(0.0, INF);
        ++row;
    };
    auto addSoftUpper = [&](const std::vector<std::pair<int, double>>& terms,
                            double upper, double slackWeight) {
        const int sIdx = addSlackVar(slackWeight);
        for (const auto& t : terms) Atr.emplace_back(row, t.first, t.second);
        Atr.emplace_back(row, sIdx, -1.0);              // a*x - s <= upper
        addBound(-INF, upper);
        ++row;
        addSlackNonnegative(sIdx);
    };
    auto addSoftUpperWithSlack = [&](const std::vector<std::pair<int, double>>& terms,
                                     double upper, int slackIdx) {
        for (const auto& t : terms) Atr.emplace_back(row, t.first, t.second);
        Atr.emplace_back(row, slackIdx, -1.0);           // a*x - s <= upper
        addBound(-INF, upper);
        ++row;
    };
    auto addSoftLower = [&](const std::vector<std::pair<int, double>>& terms,
                            double lower, double slackWeight) {
        const int sIdx = addSlackVar(slackWeight);
        for (const auto& t : terms) Atr.emplace_back(row, t.first, t.second);
        Atr.emplace_back(row, sIdx, 1.0);               // a*x + s >= lower
        addBound(lower, INF);
        ++row;
        addSlackNonnegative(sIdx);
    };

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

    const int firstAvoidK = std::min(N, std::max(1, (int)std::ceil(0.15 / std::max(1e-3, dt_))));

    if (!branch.overTake) {
        // Paper Eq. (9d): original collision constraints, linearized as tangent
        // half-planes around the guidance/nominal point. These are separate from
        // the topology constraint below and are soft only to avoid solver-level
        // infeasibility; any colliding solution is still rejected by post-check.
        for (int k = firstAvoidK; k <= N; ++k) {
            Eigen::Vector2d anchor;
            if (k < (int)guidancePath.size()) {
                anchor = guidancePath[k].head<2>();
            } else if (k < (int)localRef_.size()) {
                anchor = localRef_[k].head<2>();
            } else {
                anchor = currPos_.head<2>();
            }
            for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                if (k >= (int)obsPredPos_[j].size()) continue;
                const Eigen::Vector2d op = obsPredPos_[j][k].head<2>();
                const double required = rUav_ + obsRadius_[j] + safetyMargin_;
                Eigen::Vector2d n = anchor - op;
                if (n.norm() < 1e-5) {
                    n = currPos_.head<2>() - op;
                }
                if (n.norm() < 1e-5) {
                    Eigen::Vector2d tang = localTangent(guidancePath, std::min(k, (int)guidancePath.size() - 1));
                    if (tang.norm() < 1e-6) tang = Eigen::Vector2d(1, 0);
                    Eigen::Vector2d nrm(-tang.y(), tang.x());
                    n = (nrm.dot(currPos_.head<2>() - op) >= 0.0) ? nrm : -nrm;
                }
                n.normalize();
                addSoftLower({{xi(k) + 0, n.x()}, {xi(k) + 1, n.y()}},
                             n.dot(op) + required, wCollisionSlack);
            }
        }

        // Paper Eq. (9e): homotopy-preserving constraints derived from the guidance
        // trajectory. Only guided topology branches receive these constraints; the
        // T-MPC++ free branch keeps the original local planner behavior.
        if (branch.guided) {
            for (int k = firstAvoidK; k <= N && k < (int)guidancePath.size(); ++k) {
                const Eigen::Vector2d gp = guidancePath[k].head<2>();
                for (size_t j = 0; j < obsPredPos_.size(); ++j) {
                    if (k >= (int)obsPredPos_[j].size()) continue;
                    Eigen::Vector2d A_k; double b_k;
                    const double rSum = rUav_ + obsRadius_[j];
                    const Eigen::Vector2d op = obsPredPos_[j][k].head<2>();
                    if (!homotopyHalfPlane(gp, op, rSum, A_k, b_k)) {
                        // Obstacle sits exactly on the guidance point; derive the side
                        // from the local tangent and current relative position.
                        Eigen::Vector2d tang(1, 0);
                        if (k + 1 < (int)guidancePath.size())
                            tang = guidancePath[k + 1].head<2>() - gp;
                        if (tang.norm() < 1e-6) tang = Eigen::Vector2d(1, 0);
                        tang.normalize();
                        Eigen::Vector2d nrm(-tang.y(), tang.x());
                        double sign = (nrm.dot(currPos_.head<2>() - op) >= 0.0) ? 1.0 : -1.0;
                        A_k = -sign * nrm;
                        b_k = A_k.dot(op) - (rSum + safetyMargin_);
                    }
                    addSoftUpper({{xi(k) + 0, A_k.x()}, {xi(k) + 1, A_k.y()}},
                                 b_k, wHomotopySlack);
                }
            }
        }
    } else {
        // 3-D "fly-over" branch: where the reference passes horizontally near an
        // obstacle, force z_k >= obstacle_top + clearance. The nearness test uses the
        // (fixed) reference trajectory, so the resulting constraint stays linear in z_k.
        for (int k = firstAvoidK; k <= N && k < (int)branch.guidanceTraj.size(); ++k) {
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

        for (int k = firstAvoidK; k <= N && k < (int)guidancePath.size(); ++k) {
            Eigen::Vector3d gp3 = guidancePath[k];
            gp3.z() = zLap_;
            int staticSlackIdx = -1;
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
                if (staticSlackIdx < 0) {
                    staticSlackIdx = addSlackVar(wStaticSlack);
                    addSlackNonnegative(staticSlackIdx);
                }
                addSoftUpperWithSlack({{xi(k)+0, A.x()}, {xi(k)+1, A.y()}}, b, staticSlackIdx);
            }
        }
    }

    Eigen::SparseMatrix<double> P(nVar, nVar);
    P.setFromTriplets(Ptr.begin(), Ptr.end());
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nVar);
    for (int i = 0; i < std::min(nVar, (int)qVals.size()); ++i) q(i) = qVals[i];

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
    if (!solver.data()->setHessianMatrix(P))            { branch.feasible = false; branch.status = "setup_hessian"; return; }
    if (!solver.data()->setGradient(q))                 { branch.feasible = false; branch.status = "setup_gradient"; return; }
    if (!solver.data()->setLinearConstraintsMatrix(Ac)) { branch.feasible = false; branch.status = "setup_constraints"; return; }
    if (!solver.data()->setLowerBound(l))               { branch.feasible = false; branch.status = "setup_lower"; return; }
    if (!solver.data()->setUpperBound(u))               { branch.feasible = false; branch.status = "setup_upper"; return; }
    if (!solver.initSolver())                           { branch.feasible = false; branch.status = "init_solver"; return; }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.status = "solve_error";
        return;
    }
    const OsqpEigen::Status osqpStatus = solver.getStatus();
    if (osqpStatus != OsqpEigen::Status::Solved &&
        osqpStatus != OsqpEigen::Status::SolvedInaccurate) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        if (osqpStatus == OsqpEigen::Status::PrimalInfeasible ||
            osqpStatus == OsqpEigen::Status::PrimalInfeasibleInaccurate) {
            branch.status = "solve_primal_infeasible";
        } else if (osqpStatus == OsqpEigen::Status::DualInfeasible ||
                   osqpStatus == OsqpEigen::Status::DualInfeasibleInaccurate) {
            branch.status = "solve_dual_infeasible";
        } else if (osqpStatus == OsqpEigen::Status::MaxIterReached) {
            branch.status = "solve_max_iter";
        } else {
            branch.status = "solve_not_solved";
        }
        return;
    }
    const Eigen::VectorXd sol = solver.getSolution();

    // Reject garbage solutions (non-finite or absurdly large). Without this a failed
    // / diverged solve could be streamed to the controller as a setpoint and fly the
    // drone off into space. World is ~+-15 m, so 1e4 only catches true garbage.
    if (!sol.allFinite() || sol.cwiseAbs().maxCoeff() > 1e4) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.status = "numeric";
        return;
    }

    // optimal cost J = 0.5 z'Pz + q'z + reference constants. The constants matter
    // now because guided branches track different topology paths.
    branch.cost = 0.5 * sol.dot(P * sol) + q.dot(sol) + objectiveConstant;
    branch.feasible = true;

    // extract full 3-D states [x,y,z,vx,vy,vz]
    branch.statesSol.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        Eigen::VectorXd s(6);
        s << sol(xi(k)+0), sol(xi(k)+1), sol(xi(k)+2),
             sol(xi(k)+3), sol(xi(k)+4), sol(xi(k)+5);
        branch.statesSol[k] = s;
    }
    branch.controlsSol.resize(N);
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd u(3);
        u << sol(ui(k)+0), sol(ui(k)+1), sol(ui(k)+2);
        branch.controlsSol[k] = u;
    }

    if (trajectoryHitsStaticMap(branch.statesSol)) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.status = "static_collision";
        branch.statesSol.clear();
        branch.controlsSol.clear();
    } else if (trajectoryHitsDynamicObstacles(branch.statesSol, branch.overTake)) {
        branch.feasible = false;
        branch.cost = std::numeric_limits<double>::infinity();
        branch.status = "dynamic_collision";
        branch.statesSol.clear();
        branch.controlsSol.clear();
    } else {
        branch.status = "feasible";
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
    }
}

bool tmpcPlanner::plan() {
    auto t0 = std::chrono::steady_clock::now();
    resetDiagnostics();

    buildConstantVelocityPredictions();
    buildGoalGrid();
    buildStaticAwareReference();
    buildGoalGridFromLocalRef();
    if ((int)localRef_.size() < horizon_ + 1) {   // no valid reference path was set
        ROS_WARN_THROTTLE(2.0, "[tmpcPlanner] empty/short local reference "
                               "(reference path not set?); skipping plan - drone holds.");
        planTimeMs_ = 0.0;
        bestIdx_ = -1;
        lastPlanStatus_ = "short_ref";
        return false;
    }
    if (!runGuidance()) {
        planTimeMs_ = 0.0;
        lastPlanStatus_ = "guidance_failed";
        return false;
    }

    // Solve each branch's local MPC. Each OsqpEigen solver is constructed locally
    // inside solveBranch (independent state), so the branches can be solved in
    // parallel; this mirrors the paper's P+1 parallel local planners. Set
    // solve_sequential:true for deterministic single-thread timing.
#ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) num_threads(parallelThreads_) if(!solveSequential_)
#endif
    for (int i = 0; i < (int)branches_.size(); ++i) solveBranch(branches_[i]);

    updateDiagnosticsAfterSolve();
    decide();

    auto t1 = std::chrono::steady_clock::now();
    planTimeMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    lastPlanStatus_ = (bestIdx_ >= 0) ? "success" : "no_feasible_branch";
    ROS_INFO_THROTTLE(1.0,
        "[tmpcPlanner] status=%s obs=%zu branches=%d guided=%d feasible=%d best=%d class=%d "
        "reject(static=%d dynamic=%d solve=%d setup=%d numeric=%d) "
        "topo(nodes=%d goals=%d expansions=%d) static(blocked=%d astar=%d astar_fail=%d) plan=%.1fms",
        lastPlanStatus_.c_str(), obsPredPos_.size(), lastTotalBranches_, lastGuidedBranches_,
        lastFeasibleBranches_, bestIdx_, bestClassId_,
        lastStaticRejects_, lastDynamicRejects_, lastSolveRejects_, lastSetupRejects_, lastNumericRejects_,
        lastGuidanceNodes_, lastGuidanceGoals_, lastGuidanceExpansions_,
        (int)lastStaticDirectBlocked_, (int)lastStaticAstarActive_, (int)lastStaticAstarFailed_,
        planTimeMs_);
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

bool tmpcPlanner::getBestControls(std::vector<Eigen::VectorXd>& controls) const {
    controls.clear();
    if (bestIdx_ < 0) return false;
    controls = branches_[bestIdx_].controlsSol;
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
    double prevMargin = 0.0;
    const double step = std::max(0.05, map_->getRes());
    for (size_t k = 0; k < states.size(); ++k) {
        const auto& s = states[k];
        if (s.size() < 3 || !s.allFinite()) return true;
        Eigen::Vector3d p(s(0), s(1), s(2));
        // Match the guidance checker: only the already-executed start sample can use
        // zero extra clearance; every future sample must keep the configured static
        // margin. The previous 0.25 s ramp let the first few commanded states skim
        // static obstacles before the post-check became strict.
        const double margin = (k == 0) ? 0.0 : staticPostCheckClearance_;
        if (pointHitsStaticMapWithMargin(p, margin)) return true;
        if (havePrev && (p - prev).norm() > 1e-4) {
            const int samples = std::max(1, (int)std::ceil((p - prev).norm() / step));
            for (int i = 1; i < samples; ++i) {
                const double u = (double)i / (double)samples;
                const double sampleMargin = std::max(prevMargin, margin);
                if (pointHitsStaticMapWithMargin(prev + u * (p - prev), sampleMargin)) return true;
            }
        }
        prev = p;
        prevMargin = margin;
        havePrev = true;
    }
    return false;
}

bool tmpcPlanner::pointHitsStaticMapWithMargin(const Eigen::Vector3d& p, double margin) const {
    if (!map_) return false;
    // Static obstacles are only considered within the sensor FOV range of the drone
    // (consistent with dynamic obstacles, which come from getObstaclesInSensorRange).
    // Beyond it the prebuilt global map is ignored (treated as free); the drone re-plans
    // as it approaches. staticFovRange_ mirrors the map raycast range.
    if ((p.head<2>() - currPos_.head<2>()).norm() > staticFovRange_) return false;
    if (map_->isInflatedOccupied(p)) return true;
    if (margin <= 1e-6) return false;

    const double step = std::max(0.05, map_->getRes());
    const int radialSteps = std::max(1, (int)std::ceil(margin / step));
    const int dirs = 8;
    for (int r = 1; r <= radialSteps; ++r) {
        const double radius = std::min(margin, step * (double)r);
        Eigen::Vector3d qz = p;
        qz.z() += radius;
        if (map_->isInflatedOccupied(qz)) return true;
        qz = p;
        qz.z() -= radius;
        if (map_->isInflatedOccupied(qz)) return true;
        for (int d = 0; d < dirs; ++d) {
            const double th = 2.0 * M_PI * (double)d / (double)dirs;
            Eigen::Vector3d q = p;
            q.x() += radius * std::cos(th);
            q.y() += radius * std::sin(th);
            if (map_->isInflatedOccupied(q)) return true;
        }
    }
    return false;
}

bool tmpcPlanner::segmentHitsStaticMapWithMargin(const Eigen::Vector3d& a,
                                                 const Eigen::Vector3d& b,
                                                 double margin) const {
    if (!map_) return false;
    if (pointHitsStaticMapWithMargin(a, margin) ||
        pointHitsStaticMapWithMargin(b, margin)) {
        return true;
    }
    if ((b - a).squaredNorm() <= 1e-10) return false;

    const double step = std::max(0.05, map_->getRes());
    const int samples = std::max(1, (int)std::ceil((b - a).norm() / step));
    for (int i = 1; i < samples; ++i) {
        const double u = (double)i / (double)samples;
        if (pointHitsStaticMapWithMargin(a + u * (b - a), margin)) return true;
    }
    return false;
}

bool tmpcPlanner::trajectoryHitsDynamicObstacles(const std::vector<Eigen::VectorXd>& states,
                                                 bool allowVerticalOvertake) const {
    if (obsPredPos_.empty()) return false;
    for (size_t k = 1; k < states.size(); ++k) {
        const auto& s = states[k];
        if (s.size() < 3 || !s.allFinite()) return true;
        const Eigen::Vector2d p(s(0), s(1));
        for (size_t j = 0; j < obsPredPos_.size(); ++j) {
            if (k >= obsPredPos_[j].size()) continue;
            const double required = rUav_ + obsRadius_[j] + safetyMargin_;
            const double dxy = (p - obsPredPos_[j][k].head<2>()).norm();
            if (dxy >= required) continue;
            if (allowVerticalOvertake && j < obsTop_.size() &&
                s(2) >= obsTop_[j] + vClearance_) {
                continue;
            }
            return true;
        }
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
    if (!publishGuidanceMarkers_) return;
    if (guidancePathsPub_.getNumSubscribers() == 0) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "tmpc_guidance";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int mid = 0;
    for (size_t i = 0; i < branches_.size(); ++i) {
        if (!branches_[i].guided || branches_[i].overTake) continue;
        auto m = lineMarker(mid++, 0.7, 0.3, 0.85, 0.05, "tmpc_guidance");
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
    if (!publishOptimizedMarkers_) return;
    if (optimizedTrajPub_.getNumSubscribers() == 0) return;
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
    if (!publishObstaclePredictionMarkers_) return;
    if (dynObsPub_.getNumSubscribers() == 0) return;
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

void tmpcPlanner::publishVisibleStaticObstacles() const {
    if (!publishVisibleStaticMarkers_) return;
    if (!map_ || visibleStaticPub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "tmpc_visible_static";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    Eigen::Vector3d mapMin, mapMax;
    Eigen::Vector3d currMin, currMax;
    map_->getMapRange(mapMin, mapMax);
    map_->getCurrMapRange(currMin, currMax);

    const double range = std::max(1.0, staticFovRange_);
    Eigen::Vector3d lo(currPos_.x() - range, currPos_.y() - range, mapMin.z());
    Eigen::Vector3d hi(currPos_.x() + range, currPos_.y() + range, mapMax.z());
    lo = lo.cwiseMax(mapMin);
    hi = hi.cwiseMin(mapMax);

    // If the map exposes a current sensor-update range, intersect with it so the
    // marker shows what the planner currently sees instead of the whole global map.
    if ((currMax - currMin).norm() > 1e-3) {
        lo = lo.cwiseMax(currMin);
        hi = hi.cwiseMin(currMax);
    }

    const double res = std::max(0.03, map_->getRes());
    const int stride = std::max(1, visibleStaticMarkerStride_);
    const double step = res * (double)stride;

    visualization_msgs::Marker vox;
    vox.header.frame_id = "map";
    vox.header.stamp = ros::Time::now();
    vox.ns = "tmpc_visible_static";
    vox.id = 0;
    vox.type = visualization_msgs::Marker::CUBE_LIST;
    vox.action = visualization_msgs::Marker::ADD;
    vox.pose.orientation.w = 1.0;
    vox.scale.x = vox.scale.y = vox.scale.z = step;
    vox.color.r = 0.25;
    vox.color.g = 0.55;
    vox.color.b = 0.95;
    vox.color.a = 0.45;
    vox.lifetime = ros::Duration(0.6);
    vox.points.reserve((size_t)std::min(visibleStaticMarkerMaxPoints_, 6000));

    bool full = false;
    for (double x = lo.x(); x <= hi.x() + 1e-9 && !full; x += step) {
        for (double y = lo.y(); y <= hi.y() + 1e-9 && !full; y += step) {
            if ((Eigen::Vector2d(x, y) - currPos_.head<2>()).norm() > range) continue;
            for (double z = lo.z(); z <= hi.z() + 1e-9; z += step) {
                Eigen::Vector3d p(x, y, z);
                if (!map_->isInMap(p)) continue;
                if (!map_->isInflatedOccupied(p)) continue;
                geometry_msgs::Point pt;
                pt.x = x; pt.y = y; pt.z = z;
                vox.points.push_back(pt);
                if ((int)vox.points.size() >= visibleStaticMarkerMaxPoints_) {
                    full = true;
                    break;
                }
            }
        }
    }

    arr.markers.push_back(vox);
    visibleStaticPub_.publish(arr);
}

bool tmpcPlanner::hasVisualizationSubscribers() const {
    return (publishGuidanceMarkers_ && guidancePathsPub_.getNumSubscribers() > 0) ||
           (publishOptimizedMarkers_ && optimizedTrajPub_.getNumSubscribers() > 0) ||
           (publishObstaclePredictionMarkers_ && dynObsPub_.getNumSubscribers() > 0) ||
           (publishVisibleStaticMarkers_ && visibleStaticPub_.getNumSubscribers() > 0);
}

} // namespace trajPlanner
