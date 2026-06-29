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
#include <trajectory_planner/third_party/OsqpEigen/OsqpEigen.h>

#include <chrono>
#include <cmath>
#include <limits>

namespace trajPlanner {

// State/control dimensions of the local MPC (3-D double integrator).
static constexpr int NS = 6;   // [x, y, z, vx, vy, vz]
static constexpr int NU = 3;   // [ax, ay, az]

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
    nh_.param("tmpc/enable_vertical_avoidance", vertical_,  true);
    nh_.param("tmpc/vertical_clearance",  vClearance_,      0.4);
    nh_.param<std::string>("tmpc/prediction_source", predictionSource_, "constant_velocity");
    nh_.param("tmpc/max_obstacles",       maxObstacles_,    12);

    // cost weights
    nh_.param("tmpc/cost_weights/w_contour", wContour_, 1.0);
    nh_.param("tmpc/cost_weights/w_lag",     wLag_,     1.0);
    nh_.param("tmpc/cost_weights/w_vel",     wVel_,     0.1);
    nh_.param("tmpc/cost_weights/w_acc",     wAcc_,     0.05);

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

    // arc-length parametrization forward from nearest
    auto refDir = [&](int i) -> Eigen::Vector2d {
        int a = std::min<int>(i, (int)refPath_.size() - 2);
        Eigen::Vector2d t = (refPath_[a + 1].head<2>() - refPath_[a].head<2>());
        if (t.norm() < 1e-6) return Eigen::Vector2d(1, 0);
        return t.normalized();
    };

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

    // goal grid centered on the look-ahead point
    int lookIdx = nearest;
    {
        double remaining = goalLongDist_;
        while (lookIdx < (int)refPath_.size() - 1 && remaining > 0.0) {
            double segLen = (refPath_[lookIdx + 1].head<2>() - refPath_[lookIdx].head<2>()).norm();
            remaining -= segLen; ++lookIdx;
        }
    }
    Eigen::Vector2d center = refPath_[std::min(lookIdx, (int)refPath_.size() - 1)].head<2>();
    Eigen::Vector2d tang   = refDir(lookIdx);
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

// ---------------------------------------------------------------------------
// Guidance: call the vendored guidance_planner to get P topology-distinct
// trajectories. Fallback (package absent / failure): a single straight branch to
// the central look-ahead goal so the local MPC still runs.
// ---------------------------------------------------------------------------
bool tmpcPlanner::runGuidance() {
    branches_.clear();

#if TMPC_HAVE_GUIDANCE_PLANNER
    if (!guidance_) guidance_.reset(new GuidancePlanner::GlobalGuidance());

    // start
    guidance_->SetStart(currPos_.head<2>(), currYaw_, currVel_.head<2>().norm());
    guidance_->SetReferenceVelocity(vRef_);

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

    // goals
    std::vector<GuidancePlanner::Goal> goals;
    for (size_t i = 0; i < goalGrid_.size(); ++i) {
        // cost = distance of this goal to the ideal look-ahead point on the ref path
        double cost = (goalGrid_[i].head<2>() - localRef_.back().head<2>()).norm();
        goals.emplace_back(goalGrid_[i].head<2>(), cost);
    }
    guidance_->SetGoals(goals);

    if (!guidance_->Update() || guidance_->NumberOfGuidanceTrajectories() == 0) {
        ROS_WARN_THROTTLE(2.0, "[tmpcPlanner] guidance produced no trajectories; using fallback branch.");
    } else {
        int nTraj = std::min(numTrajP_, guidance_->NumberOfGuidanceTrajectories());
        for (int i = 0; i < nTraj; ++i) {
            auto& out = guidance_->GetGuidanceTrajectory(i);
            auto traj2d = out.spline.GetTrajectory();
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
    }
#endif

    // Unguided branch (T-MPC++) and/or fallback: warm-start from the reference path.
    if (branches_.empty() || addUnguided_) {
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

    buildConstantVelocityPredictions();
    buildGoalGrid();
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
    #pragma omp parallel for schedule(dynamic) if(!solveSequential_)
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
