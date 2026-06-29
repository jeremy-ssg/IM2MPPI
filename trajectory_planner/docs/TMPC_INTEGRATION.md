# T-MPC++ Integration Plan & Progress Tracker

> **Goal:** add a new baseline method **T-MPC++** (Topology-driven Model Predictive
> Control, de Groot et al., *IEEE T-RO* vol.41 2025) to the IM2-MPPI benchmark as
> method id **`M6_tmpc`** (display name **"T-MPC++"**, plot color `#b452d8`).
>
> **This file is the single source of truth.** If a working session runs out of
> budget/context, the next session resumes from the **"NEXT STEP"** box below and
> the checkboxes in §6. Keep this file updated at the end of every working turn.

---

## 0. NEXT STEP  ← always keep this current

```
STATUS: ALL new code written + wired into CMake. NOT compiled (dev box is Windows).
        Fork resolved: local planner is a self-contained OsqpEigen linear MPC (docs §8 opt.1
        variant) — no dependency on the ACADO path. guidance_planner is OPTIONAL via
        find_package(QUIET) + __has_include guard (fallback branch if absent).
DO THIS NEXT (on the Linux ROS machine):
   1. git clone https://github.com/tud-amr/guidance_planner into the workspace src/ and
      build it (catkin). Then add its <depend> to the two package.xml (see §5) and
      uncomment the guidance config rosparam in tmpc_demo.launch.
   2. catkin build trajectory_planner autonomous_flight  → FIX COMPILE ERRORS (code was
      written blind; expect a few signature mismatches vs the real guidance_planner API
      — check Obstacle/Goal/GlobalGuidance against the cloned headers).
   3. roslaunch autonomous_flight tmpc_demo.launch  → sanity-check /tmpc/best_trajectory.
   4. Run the 5-method benchmark; then do D3 (add M6_tmpc to plot METHODS) and regenerate.
ROLLBACK (if you need the old build while debugging): comment out the 3 tmpc lines in
   autonomous_flight/CMakeLists.txt (lib src + add_executable + target_link) and the
   tmpcPlanner.cpp line in trajectory_planner/CMakeLists.txt add_library.
SAFE STATE:    Existing 4-method code is UNCHANGED. New code is wired but unverified;
               if it fails to compile, use ROLLBACK above to restore the green build.
```

---

## 1. Decisions locked with the user (2026-06-29)

1. **Variant:** implement **T-MPC++** (= T-MPC + one extra *unguided* parallel MPC).
   Keep `add_unguided_planner: true/false` in yaml so the plain-T-MPC ablation is one flag away.
2. **Source strategy:** **vendor the official `guidance_planner` package** (Apache-2.0)
   for the hard part (Visibility-PRM, H-signature, homotopy propagation). Do **NOT**
   vendor `mpc_planner` (it depends on Forces Pro + a Python solver generator). The
   local MPC is our existing ACADO `trajPlanner::mpcPlanner`.
3. **Obstacle prediction:** **constant-velocity** (matches the paper). Take each
   detected obstacle's current pose+velocity from the detector and propagate linearly
   over the horizon. (Later we can add an `intent_mean` source for a second table.)
4. **Paper name:** report it literally as **"T-MPC++ (de Groot et al., T-RO 2025)"**.

### UAV-vs-UGV adaptation (the part the user flagged)
The paper is a **ground robot in 2D**. Our system is a **UAV in 3D**. Handling:

| Concern | Resolution |
|---|---|
| Topology dimensionality | Run guidance in the **horizontal plane** `(x,y,t)` at the lap altitude `z_lap`. Homotopy classes only make sense laterally for our lap task, so this is correct, not a hack. |
| Local planner dimensionality | Self-contained FULL 3-D double-integrator MPC: state [x,y,z,vx,vy,vz], control [ax,ay,az], with separate vertical limits (vz_max, az_max). Altitude reference auto-derived as the mean z of the reference path (`setReference()`). Horizontal homotopy half-planes (Eq.8) act on (x,y); a 3-D "fly-over" branch (`overTake`, classId -2) adds a convex vertical-clearance floor `z_k >= obs_top + vertical_clearance` where the path passes near an obstacle, so the planner can also avoid by climbing over. Toggle via `enable_vertical_avoidance`. |
| Robot/obstacle geometry | Disc model: `r = r_uav + r_obs`. `r_uav` from drone radius (~0.3 m), `r_obs` from detector bbox. |
| Kinematic limits | Use UAV limits from `im2_mppi.yaml`: `v_max=2.0`, `a_max=3.0` for both guidance feasibility checks and the local MPC. |
| Reference / goals | Lap is a circle (radius ~7 m), ref = `autonomous_flight/cfg/mpc_navigation/ref_trajectory.txt`. Place the guidance **goal grid** along the ref path in Frenet coords (lateral spread + look-ahead). |

---

## 2. What we reuse vs. write new

**Reuse (no new code):**
- `mapManager::dynamicMap` / `occMap`  — static occupancy for PRM visibility + MPC static avoidance
- `onboardDetector::fakeDetector`       — dynamic obstacle bbox + velocity
- `dynamicPredictor::predictor`         — present but for T-MPC we bypass it and build constant-velocity predictions (decision #3)
- `trajPlanner::mpcPlanner` (ACADO)     — the local MPC; instantiate **P+1** of them
- `AutoFlight::flightBase`              — takeoff, target stream, control interface
- `evaluate_intent_mpc_im2mppi.py`      — metrics auto-work once we publish a `nav_msgs/Path`
- `compare_planner_eval.py`, `print_summary_table.py` — no change (key off `algorithm` field)

**Vendor (drop-in ROS package):**
- `guidance_planner/`  (clone `https://github.com/tud-amr/guidance_planner`, Apache-2.0, ROS1 Noetic). Public API: `#include <guidance_planner/global_guidance.h>`, example `src/ros1_example.cpp`.

**Write new:**
- `trajectory_planner/include/trajectory_planner/tmpc/tmpcPlanner.{h,cpp}` — orchestrate guidance → P parallel ACADO MPC → decision
- `autonomous_flight/include/autonomous_flight/tmpcNavigation.{h,cpp}` — mirror `im2MppiNavigation`
- `autonomous_flight/src/tmpc_navigation_node.cpp`
- `autonomous_flight/launch/tmpc_demo.launch`
- `trajectory_planner/cfg/tmpc.yaml`

---

## 3. ROS topics (T-MPC++ outputs, under node ns `/tmpc`)
```
/tmpc/best_trajectory          nav_msgs/Path          executed winner   (← evaluator subscribes here)
/tmpc/guidance_paths           MarkerArray            all P guidance trajectories (rviz)
/tmpc/optimized_trajectories   MarkerArray            all P+1 optimized trajectories (rviz)
/tmpc/reference_path           nav_msgs/Path          local horizon reference (yellow)
/tmpc/dynamic_obstacle_predictions  MarkerArray       const-vel obstacle tubes
/tmpc/goal                     MarkerArray            goal grid / global goal
/tmpc/plan_time_ms             std_msgs/Float64       plan() duration
```

---

## 4. Component specs

### A. `tmpc.yaml`  — DONE (see trajectory_planner/cfg/tmpc.yaml)
Namespace `tmpc:`. Mirrors im2_mppi.yaml conventions. Key knobs:
`num_trajectories_P`, `add_unguided_planner`, `consistency_ci`, `beta_relax`,
`homotopy_method`, goal-grid params, `v_max`/`a_max`, `prediction_source: constant_velocity`.

### B. `tmpc_demo.launch` — DONE (see autonomous_flight/launch/tmpc_demo.launch)
Mirror of `im2_mppi_demo.launch`: loads controller/detector/flight_base/mapping/
predictor params + `tmpc.yaml` + (vendored) `guidance_planner` config, starts
`tracking_controller_node` + `tmpc_navigation_node` + rviz.

### C. `tmpcPlanner.{h,cpp}`  — TODO (NEXT)
Class `trajPlanner::tmpcPlanner`. Responsibilities:
1. `setObstacles(...)`: take detector obstacles → build **constant-velocity** predicted
   positions per horizon step; convert to `guidance_planner` obstacle format.
2. `setReference(refPath, currState)`: build the **goal grid** along ref in Frenet frame.
3. `runGuidance()`: call `GlobalGuidance::Update()`; pull P topology-distinct guidance
   trajectories (2D `(x,y,t)`), lift to 3D with `z=z_lap`.
4. `optimizeParallel()`: for each guidance traj i, configure a dedicated ACADO
   `mpcPlanner` instance:
      - warm-start `x` with guidance traj,
      - add homotopy half-plane constraints Eq.(8) per (k,obstacle) with `beta_relax`,
        acting on (x,y) only,
      - solve. Record optimal cost `J_i`.
   Plus (T-MPC++) one **unguided** instance (no homotopy constraints, previous-solution warm start).
   Run with `std::thread` / OpenMP; **per-thread ACADO workspace** (ACADO generated code
   is NOT thread-safe — each parallel MPC needs its own solver state. See §5 risk.)
5. `decide()`: pick `i* = argmin_i w_i J_i*` with `w_i = consistency_ci` if traj i was the
   previously executed homotopy class, else 1 (Eq.12). Infeasible ⇒ `J=inf`.
6. Expose winner trajectory + all candidates for viz.

**Homotopy constraint (Eq.8), (x,y) only, per step k / obstacle j:**
```
n   = (o_k - tau_{i,k}) / ||o_k - tau_{i,k}||      // unit vector, guidance->obstacle
A_k = n
b_k = n . (o_k - n * beta*(r_uav + r_obs))
constraint:  A_k . p_k <= b_k     // keep ego on guidance side of obstacle
```

### D. `tmpcNavigation.{h,cpp}` — TODO
Copy `im2MppiNavigation.{h,cpp}` structure. Swap `IM2MPPIPlanner` → `tmpcPlanner`.
Same timers (plan ~10 Hz, trajExe 100 Hz, vis ~5 Hz), same flightBase usage, same
reference-path slicing. Publish topics in §3.

### E. `tmpc_navigation_node.cpp` — TODO
Trivial: `ros::init` → `AutoFlight::tmpcNavigation nav(nh); nav.run();` + AsyncSpinner(3).

---

## 5. Wiring diffs to apply LAST (keeps build green until then)

### `trajectory_planner/CMakeLists.txt`
- `find_package(catkin ... COMPONENTS ... guidance_planner)`
- add `tmpc_planner` to the built library / link `${catkin_LIBRARIES}` (OpenMP: `find_package(OpenMP)` + link `OpenMP::OpenMP_CXX`).
### `trajectory_planner/package.xml`
- `<build_depend>guidance_planner</build_depend>` + `<exec_depend>guidance_planner</exec_depend>`
### `autonomous_flight/CMakeLists.txt`
- add `include/${PROJECT_NAME}/tmpcNavigation.cpp` to `add_library(...)`
- `add_executable(tmpc_navigation_node src/tmpc_navigation_node.cpp)`
- `target_link_libraries(tmpc_navigation_node ${catkin_LIBRARIES} ${PROJECT_NAME})`
### `autonomous_flight/package.xml`
- add `guidance_planner` build/exec depends (transitively via trajectory_planner, but be explicit)

### Vendoring guidance_planner
```
cd <catkin_ws>/src/IM2MPPI      # or wherever sibling packages live
git clone https://github.com/tud-amr/guidance_planner
# install its deps (it has a setup script / package.xml deps); build with catkin.
```

## RISKS / GOTCHAS
- **ACADO thread-safety:** generated ACADO solver uses global workspace → P parallel
  solves will corrupt each other unless each runs in its own process/workspace.
  Mitigations (pick one when implementing): (a) solve guidance branches **sequentially**
  first (correct but slower; fine for offline benchmark), (b) compile P copies of the
  ACADO workspace into separate translation units, (c) guard with a mutex (serializes —
  defeats parallelism). For the benchmark, **start with sequential (a)** to get correct
  numbers, optimize later. Record plan_time honestly.
- **guidance_planner deps:** it pulls extra deps (e.g. its own `ros_tools`). Verify they
  build on the user's Noetic machine. Document any missing apt/rosdep packages here when found.
- **Cannot compile on this Windows box.** All C++ is written blind; the user builds on
  Linux. Keep code close to existing patterns (im2MppiNavigation, mpcPlanner) to minimize
  compile errors. Prefer adapting tested upstream code over inventing.

---

## 6. Milestone checklist
- [x] A1 master plan/progress doc (this file)
- [x] A2 memory pointer written
- [x] B1 `trajectory_planner/cfg/tmpc.yaml`
- [x] B6 `autonomous_flight/launch/tmpc_demo.launch` (+ commented guidance config rosparam)
- [x] B7 `autonomous_flight/cfg/tmpc_navigation.rviz` (dedicated RViz config; /tmpc/* displays,
        does NOT touch im2_mppi_navigation.rviz). Optimized-branch markers color-coded
        cyan=fly-over / gray=free / warm=left-right, chosen branch labeled OVER/FREE/L/R.
- [x] B3 `tmpc/tmpcPlanner.{h,cpp}`  (self-contained OSQP local MPC + guidance integration)
- [x] B4 `tmpcNavigation.{h,cpp}`
- [x] B5 `tmpc_navigation_node.cpp`
- [x] C1 `trajectory_planner/CMakeLists.txt` (tmpcPlanner.cpp + optional guidance_planner)
- [x] C2 `autonomous_flight/CMakeLists.txt` (tmpcNavigation.cpp + tmpc_navigation_node)
- [x] D1 `run_four_methods_full_lap_bag.sh` → M6_tmpc row + `/tmpc/*` bag topics
- [x] D2 `evaluate_intent_mpc_im2mppi.py` `default_path_topic`: `tmpc → /tmpc/best_trajectory`
- [ ] C3 vendor guidance_planner (clone+build) + add `<depend>` to both package.xml
        + uncomment guidance config in tmpc_demo.launch  ← do on Linux
- [ ] BUILD: catkin build on Linux; fix blind-write compile errors (esp. guidance API)
- [ ] D3 add `("M6_tmpc","T-MPC++","#b452d8")` to METHODS in plot scripts
        (plot_seed12_with_obstacles.py + make_combined.py STYLES `tmpc` color + cfg_to_key).
        DEFERRED on purpose: adding it before M6 CSVs exist would crash existing plotting.
- [ ] E  run 7-seed full-lap bag (5 methods) + regenerate comparison figures

### Deliberately NOT done yet (to keep current pipeline green)
- package.xml `<depend>guidance_planner</depend>` — would break `catkin build` until the
  package is cloned. Add it at vendoring time (C3).
- plot METHODS entries — would crash make_combined.py (reads non-existent M6 CSVs) (D3).

## 8. LOCAL-PLANNER CONSTRAINT FORK  ← architectural decision needed

The paper enforces the homotopy class in each local MPC by adding linear half-plane
constraints (Eq.8) at runtime, solved with Forces Pro. Our workspace has two MPC paths
inside `mpcPlanner.h`:

- **ACADO path** (`solveTraj()`, used by M0 Intent-MPC): constraint structure is fixed at
  ACADO code-generation time. **Cannot accept arbitrary runtime half-plane constraints.**
- **OSQP path** (`castMPCToQPConstraintMatrix()` / `castMPCToQPConstraintVectors()` /
  `updateObstacleParam()`, OsqpEigen vendored under third_party/): assembles the
  constraint matrix at runtime from obstacle params. **Can accept extra half-plane rows.**

**Three implementable options (pick one):**
1. **OSQP path (recommended).** Build the per-branch local MPC on the OSQP assembly:
   reuse `castMPCToQP*`, append P-specific homotopy rows `A_k·p_k ≤ b_k` (Eq.8, xy only).
   Most faithful to the paper, uses already-vendored OsqpEigen, no codegen. Cost: must
   verify/extend the OSQP assembly (it may be partially stubbed — `setDynamicsMatrices`
   etc. are marked TODO in the header).
2. **ACADO + soft guidance penalty (approximation).** Run P ACADO `solveTraj()` warm-
   started from each guidance trajectory, add a light contour penalty pulling each branch
   toward its guidance traj; pick min-cost. Easiest (reuses M0 path verbatim) but the
   paper's Fig.6 shows init-only branches can collapse to the same class — so this is a
   *T-MPC-flavored* baseline, not exactly T-MPC++. Acceptable only if labeled honestly.
3. **Regenerate ACADO with homotopy constraints.** Heavy (touch solver_generator /
   mpc_solver_setup.cpp). Not worth it for a benchmark baseline.

Until resolved, `tmpcPlanner.h` is written solver-agnostic: it owns the guidance + decision
logic and calls a `solveBranch(...)` hook whose body is filled per the chosen option.

## 7. Source references
- Paper text (extracted): `scratchpad/tdpto.txt` (session-local; re-extract with
  `pdftotext -layout` from the Desktop PDF if gone).
- Guidance planner: https://github.com/tud-amr/guidance_planner  (Apache-2.0)
- MPC planner (NOT vendored, reference only): https://github.com/tud-amr/mpc_planner
- Existing method registry: `plot_seed12_with_obstacles.py` lines 15-20 `METHODS`.
- Existing method driver: `trajectory_planner/scripts/run_four_methods_full_lap_bag.sh` `CONFIGS` array.
