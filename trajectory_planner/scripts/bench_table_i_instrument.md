# C++ instrumentation for Table I

This document describes the exact code changes needed to instrument
`im2_mppi_planner.{h,cpp}` so that every call to `plan()` emits a
per-stage wall-clock breakdown that `bench_table_i.py` can aggregate
into the LaTeX table.

The instrumentation:
- Uses `std::chrono::steady_clock` for CPU stages
- Uses `cudaEvent_t` for GPU stages (correct for async kernels)
- Writes one JSON line per `plan()` call to the path configured by
  the ROS parameter `im2_mppi/stage_timing_log`
- Falls back to a no-op if the parameter is empty (zero overhead in
  production)

Estimated overhead: ~3 µs per `plan()` tick from the eight chrono
queries and one file write. The cudaEvent pair adds one stream sync
which is already required by the existing memcpy-out, so it is
effectively free.

## 1. Add fields to `im2_mppi_planner.h`

In the `private:` section of `IM2MPPIPlanner` (around line 290, after
the existing CUDA forward declarations), add:

```cpp
    // ── Stage timing instrumentation (bench_table_i) ─────────────────────
    // When stage_timing_log_ is non-empty, every plan() tick writes a
    // single JSON line with per-stage wall-clock times in milliseconds.
    // Empty path = disabled = zero overhead.
    std::string stage_timing_log_;
    void writeStageTimingRecord(const std::map<std::string, double>& times) const;
```

You'll need `#include <map>` and `#include <fstream>` at the top of
the header (or `.cpp` only if you prefer).

## 2. Load the parameter in `loadParams()`

In `im2_mppi_planner.cpp`, find `void IM2MPPIPlanner::loadParams()` and
add at the end:

```cpp
    nh_.param<std::string>("im2_mppi/stage_timing_log", stage_timing_log_, std::string{});
```

## 3. Add the writer helper

Add to the bottom of `im2_mppi_planner.cpp`:

```cpp
void IM2MPPIPlanner::writeStageTimingRecord(
    const std::map<std::string, double>& times) const
{
    if (stage_timing_log_.empty()) return;
    std::ofstream f(stage_timing_log_, std::ios::app);
    if (!f) return;
    f << "{";
    bool first = true;
    for (const auto& [k, v] : times) {
        if (!first) f << ", ";
        first = false;
        f << "\"" << k << "\": " << v;
    }
    f << "}\n";
}
```

## 4. Instrument `planCPU()`

Replace the body of `IM2MPPIPlanner::planCPU()` (around line 1514) so
that each numbered stage is bracketed by chrono timers. The stage tags
**must** match the ones expected by `bench_table_i.py` (see `STAGES`
in that script).

```cpp
bool IM2MPPIPlanner::planCPU()
{
    using clk = std::chrono::steady_clock;
    auto ms = [](clk::duration d) {
        return std::chrono::duration<double, std::milli>(d).count();
    };
    std::map<std::string, double> t;
    auto t_total_0 = clk::now();

    // 1. Warm-start + 7. Joint-mode tree
    auto t0 = clk::now();
    shiftControlSequence();
    buildJointModes();
    t["joint_mode_tree"] = ms(clk::now() - t0);

    if (joint_modes_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[IM2-MPPI] No joint modes — skipping plan().");
        return false;
    }

    // 2. Sample noise
    t0 = clk::now();
    std::vector<std::vector<Control>> noise;
    sampleControlNoise(noise);
    t["noise_sampling"] = ms(clk::now() - t0);

    // 3. Rollout dynamics
    t0 = clk::now();
    std::vector<RolloutResult> base_rollouts = rolloutDynamics(noise);
    t["rollout_dynamics_cpu"] = ms(clk::now() - t0);

    // 4. Per-mode base cost
    t0 = clk::now();
    const bool is_cvar = (params_.method_type == "cvar_mppi");
    const bool is_dra  = (params_.method_type == "dra_mppi");
    std::vector<std::vector<RolloutResult>> all_results;
    all_results.reserve(joint_modes_.size());
    for (const auto& jm : joint_modes_) {
        std::vector<RolloutResult> mode_results = base_rollouts;
        for (auto& r : mode_results) {
            double c = 0.0;
            c += computeGoalCost(r);
            c += computePathCost(r);
            c += computeSmoothnessCost(r);
            c += computeStaticObstacleCost(r);
            c += computeMapObstacleCost(r);
            if (!is_dra) c += computeDynamicObstacleCost(r, jm);
            r.cost = c;
        }
        all_results.push_back(std::move(mode_results));
    }
    // CPU path lumps "K1 rollout + cost" because they aren't separable here.
    t["rollout_cost"] = t["rollout_dynamics_cpu"] + ms(clk::now() - t0);
    t.erase("rollout_dynamics_cpu");

    // 5. CVaR Monte-Carlo
    t0 = clk::now();
    if (is_cvar) {
        std::vector<std::vector<double>> delta_S;
        computeObstacleCVaRCost(base_rollouts, joint_modes_, delta_S);
        for (size_t m = 0; m < all_results.size() && m < delta_S.size(); ++m) {
            for (size_t i = 0; i < all_results[m].size() && i < delta_S[m].size(); ++i) {
                all_results[m][i].cost += delta_S[m][i];
            }
        }
    }
    t["cvar_mc"] = ms(clk::now() - t0);

    // 6. DR collision-probability path (if DR-MPPI) + hard floor
    t0 = clk::now();
    if (is_dra) {
        std::vector<std::vector<double>> delta_S;
        computeDRACollisionProbabilityCost(base_rollouts, joint_modes_, delta_S);
        for (size_t m = 0; m < all_results.size() && m < delta_S.size(); ++m) {
            for (size_t i = 0; i < all_results[m].size() && i < delta_S[m].size(); ++i) {
                all_results[m][i].cost += delta_S[m][i];
            }
        }
    }
    applyHardFloorFilter(joint_modes_, all_results);
    if (is_cvar) addIntentCVaRRiskPremium(joint_modes_, all_results);
    t["voxel_map"] = ms(clk::now() - t0);

    // 7. MPPI weighted update
    t0 = clk::now();
    updateControlSequence(joint_modes_, all_results);
    t["reduction_fusion"] = ms(clk::now() - t0);

    // 8. Output trajectory + yaw + viz cache (lumped as "yaw_warmstart")
    t0 = clk::now();
    planned_traj_.assign(params_.horizon_steps + 1, TrajectoryPoint{});
    State s = current_state_;
    planned_traj_[0].p = s.p;
    planned_traj_[0].v = s.v;
    for (int k = 0; k < params_.horizon_steps; ++k) {
        planned_traj_[k].a   = u_nominal_[k].a;
        s = clampVelocity(propagate(s, u_nominal_[k]));
        planned_traj_[k + 1].p = s.p;
        planned_traj_[k + 1].v = s.v;
    }
    generateYawReference(planned_traj_);
    cacheVisualizationData(base_rollouts, all_results);
    t["yaw_warmstart"] = ms(clk::now() - t0);

    t["total"] = ms(clk::now() - t_total_0);
    writeStageTimingRecord(t);
    return true;
}
```

## 5. Instrument `planGPU()`

For the GPU path, use `cudaEvent_t` around each kernel launch so the
async work is captured correctly. Replace the corresponding stages in
`planGPU()` with:

```cpp
    cudaEvent_t e_start, e_jmt, e_noise, e_k1, e_k2, e_vox, e_k3, e_yaw;
    for (auto* e : {&e_start, &e_jmt, &e_noise, &e_k1, &e_k2,
                    &e_vox, &e_k3, &e_yaw}) {
        cudaEventCreate(e);
    }
    cudaEventRecord(e_start);
```

Place `cudaEventRecord(e_jmt)` immediately after `buildJointModes()`,
`cudaEventRecord(e_noise)` after the host-side noise generation,
`cudaEventRecord(e_k1)` after the rollout-and-cost kernel launch,
`cudaEventRecord(e_k2)` after the CVaR Monte-Carlo kernel launch,
`cudaEventRecord(e_vox)` after the voxel-map collision pass,
`cudaEventRecord(e_k3)` after the reduction kernel,
`cudaEventRecord(e_yaw)` at the very end (after yaw + warm-start).

Then collect and write:

```cpp
    cudaEventSynchronize(e_yaw);
    std::map<std::string, double> t;
    auto elapsed = [](cudaEvent_t a, cudaEvent_t b) {
        float v = 0.0f; cudaEventElapsedTime(&v, a, b); return double(v);
    };
    t["joint_mode_tree"]  = elapsed(e_start, e_jmt);
    t["noise_sampling"]   = elapsed(e_jmt,   e_noise);
    t["rollout_cost"]     = elapsed(e_noise, e_k1);
    t["cvar_mc"]          = elapsed(e_k1,    e_k2);
    t["voxel_map"]        = elapsed(e_k2,    e_vox);
    t["reduction_fusion"] = elapsed(e_vox,   e_k3);
    t["yaw_warmstart"]    = elapsed(e_k3,    e_yaw);
    t["total"]            = elapsed(e_start, e_yaw);
    writeStageTimingRecord(t);
    for (auto e : {e_start, e_jmt, e_noise, e_k1, e_k2,
                   e_vox, e_k3, e_yaw}) {
        cudaEventDestroy(e);
    }
```

## 6. Wire up the log path

Pass the log path to the planner from your launch file:

```xml
  <param name="im2_mppi/stage_timing_log"
         value="$(env HOME)/im2mppi_stage_timings.jsonl"/>
```

or directly in a benchmark shell script:

```bash
export STAGE_LOG=/tmp/im2_stage_$(date +%s).jsonl
rosparam set /im2_mppi/stage_timing_log "$STAGE_LOG"
roslaunch trajectory_planner ... &
sleep 90
rosnode kill -a
python3 ~/catkin_ws/src/IM2MPPI/trajectory_planner/scripts/bench_table_i.py "$STAGE_LOG"
```

## 7. Verify

After rebuilding (`catkin_make`), the log file should contain one
JSON line per `plan()` call:

```
{"joint_mode_tree": 0.18, "noise_sampling": 0.09, "rollout_cost": 1.94, "cvar_mc": 0.86, "voxel_map": 0.17, "reduction_fusion": 0.13, "yaw_warmstart": 0.06, "total": 3.43}
{"joint_mode_tree": 0.21, ...}
```

Run `bench_table_i.py` on the log; the printed Total should match the
existing `planner.plan_latency_mean_ms` field in `_summary.json`
within rounding error. If not, the chrono brackets are mis-placed.

## 8. Reverting

The instrumentation is opt-in via the `stage_timing_log` parameter, so
shipping the changes into the production planner is safe. If you'd
rather keep `master` clean, apply the patch on a feature branch
`stage-timing-instrumentation` and merge only when running the
Table I benchmark.
