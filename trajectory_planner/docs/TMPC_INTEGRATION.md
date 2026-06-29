# T-MPC++ Integration Notes

## Current Status

T-MPC++ is implemented as method `M6_tmpc` and is self-contained inside this
repository. It does not require `tud-amr/guidance_planner`, `ros_tools`, or
`mpc_planner` as external catkin packages.

The implementation follows the official T-MPC++ pipeline at the algorithm level:

1. Build constant-velocity dynamic-obstacle predictions.
2. Build a static-aware local reference from the lap reference. If the inflated
   occupancy map blocks the local segment, replace it with an A* reference.
3. Run an internal lightweight Visibility-PRM in `(x,y,t)`.
4. Filter graph-search results by topology signatures so the local optimizer sees
   topology-distinct guidance branches.
5. Solve one local OSQP MPC per branch, plus the optional unguided T-MPC++ branch.
6. Select the best feasible branch with consistency weighting.

## UAV Adaptation

The paper's topology search is planar. For this UAV benchmark, topology search is
performed in the horizontal plane `(x,y,t)` at `z_lap`. The local optimizer remains
3-D with state `[x,y,z,vx,vy,vz]` and control `[ax,ay,az]`.

Static obstacles are handled in three places:

- The local reference can be replaced by an A* path through the inflated occupancy
  map.
- The internal Visibility-PRM rejects edges that intersect the inflated map.
- The OSQP branch solve adds local static half-plane constraints and rejects any
  final trajectory that collides with the inflated map.

Dynamic obstacles are represented as bounding boxes from the detector. The planner
uses their center, size, and velocity to build constant-velocity obstacle tubes.
RViz displays the same detector bounding boxes as the other methods; no intent
prediction markers are required for T-MPC++.

## Key Files

- `trajectory_planner/include/trajectory_planner/tmpc/tmpcPlanner.h`
- `trajectory_planner/include/trajectory_planner/tmpc/tmpcPlanner.cpp`
- `trajectory_planner/cfg/tmpc.yaml`
- `autonomous_flight/include/autonomous_flight/tmpcNavigation.cpp`
- `autonomous_flight/launch/tmpc_demo.launch`
- `autonomous_flight/cfg/tmpc_navigation.rviz`

## Build And Run

No extra topology package is needed.

```bash
catkin_make
source devel/setup.bash
roslaunch autonomous_flight tmpc_demo.launch
```

If planning fails, first check `/tmpc/guidance_paths`, `/tmpc/optimized_trajectories`,
`/tmpc/best_trajectory`, and the console warnings emitted by `tmpcPlanner`.

## Important Parameters

All parameters live under `tmpc:` in `trajectory_planner/cfg/tmpc.yaml`.

- `num_trajectories_P`: number of topology-distinct guided branches to keep.
- `add_unguided_planner`: enables the additional T-MPC++ unguided local planner.
- `prm_samples_n`: internal Visibility-PRM sample budget.
- `goal_grid_lat`, `goal_grid_long`, `goal_lat_spread`: reference-corridor and goal
  sampling controls.
- `use_static_astar`: replace blocked local references with A* paths.
- `static_halfplane_*`: local static-map half-plane constraints used by OSQP.
- `enable_vertical_avoidance`: optional UAV-specific fly-over branch. It is off by
  default because the paper's topology planner is 2-D.

## Remaining Verification

The Windows development machine does not have a ROS/catkin/C++ build chain. Verify
on the Linux ROS machine with `catkin_make`, then run `tmpc_demo.launch` and inspect
the RViz topics above.
