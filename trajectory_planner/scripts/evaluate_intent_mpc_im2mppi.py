#!/usr/bin/env python3
"""
Runtime evaluator for Intent-MPC and IM2-MPPI.

Records safety / tracking / smoothness / planning-latency metrics for one
planner per launch. Outputs JSON summary + CSV time-series for downstream
statistical comparison.

Improvements over the initial version:
  * Tiered collision metrics (strict + near-miss + tail).
  * Empirical CVaR over the worst-α clearance tail (matches the theoretical
    CVaR objective the planner optimizes).
  * Acceleration / jerk computed from the COMMANDED target (no odom noise).
  * Stops sampling once the goal is reached so post-arrival hover doesn't
    pollute mean speed / tracking metrics.
  * Subscribes to /im2mppi/plan_time_ms (or any configured plan-time topic)
    to record actual planning latency (mean / p95 / max).
"""

import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from tracking_controller.msg import Target
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────────────────
#  Math helpers
# ─────────────────────────────────────────────────────────────────────────────

def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dist3(a, b):
    return norm3(sub3(a, b))


def point_from_pose(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def path_points(path_msg):
    return [point_from_pose(ps.pose) for ps in path_msg.poses]


def aabb_clearance(point, center, size):
    """Same SDF as the C++ planner (aabbSDF in im2_mppi_planner.cpp)."""
    half = (max(size[0], 1e-6) * 0.5,
            max(size[1], 1e-6) * 0.5,
            max(size[2], 1e-6) * 0.5)
    q = (abs(point[0] - center[0]) - half[0],
         abs(point[1] - center[1]) - half[1],
         abs(point[2] - center[2]) - half[2])
    outside = norm3((max(q[0], 0.0), max(q[1], 0.0), max(q[2], 0.0)))
    inside = min(max(q[0], q[1], q[2]), 0.0)
    return outside + inside


def mean(values):
    return sum(values) / len(values) if values else None


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def percentile(values, q):
    if not values:
        return None
    vals = sorted(values)
    idx = int(round((len(vals) - 1) * q))
    return vals[max(0, min(idx, len(vals) - 1))]


def empirical_cvar(values, alpha, worst="low"):
    """
    Empirical CVaR over the worst α fraction of `values`.
      worst="low"  : low values are bad (e.g., clearance — small = dangerous)
                     returns mean of the smallest α fraction.
      worst="high" : high values are bad (e.g., cost, latency)
                     returns mean of the largest α fraction.
    """
    if not values:
        return None
    n = len(values)
    k = max(1, int(round(n * alpha)))
    s = sorted(values)
    tail = s[:k] if worst == "low" else s[-k:]
    return sum(tail) / len(tail)


def path_length(points):
    return sum(dist3(points[i], points[i - 1]) for i in range(1, len(points)))


def finite_diff_norms(samples):
    out = []
    for i in range(1, len(samples)):
        t0, v0 = samples[i - 1]
        t1, v1 = samples[i]
        dt = t1 - t0
        if dt > 1e-6:
            out.append(norm3(sub3(v1, v0)) / dt)
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  Evaluator
# ─────────────────────────────────────────────────────────────────────────────

class Evaluator:
    def __init__(self):
        self.algorithm = rospy.get_param("~algorithm", "im2_mppi")
        self.duration = float(rospy.get_param("~duration", 120.0))
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(os.path.expanduser("~"), ".ros", "mppi_eval"),
        )
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.5))
        self.sample_hz = float(rospy.get_param("~sample_hz", 20.0))

        # Tiered collision thresholds (m).
        self.collision_strict   = float(rospy.get_param("~collision_strict",   0.15))
        self.collision_near_miss = float(rospy.get_param("~collision_near_miss", 0.30))
        self.collision_tail     = float(rospy.get_param("~collision_tail",     0.50))

        # CVaR α for the safety tail (worst α fraction of clearances).
        self.cvar_alpha = float(rospy.get_param("~cvar_alpha", 0.05))

        # Topics
        self.odom_topic     = rospy.get_param("~odom_topic", "/CERLAB/quadcopter/odom")
        self.target_topic   = rospy.get_param("~target_topic", "/autonomous_flight/target_state")
        self.goal_topic     = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/onboard_detector/GT_obstacle_bbox")
        self.path_topic     = rospy.get_param("~path_topic", "")
        self.plan_time_topic = rospy.get_param(
            "~plan_time_topic", "/im2mppi/plan_time_ms")
        if not self.path_topic:
            self.path_topic = self.default_path_topic(self.algorithm)

        self.lock = threading.Lock()
        self.start_time = None
        self.goal = None
        self.latest_odom = None
        self.latest_target = None
        self.latest_obstacles = []
        self.latest_path = None

        # Per-sample storage (in-flight only — stops at arrival).
        self.odom_samples = []
        self.target_errors = []
        self.clearances = []
        self.cmd_accel_samples = []   # (t, (ax,ay,az)) from target msg
        self.path_metrics = []
        self.path_arrival_times = []
        self.plan_time_samples_ms = []

        # Tiered collision counters.
        self.collision_strict_samples    = 0
        self.collision_near_miss_samples = 0
        self.collision_tail_samples      = 0
        self.collision_strict_time = 0.0
        self.last_sample_time = None

        # Flight phase control.
        self.flight_ended = False
        self.time_to_goal = None
        self.success = False

        os.makedirs(self.output_dir, exist_ok=True)

        rospy.Subscriber(self.odom_topic,     Odometry,    self.odom_cb,     queue_size=50)
        rospy.Subscriber(self.target_topic,   Target,      self.target_cb,   queue_size=50)
        rospy.Subscriber(self.goal_topic,     PoseStamped, self.goal_cb,     queue_size=5)
        rospy.Subscriber(self.path_topic,     Path,        self.path_cb,     queue_size=20)
        rospy.Subscriber(self.obstacle_topic, MarkerArray, self.obstacle_cb, queue_size=10)
        rospy.Subscriber(self.plan_time_topic, Float64,    self.plan_time_cb, queue_size=50)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.sample_hz, 1.0)),
                                 self.sample_cb)
        self.shutdown_timer = rospy.Timer(rospy.Duration(max(self.duration, 1.0)),
                                          self.finish_cb, oneshot=True)

    # ── Topic defaults ────────────────────────────────────────────────────

    @staticmethod
    def default_path_topic(algorithm):
        if algorithm in ("intent_mpc", "intend_mpc", "mpc"):
            return "/mpcNavigation/mpc_trajectory"
        return "/im2mppi/best_trajectory"

    def now_rel(self):
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        return (now - self.start_time).to_sec()

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def target_cb(self, msg):
        # Record COMMANDED acceleration — noise-free, matches what the
        # tracking controller is asked to execute.
        t = self.now_rel()
        with self.lock:
            self.latest_target = msg
            if not self.flight_ended:
                a = (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)
                self.cmd_accel_samples.append((t, a))

    def goal_cb(self, msg):
        with self.lock:
            self.goal = point_from_pose(msg.pose)

    def path_cb(self, msg):
        t = self.now_rel()
        points = path_points(msg)
        with self.lock:
            if self.flight_ended:
                return
            self.latest_path = points
            self.path_arrival_times.append(t)
            if points:
                metric = self.compute_path_metric(t, points)
                self.path_metrics.append(metric)

    def plan_time_cb(self, msg):
        with self.lock:
            if self.flight_ended:
                return
            self.plan_time_samples_ms.append(float(msg.data))

    def obstacle_cb(self, msg):
        boxes = []
        line_groups = {}
        for marker in msg.markers:
            if marker.type == Marker.LINE_LIST and marker.points:
                key = marker.ns if marker.ns else str(marker.id)
                line_groups.setdefault(key, []).extend(marker.points)
                if len(marker.points) >= 24:
                    boxes.extend(self.line_points_to_boxes(marker.points))
                continue
            box = self.marker_to_box(marker)
            if box is not None:
                boxes.append(box)
        for points in line_groups.values():
            boxes.extend(self.line_points_to_boxes(points))
        with self.lock:
            self.latest_obstacles = boxes

    def line_points_to_boxes(self, points):
        boxes = []
        if not points:
            return boxes
        chunk_size = 24 if len(points) >= 24 else len(points)
        for start in range(0, len(points), chunk_size):
            chunk = points[start:start + chunk_size]
            if len(chunk) < 2:
                continue
            xs = [p.x for p in chunk]
            ys = [p.y for p in chunk]
            zs = [p.z for p in chunk]
            size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            if max(size) < 1e-6:
                continue
            center = ((min(xs) + max(xs)) * 0.5,
                      (min(ys) + max(ys)) * 0.5,
                      (min(zs) + max(zs)) * 0.5)
            boxes.append((center, size))
        return boxes

    def marker_to_box(self, marker):
        if marker.type == Marker.CUBE:
            center = point_from_pose(marker.pose)
            size = (marker.scale.x, marker.scale.y, marker.scale.z)
            return center, size

        if marker.type == Marker.LINE_LIST and marker.points:
            xs = [p.x for p in marker.points]
            ys = [p.y for p in marker.points]
            zs = [p.z for p in marker.points]
            if not xs:
                return None
            center = ((min(xs) + max(xs)) * 0.5,
                      (min(ys) + max(ys)) * 0.5,
                      (min(zs) + max(zs)) * 0.5)
            size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            return center, size

        if marker.type in (Marker.SPHERE, Marker.CYLINDER):
            center = point_from_pose(marker.pose)
            size = (marker.scale.x, marker.scale.y, marker.scale.z)
            return center, size

        return None

    def compute_path_metric(self, t, points):
        clearances = []
        for p in points:
            c = self.min_clearance_locked(p)
            if c is not None:
                clearances.append(c)
        segment_lengths = [dist3(points[i], points[i - 1]) for i in range(1, len(points))]
        return {
            "t": t,
            "num_points": len(points),
            "path_length": path_length(points),
            "mean_segment_length": mean(segment_lengths),
            "min_obstacle_clearance": min(clearances) if clearances else None,
            "near_miss_points": sum(1 for c in clearances if c < self.collision_near_miss),
            "strict_collision_points": sum(1 for c in clearances if c < self.collision_strict),
        }

    def min_clearance_locked(self, point):
        if not self.latest_obstacles:
            return None
        return min(aabb_clearance(point, center, size)
                   for center, size in self.latest_obstacles)

    # ── Periodic sample ───────────────────────────────────────────────────

    def sample_cb(self, _event):
        with self.lock:
            if self.flight_ended:
                return
            if self.latest_odom is None:
                return

            t = self.now_rel()
            odom = self.latest_odom
            pos = point_from_pose(odom.pose.pose)
            vel = (odom.twist.twist.linear.x,
                   odom.twist.twist.linear.y,
                   odom.twist.twist.linear.z)

            target_error = None
            if self.latest_target is not None:
                tgt = (self.latest_target.position.x,
                       self.latest_target.position.y,
                       self.latest_target.position.z)
                target_error = dist3(pos, tgt)
                self.target_errors.append(target_error)

            clearance = self.min_clearance_locked(pos)
            if clearance is not None:
                self.clearances.append(clearance)
                if clearance < self.collision_strict:
                    self.collision_strict_samples += 1
                    if self.last_sample_time is not None:
                        self.collision_strict_time += max(0.0, t - self.last_sample_time)
                if clearance < self.collision_near_miss:
                    self.collision_near_miss_samples += 1
                if clearance < self.collision_tail:
                    self.collision_tail_samples += 1

            goal_dist = dist3(pos, self.goal) if self.goal is not None else None
            self.odom_samples.append({
                "t": t,
                "x": pos[0], "y": pos[1], "z": pos[2],
                "vx": vel[0], "vy": vel[1], "vz": vel[2],
                "speed": norm3(vel),
                "target_error": target_error,
                "obstacle_clearance": clearance,
                "goal_distance": goal_dist,
            })
            self.last_sample_time = t

            # Stop sampling at arrival so smoothness / speed metrics aren't
            # diluted by post-arrival hover.
            if (self.goal is not None and goal_dist is not None
                    and goal_dist <= self.goal_radius):
                self.success = True
                self.time_to_goal = t
                self.flight_ended = True
                rospy.loginfo(
                    "[eval] Goal reached at t=%.2fs — freezing sampling.", t)

    def finish_cb(self, _event):
        self.write_outputs()
        rospy.signal_shutdown("evaluation finished")

    # ── Summary ───────────────────────────────────────────────────────────

    def summarize(self):
        positions = [(s["x"], s["y"], s["z"]) for s in self.odom_samples]
        velocities = [(s["t"], (s["vx"], s["vy"], s["vz"])) for s in self.odom_samples]
        speeds = [s["speed"] for s in self.odom_samples]

        # Smoothness from COMMANDED target acceleration (preferred — no odom noise).
        cmd_accel_norms = [norm3(a) for _t, a in self.cmd_accel_samples]
        cmd_jerk_norms = finite_diff_norms(self.cmd_accel_samples)

        # Fallback smoothness from odom (kept as a secondary, noisy reference).
        odom_accel_samples = []
        for i in range(1, len(velocities)):
            dt = velocities[i][0] - velocities[i - 1][0]
            if dt > 1e-6:
                dv = sub3(velocities[i][1], velocities[i - 1][1])
                odom_accel_samples.append(
                    (velocities[i][0], (dv[0] / dt, dv[1] / dt, dv[2] / dt)))
        odom_accel_norms = [norm3(a) for _t, a in odom_accel_samples]
        odom_jerk_norms = finite_diff_norms(odom_accel_samples)

        final_goal_distance = None
        if self.goal is not None and self.odom_samples:
            final_goal_distance = self.odom_samples[-1]["goal_distance"]

        plan_periods = [self.path_arrival_times[i] - self.path_arrival_times[i - 1]
                        for i in range(1, len(self.path_arrival_times))]

        path_clearances = [m["min_obstacle_clearance"] for m in self.path_metrics
                           if m["min_obstacle_clearance"] is not None]
        path_lengths = [m["path_length"] for m in self.path_metrics]

        # Empirical CVaR over the worst α fraction of the clearance series —
        # directly comparable to the planner's CVaR objective.
        cvar_05 = empirical_cvar(self.clearances, 0.05, worst="low")
        cvar_10 = empirical_cvar(self.clearances, 0.10, worst="low")
        cvar_user = empirical_cvar(self.clearances, self.cvar_alpha, worst="low")

        # Planner latency (P95 / max are the meaningful numbers for real-time).
        plan_latency_mean = mean(self.plan_time_samples_ms)
        plan_latency_p95 = percentile(self.plan_time_samples_ms, 0.95)
        plan_latency_max = max(self.plan_time_samples_ms) if self.plan_time_samples_ms else None

        summary = {
            "algorithm": self.algorithm,
            "topics": {
                "odom": self.odom_topic,
                "target": self.target_topic,
                "planned_path": self.path_topic,
                "obstacles": self.obstacle_topic,
                "goal": self.goal_topic,
                "plan_time": self.plan_time_topic,
            },
            "samples": {
                "odom": len(self.odom_samples),
                "cmd_accel": len(self.cmd_accel_samples),
                "planned_paths": len(self.path_metrics),
                "obstacle_clearance": len(self.clearances),
                "plan_time_msgs": len(self.plan_time_samples_ms),
            },
            "task": {
                "success": self.success,
                "goal_radius": self.goal_radius,
                "time_to_goal_s": self.time_to_goal,
                "final_goal_distance_m": final_goal_distance,
                "executed_path_length_m": path_length(positions),
                "flight_duration_s": self.odom_samples[-1]["t"] if self.odom_samples else None,
            },
            "safety": {
                # Raw clearance stats
                "min_clearance_m": min(self.clearances) if self.clearances else None,
                "mean_clearance_m": mean(self.clearances),
                "p05_clearance_m": percentile(self.clearances, 0.05),
                # Tiered collision metrics
                "collision_strict_threshold_m":    self.collision_strict,
                "collision_near_miss_threshold_m": self.collision_near_miss,
                "collision_tail_threshold_m":      self.collision_tail,
                "collision_strict_samples":        self.collision_strict_samples,
                "collision_near_miss_samples":     self.collision_near_miss_samples,
                "collision_tail_samples":          self.collision_tail_samples,
                "collision_strict_rate":           (self.collision_strict_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_near_miss_rate":        (self.collision_near_miss_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_tail_rate":             (self.collision_tail_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_strict_time_s":         self.collision_strict_time,
                # Empirical CVaR
                "cvar_alpha":               self.cvar_alpha,
                "empirical_cvar_5pct_m":    cvar_05,
                "empirical_cvar_10pct_m":   cvar_10,
                "empirical_cvar_user_m":    cvar_user,
                # Planned-trajectory safety (offline check of planner's intent)
                "planned_min_clearance_m":      min(path_clearances) if path_clearances else None,
                "planned_near_miss_points":     sum(m["near_miss_points"] for m in self.path_metrics),
                "planned_strict_collision_points": sum(m["strict_collision_points"] for m in self.path_metrics),
            },
            "tracking": {
                "mean_target_error_m": mean(self.target_errors),
                "rms_target_error_m":  rms(self.target_errors),
                "max_target_error_m":  max(self.target_errors) if self.target_errors else None,
            },
            "smoothness": {
                # Speed
                "mean_speed_mps": mean(speeds),
                "max_speed_mps":  max(speeds) if speeds else None,
                # Acceleration / jerk from COMMANDED target (preferred)
                "cmd_mean_accel_mps2": mean(cmd_accel_norms),
                "cmd_max_accel_mps2":  max(cmd_accel_norms) if cmd_accel_norms else None,
                "cmd_rms_accel_mps2":  rms(cmd_accel_norms),
                "cmd_mean_jerk_mps3":  mean(cmd_jerk_norms),
                "cmd_max_jerk_mps3":   max(cmd_jerk_norms) if cmd_jerk_norms else None,
                "cmd_rms_jerk_mps3":   rms(cmd_jerk_norms),
                # Acceleration / jerk from odom finite-difference (noisy reference)
                "odom_rms_accel_mps2": rms(odom_accel_norms),
                "odom_rms_jerk_mps3":  rms(odom_jerk_norms),
            },
            "planner": {
                # Output cadence (publish rate of planned path)
                "publish_rate_hz":         (1.0 / mean(plan_periods))
                                            if plan_periods and mean(plan_periods) else None,
                "mean_publish_period_s":   mean(plan_periods),
                "max_publish_period_s":    max(plan_periods) if plan_periods else None,
                # Actual planning latency (from /im2mppi/plan_time_ms)
                "plan_latency_mean_ms":    plan_latency_mean,
                "plan_latency_p95_ms":     plan_latency_p95,
                "plan_latency_max_ms":     plan_latency_max,
                # Planned path geometry
                "mean_planned_path_length_m": mean(path_lengths),
                "max_planned_path_length_m":  max(path_lengths) if path_lengths else None,
            },
        }
        return summary

    def write_outputs(self):
        with self.lock:
            summary = self.summarize()
            prefix = os.path.join(self.output_dir, self.algorithm)

            with open(prefix + "_summary.json", "w") as f:
                json.dump(summary, f, indent=2, sort_keys=True)

            with open(prefix + "_timeseries.csv", "w", newline="") as f:
                fields = ["t", "x", "y", "z", "vx", "vy", "vz", "speed",
                          "target_error", "obstacle_clearance", "goal_distance"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.odom_samples:
                    writer.writerow(row)

            with open(prefix + "_path_metrics.csv", "w", newline="") as f:
                fields = ["t", "num_points", "path_length", "mean_segment_length",
                          "min_obstacle_clearance",
                          "near_miss_points", "strict_collision_points"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.path_metrics:
                    writer.writerow(row)

            with open(prefix + "_plan_time.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["plan_time_ms"])
                for v in self.plan_time_samples_ms:
                    writer.writerow([v])

            rospy.loginfo("[eval] Wrote summary to %s_summary.json", prefix)
            rospy.loginfo("[eval] success=%s  t_goal=%s  min_clr=%.3f  cvar5=%s  p95_lat=%s ms",
                          summary["task"]["success"],
                          summary["task"]["time_to_goal_s"],
                          summary["safety"]["min_clearance_m"] or float("nan"),
                          summary["safety"]["empirical_cvar_5pct_m"],
                          summary["planner"]["plan_latency_p95_ms"])


if __name__ == "__main__":
    rospy.init_node("planner_evaluator")
    Evaluator()
    rospy.spin()
