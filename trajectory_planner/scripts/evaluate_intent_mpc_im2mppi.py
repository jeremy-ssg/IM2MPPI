#!/usr/bin/env python3
"""
Runtime evaluator for Intent-MPC and IM2-MPPI.

Run one planner at a time, point this node at the planner trajectory topic, and
it records comparable safety, tracking, smoothness, and timing metrics.
"""

import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tracking_controller.msg import Target
from visualization_msgs.msg import Marker, MarkerArray


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


class Evaluator:
    def __init__(self):
        self.algorithm = rospy.get_param("~algorithm", "im2_mppi")
        self.duration = float(rospy.get_param("~duration", 120.0))
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(os.path.expanduser("~"), ".ros", "mppi_eval"),
        )
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.5))
        self.collision_clearance = float(rospy.get_param("~collision_clearance", 0.0))
        self.sample_hz = float(rospy.get_param("~sample_hz", 20.0))
        self.odom_topic = rospy.get_param("~odom_topic", "/CERLAB/quadcopter/odom")
        self.target_topic = rospy.get_param("~target_topic", "/autonomous_flight/target_state")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/onboard_detector/GT_obstacle_bbox")
        self.path_topic = rospy.get_param("~path_topic", "")
        if not self.path_topic:
            self.path_topic = self.default_path_topic(self.algorithm)

        self.lock = threading.Lock()
        self.start_time = None
        self.goal = None
        self.latest_odom = None
        self.latest_target = None
        self.latest_obstacles = []
        self.latest_path = None

        self.odom_samples = []
        self.target_errors = []
        self.clearances = []
        self.path_metrics = []
        self.path_arrival_times = []
        self.collision_time = 0.0
        self.collision_samples = 0
        self.last_sample_time = None

        os.makedirs(self.output_dir, exist_ok=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(self.target_topic, Target, self.target_cb, queue_size=50)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=5)
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=20)
        rospy.Subscriber(self.obstacle_topic, MarkerArray, self.obstacle_cb, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.sample_hz, 1.0)), self.sample_cb)
        self.shutdown_timer = rospy.Timer(rospy.Duration(max(self.duration, 1.0)), self.finish_cb, oneshot=True)

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

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def target_cb(self, msg):
        with self.lock:
            self.latest_target = msg

    def goal_cb(self, msg):
        with self.lock:
            self.goal = point_from_pose(msg.pose)

    def path_cb(self, msg):
        t = self.now_rel()
        points = path_points(msg)
        with self.lock:
            self.latest_path = points
            self.path_arrival_times.append(t)
            if points:
                metric = self.compute_path_metric(t, points)
                self.path_metrics.append(metric)

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
        # Most bbox visualizers emit 12 line segments = 24 points per box.
        # Some publish each edge as a separate marker with a shared namespace,
        # so grouping by namespace above reconstructs the same 24-point chunk.
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
            "collision_points": sum(1 for c in clearances if c < self.collision_clearance),
        }

    def min_clearance_locked(self, point):
        if not self.latest_obstacles:
            return None
        return min(aabb_clearance(point, center, size)
                   for center, size in self.latest_obstacles)

    def sample_cb(self, _event):
        t = self.now_rel()
        with self.lock:
            if self.latest_odom is None:
                return
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
                if clearance < self.collision_clearance:
                    self.collision_samples += 1
                    if self.last_sample_time is not None:
                        self.collision_time += max(0.0, t - self.last_sample_time)

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

    def finish_cb(self, _event):
        self.write_outputs()
        rospy.signal_shutdown("evaluation finished")

    def summarize(self):
        positions = [(s["x"], s["y"], s["z"]) for s in self.odom_samples]
        velocities = [(s["t"], (s["vx"], s["vy"], s["vz"])) for s in self.odom_samples]
        speeds = [s["speed"] for s in self.odom_samples]
        accel_norms = finite_diff_norms(velocities)
        accel_samples = []
        for i in range(1, len(velocities)):
            dt = velocities[i][0] - velocities[i - 1][0]
            if dt > 1e-6:
                dv = sub3(velocities[i][1], velocities[i - 1][1])
                accel_samples.append((velocities[i][0], (dv[0] / dt, dv[1] / dt, dv[2] / dt)))
        jerk_norms = finite_diff_norms(accel_samples)

        final_goal_distance = None
        success = False
        time_to_goal = None
        if self.goal is not None and self.odom_samples:
            for s in self.odom_samples:
                if s["goal_distance"] is not None and s["goal_distance"] <= self.goal_radius:
                    success = True
                    time_to_goal = s["t"]
                    break
            final_goal_distance = self.odom_samples[-1]["goal_distance"]

        plan_periods = [self.path_arrival_times[i] - self.path_arrival_times[i - 1]
                        for i in range(1, len(self.path_arrival_times))]

        path_clearances = [m["min_obstacle_clearance"] for m in self.path_metrics
                           if m["min_obstacle_clearance"] is not None]
        path_lengths = [m["path_length"] for m in self.path_metrics]

        summary = {
            "algorithm": self.algorithm,
            "topics": {
                "odom": self.odom_topic,
                "target": self.target_topic,
                "planned_path": self.path_topic,
                "obstacles": self.obstacle_topic,
                "goal": self.goal_topic,
            },
            "samples": {
                "odom": len(self.odom_samples),
                "planned_paths": len(self.path_metrics),
                "obstacle_clearance": len(self.clearances),
            },
            "task": {
                "success": success,
                "goal_radius": self.goal_radius,
                "time_to_goal_s": time_to_goal,
                "final_goal_distance_m": final_goal_distance,
                "executed_path_length_m": path_length(positions),
            },
            "safety": {
                "min_clearance_m": min(self.clearances) if self.clearances else None,
                "mean_clearance_m": mean(self.clearances),
                "p05_clearance_m": percentile(self.clearances, 0.05),
                "collision_clearance_threshold_m": self.collision_clearance,
                "collision_samples": self.collision_samples,
                "collision_time_s": self.collision_time,
                "planned_min_clearance_m": min(path_clearances) if path_clearances else None,
                "planned_collision_points": sum(m["collision_points"] for m in self.path_metrics),
            },
            "tracking": {
                "mean_target_error_m": mean(self.target_errors),
                "rms_target_error_m": rms(self.target_errors),
                "max_target_error_m": max(self.target_errors) if self.target_errors else None,
            },
            "smoothness": {
                "mean_speed_mps": mean(speeds),
                "max_speed_mps": max(speeds) if speeds else None,
                "mean_accel_mps2": mean(accel_norms),
                "max_accel_mps2": max(accel_norms) if accel_norms else None,
                "rms_accel_mps2": rms(accel_norms),
                "mean_jerk_mps3": mean(jerk_norms),
                "max_jerk_mps3": max(jerk_norms) if jerk_norms else None,
                "rms_jerk_mps3": rms(jerk_norms),
            },
            "planner": {
                "publish_rate_hz": (1.0 / mean(plan_periods)) if plan_periods and mean(plan_periods) else None,
                "mean_publish_period_s": mean(plan_periods),
                "max_publish_period_s": max(plan_periods) if plan_periods else None,
                "mean_planned_path_length_m": mean(path_lengths),
                "max_planned_path_length_m": max(path_lengths) if path_lengths else None,
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
                          "min_obstacle_clearance", "collision_points"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.path_metrics:
                    writer.writerow(row)

            rospy.loginfo("Wrote evaluation summary to %s_summary.json", prefix)


if __name__ == "__main__":
    rospy.init_node("planner_evaluator")
    Evaluator()
    rospy.spin()
