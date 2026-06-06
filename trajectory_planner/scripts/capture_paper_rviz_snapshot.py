#!/usr/bin/env python3

import math
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

try:
    from rviz.srv import SendFilePath
except ImportError:
    SendFilePath = None


class PaperSnapshotCapture:
    def __init__(self):
        self.output_dir = os.path.abspath(
            os.path.expanduser(rospy.get_param("~output_dir"))
        )
        self.filename_prefix = rospy.get_param(
            "~filename_prefix", "im2_mppi_paper_snapshot"
        )
        self.save_service = rospy.get_param("~save_service", "/rviz/save_image")
        self.first_capture_distance = float(
            rospy.get_param("~first_capture_distance", 4.0)
        )
        self.capture_interval = float(
            rospy.get_param("~capture_interval", 3.0)
        )
        self.capture_count = int(rospy.get_param("~capture_count", 10))
        self.min_elapsed = float(rospy.get_param("~min_elapsed", 3.0))
        self.timeout = float(rospy.get_param("~timeout", 120.0))
        self.min_rollout_markers = int(
            rospy.get_param("~min_rollout_markers", 150)
        )

        self.start_wall_time = time.monotonic()
        self.previous_position = None
        self.current_position = None
        self.travelled_distance = 0.0
        self.rollout_count = 0
        self.saved_count = 0

        rospy.Subscriber(
            "/CERLAB/quadcopter/odom",
            Odometry,
            self._odom_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/im2mppi/sampled_rollouts",
            MarkerArray,
            self._rollout_callback,
            queue_size=1,
        )

    def _odom_callback(self, message):
        point = message.pose.pose.position
        position = (point.x, point.y, point.z)
        if self.previous_position is not None:
            step = math.hypot(
                position[0] - self.previous_position[0],
                position[1] - self.previous_position[1],
            )
            if step <= 2.0:
                self.travelled_distance += step
        self.previous_position = position
        self.current_position = position

    def _rollout_callback(self, message):
        self.rollout_count = len(message.markers)

    def _next_capture_distance(self):
        return (
            self.first_capture_distance
            + self.saved_count * self.capture_interval
        )

    def _ready(self):
        elapsed = time.monotonic() - self.start_wall_time
        return (
            elapsed >= self.min_elapsed
            and self.travelled_distance >= self._next_capture_distance()
            and self.rollout_count >= self.min_rollout_markers
        )

    def _output_file(self):
        return os.path.join(
            self.output_dir,
            "{}_{:02d}.png".format(
                self.filename_prefix,
                self.saved_count + 1,
            ),
        )

    def _save(self, output_file):
        if SendFilePath is None:
            rospy.logerr("rviz/SendFilePath is unavailable")
            return False

        os.makedirs(self.output_dir, exist_ok=True)

        rospy.wait_for_service(self.save_service, timeout=10.0)
        save_image = rospy.ServiceProxy(self.save_service, SendFilePath)
        response = save_image(output_file)
        return not hasattr(response, "success") or response.success

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed = time.monotonic() - self.start_wall_time
            if elapsed >= self.timeout:
                rospy.logerr(
                    "Snapshot timeout after %d/%d images: travelled=%.2f m, "
                    "rollout markers=%d",
                    self.saved_count,
                    self.capture_count,
                    self.travelled_distance,
                    self.rollout_count,
                )
                return 1

            if self._ready():
                rospy.sleep(0.5)
                output_file = self._output_file()
                if not self._save(output_file):
                    return 1
                self.saved_count += 1
                rospy.loginfo(
                    "Saved paper snapshot %d/%d at %.2f m: %s",
                    self.saved_count,
                    self.capture_count,
                    self.travelled_distance,
                    output_file,
                )
                if self.saved_count >= self.capture_count:
                    return 0

            rospy.loginfo_throttle(
                2.0,
                "Waiting for snapshot %d/%d: travelled=%.2f/%.2f m, "
                "rollouts=%d/%d",
                self.saved_count + 1,
                self.capture_count,
                self.travelled_distance,
                self._next_capture_distance(),
                self.rollout_count,
                self.min_rollout_markers,
            )
            rate.sleep()
        return 1


if __name__ == "__main__":
    rospy.init_node("capture_paper_rviz_snapshot")
    sys.exit(PaperSnapshotCapture().run())
