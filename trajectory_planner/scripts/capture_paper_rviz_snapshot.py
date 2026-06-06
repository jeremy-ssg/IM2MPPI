#!/usr/bin/env python3

import math
import os
import re
import shutil
import subprocess
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
        self.window_title = rospy.get_param("~window_title", "RViz")
        self.screenshot_backend = rospy.get_param(
            "~screenshot_backend", "auto"
        ).lower()
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
        self.capture_backend = self._select_capture_backend()

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

    def _service_available(self):
        if SendFilePath is None:
            return False
        try:
            rospy.wait_for_service(self.save_service, timeout=0.5)
            return True
        except rospy.ROSException:
            return False

    @staticmethod
    def _imagemagick_command():
        if shutil.which("import"):
            return ["import"]
        if shutil.which("magick"):
            return ["magick", "import"]
        return None

    def _select_capture_backend(self):
        requested = self.screenshot_backend
        if requested in ("auto", "service") and self._service_available():
            rospy.loginfo("RViz screenshot backend: service %s", self.save_service)
            return "service"

        has_locator = shutil.which("xdotool") or shutil.which("xwininfo")
        if requested in ("auto", "window") and self._imagemagick_command() and has_locator:
            rospy.loginfo("RViz screenshot backend: direct window capture")
            return "window"

        rospy.logerr(
            "No RViz screenshot backend is available. Install the window "
            "capture tools with: sudo apt install imagemagick xdotool"
        )
        return None

    def _find_rviz_window(self):
        if shutil.which("xdotool"):
            searches = (
                ["xdotool", "search", "--onlyvisible", "--class", "rviz"],
                [
                    "xdotool",
                    "search",
                    "--onlyvisible",
                    "--name",
                    self.window_title,
                ],
            )
            for command in searches:
                result = subprocess.run(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                )
                window_ids = result.stdout.split()
                if result.returncode == 0 and window_ids:
                    return window_ids[-1]

        if shutil.which("xwininfo"):
            result = subprocess.run(
                ["xwininfo", "-root", "-tree"],
                check=False,
                capture_output=True,
                text=True,
            )
            candidates = []
            for line in result.stdout.splitlines():
                if "rviz" not in line.lower():
                    continue
                match = re.match(r"\s*(0x[0-9a-fA-F]+)\s+", line)
                if match:
                    candidates.append(match.group(1))
            if candidates:
                return candidates[-1]

        return None

    def _save_with_service(self, output_file):
        save_image = rospy.ServiceProxy(self.save_service, SendFilePath)
        response = save_image(output_file)
        return not hasattr(response, "success") or response.success

    def _save_window(self, output_file):
        window_id = self._find_rviz_window()
        if window_id is None:
            rospy.logerr(
                "Could not find a visible RViz window (title=%s)",
                self.window_title,
            )
            return False

        command = self._imagemagick_command()
        command.extend(["-window", window_id, output_file])
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            rospy.logerr(
                "RViz window capture failed (window=%s): %s",
                window_id,
                result.stderr.strip(),
            )
            return False
        return os.path.isfile(output_file) and os.path.getsize(output_file) > 0

    def _save(self, output_file):
        if self.capture_backend is None:
            return False

        os.makedirs(self.output_dir, exist_ok=True)
        if self.capture_backend == "service":
            return self._save_with_service(output_file)
        return self._save_window(output_file)

    def run(self):
        if self.capture_backend is None:
            return 1

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
