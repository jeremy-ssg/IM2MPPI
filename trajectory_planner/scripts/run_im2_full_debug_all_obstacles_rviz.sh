#!/usr/bin/env bash
# Compatibility entry point.
#
# Do not keep the real implementation in this file: run_im2_full_debug.sh calls
# `pkill -f rviz` during cleanup, so a long-running process whose command line
# contains this filename would kill itself immediately.

set -u

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
exec -a im2_all_obstacles_overlay bash "${SCRIPT_DIR}/run_im2_full_debug_all_obstacles.sh" "$@"
