#!/usr/bin/env bash
# ============================================================================
#  run_im2_full_debug_all_obstacles_rviz.sh
# ----------------------------------------------------------------------------
#  Run the normal IM2 full debug experiment, but add a visualization-only node
#  that republishes every Gazebo dynamic obstacle as detector-style blue bbox
#  markers.  RViz already displays /onboard_detector/dynamic_bboxes, so this
#  makes the "Dynamic Obstacles" panel show all dynamic obstacles instead of
#  only the locally perceived subset.
#
#  This does not change the planner, fake detector service, predictor, or
#  evaluation metrics.  It only adds an RViz MarkerArray publisher.
#
#  Usage:
#    bash $(rospack find trajectory_planner)/scripts/run_im2_full_debug_all_obstacles_rviz.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Useful environment overrides:
#    ALL_DYNAMIC_BBOX_TOPIC=/onboard_detector/dynamic_bboxes
#    ALL_DYNAMIC_BBOX_PREFIXES="dynamic_cylinder,dynamic_box,person"
#    ALL_DYNAMIC_BBOX_LINE_WIDTH=0.06
# ============================================================================

set -u

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
BASE_RUNNER="${SCRIPT_DIR}/run_im2_full_debug.sh"
VIS_NODE="${SCRIPT_DIR}/publish_all_dynamic_bboxes.py"

ALL_DYNAMIC_BBOX_TOPIC="${ALL_DYNAMIC_BBOX_TOPIC:-/onboard_detector/dynamic_bboxes}"
ALL_DYNAMIC_BBOX_PREFIXES="${ALL_DYNAMIC_BBOX_PREFIXES:-dynamic_cylinder,dynamic_box,person}"
ALL_DYNAMIC_BBOX_LINE_WIDTH="${ALL_DYNAMIC_BBOX_LINE_WIDTH:-0.06}"
ALL_DYNAMIC_BBOX_LIFETIME="${ALL_DYNAMIC_BBOX_LIFETIME:-0.15}"
ALL_DYNAMIC_BBOX_COLOR_R="${ALL_DYNAMIC_BBOX_COLOR_R:-0.0}"
ALL_DYNAMIC_BBOX_COLOR_G="${ALL_DYNAMIC_BBOX_COLOR_G:-0.0}"
ALL_DYNAMIC_BBOX_COLOR_B="${ALL_DYNAMIC_BBOX_COLOR_B:-1.0}"
ALL_DYNAMIC_BBOX_COLOR_A="${ALL_DYNAMIC_BBOX_COLOR_A:-1.0}"
MONITOR_PERIOD="${MONITOR_PERIOD:-2}"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${HOME}/IM2MPPI/results/all_obstacles_rviz_${STAMP}"
mkdir -p "${LOG_ROOT}"

RUNNER_PID=""
VIS_PID=""
VIS_LAUNCH_COUNT=0

stop_visualizer() {
    if [[ -n "${VIS_PID}" ]]; then
        kill "${VIS_PID}" >/dev/null 2>&1 || true
        wait "${VIS_PID}" >/dev/null 2>&1 || true
        VIS_PID=""
    fi
}

cleanup() {
    stop_visualizer
    if [[ -n "${RUNNER_PID}" ]]; then
        kill "${RUNNER_PID}" >/dev/null 2>&1 || true
        wait "${RUNNER_PID}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

ros_topic_ready() {
    rostopic type /gazebo/model_states >/dev/null 2>&1
}

visualizer_alive() {
    [[ -n "${VIS_PID}" ]] && kill -0 "${VIS_PID}" >/dev/null 2>&1
}

start_visualizer() {
    if visualizer_alive; then
        return
    fi

    VIS_LAUNCH_COUNT=$((VIS_LAUNCH_COUNT + 1))
    echo "[all-obstacles-rviz] starting all-obstacle bbox publisher -> ${ALL_DYNAMIC_BBOX_TOPIC}"
    python3 "${VIS_NODE}" \
        _output_topic:="${ALL_DYNAMIC_BBOX_TOPIC}" \
        _target_prefixes:="${ALL_DYNAMIC_BBOX_PREFIXES}" \
        _line_width:="${ALL_DYNAMIC_BBOX_LINE_WIDTH}" \
        _lifetime:="${ALL_DYNAMIC_BBOX_LIFETIME}" \
        _color_r:="${ALL_DYNAMIC_BBOX_COLOR_R}" \
        _color_g:="${ALL_DYNAMIC_BBOX_COLOR_G}" \
        _color_b:="${ALL_DYNAMIC_BBOX_COLOR_B}" \
        _color_a:="${ALL_DYNAMIC_BBOX_COLOR_A}" \
        >"${LOG_ROOT}/all_dynamic_bboxes_${VIS_LAUNCH_COUNT}.log" 2>&1 &
    VIS_PID=$!
}

echo
echo "================================================================"
echo "  IM2 full debug + all dynamic obstacle RViz overlay"
echo "  base runner : ${BASE_RUNNER}"
echo "  overlay     : ${VIS_NODE}"
echo "  output topic: ${ALL_DYNAMIC_BBOX_TOPIC}"
echo "  prefixes    : ${ALL_DYNAMIC_BBOX_PREFIXES}"
echo "  logs        : ${LOG_ROOT}"
echo "================================================================"

bash "${BASE_RUNNER}" "$@" &
RUNNER_PID=$!

while kill -0 "${RUNNER_PID}" >/dev/null 2>&1; do
    if ros_topic_ready; then
        start_visualizer
    else
        stop_visualizer
    fi
    sleep "${MONITOR_PERIOD}"
done

wait "${RUNNER_PID}"
STATUS=$?
RUNNER_PID=""
stop_visualizer
exit "${STATUS}"
