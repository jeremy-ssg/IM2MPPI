#!/usr/bin/env bash
# Capture one high-resolution, drone-centred RViz snapshot of M4_im2_full.
#
# Usage:
#   ./run_im2_paper_snapshot.sh [SEED] [CAPTURE_COUNT] [VIEW_HALF_WIDTH_M]
#                                [IMAGE_WIDTH_PX] [IMAGE_HEIGHT_PX]
#
# Example:
#   ./run_im2_paper_snapshot.sh 12 10 7.5 3840 2160

set -u

SEED="${1:-12}"
CAPTURE_COUNT="${2:-10}"
VIEW_HALF_WIDTH="${3:-7.5}"
IMAGE_WIDTH="${4:-3840}"
IMAGE_HEIGHT="${5:-2160}"
RVIZ_WINDOW_WIDTH="${RVIZ_WINDOW_WIDTH:-1600}"
RVIZ_WINDOW_HEIGHT="${RVIZ_WINDOW_HEIGHT:-900}"
FIRST_CAPTURE_DISTANCE="${FIRST_CAPTURE_DISTANCE:-4.0}"
CAPTURE_INTERVAL="${CAPTURE_INTERVAL:-3.0}"
CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-120}"
MIN_ELAPSED="${MIN_ELAPSED:-3}"
VIZ_ROLLOUTS="${VIZ_ROLLOUTS:-200}"
GAZEBO_GUI="${GAZEBO_GUI:-true}"

if ! command -v import >/dev/null 2>&1 \
    && ! command -v magick >/dev/null 2>&1; then
    echo "ERROR: RViz window capture requires ImageMagick."
    echo "Install it once with:"
    echo "  sudo apt update && sudo apt install -y imagemagick xdotool"
    exit 1
fi

if ! command -v xdotool >/dev/null 2>&1 \
    && ! command -v xwininfo >/dev/null 2>&1; then
    echo "ERROR: RViz window capture requires xdotool or xwininfo."
    echo "Install the recommended tool once with:"
    echo "  sudo apt update && sudo apt install -y imagemagick xdotool"
    exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR_OVERRIDE:-${HOME}/IM2MPPI/results/paper_snapshot_${STAMP}}"
LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
BASE_RVIZ="$(rospack find trajectory_planner)/rviz/im2_paper_snapshot.rviz"
RUNTIME_RVIZ="${OUT_DIR}/im2_paper_snapshot_runtime.rviz"
CAPTURE_NODE="$(rospack find trajectory_planner)/scripts/capture_paper_rviz_snapshot.py"
SNAPSHOT_DIR="${OUT_DIR}/snapshots"
YAML_BACKUP="${OUT_DIR}/im2_mppi.yaml.backup"

cleanup_all() {
    for SIG in 15 9; do
        pkill -${SIG} -f capture_paper_rviz_snapshot.py 2>/dev/null || true
        pkill -${SIG} -f im2_mppi_navigation_node      2>/dev/null || true
        pkill -${SIG} -f tracking_controller_node     2>/dev/null || true
        pkill -${SIG} -f onboard_detector             2>/dev/null || true
        pkill -${SIG} -f dynamic_predictor            2>/dev/null || true
        pkill -${SIG} -f keyboard_control             2>/dev/null || true
        pkill -${SIG} -f gzclient                     2>/dev/null || true
        pkill -${SIG} -f gzserver                     2>/dev/null || true
        pkill -${SIG} -f gazebo                       2>/dev/null || true
        pkill -${SIG} -f rviz                         2>/dev/null || true
        pkill -${SIG} -f roslaunch                    2>/dev/null || true
        pkill -${SIG} -f rosmaster                    2>/dev/null || true
        pkill -${SIG} -f rosout                       2>/dev/null || true
        [[ "${SIG}" == "15" ]] && sleep 2
    done
}

restore_and_exit() {
    local status=$?
    trap - EXIT INT TERM
    if [[ -f "${YAML_BACKUP}" ]]; then
        cp "${YAML_BACKUP}" "${YAML_PLANNER}"
    fi
    cleanup_all
    if [[ "${status}" -eq 0 && -d "${SNAPSHOT_DIR}" ]]; then
        echo
        echo "Paper snapshot candidates saved:"
        echo "  ${SNAPSHOT_DIR}"
    fi
    exit "${status}"
}

set_yaml_str() {
    local key=$1
    local value=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 \"${value}\"|g" "${YAML_PLANNER}"
}

set_yaml_num() {
    local key=$1
    local value=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 ${value}|g" "${YAML_PLANNER}"
}

cleanup_all
cp "${YAML_PLANNER}" "${YAML_BACKUP}"
trap restore_and_exit EXIT INT TERM

set_yaml_str "method_type" "cvar_mppi"
set_yaml_str "fusion_mode" "adaptive"
set_yaml_num "random_seed" "${SEED}"
set_yaml_num "viz_num_rollouts" "${VIZ_ROLLOUTS}"

VIEW_SCALE="$(python3 - "${VIEW_HALF_WIDTH}" "${RVIZ_WINDOW_WIDTH}" <<'PY'
import sys
half_width = max(1.0, float(sys.argv[1]))
window_width = max(640, int(sys.argv[2]))
print(float(window_width) / (2.0 * half_width))
PY
)"
sed -E \
    -e "s|^([[:space:]]*Scale:).*$|\1 ${VIEW_SCALE}|" \
    -e "s|^([[:space:]]*Width:).*$|\1 ${RVIZ_WINDOW_WIDTH}|" \
    -e "s|^([[:space:]]*Height:).*$|\1 ${RVIZ_WINDOW_HEIGHT}|" \
    "${BASE_RVIZ}" > "${RUNTIME_RVIZ}"

echo "============================================================"
echo "  IM2-MPPI paper snapshot"
echo "  seed: ${SEED}  displayed rollouts: ${VIZ_ROLLOUTS}"
echo "  captures: ${CAPTURE_COUNT}, first at ${FIRST_CAPTURE_DISTANCE} m,"
echo "            then every ${CAPTURE_INTERVAL} m"
echo "  horizontal view: +/- ${VIEW_HALF_WIDTH} m"
echo "  RViz window: ${RVIZ_WINDOW_WIDTH}x${RVIZ_WINDOW_HEIGHT} (single screen)"
echo "  output image: ${IMAGE_WIDTH}x${IMAGE_HEIGHT}"
echo "  output: ${SNAPSHOT_DIR}"
echo "============================================================"

roscore > "${LOG_DIR}/roscore.log" 2>&1 &
sleep 3

rosparam set /autonomous_flight/closed_loop_intent_enabled true \
    >/dev/null 2>&1 || true

OBS_BRANCH_SEED="${SEED}" \
roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" \
    > "${LOG_DIR}/sim.log" 2>&1 &
sleep 10

pkill -9 -f keyboard_control 2>/dev/null || true

roslaunch autonomous_flight im2_mppi_demo.launch enable_rviz:=false \
    > "${LOG_DIR}/stack.log" 2>&1 &
sleep 8

rviz -d "${RUNTIME_RVIZ}" > "${LOG_DIR}/rviz.log" 2>&1 &
sleep 5

python3 "${CAPTURE_NODE}" \
    _output_dir:="${SNAPSHOT_DIR}" \
    _filename_prefix:="im2_mppi_seed${SEED}" \
    _first_capture_distance:="${FIRST_CAPTURE_DISTANCE}" \
    _capture_interval:="${CAPTURE_INTERVAL}" \
    _capture_count:="${CAPTURE_COUNT}" \
    _min_elapsed:="${MIN_ELAPSED}" \
    _timeout:="${CAPTURE_TIMEOUT}" \
    _output_width:="${IMAGE_WIDTH}" \
    _output_height:="${IMAGE_HEIGHT}" \
    _min_rollout_markers:="$((VIZ_ROLLOUTS * 3 / 4))"
