#!/usr/bin/env bash
# Show saved seed trajectories on top of the real Gazebo/RViz perception scene.
#
# This is intentionally different from show_selected_seed_trajectories_rviz.sh:
# it does not draw obstacle markers from a parsed world file.  Static obstacles
# come from the normal dynamic_map inflated voxel-map topic, and dynamic
# obstacles come from fakeDetector's Gazebo-ground-truth bounding boxes.
#
# Usage:
#   bash $(rospack find trajectory_planner)/scripts/show_real_scene_seed_trajectories_rviz.sh [RESULTS_DIR] [SEEDS]
#
# Example:
#   bash $(rospack find trajectory_planner)/scripts/show_real_scene_seed_trajectories_rviz.sh \
#     ~/IM2MPPI/results/full_lap_bag_20260528_080853 \
#     12,19,21,24,25,28,29
#
# Without a recorded bag, old CSV trajectories cannot be exactly synchronized
# with a newly launched Gazebo scene.  This script therefore restarts Gazebo per
# seed with OBS_BRANCH_SEED=<seed> and waits until CSV_T0_SIM_TIME before
# playing that seed.  That keeps the live dynamic obstacles seed-consistent.

set -u

DEFAULT_RESULTS="${HOME}/IM2MPPI/results/full_lap_bag_20260528_080853"
REQUESTED_RESULTS_DIR="${1:-${RESULTS_DIR:-${DEFAULT_RESULTS}}}"
SEEDS="${2:-${SEEDS:-12,19,21,24,25,28,29}}"

PLAYBACK_SPEED="${PLAYBACK_SPEED:-1.0}"
PLAYBACK_RATE="${PLAYBACK_RATE:-20.0}"
PAUSE_BETWEEN_SEEDS="${PAUSE_BETWEEN_SEEDS:-10.0}"
LOOP_PLAYBACK="${LOOP_PLAYBACK:-false}"
REPLAY_EXIT_DELAY="${REPLAY_EXIT_DELAY:-0.5}"
SAVE_RVIZ_SCREENSHOTS="${SAVE_RVIZ_SCREENSHOTS:-true}"
RVIZ_SCREENSHOT_DIR="${RVIZ_SCREENSHOT_DIR:-}"
RVIZ_SAVE_IMAGE_SERVICE="${RVIZ_SAVE_IMAGE_SERVICE:-/rviz/save_image}"
RVIZ_SCREENSHOT_DELAY="${RVIZ_SCREENSHOT_DELAY:-0.5}"
SCREENSHOT_ONCE_PER_SEED="${SCREENSHOT_ONCE_PER_SEED:-true}"
SHOW_TRAJECTORY_LABELS="${SHOW_TRAJECTORY_LABELS:-false}"

GAZEBO_GUI="${GAZEBO_GUI:-true}"
WORLD_FILE="${WORLD_FILE:-$(rospack find uav_simulator)/worlds/generated_env/generated_env.world}"
RVIZ_STARTUP_WAIT="${RVIZ_STARTUP_WAIT:-5}"
TOPIC_WAIT_TIMEOUT="${TOPIC_WAIT_TIMEOUT:-45}"
CSV_T0_SIM_TIME="${CSV_T0_SIM_TIME:-12.0}"
GT_COLOR_DISTANCE="${GT_COLOR_DISTANCE:-}"
SHOW_INFLATED_DYNAMIC_BBOX="${SHOW_INFLATED_DYNAMIC_BBOX:-false}"
INFLATED_BBOX_INPUT_TOPIC="${INFLATED_BBOX_INPUT_TOPIC:-/onboard_detector/GT_obstacle_bbox}"
INFLATED_BBOX_OUTPUT_TOPIC="${INFLATED_BBOX_OUTPUT_TOPIC:-/onboard_detector/GT_obstacle_bbox_inflated}"
INFLATED_BBOX_LINE_WIDTH="${INFLATED_BBOX_LINE_WIDTH:-0.08}"
INFLATED_BBOX_EXTRA_MARGIN_XY="${INFLATED_BBOX_EXTRA_MARGIN_XY:-0.0}"
INFLATED_BBOX_EXTRA_MARGIN_Z="${INFLATED_BBOX_EXTRA_MARGIN_Z:-0.0}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
RVIZ_CONFIG="$(rospack find autonomous_flight)/cfg/im2_mppi_navigation.rviz"
REPLAY_NODE="${SCRIPT_DIR}/visualize_seed_trajectories_rviz.py"
INFLATED_BBOX_NODE="${SCRIPT_DIR}/publish_inflated_dynamic_bboxes.py"

has_timeseries_csv() {
    compgen -G "$1/*_timeseries.csv" >/dev/null 2>&1
}

normalize_results_dir() {
    local dir="$1"
    local base
    base="$(basename "${dir}")"

    if [[ ! -d "${dir}" ]]; then
        return 1
    fi

    if has_timeseries_csv "${dir}"; then
        printf '%s\n' "${dir}"
        return 0
    fi

    if [[ -d "${dir}/${base}" ]] && has_timeseries_csv "${dir}/${base}"; then
        printf '%s\n' "${dir}/${base}"
        return 0
    fi

    printf '%s\n' "${dir}"
    return 0
}

resolve_results_dir() {
    local requested="$1"
    local base
    local candidate
    local resolved

    base="$(basename "${requested}")"
    if resolved="$(normalize_results_dir "${requested}")"; then
        printf '%s\n' "${resolved}"
        return 0
    fi

    local candidates=(
        "${HOME}/IM2MPPI/results/${base}"
        "${HOME}/MOTIONPLANNING/IM2MPPI/results/${base}"
        "${HOME}/MOTIONPLANNING/IM2MPPI/src/IM2MPPI/results/${base}"
        "/mnt/c/Users/WIN11/IM2MPPI/analysis_results/${base}"
        "/mnt/c/Users/WIN11/IM2MPPI/analysis_results/${base}/${base}"
    )

    for candidate in "${candidates[@]}"; do
        if resolved="$(normalize_results_dir "${candidate}")"; then
            printf '%s\n' "${resolved}"
            return 0
        fi
    done

    while IFS= read -r candidate; do
        if resolved="$(normalize_results_dir "${candidate}")"; then
            printf '%s\n' "${resolved}"
            return 0
        fi
    done < <(find "${HOME}" -maxdepth 5 -type d -name "${base}" 2>/dev/null | head -n 20)

    return 1
}

wait_for_ros_master() {
    local timeout_s="$1"
    local start
    local current
    start="$(date +%s)"
    until rostopic list >/dev/null 2>&1; do
        current="$(date +%s)"
        if (( current - start >= timeout_s )); then
            echo "[real-scene-rviz] ROS master did not become ready within ${timeout_s}s" >&2
            return 1
        fi
        sleep 1
    done
    return 0
}

wait_topic_optional() {
    local topic="$1"
    local timeout_s="${2:-${TOPIC_WAIT_TIMEOUT}}"

    echo "[real-scene-rviz] waiting for ${topic} (${timeout_s}s max)"
    if timeout "${timeout_s}" bash -c "until rostopic echo -n 1 '${topic}' >/dev/null 2>&1; do sleep 1; done"; then
        echo "[real-scene-rviz] topic ready: ${topic}"
        return 0
    fi

    echo "[real-scene-rviz] warning: no message received from ${topic}" >&2
    return 1
}

wait_sim_time() {
    local target_time="$1"

    echo "[real-scene-rviz] waiting until /clock >= ${target_time}s"
    python3 - "${target_time}" <<'PY'
import sys
import rospy
from rosgraph_msgs.msg import Clock

target = float(sys.argv[1])
rospy.init_node("wait_for_csv_t0_sim_time", anonymous=True, disable_signals=True)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    try:
        msg = rospy.wait_for_message("/clock", Clock, timeout=1.0)
    except Exception:
        rate.sleep()
        continue
    if msg.clock.to_sec() >= target:
        break
    rate.sleep()
PY
}

parse_seed_list() {
    local raw="$1"
    local parts
    local seed
    IFS=',' read -r -a parts <<< "${raw}"
    for seed in "${parts[@]}"; do
        seed="${seed//[[:space:]]/}"
        if [[ -n "${seed}" ]]; then
            printf '%s\n' "${seed}"
        fi
    done
}

if ! RESULTS_DIR="$(resolve_results_dir "${REQUESTED_RESULTS_DIR}")"; then
    echo "[real-scene-rviz] results directory not found: ${REQUESTED_RESULTS_DIR}" >&2
    echo "[real-scene-rviz] pass the directory explicitly as the first argument." >&2
    exit 1
fi

if [[ -z "${RVIZ_SCREENSHOT_DIR}" ]]; then
    RVIZ_SCREENSHOT_DIR="${RESULTS_DIR}/rviz_real_scene_screenshots"
fi

LOG_DIR="${RESULTS_DIR}/rviz_real_scene_logs"
mkdir -p "${LOG_DIR}" "${RVIZ_SCREENSHOT_DIR}"

ROSCORE_PID=""
SIM_PID=""
MAP_PID=""
FAKE_PID=""
INFLATED_PID=""
RVIZ_PID=""
REPLAY_PID=""

stop_pid() {
    local pid="$1"
    if [[ -n "${pid}" ]]; then
        kill "${pid}" >/dev/null 2>&1 || true
        wait "${pid}" >/dev/null 2>&1 || true
    fi
}

stop_scene() {
    stop_pid "${REPLAY_PID}"
    stop_pid "${INFLATED_PID}"
    stop_pid "${FAKE_PID}"
    stop_pid "${MAP_PID}"
    stop_pid "${SIM_PID}"
    REPLAY_PID=""
    INFLATED_PID=""
    FAKE_PID=""
    MAP_PID=""
    SIM_PID=""
}

cleanup() {
    stop_scene
    for pid in "${RVIZ_PID}" "${ROSCORE_PID}"; do
        if [[ -n "${pid}" ]]; then
            kill "${pid}" >/dev/null 2>&1 || true
            wait "${pid}" >/dev/null 2>&1 || true
        fi
    done
}
trap cleanup EXIT INT TERM

echo "[real-scene-rviz] results  : ${RESULTS_DIR}"
echo "[real-scene-rviz] seeds    : ${SEEDS}"
echo "[real-scene-rviz] world    : ${WORLD_FILE}"
echo "[real-scene-rviz] rviz     : ${RVIZ_CONFIG}"
echo "[real-scene-rviz] logs     : ${LOG_DIR}"
echo "[real-scene-rviz] shots    : ${RVIZ_SCREENSHOT_DIR}"
echo "[real-scene-rviz] dynamic  : /onboard_detector/GT_obstacle_bbox (planning-time fakeDetector bbox style)"
if [[ "${SHOW_INFLATED_DYNAMIC_BBOX}" == "true" || "${SHOW_INFLATED_DYNAMIC_BBOX}" == "1" ]]; then
    echo "[real-scene-rviz] inflated : ${INFLATED_BBOX_OUTPUT_TOPIC}"
else
    echo "[real-scene-rviz] inflated : disabled"
fi
echo "[real-scene-rviz] static   : /dynamic_map/inflated_voxel_map"
echo "[real-scene-rviz] csv t0   : Gazebo /clock ${CSV_T0_SIM_TIME}s"

if ! rostopic list >/dev/null 2>&1; then
    echo "[real-scene-rviz] starting roscore"
    roscore >"${LOG_DIR}/roscore.log" 2>&1 &
    ROSCORE_PID=$!
    wait_for_ros_master 30 || exit 1
fi

echo "[real-scene-rviz] opening RViz"
rviz -d "${RVIZ_CONFIG}" >"${LOG_DIR}/rviz.log" 2>&1 &
RVIZ_PID=$!
sleep "${RVIZ_STARTUP_WAIT}"

mapfile -t SEED_LIST < <(parse_seed_list "${SEEDS}")
if [[ "${#SEED_LIST[@]}" -eq 0 ]]; then
    echo "[real-scene-rviz] no valid seeds requested: ${SEEDS}" >&2
    exit 1
fi

for idx in "${!SEED_LIST[@]}"; do
    seed="${SEED_LIST[$idx]}"
    echo
    echo "================================================================"
    echo "[real-scene-rviz] seed ${seed}: starting seed-synchronized Gazebo scene"

    OBS_BRANCH_SEED="${seed}" \
    roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" world_name:="${WORLD_FILE}" \
        >"${LOG_DIR}/seed${seed}_gazebo_start.log" 2>&1 &
    SIM_PID=$!

    wait_topic_optional "/gazebo/model_states" 60 || true

    echo "[real-scene-rviz] seed ${seed}: loading map/detector parameters"
    rosparam load "$(rospack find autonomous_flight)/cfg/mpc_navigation/mapping_param.yaml" /dynamic_map
    rosparam load "$(rospack find autonomous_flight)/cfg/mpc_navigation/dynamic_detector_param.yaml" /onboard_detector
    rosparam load "$(rospack find autonomous_flight)/cfg/mpc_navigation/fake_detector_param.yaml"
    rosparam set odom_topic "/CERLAB/quadcopter/odom"
    if [[ -n "${GT_COLOR_DISTANCE}" ]]; then
        rosparam set color_distance "${GT_COLOR_DISTANCE}"
    fi

    echo "[real-scene-rviz] seed ${seed}: starting dynamic map node"
    rosrun map_manager dynamic_map_node >"${LOG_DIR}/seed${seed}_dynamic_map_node.log" 2>&1 &
    MAP_PID=$!

    echo "[real-scene-rviz] seed ${seed}: starting fake detector node"
    rosrun onboard_detector fake_detector_node >"${LOG_DIR}/seed${seed}_fake_detector_node.log" 2>&1 &
    FAKE_PID=$!

    if [[ "${SHOW_INFLATED_DYNAMIC_BBOX}" == "true" || "${SHOW_INFLATED_DYNAMIC_BBOX}" == "1" ]]; then
        echo "[real-scene-rviz] seed ${seed}: starting inflated dynamic bbox visualizer"
        python3 "${INFLATED_BBOX_NODE}" \
            _input_topic:="${INFLATED_BBOX_INPUT_TOPIC}" \
            _output_topic:="${INFLATED_BBOX_OUTPUT_TOPIC}" \
            _line_width:="${INFLATED_BBOX_LINE_WIDTH}" \
            _extra_margin_xy:="${INFLATED_BBOX_EXTRA_MARGIN_XY}" \
            _extra_margin_z:="${INFLATED_BBOX_EXTRA_MARGIN_Z}" \
            >"${LOG_DIR}/seed${seed}_inflated_dynamic_bbox_visualizer.log" 2>&1 &
        INFLATED_PID=$!
    fi

    wait_topic_optional "/dynamic_map/inflated_voxel_map" "${TOPIC_WAIT_TIMEOUT}" || true
    wait_topic_optional "/onboard_detector/GT_obstacle_bbox" "${TOPIC_WAIT_TIMEOUT}" || true
    if [[ -n "${INFLATED_PID}" ]]; then
        wait_topic_optional "${INFLATED_BBOX_OUTPUT_TOPIC}" "${TOPIC_WAIT_TIMEOUT}" || true
    fi

    wait_sim_time "${CSV_T0_SIM_TIME}"

    echo "[real-scene-rviz] seed ${seed}: starting saved trajectory playback"
    python3 "${REPLAY_NODE}" \
        _results_dir:="${RESULTS_DIR}" \
        _seeds:="${seed}" \
        _playback_speed:="${PLAYBACK_SPEED}" \
        _rate:="${PLAYBACK_RATE}" \
        _pause_between_seeds:="${REPLAY_EXIT_DELAY}" \
        _loop:="${LOOP_PLAYBACK}" \
        _replay_bags:=false \
        _use_world_obstacles:=false \
        _save_rviz_screenshots:="${SAVE_RVIZ_SCREENSHOTS}" \
        _screenshot_dir:="${RVIZ_SCREENSHOT_DIR}" \
        _rviz_save_image_service:="${RVIZ_SAVE_IMAGE_SERVICE}" \
        _screenshot_delay:="${RVIZ_SCREENSHOT_DELAY}" \
        _screenshot_once_per_seed:="${SCREENSHOT_ONCE_PER_SEED}" \
        _show_labels:="${SHOW_TRAJECTORY_LABELS}" &
    REPLAY_PID=$!

    wait "${REPLAY_PID}" || true
    REPLAY_PID=""

    echo "[real-scene-rviz] seed ${seed}: playback finished; stopping scene"
    stop_scene

    if [[ "$idx" -lt "$((${#SEED_LIST[@]} - 1))" ]]; then
        echo "[real-scene-rviz] waiting ${PAUSE_BETWEEN_SEEDS}s before next seed"
        sleep "${PAUSE_BETWEEN_SEEDS}"
    fi
done

echo "[real-scene-rviz] all requested seed playbacks finished"
