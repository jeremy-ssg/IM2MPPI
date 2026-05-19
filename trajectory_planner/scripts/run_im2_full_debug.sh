#!/usr/bin/env bash
# ============================================================================
#  run_im2_full_debug.sh
# ----------------------------------------------------------------------------
#  Debug-only driver for the full IM2-MPPI method.
#
#  It runs only:
#    M4_im2_full = cvar_mppi + adaptive fusion + closed-loop intent
#
#  Outputs are saved under:
#    ${HOME}/IM2MPPI/results/im2_full_debug_<stamp>/
#
#  Usage:
#    ./run_im2_full_debug.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_im2_full_debug.sh          # 1 seed, 90 s
#    ./run_im2_full_debug.sh 3 90     # 3 seeds, 90 s each
#    ./run_im2_full_debug.sh 1 45     # quick crash reproduction
# ============================================================================

set -u

SEEDS=${1:-1}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}

if [[ -n "${OUT_DIR_OVERRIDE:-}" ]]; then
    OUT_DIR="${OUT_DIR_OVERRIDE}"
    STAMP="$(basename "${OUT_DIR}")"
    echo "[resume] using existing OUT_DIR = ${OUT_DIR}"
else
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_DIR="${HOME}/IM2MPPI/results/im2_full_debug_${STAMP}"
fi

LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"

cleanup_all() {
    for SIG in 15 9; do
        pkill -${SIG} -f evaluate_intent_mpc_im2mppi 2>/dev/null || true
        pkill -${SIG} -f im2_mppi_navigation_node    2>/dev/null || true
        pkill -${SIG} -f mpcNavigation              2>/dev/null || true
        pkill -${SIG} -f tracking_controller_node   2>/dev/null || true
        pkill -${SIG} -f onboard_detector           2>/dev/null || true
        pkill -${SIG} -f dynamic_predictor          2>/dev/null || true
        pkill -${SIG} -f teleop_twist_keyboard      2>/dev/null || true
        pkill -${SIG} -f keyboard_control           2>/dev/null || true
        pkill -${SIG} -f key_teleop                 2>/dev/null || true
        pkill -${SIG} -f keyboardCtrl               2>/dev/null || true
        pkill -${SIG} -f gzclient                   2>/dev/null || true
        pkill -${SIG} -f gzserver                   2>/dev/null || true
        pkill -${SIG} -f gazebo                     2>/dev/null || true
        pkill -${SIG} -f rviz                       2>/dev/null || true
        pkill -${SIG} -f roslaunch                  2>/dev/null || true
        pkill -${SIG} -f rosmaster                  2>/dev/null || true
        pkill -${SIG} -f rosout                     2>/dev/null || true
        [[ "${SIG}" == "15" ]] && sleep 3
    done
    sleep 5
}

restore_yaml_and_exit() {
    trap - EXIT INT TERM
    echo "[debug] restoring yaml"
    if [[ -f "${YAML_PLANNER}.debugbak" ]]; then
        cp "${YAML_PLANNER}.debugbak" "${YAML_PLANNER}"
        rm -f "${YAML_PLANNER}.debugbak"
    fi
    cleanup_all
    exit
}

cp "${YAML_PLANNER}" "${YAML_PLANNER}.debugbak"
trap restore_yaml_and_exit EXIT INT TERM

set_yaml_str() {
    local key=$1
    local val=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 \"${val}\"|g" "${YAML_PLANNER}"
}

set_yaml_num() {
    local key=$1
    local val=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 ${val}|g" "${YAML_PLANNER}"
}

archive_one() {
    local tmp_out=$1
    local tag=$2
    local eval_algo="im2_mppi"

    if [[ -f "${tmp_out}/${eval_algo}_summary.json" ]]; then
        mv "${tmp_out}/${eval_algo}_summary.json" "${OUT_DIR}/${tag}_summary.json"
        mv "${tmp_out}/${eval_algo}_timeseries.csv" "${OUT_DIR}/${tag}_timeseries.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_path_metrics.csv" "${OUT_DIR}/${tag}_path_metrics.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_plan_time.csv" "${OUT_DIR}/${tag}_plan_time.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_plan_time_timeline.csv" "${OUT_DIR}/${tag}_plan_time_timeline.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_target_state.csv" "${OUT_DIR}/${tag}_target_state.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_cmd_accel.csv" "${OUT_DIR}/${tag}_cmd_accel.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_collision_events.csv" "${OUT_DIR}/${tag}_collision_events.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_diagnostics.json" "${OUT_DIR}/${tag}_diagnostics.json" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_diagnostic_events.csv" "${OUT_DIR}/${tag}_diagnostic_events.csv" 2>/dev/null || true
        echo "    [OK] -> ${tag}_summary.json"
    else
        echo "    [FAIL] no summary produced; see ${LOG_DIR}/${tag}_*.log"
    fi
}

write_diagnostics_index() {
    python3 - "${OUT_DIR}" <<'PY'
import csv
import glob
import json
import os
import sys

out_dir = sys.argv[1]
rows = []
for path in sorted(glob.glob(os.path.join(out_dir, "M4_im2_full_seed*_summary.json"))):
    tag = os.path.basename(path).replace("_summary.json", "")
    with open(path) as f:
        summary = json.load(f)
    diag = summary.get("diagnostics", {})
    planner = summary.get("planner", {})
    tracking = summary.get("tracking", {})
    safety = summary.get("safety", {})
    latency = diag.get("plan_latency", {})
    target = diag.get("target_stream", {})
    command = diag.get("target_command", {})
    rows.append({
        "tag": tag,
        "verdict": diag.get("verdict"),
        "lat_avg_ms": planner.get("plan_latency_mean_ms"),
        "lat_p95_ms": planner.get("plan_latency_p95_ms"),
        "lat_max_ms": planner.get("plan_latency_max_ms"),
        "lat_gt_50ms": latency.get("count_gt_50ms"),
        "lat_gt_100ms": latency.get("count_gt_100ms"),
        "target_gap_max_s": target.get("max_gap_s"),
        "target_jump_max_m": command.get("max_position_jump_m"),
        "target_jump_gt_1m": command.get("count_position_jump_gt_1m"),
        "track_rms_m": tracking.get("target_tracking_rms_m"),
        "track_max_m": tracking.get("target_tracking_max_m"),
        "max_cmd_accel_mps2": command.get("max_cmd_accel_mps2"),
        "max_cmd_jerk_mps3": command.get("max_cmd_jerk_mps3"),
        "min_clearance_m": safety.get("min_clearance_m"),
        "collision_strict_events": safety.get("collision_strict_events"),
        "collision_near_miss_events": safety.get("collision_near_miss_events"),
    })

index_path = os.path.join(out_dir, "diagnostics_index.csv")
fields = [
    "tag", "verdict",
    "lat_avg_ms", "lat_p95_ms", "lat_max_ms", "lat_gt_50ms", "lat_gt_100ms",
    "target_gap_max_s", "target_jump_max_m", "target_jump_gt_1m",
    "track_rms_m", "track_max_m",
    "max_cmd_accel_mps2", "max_cmd_jerk_mps3",
    "min_clearance_m", "collision_strict_events", "collision_near_miss_events",
]
with open(index_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)

print(index_path)
PY
}

echo
echo "================================================================"
echo "  IM2 full debug runner"
echo "  method: cvar_mppi  fusion: adaptive  closed_loop: true"
echo "  seeds: ${SEEDS}  duration/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  results: ${OUT_DIR}"
echo "================================================================"

for SEED in $(seq 1 "${SEEDS}"); do
    TAG="M4_im2_full_seed${SEED}"

    if [[ -f "${OUT_DIR}/${TAG}_summary.json" ]]; then
        echo "[skip] ${TAG} already exists"
        continue
    fi

    echo
    echo "================================================================"
    echo "[$(date +%T)] ${TAG}"
    echo "================================================================"

    cleanup_all

    set_yaml_str "method_type" "cvar_mppi"
    set_yaml_str "fusion_mode" "adaptive"
    set_yaml_num "random_seed" "${SEED}"

    roscore > "${LOG_DIR}/${TAG}_roscore.log" 2>&1 &
    sleep 3

    rosparam set /autonomous_flight/closed_loop_intent_enabled true >/dev/null 2>&1 || true

    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch uav_simulator start.launch \
        > "${LOG_DIR}/${TAG}_sim.log" 2>&1 &
    sleep 10

    pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
    pkill -9 -f keyboard_control      2>/dev/null || true
    pkill -9 -f key_teleop            2>/dev/null || true
    pkill -9 -f keyboardCtrl          2>/dev/null || true

    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch autonomous_flight im2_mppi_demo.launch \
        > "${LOG_DIR}/${TAG}_stack.log" 2>&1 &
    sleep 12

    TMP_OUT="${OUT_DIR}/_tmp_${TAG}"
    mkdir -p "${TMP_OUT}"

    timeout --kill-after=10 $((DURATION + 30)) \
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="im2_mppi" \
            duration:="${DURATION}" \
            goal_radius:="${GOAL_RADIUS}" \
            output_dir:="${TMP_OUT}" \
        > "${LOG_DIR}/${TAG}_eval.log" 2>&1 || true

    archive_one "${TMP_OUT}" "${TAG}"
    rm -rf "${TMP_OUT}"

    cleanup_all
done

echo
echo "================================================================"
echo "  Aggregating IM2 full debug outputs..."
echo "================================================================"

if compgen -G "${OUT_DIR}/*_summary.json" > /dev/null; then
    python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
        "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json \
        > "${LOG_DIR}/aggregate.log" 2>&1 || true
    python3 "${SCRIPT_DIR}/print_summary_table.py" "${OUT_DIR}" || true
    INDEX_PATH="$(write_diagnostics_index)"
    echo "  diagnostics index: ${INDEX_PATH}"
else
    echo "  no summaries produced"
fi

echo
echo "================================================================"
echo "  DONE. Results dir: ${OUT_DIR}"
echo "================================================================"
