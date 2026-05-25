#!/usr/bin/env bash
# ============================================================================
#  run_intent_mpc_latency_experiments.sh
# ----------------------------------------------------------------------------
#  Single-method batch driver for Intent-MPC latency measurement.
#
#  It runs only:
#    M0_intent_mpc = autonomous_flight intent_mpc_demo.launch
#
#  The simulator world is start.launch's default map:
#    roslaunch uav_simulator start.launch
#
#  The evaluator stops after one predefined lap, or when DURATION_SEC expires.
#  Planning latency is read from:
#    /mpcNavigation/plan_time_ms
#
#  Usage:
#    bash run_intent_mpc_latency_experiments.sh [RUNS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    bash run_intent_mpc_latency_experiments.sh
#    bash run_intent_mpc_latency_experiments.sh 10 90 0.8
#    SEED_START=21 bash run_intent_mpc_latency_experiments.sh 10 120 0.8
#
#  Optional environment overrides:
#    OUT_DIR_OVERRIDE=/home/user/IM2MPPI/results/intent_mpc_latency_resume
#    SEED_START=1
#    GAZEBO_GUI=true
#    INTENT_MPC_LAUNCH="autonomous_flight intent_mpc_demo.launch"
#    WORLD_FILE=/abs/path/to/world.world
# ============================================================================

set -u

RUNS=${1:-1}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}
SEED_START=${SEED_START:-1}
GAZEBO_GUI="${GAZEBO_GUI:-true}"
WORLD_FILE="${WORLD_FILE:-}"
INTENT_MPC_LAUNCH="${INTENT_MPC_LAUNCH:-autonomous_flight intent_mpc_demo.launch}"
PLAN_TIME_TOPIC="${PLAN_TIME_TOPIC:-/mpcNavigation/plan_time_ms}"

if [[ -n "${OUT_DIR_OVERRIDE:-}" ]]; then
    OUT_DIR="${OUT_DIR_OVERRIDE}"
    STAMP="$(basename "${OUT_DIR}")"
    echo "[resume] using existing OUT_DIR = ${OUT_DIR}"
else
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_DIR="${HOME}/IM2MPPI/results/intent_mpc_latency_${STAMP}"
fi

LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"

cleanup_all() {
    for SIG in 15 9; do
        pkill -${SIG} -f evaluate_intent_mpc_im2mppi 2>/dev/null || true
        pkill -${SIG} -f mpc_navigation_node        2>/dev/null || true
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

    if [[ -d "${HOME}/.ros/log" ]]; then
        find "${HOME}/.ros/log" -mindepth 1 -maxdepth 1 -type d \
            | sort | head -n -3 | xargs -r rm -rf 2>/dev/null || true
    fi

    sleep 5
}

restore_and_exit() {
    trap - EXIT INT TERM
    cleanup_all
    exit
}

trap restore_and_exit EXIT INT TERM

archive_one() {
    local tmp_out=$1
    local tag=$2
    local eval_algo="intent_mpc"

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

TOTAL="${RUNS}"
T_START=$(date +%s)
EST_SEC=$(( TOTAL * (DURATION + 40) ))
EST_HMS=$(printf '%02dh%02dm' $((EST_SEC/3600)) $(((EST_SEC%3600)/60)))
EST_END=$(date -d "+${EST_SEC} seconds" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || \
          date -v+${EST_SEC}S '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo "?")

echo
echo "================================================================"
echo "  Intent-MPC latency batch driver"
echo "  method: M0_intent_mpc"
echo "  world:  ${WORLD_FILE:-<start.launch default>}"
echo "  launch: ${INTENT_MPC_LAUNCH}"
echo "  plan latency topic: ${PLAN_TIME_TOPIC}"
echo "  runs: ${RUNS}  seed_start: ${SEED_START}"
echo "  timeout/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  gazebo gui: ${GAZEBO_GUI}"
echo "  estimated total wall clock: ${EST_HMS}  (done around ${EST_END})"
echo "  results: ${OUT_DIR}"
echo "================================================================"

for IDX in $(seq 0 $((RUNS - 1))); do
    SEED=$((SEED_START + IDX))
    TAG="M0_intent_mpc_seed${SEED}"

    if [[ -f "${OUT_DIR}/${TAG}_summary.json" ]]; then
        echo "[skip] ${TAG} already exists"
        continue
    fi

    ELAPSED=$(( $(date +%s) - T_START ))
    REMAINING=$(( (RUNS - IDX) * (DURATION + 40) ))

    echo
    echo "================================================================"
    echo "[$(date +%T)] ${TAG}  run=$((IDX + 1))/${RUNS}  elapsed=${ELAPSED}s  est_remaining=${REMAINING}s"
    echo "================================================================"

    cleanup_all

    roscore > "${LOG_DIR}/${TAG}_roscore.log" 2>&1 &
    sleep 3

    SIM_EXTRA=""
    if [[ -n "${WORLD_FILE}" ]]; then
        SIM_EXTRA="world_name:=${WORLD_FILE}"
    fi
    OBS_BRANCH_SEED="${SEED}" \
    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" ${SIM_EXTRA} \
        > "${LOG_DIR}/${TAG}_sim.log" 2>&1 &
    sleep 10

    pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
    pkill -9 -f keyboard_control      2>/dev/null || true
    pkill -9 -f key_teleop            2>/dev/null || true
    pkill -9 -f keyboardCtrl          2>/dev/null || true

    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch ${INTENT_MPC_LAUNCH} \
        > "${LOG_DIR}/${TAG}_stack.log" 2>&1 &
    sleep 12

    TMP_OUT="${OUT_DIR}/_tmp_${TAG}"
    mkdir -p "${TMP_OUT}"

    timeout --kill-after=10 $((DURATION + 30)) \
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="intent_mpc" \
            duration:="${DURATION}" \
            completion_mode:="lap" \
            shutdown_on_success:="true" \
            goal_radius:="${GOAL_RADIUS}" \
            lap_completion_radius:="${GOAL_RADIUS}" \
            plan_time_topic:="${PLAN_TIME_TOPIC}" \
            output_dir:="${TMP_OUT}" \
        > "${LOG_DIR}/${TAG}_eval.log" 2>&1 || true

    archive_one "${TMP_OUT}" "${TAG}"
    rm -rf "${TMP_OUT}"

    cleanup_all
done

echo
echo "================================================================"
echo "  Aggregating Intent-MPC latency outputs..."
echo "================================================================"

if compgen -G "${OUT_DIR}/*_summary.json" > /dev/null; then
    python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
        "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json \
        > "${LOG_DIR}/aggregate.log" 2>&1 || true
    echo "  wrote ${OUT_DIR}/compare_all.csv"
    python3 "${SCRIPT_DIR}/print_summary_table.py" "${OUT_DIR}" || true
    python3 - "${OUT_DIR}/compare_all.csv" <<'PY'
import csv
import math
import sys

path = sys.argv[1]
vals = {"plan_latency_mean_ms": [], "plan_latency_p95_ms": [], "plan_latency_max_ms": []}
with open(path, newline="") as f:
    for row in csv.DictReader(f):
        for k in vals:
            s = row.get(k, "")
            try:
                v = float(s)
            except ValueError:
                continue
            if math.isfinite(v):
                vals[k].append(v)

def avg(xs):
    return sum(xs) / len(xs) if xs else float("nan")

print()
print("Intent-MPC planning latency over completed evaluator files:")
print("  Lat_avg = %.3f ms" % avg(vals["plan_latency_mean_ms"]))
print("  Lat_p95 = %.3f ms" % avg(vals["plan_latency_p95_ms"]))
print("  Lat_max = %.3f ms" % avg(vals["plan_latency_max_ms"]))
PY
else
    echo "  no summaries produced"
fi

echo
echo "================================================================"
echo "  DONE. Results dir: ${OUT_DIR}"
echo "================================================================"
