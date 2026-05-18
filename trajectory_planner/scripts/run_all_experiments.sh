#!/usr/bin/env bash
# ============================================================================
#  run_all_experiments.sh
# ----------------------------------------------------------------------------
#  One-shot driver for the IM2-MPPI comparison + ablation suite.
#
#  For each (config, seed):
#    1. Patches im2_mppi.yaml (method_type / fusion_mode / random_seed)
#    2. Brings up its own roscore + presets autonomous_flight rosparams
#       (closed_loop_intent_enabled)
#    3. Launches the simulator + the right planner stack
#    4. Launches the evaluator, blocking until DURATION seconds elapse
#    5. Archives the summary JSON tagged <CONFIG>_seed<N>_summary.json
#    6. Aggressively tears everything down before the next run
#
#  After all runs, aggregates the JSONs via compare_planner_eval.py and
#  prints a per-config mean ± std table via print_summary_table.py.
#
#  Usage:
#    ./run_all_experiments.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#    SEEDS         — number of random seeds per config (default 24 = 3 × 8)
#    DURATION_SEC  — evaluator window per run (default 90)
#    GOAL_RADIUS   — success threshold in metres (default 0.8)
#
#  Examples:
#    ./run_all_experiments.sh             # default (24 seeds × 8 configs, ~7 h)
#    ./run_all_experiments.sh 3 90        # quick sanity sweep, ~25 min
#    ./run_all_experiments.sh 10 120      # mid-size paper run, ~4 h
#
#  Total wall clock ≈ SEEDS × 8 configs × (DURATION + 40 s overhead).
#
#  Per-run raw artefacts saved under results/<stamp>/:
#    <CFG>_seed<N>_summary.json         — aggregated metrics
#    <CFG>_seed<N>_timeseries.csv       — odom + clearance + target_error
#    <CFG>_seed<N>_path_metrics.csv     — per-planned-trajectory stats
#    <CFG>_seed<N>_plan_time.csv        — per-tick planning latency in ms
#    <CFG>_seed<N>_cmd_accel.csv        — commanded acceleration time series
#    <CFG>_seed<N>_collision_events.csv — rising-edge collision event log
#    logs/<CFG>_seed<N>_{sim,stack,eval,roscore}.log — stdout / stderr
# ============================================================================

set -u

SEEDS=${1:-24}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${HOME}/IM2MPPI/results/${STAMP}"
LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"

# Back up the planner yaml so we always restore it on exit, even on Ctrl+C.
cp "${YAML_PLANNER}" "${YAML_PLANNER}.batchbak"
trap 'echo "[batch] restoring yaml"; cp "${YAML_PLANNER}.batchbak" "${YAML_PLANNER}"; rm -f "${YAML_PLANNER}.batchbak"; cleanup_all; exit' EXIT INT TERM

# ────────────────────────────────────────────────────────────────────────────
#  Config table: name | launch | method_type | fusion_mode | closed_loop
#    "N/A" entries are skipped (Intent-MPC has no MPPI yaml).
# ────────────────────────────────────────────────────────────────────────────
CONFIGS=(
  "M0_intent_mpc|autonomous_flight intent_mpc_demo.launch|N/A|N/A|N/A"
  "M1_vanilla|autonomous_flight im2_mppi_demo.launch|vanilla_mppi|soft|true"
  "M2_mean_pred|autonomous_flight im2_mppi_demo.launch|mean_prediction_mppi|soft|true"
  "M3_mode_aware|autonomous_flight im2_mppi_demo.launch|mode_aware_mppi|soft|false"
  "M4_im2_full|autonomous_flight im2_mppi_demo.launch|cvar_mppi|adaptive|true"
  "A1_no_cvar|autonomous_flight im2_mppi_demo.launch|mode_aware_mppi|adaptive|true"
  "A2_no_fusion|autonomous_flight im2_mppi_demo.launch|cvar_mppi|soft|true"
  "A3_no_cl|autonomous_flight im2_mppi_demo.launch|cvar_mppi|adaptive|false"
)

# ────────────────────────────────────────────────────────────────────────────
#  Helpers
# ────────────────────────────────────────────────────────────────────────────

cleanup_all() {
    # Hard-kill in the order that minimizes "still attached to dead master" errors.
    pkill -9 -f evaluate_intent_mpc_im2mppi 2>/dev/null || true
    pkill -9 -f im2_mppi_navigation_node    2>/dev/null || true
    pkill -9 -f mpcNavigation              2>/dev/null || true
    pkill -9 -f tracking_controller_node   2>/dev/null || true
    pkill -9 -f onboard_detector           2>/dev/null || true
    pkill -9 -f dynamic_predictor          2>/dev/null || true
    pkill -9 -f gzclient                   2>/dev/null || true
    pkill -9 -f gzserver                   2>/dev/null || true
    pkill -9 -f gazebo                     2>/dev/null || true
    pkill -9 -f rviz                       2>/dev/null || true
    pkill -9 -f roslaunch                  2>/dev/null || true
    pkill -9 -f rosmaster                  2>/dev/null || true
    pkill -9 -f rosout                     2>/dev/null || true
    sleep 4
}

set_yaml_str() {
    local key=$1; local val=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 \"${val}\"|g" "${YAML_PLANNER}"
}

set_yaml_num() {
    local key=$1; local val=$2
    sed -i -E "s|^([[:space:]]*${key}:).*$|\1 ${val}|g" "${YAML_PLANNER}"
}

run_one() {
    local NAME=$1; local LAUNCH=$2; local METHOD=$3; local FUSION=$4; local CL=$5; local SEED=$6
    local TAG="${NAME}_seed${SEED}"

    echo
    echo "================================================================"
    echo "[$(date +%T)]  ${TAG}"
    echo "    method=${METHOD}  fusion=${FUSION}  CL=${CL}"
    echo "================================================================"

    cleanup_all

    # 1. Patch yaml only for IM2-MPPI variants.
    if [[ "${METHOD}" != "N/A" ]]; then
        set_yaml_str "method_type" "${METHOD}"
        set_yaml_str "fusion_mode" "${FUSION}"
        set_yaml_num "random_seed" "${SEED}"
    fi

    # 2. Bring up a dedicated roscore so we can preset rosparams.
    roscore > "${LOG_DIR}/${TAG}_roscore.log" 2>&1 &
    sleep 3

    # 3. Pre-set the closed-loop intent flag (autonomous_flight namespace).
    if [[ "${CL}" != "N/A" ]]; then
        rosparam set /autonomous_flight/closed_loop_intent_enabled "${CL}" \
            >/dev/null 2>&1 || true
    fi

    # 4. Launch the simulator.
    roslaunch uav_simulator start.launch \
        > "${LOG_DIR}/${TAG}_sim.log" 2>&1 &
    sleep 10

    # 5. Launch the planner stack (Intent-MPC or IM2-MPPI).
    roslaunch ${LAUNCH} \
        > "${LOG_DIR}/${TAG}_stack.log" 2>&1 &
    sleep 12   # wait for takeoff + planner init

    # 6. Choose evaluator algorithm tag.
    local EVAL_ALGO="im2_mppi"
    if [[ "${LAUNCH}" =~ intent_mpc ]]; then
        EVAL_ALGO="intent_mpc"
    fi

    # 7. Launch the evaluator (blocks for DURATION seconds, then exits).
    local TMP_OUT="${OUT_DIR}/_tmp_${TAG}"
    mkdir -p "${TMP_OUT}"
    roslaunch trajectory_planner evaluate_planner.launch \
        algorithm:="${EVAL_ALGO}" \
        duration:="${DURATION}" \
        goal_radius:="${GOAL_RADIUS}" \
        output_dir:="${TMP_OUT}" \
        > "${LOG_DIR}/${TAG}_eval.log" 2>&1 || true

    # 8. Archive results with seed-tagged names.
    if [[ -f "${TMP_OUT}/${EVAL_ALGO}_summary.json" ]]; then
        mv  "${TMP_OUT}/${EVAL_ALGO}_summary.json"     "${OUT_DIR}/${TAG}_summary.json"
        mv  "${TMP_OUT}/${EVAL_ALGO}_timeseries.csv"   "${OUT_DIR}/${TAG}_timeseries.csv"  2>/dev/null || true
        mv  "${TMP_OUT}/${EVAL_ALGO}_path_metrics.csv" "${OUT_DIR}/${TAG}_path_metrics.csv" 2>/dev/null || true
        mv  "${TMP_OUT}/${EVAL_ALGO}_plan_time.csv"    "${OUT_DIR}/${TAG}_plan_time.csv"   2>/dev/null || true
        echo "    [OK]  -> ${TAG}_summary.json"
    else
        echo "    [FAIL]  no summary produced (see ${LOG_DIR}/${TAG}_*.log)"
    fi
    rm -rf "${TMP_OUT}"

    cleanup_all
}

# ────────────────────────────────────────────────────────────────────────────
#  Main loop
# ────────────────────────────────────────────────────────────────────────────

TOTAL=$((${#CONFIGS[@]} * SEEDS))
COUNT=0
T_START=$(date +%s)

EST_SEC=$(( TOTAL * (DURATION + 40) ))
EST_HMS=$(printf '%02dh%02dm' $((EST_SEC/3600)) $(((EST_SEC%3600)/60)))
EST_END=$(date -d "+${EST_SEC} seconds" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || \
          date -v+${EST_SEC}S '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo "?")

echo
echo "================================================================"
echo "  IM2-MPPI batch driver"
echo "  configs: ${#CONFIGS[@]}   seeds/config: ${SEEDS}   total runs: ${TOTAL}"
echo "  duration/run: ${DURATION}s    goal_radius: ${GOAL_RADIUS} m"
echo "  estimated total wall clock: ${EST_HMS}  (≈ done ${EST_END})"
echo "  results:  ${OUT_DIR}"
echo "================================================================"

for CONFIG in "${CONFIGS[@]}"; do
    IFS='|' read -r NAME LAUNCH METHOD FUSION CL <<< "${CONFIG}"
    for SEED in $(seq 1 ${SEEDS}); do
        COUNT=$((COUNT + 1))
        ELAPSED=$(( $(date +%s) - T_START ))
        REMAINING=$(( (TOTAL - COUNT + 1) * (DURATION + 40) ))
        printf "\n>>> [%d/%d]  elapsed %ds  est. remaining %ds\n" \
            "${COUNT}" "${TOTAL}" "${ELAPSED}" "${REMAINING}"
        run_one "${NAME}" "${LAUNCH}" "${METHOD}" "${FUSION}" "${CL}" "${SEED}"
    done
done

echo
echo "================================================================"
echo "  All runs done. Aggregating..."
echo "================================================================"

# Aggregate every per-run summary into one CSV.
if compgen -G "${OUT_DIR}/*_summary.json" > /dev/null; then
    python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
        "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json \
        > "${LOG_DIR}/aggregate.log" 2>&1
    echo "  ✓ ${OUT_DIR}/compare_all.csv"
else
    echo "  ✗ no summaries to aggregate."
    exit 1
fi

# Per-config mean ± std summary table to stdout.
python3 "${SCRIPT_DIR}/print_summary_table.py" "${OUT_DIR}"

# Wilcoxon: only meaningful when N≥5 paired seeds across two algorithms.
if [[ "${SEEDS}" -ge 5 ]]; then
    echo
    echo "==== Wilcoxon: M4_im2_full  vs  M3_mode_aware ===="
    python3 "${SCRIPT_DIR}/stats_compare.py" \
        --baseline-glob  "${OUT_DIR}/M3_mode_aware_seed*_summary.json" \
        --treatment-glob "${OUT_DIR}/M4_im2_full_seed*_summary.json" \
        --baseline-name  "mode_aware" --treatment-name "im2_full" \
        --out            "${OUT_DIR}/stats_im2_vs_mode_aware.csv" \
        2>/dev/null || true

    echo
    echo "==== Wilcoxon: M4_im2_full  vs  M0_intent_mpc ===="
    python3 "${SCRIPT_DIR}/stats_compare.py" \
        --baseline-glob  "${OUT_DIR}/M0_intent_mpc_seed*_summary.json" \
        --treatment-glob "${OUT_DIR}/M4_im2_full_seed*_summary.json" \
        --baseline-name  "intent_mpc" --treatment-name "im2_full" \
        --out            "${OUT_DIR}/stats_im2_vs_intent_mpc.csv" \
        2>/dev/null || true
else
    echo
    echo "  (Wilcoxon skipped: SEEDS=${SEEDS} < 5)"
fi

echo
echo "================================================================"
echo "  DONE.  Results dir: ${OUT_DIR}"
echo "================================================================"
