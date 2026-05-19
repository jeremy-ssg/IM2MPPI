#!/usr/bin/env bash
# Batch driver: runs each algorithm × seed combination once, archives the
# resulting summary JSON files under a per-experiment directory, then runs
# the Wilcoxon comparison at the end.
#
# Assumptions:
#   * The planner reads `im2_mppi/random_seed` from rosparam at startup.
#   * Both launch files exit cleanly after `duration` seconds (via the
#     evaluate_planner.launch shutdown_timer).
#   * roscore can be (re)started between runs.
#
# Usage:
#   ./run_batch_eval.sh  [N_SEEDS]  [DURATION_SEC]
# Example:
#   ./run_batch_eval.sh 30 60
#
# Outputs:
#   results/<timestamp>/<algo>_seed<N>_summary.json
#   results/<timestamp>/compare_all.csv
#   results/<timestamp>/stats_im2_mppi_vs_intent_mpc.csv

set -euo pipefail

N_SEEDS="${1:-30}"
DURATION="${2:-60}"

ALGORITHMS=("intent_mpc" "im2_mppi")

# Launch files associated with each algorithm.
declare -A LAUNCH_FILE=(
    [intent_mpc]="autonomous_flight intent_mpc_demo.launch"
    [im2_mppi]="autonomous_flight im2_mppi_demo.launch"
)

# Yaml path containing random_seed for each algorithm (we'll sed it per run).
declare -A YAML_PATH=(
    [intent_mpc]=""    # intent-MPC has no MPPI seed; harmless
    [im2_mppi]="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
)

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$(pwd)/results/${STAMP}"
mkdir -p "${OUT_DIR}"
echo "[batch] writing to ${OUT_DIR}"

# Helper: set random_seed in a yaml file (in-place).
set_seed() {
    local yaml="$1" seed="$2"
    [[ -z "${yaml}" || ! -f "${yaml}" ]] && return
    # Match both `  random_seed: 42` and `random_seed: 42`
    sed -i -E "s/^([[:space:]]*random_seed[[:space:]]*:).*$/\1 ${seed}/g" "${yaml}"
}

for SEED in $(seq 1 "${N_SEEDS}"); do
    for ALGO in "${ALGORITHMS[@]}"; do
        echo "─────────────────────────────────────────────────────────────"
        echo "[batch] algo=${ALGO}  seed=${SEED}/${N_SEEDS}"
        echo "─────────────────────────────────────────────────────────────"

        # 1. Patch the seed into the planner's yaml.
        set_seed "${YAML_PATH[${ALGO}]:-}" "${SEED}"

        # 2. Start the planner stack in the background.
        roslaunch ${LAUNCH_FILE[${ALGO}]} \
            > "${OUT_DIR}/${ALGO}_seed${SEED}_stack.log" 2>&1 &
        STACK_PID=$!

        # Wait a few seconds for nodes to come up before launching evaluator.
        sleep 8

        # 3. Run the evaluator (blocks for `duration` seconds, then exits).
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="${ALGO}" \
            duration:="${DURATION}" \
            output_dir:="${OUT_DIR}/_tmp_${ALGO}_${SEED}" \
            > "${OUT_DIR}/${ALGO}_seed${SEED}_eval.log" 2>&1 || true

        # 4. Archive evaluator output with a seed-tagged name.
        if [[ -f "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_summary.json" ]]; then
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_summary.json" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_summary.json"
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_timeseries.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_timeseries.csv" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_plan_time.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_plan_time.csv"  2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_plan_time_timeline.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_plan_time_timeline.csv" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_target_state.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_target_state.csv" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_cmd_accel.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_cmd_accel.csv" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_collision_events.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_collision_events.csv" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_diagnostics.json" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_diagnostics.json" 2>/dev/null || true
            mv "${OUT_DIR}/_tmp_${ALGO}_${SEED}/${ALGO}_diagnostic_events.csv" \
               "${OUT_DIR}/${ALGO}_seed${SEED}_diagnostic_events.csv" 2>/dev/null || true
        else
            echo "[batch] WARNING: no summary produced for ${ALGO} seed ${SEED}"
        fi
        rm -rf "${OUT_DIR}/_tmp_${ALGO}_${SEED}"

        # 5. Tear down the planner stack.
        kill -INT  "${STACK_PID}" 2>/dev/null || true
        sleep 1
        kill -TERM "${STACK_PID}" 2>/dev/null || true
        wait "${STACK_PID}" 2>/dev/null || true
        # Make sure stragglers (rviz, gazebo, etc.) are also gone.
        pkill -f "gazebo"         2>/dev/null || true
        pkill -f "rviz"           2>/dev/null || true
        pkill -f "im2_mppi_navigation_node"   2>/dev/null || true
        pkill -f "mpcNavigation"  2>/dev/null || true
        sleep 2
    done
done

# 6. Aggregate per-seed summaries.
SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
echo "[batch] aggregating ${OUT_DIR}/*_summary.json"
python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
    "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json

# 7. Paired statistical comparison (IM2-MPPI vs Intent-MPC).
echo "[batch] running Wilcoxon test"
python3 "${SCRIPT_DIR}/stats_compare.py" \
    --baseline-glob  "${OUT_DIR}/intent_mpc_seed*_summary.json" \
    --treatment-glob "${OUT_DIR}/im2_mppi_seed*_summary.json" \
    --baseline-name  "intent_mpc" \
    --treatment-name "im2_mppi" \
    --out            "${OUT_DIR}/stats_im2_mppi_vs_intent_mpc.csv" || true

echo "[batch] done. results -> ${OUT_DIR}"
