#!/usr/bin/env bash
# ============================================================================
#  run_generated_env_grid_experiments.sh
# ----------------------------------------------------------------------------
#  Fixed-trajectory dynamic-obstacle benchmark over four generated_env variants:
#
#    Sparse-Slow     20 dynamic cylinders, 0.3-0.5 m/s
#    Medium-Normal   40 dynamic cylinders, 0.5-0.8 m/s
#    Dense-Normal    80 dynamic cylinders, 0.5-0.8 m/s
#    Dense-Fast      80 dynamic cylinders, 0.8-1.1 m/s
#
#  It compares only four methods:
#    M4_im2_full    proposed method
#    M1_vanilla     original MPPI
#    M0_intent_mpc  Intent-MPC baseline
#    M5_dra_mppi    DRA-MPPI baseline
#
#  Usage:
#    ./run_generated_env_grid_experiments.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_generated_env_grid_experiments.sh 5 90 0.8
#    GAZEBO_GUI=false ENABLE_RVIZ=false ./run_generated_env_grid_experiments.sh 50 90
#
#  Optional overrides:
#    OUT_ROOT_OVERRIDE=/home/user/IM2MPPI/results/generated_env_grid_resume
#    SEED_START=1
#    GAZEBO_GUI=true|false
#    ENABLE_RVIZ=true|false
# ============================================================================

set -u

SEEDS=${1:-5}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}
SEED_START=${SEED_START:-1}
GAZEBO_GUI="${GAZEBO_GUI:-true}"
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
WORLD_DIR="$(rospack find uav_simulator)/worlds/generated_env"
YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"

INTENT_MPC_LAUNCH="${INTENT_MPC_LAUNCH:-autonomous_flight intent_mpc_demo.launch}"
IM2_MPPI_LAUNCH="${IM2_MPPI_LAUNCH:-autonomous_flight im2_mppi_demo.launch enable_rviz:=${ENABLE_RVIZ}}"

if [[ -n "${OUT_ROOT_OVERRIDE:-}" ]]; then
    OUT_ROOT="${OUT_ROOT_OVERRIDE}"
    STAMP="$(basename "${OUT_ROOT}")"
    echo "[resume] using existing OUT_ROOT = ${OUT_ROOT}"
else
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_ROOT="${HOME}/IM2MPPI/results/generated_env_grid_${STAMP}"
fi

mkdir -p "${OUT_ROOT}"

echo "[grid] generating scenario worlds from generated_env.world"
python3 "${SCRIPT_DIR}/generate_generated_env_scenarios.py" \
    --base "${WORLD_DIR}/generated_env.world" \
    --out-dir "${WORLD_DIR}"

CONFIGS=(
  "M4_im2_full|${IM2_MPPI_LAUNCH}|cvar_mppi|adaptive|true|im2_mppi"
  "M1_vanilla|${IM2_MPPI_LAUNCH}|vanilla_mppi|soft|true|im2_mppi"
  "M0_intent_mpc|${INTENT_MPC_LAUNCH}|N/A|N/A|N/A|intent_mpc"
  "M5_dra_mppi|${IM2_MPPI_LAUNCH}|dra_mppi|soft|false|im2_mppi"
)

SCENARIOS=(
  "Sparse-Slow|${WORLD_DIR}/generated_sparse_slow.world"
  "Medium-Normal|${WORLD_DIR}/generated_medium_normal.world"
  "Dense-Normal|${WORLD_DIR}/generated_dense_normal.world"
  "Dense-Fast|${WORLD_DIR}/generated_dense_fast.world"
)

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

    if [[ -d "${HOME}/.ros/log" ]]; then
        find "${HOME}/.ros/log" -mindepth 1 -maxdepth 1 -type d \
            | sort | head -n -3 | xargs -r rm -rf 2>/dev/null || true
    fi

    sleep 5
}

restore_yaml_and_exit() {
    trap - EXIT INT TERM
    echo "[grid] restoring yaml"
    if [[ -f "${YAML_PLANNER}.gridbak" ]]; then
        cp "${YAML_PLANNER}.gridbak" "${YAML_PLANNER}"
        rm -f "${YAML_PLANNER}.gridbak"
    fi
    cleanup_all
    exit
}

cp "${YAML_PLANNER}" "${YAML_PLANNER}.gridbak"
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
    local out_dir=$2
    local tag=$3
    local eval_algo=$4

    if [[ -f "${tmp_out}/${eval_algo}_summary.json" ]]; then
        mv "${tmp_out}/${eval_algo}_summary.json" "${out_dir}/${tag}_summary.json"
        mv "${tmp_out}/${eval_algo}_timeseries.csv" "${out_dir}/${tag}_timeseries.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_path_metrics.csv" "${out_dir}/${tag}_path_metrics.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_plan_time.csv" "${out_dir}/${tag}_plan_time.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_plan_time_timeline.csv" "${out_dir}/${tag}_plan_time_timeline.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_target_state.csv" "${out_dir}/${tag}_target_state.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_cmd_accel.csv" "${out_dir}/${tag}_cmd_accel.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_collision_events.csv" "${out_dir}/${tag}_collision_events.csv" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_diagnostics.json" "${out_dir}/${tag}_diagnostics.json" 2>/dev/null || true
        mv "${tmp_out}/${eval_algo}_diagnostic_events.csv" "${out_dir}/${tag}_diagnostic_events.csv" 2>/dev/null || true
        echo "    [OK] -> ${tag}_summary.json"
    else
        echo "    [FAIL] no summary produced; see logs for ${tag}"
    fi
}

run_one() {
    local scenario=$1
    local world_file=$2
    local out_dir=$3
    local log_dir=$4
    local name=$5
    local launch=$6
    local method=$7
    local fusion=$8
    local cl=$9
    local eval_algo=${10}
    local seed=${11}
    local tag="${name}_seed${seed}"

    echo
    echo "================================================================"
    echo "[$(date +%T)] ${scenario} :: ${tag}"
    echo "    world=${world_file}"
    echo "    method=${method}  fusion=${fusion}  CL=${cl}"
    echo "================================================================"

    cleanup_all

    if [[ "${method}" != "N/A" ]]; then
        set_yaml_str "method_type" "${method}"
        set_yaml_str "fusion_mode" "${fusion}"
        set_yaml_num "random_seed" "${seed}"
    fi

    roscore > "${log_dir}/${tag}_roscore.log" 2>&1 &
    sleep 3

    if [[ "${cl}" != "N/A" ]]; then
        rosparam set /autonomous_flight/closed_loop_intent_enabled "${cl}" \
            >/dev/null 2>&1 || true
    fi

    OBS_BRANCH_SEED="${seed}" \
    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" world_name:="${world_file}" \
        > "${log_dir}/${tag}_sim.log" 2>&1 &
    sleep 10

    pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
    pkill -9 -f keyboard_control      2>/dev/null || true
    pkill -9 -f key_teleop            2>/dev/null || true
    pkill -9 -f keyboardCtrl          2>/dev/null || true

    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch ${launch} \
        > "${log_dir}/${tag}_stack.log" 2>&1 &
    sleep 12

    local tmp_out="${out_dir}/_tmp_${tag}"
    mkdir -p "${tmp_out}"

    timeout --kill-after=10 $((DURATION + 30)) \
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="${eval_algo}" \
            duration:="${DURATION}" \
            completion_mode:="lap" \
            shutdown_on_success:="true" \
            goal_radius:="${GOAL_RADIUS}" \
            lap_completion_radius:="${GOAL_RADIUS}" \
            output_dir:="${tmp_out}" \
        > "${log_dir}/${tag}_eval.log" 2>&1 || true

    archive_one "${tmp_out}" "${out_dir}" "${tag}" "${eval_algo}"
    rm -rf "${tmp_out}"

    cleanup_all
}

aggregate_scenario() {
    local scenario=$1
    local out_dir=$2
    local log_dir=$3

    echo
    echo "================================================================"
    echo "  Aggregating ${scenario}"
    echo "================================================================"

    if compgen -G "${out_dir}/*_summary.json" > /dev/null; then
        python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
            "${out_dir}/compare_all.csv" "${out_dir}"/*_summary.json \
            > "${log_dir}/aggregate.log" 2>&1 || true
        echo "  wrote ${out_dir}/compare_all.csv"
        python3 "${SCRIPT_DIR}/print_summary_table.py" "${out_dir}" || true

        if [[ "${SEEDS}" -ge 5 ]]; then
            python3 "${SCRIPT_DIR}/stats_compare.py" \
                --baseline-glob "${out_dir}/M1_vanilla_seed*_summary.json" \
                --treatment-glob "${out_dir}/M4_im2_full_seed*_summary.json" \
                --baseline-name "vanilla" --treatment-name "im2_full" \
                --out "${out_dir}/stats_im2_vs_vanilla.csv" 2>/dev/null || true
            python3 "${SCRIPT_DIR}/stats_compare.py" \
                --baseline-glob "${out_dir}/M0_intent_mpc_seed*_summary.json" \
                --treatment-glob "${out_dir}/M4_im2_full_seed*_summary.json" \
                --baseline-name "intent_mpc" --treatment-name "im2_full" \
                --out "${out_dir}/stats_im2_vs_intent_mpc.csv" 2>/dev/null || true
            python3 "${SCRIPT_DIR}/stats_compare.py" \
                --baseline-glob "${out_dir}/M5_dra_mppi_seed*_summary.json" \
                --treatment-glob "${out_dir}/M4_im2_full_seed*_summary.json" \
                --baseline-name "dra_mppi" --treatment-name "im2_full" \
                --out "${out_dir}/stats_im2_vs_dra_mppi.csv" 2>/dev/null || true
        fi
    else
        echo "  no summaries produced for ${scenario}"
    fi
}

TOTAL=$((${#SCENARIOS[@]} * ${#CONFIGS[@]} * SEEDS))
COUNT=0
T_START=$(date +%s)
EST_SEC=$(( TOTAL * (DURATION + 40) ))
EST_HMS=$(printf '%02dh%02dm' $((EST_SEC/3600)) $(((EST_SEC%3600)/60)))

echo
echo "================================================================"
echo "  Generated-env density/speed grid"
echo "  scenarios: ${#SCENARIOS[@]}  configs: ${#CONFIGS[@]}  seeds/config: ${SEEDS}"
echo "  total runs: ${TOTAL}"
echo "  timeout/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  gazebo gui: ${GAZEBO_GUI}  rviz: ${ENABLE_RVIZ}"
echo "  estimated wall clock: ${EST_HMS}"
echo "  results root: ${OUT_ROOT}"
echo "================================================================"

for SCENARIO_ENTRY in "${SCENARIOS[@]}"; do
    IFS='|' read -r SCENARIO WORLD_FILE <<< "${SCENARIO_ENTRY}"
    SCENARIO_DIR="${OUT_ROOT}/${SCENARIO}"
    LOG_DIR="${SCENARIO_DIR}/logs"
    mkdir -p "${LOG_DIR}"

    echo "${WORLD_FILE}" > "${SCENARIO_DIR}/world_file.txt"

    for SEED_OFFSET in $(seq 0 $((SEEDS - 1))); do
        SEED=$((SEED_START + SEED_OFFSET))
        for CONFIG in "${CONFIGS[@]}"; do
            IFS='|' read -r NAME LAUNCH METHOD FUSION CL EVAL_ALGO <<< "${CONFIG}"
            COUNT=$((COUNT + 1))

            if [[ -f "${SCENARIO_DIR}/${NAME}_seed${SEED}_summary.json" ]]; then
                echo "[skip] ${SCENARIO} ${NAME}_seed${SEED} already exists"
                continue
            fi

            ELAPSED=$(( $(date +%s) - T_START ))
            REMAINING=$(( (TOTAL - COUNT + 1) * (DURATION + 40) ))
            printf "\n>>> [%d/%d] scenario=%s seed=%d config=%s elapsed=%ds est_remaining=%ds\n" \
                "${COUNT}" "${TOTAL}" "${SCENARIO}" "${SEED}" "${NAME}" \
                "${ELAPSED}" "${REMAINING}"

            run_one "${SCENARIO}" "${WORLD_FILE}" "${SCENARIO_DIR}" "${LOG_DIR}" \
                "${NAME}" "${LAUNCH}" "${METHOD}" "${FUSION}" "${CL}" \
                "${EVAL_ALGO}" "${SEED}"
        done
    done

    aggregate_scenario "${SCENARIO}" "${SCENARIO_DIR}" "${LOG_DIR}"
done

echo
echo "================================================================"
echo "  DONE. Results root: ${OUT_ROOT}"
echo "================================================================"
