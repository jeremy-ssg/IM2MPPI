#!/usr/bin/env bash
# ============================================================================
#  run_intent35_ablation_experiments.sh
# ----------------------------------------------------------------------------
#  Factorial ablation driver for the 35-pedestrian intent-uncertain scene.
#
#  Purpose:
#    Separate the effects of the proposed modules instead of comparing only
#    loosely named variants.
#
#  The ablation is a 2 x 2 x 2 design:
#    CVaR:        off(mode_aware_mppi) / on(cvar_mppi)
#    Fusion:      soft / adaptive
#    Closed-loop: false / true
#
#  Plus one mean-prediction reference:
#    A0_mean_pred_cl = mean_prediction_mppi + soft + closed-loop
#
#  Clean pairwise reads:
#    CVaR effect under soft+CL:       A6_cvar_soft_cl  vs A2_mode_soft_cl
#    CVaR effect under adaptive+CL:   A8_ours_full     vs A4_mode_adapt_cl
#    Fusion effect without CVaR:      A4_mode_adapt_cl vs A2_mode_soft_cl
#    Fusion effect with CVaR:         A8_ours_full     vs A6_cvar_soft_cl
#    Closed-loop effect on full:      A8_ours_full     vs A7_cvar_adapt_no_cl
#    Multi-intent vs mean prediction: A2_mode_soft_cl  vs A0_mean_pred_cl
#
#  Usage:
#    bash run_intent35_ablation_experiments.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    bash run_intent35_ablation_experiments.sh 3 90 0.8
#    GAZEBO_GUI=true ENABLE_RVIZ=true bash run_intent35_ablation_experiments.sh 1
#    OUT_DIR_OVERRIDE=~/IM2MPPI/results/intent35_ablation_resume \
#        bash run_intent35_ablation_experiments.sh 50 90
#
#  Optional environment overrides:
#    WORLD_FILE=/abs/path/to/intent_branch_35.world
#    PREDICTOR_PARAM_FILE=/abs/path/to/predictor.yaml
#    SEED_START=1
#    GAZEBO_GUI=false|true
#    ENABLE_RVIZ=false|true
#    CONFIG_SET=full|core
#
#  CONFIG_SET=core runs only the six most interpretable configs:
#    A0, A2, A4, A6, A7, A8
# ============================================================================

set -u

SEEDS=${1:-5}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}
SEED_START=${SEED_START:-1}
GAZEBO_GUI="${GAZEBO_GUI:-false}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
CONFIG_SET="${CONFIG_SET:-full}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
WORLD_DEFAULT="$(rospack find uav_simulator)/worlds/intent_uncertain/intent_branch_35.world"
PREDICTOR_PARAM_DEFAULT="$(rospack find autonomous_flight)/cfg/mpc_navigation/predictor_param.yaml"

WORLD_FILE="${WORLD_FILE:-${WORLD_DEFAULT}}"
PREDICTOR_PARAM_FILE="${PREDICTOR_PARAM_FILE:-${PREDICTOR_PARAM_DEFAULT}}"
IM2_MPPI_LAUNCH="${IM2_MPPI_LAUNCH:-autonomous_flight im2_mppi_intent_uncertain.launch enable_rviz:=${ENABLE_RVIZ} predictor_param_file:=${PREDICTOR_PARAM_FILE}}"

if [[ -n "${OUT_DIR_OVERRIDE:-}" ]]; then
    OUT_DIR="${OUT_DIR_OVERRIDE}"
    STAMP="$(basename "${OUT_DIR}")"
    echo "[resume] using existing OUT_DIR = ${OUT_DIR}"
else
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_DIR="${HOME}/IM2MPPI/results/intent35_ablation_${STAMP}"
fi

LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

CONFIGS_FULL=(
  "A0_mean_pred_cl|mean_prediction_mppi|soft|true"
  "A1_mode_soft_no_cl|mode_aware_mppi|soft|false"
  "A2_mode_soft_cl|mode_aware_mppi|soft|true"
  "A3_mode_adapt_no_cl|mode_aware_mppi|adaptive|false"
  "A4_mode_adapt_cl|mode_aware_mppi|adaptive|true"
  "A5_cvar_soft_no_cl|cvar_mppi|soft|false"
  "A6_cvar_soft_cl|cvar_mppi|soft|true"
  "A7_cvar_adapt_no_cl|cvar_mppi|adaptive|false"
  "A8_ours_full|cvar_mppi|adaptive|true"
)

CONFIGS_CORE=(
  "A0_mean_pred_cl|mean_prediction_mppi|soft|true"
  "A2_mode_soft_cl|mode_aware_mppi|soft|true"
  "A4_mode_adapt_cl|mode_aware_mppi|adaptive|true"
  "A6_cvar_soft_cl|cvar_mppi|soft|true"
  "A7_cvar_adapt_no_cl|cvar_mppi|adaptive|false"
  "A8_ours_full|cvar_mppi|adaptive|true"
)

if [[ "${CONFIG_SET}" == "core" ]]; then
    CONFIGS=("${CONFIGS_CORE[@]}")
else
    CONFIGS=("${CONFIGS_FULL[@]}")
fi

cleanup_all() {
    for SIG in 15 9; do
        pkill -${SIG} -f evaluate_intent_mpc_im2mppi 2>/dev/null || true
        pkill -${SIG} -f im2_mppi_navigation_node    2>/dev/null || true
        pkill -${SIG} -f mpc_navigation_node         2>/dev/null || true
        pkill -${SIG} -f mpcNavigation               2>/dev/null || true
        pkill -${SIG} -f tracking_controller_node    2>/dev/null || true
        pkill -${SIG} -f onboard_detector            2>/dev/null || true
        pkill -${SIG} -f dynamic_predictor           2>/dev/null || true
        pkill -${SIG} -f teleop_twist_keyboard       2>/dev/null || true
        pkill -${SIG} -f keyboard_control            2>/dev/null || true
        pkill -${SIG} -f key_teleop                  2>/dev/null || true
        pkill -${SIG} -f keyboardCtrl                2>/dev/null || true
        pkill -${SIG} -f gzclient                    2>/dev/null || true
        pkill -${SIG} -f gzserver                    2>/dev/null || true
        pkill -${SIG} -f gazebo                      2>/dev/null || true
        pkill -${SIG} -f rviz                        2>/dev/null || true
        pkill -${SIG} -f roslaunch                   2>/dev/null || true
        pkill -${SIG} -f rosmaster                   2>/dev/null || true
        pkill -${SIG} -f rosout                      2>/dev/null || true
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
    echo "[intent35-ablation] restoring yaml"
    if [[ -f "${YAML_PLANNER}.intent35bak" ]]; then
        cp "${YAML_PLANNER}.intent35bak" "${YAML_PLANNER}"
        rm -f "${YAML_PLANNER}.intent35bak"
    fi
    cleanup_all
    exit
}

cp "${YAML_PLANNER}" "${YAML_PLANNER}.intent35bak"
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

run_one() {
    local name=$1
    local method=$2
    local fusion=$3
    local cl=$4
    local seed=$5
    local tag="${name}_seed${seed}"

    echo
    echo "================================================================"
    echo "[$(date +%T)] ${tag}"
    echo "    method=${method}  fusion=${fusion}  closed_loop=${cl}"
    echo "================================================================"

    cleanup_all

    set_yaml_str "method_type" "${method}"
    set_yaml_str "fusion_mode" "${fusion}"
    set_yaml_num "random_seed" "${seed}"

    roscore > "${LOG_DIR}/${tag}_roscore.log" 2>&1 &
    sleep 3

    rosparam set /autonomous_flight/closed_loop_intent_enabled "${cl}" \
        >/dev/null 2>&1 || true

    OBS_BRANCH_SEED="${seed}" \
    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" world_name:="${WORLD_FILE}" \
        > "${LOG_DIR}/${tag}_sim.log" 2>&1 &
    sleep 10

    pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
    pkill -9 -f keyboard_control      2>/dev/null || true
    pkill -9 -f key_teleop            2>/dev/null || true
    pkill -9 -f keyboardCtrl          2>/dev/null || true

    timeout --kill-after=10 $((DURATION + 90)) \
        roslaunch ${IM2_MPPI_LAUNCH} \
        > "${LOG_DIR}/${tag}_stack.log" 2>&1 &
    sleep 12

    local tmp_out="${OUT_DIR}/_tmp_${tag}"
    mkdir -p "${tmp_out}"

    timeout --kill-after=10 $((DURATION + 30)) \
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="im2_mppi" \
            duration:="${DURATION}" \
            completion_mode:="lap" \
            shutdown_on_success:="true" \
            goal_radius:="${GOAL_RADIUS}" \
            lap_completion_radius:="${GOAL_RADIUS}" \
            output_dir:="${tmp_out}" \
        > "${LOG_DIR}/${tag}_eval.log" 2>&1 || true

    archive_one "${tmp_out}" "${tag}"
    rm -rf "${tmp_out}"

    cleanup_all
}

write_design_csv() {
    local out="${OUT_DIR}/ablation_design.csv"
    {
        echo "config,method_type,fusion_mode,closed_loop,cvar_on,adaptive_fusion_on,closed_loop_on,notes"
        for cfg in "${CONFIGS[@]}"; do
            IFS='|' read -r name method fusion cl <<< "${cfg}"
            local cvar_on="false"
            local adaptive_on="false"
            [[ "${method}" == "cvar_mppi" ]] && cvar_on="true"
            [[ "${fusion}" == "adaptive" ]] && adaptive_on="true"
            echo "${name},${method},${fusion},${cl},${cvar_on},${adaptive_on},${cl},"
        done
    } > "${out}"
}

run_stats_pair() {
    local baseline=$1
    local treatment=$2
    local name=$3

    python3 "${SCRIPT_DIR}/stats_compare.py" \
        --baseline-glob "${OUT_DIR}/${baseline}_seed*_summary.json" \
        --treatment-glob "${OUT_DIR}/${treatment}_seed*_summary.json" \
        --baseline-name "${baseline}" \
        --treatment-name "${treatment}" \
        --out "${OUT_DIR}/stats_${name}.csv" 2>/dev/null || true
}

aggregate_outputs() {
    echo
    echo "================================================================"
    echo "  Aggregating 35-pedestrian ablation outputs..."
    echo "================================================================"

    if compgen -G "${OUT_DIR}/*_summary.json" > /dev/null; then
        python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
            "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json \
            > "${LOG_DIR}/aggregate.log" 2>&1 || true
        echo "  wrote ${OUT_DIR}/compare_all.csv"
        python3 "${SCRIPT_DIR}/print_summary_table.py" "${OUT_DIR}" || true

        if [[ "${SEEDS}" -ge 5 ]]; then
            echo
            echo "  writing paired stats for clean module effects"
            run_stats_pair "A2_mode_soft_cl"      "A6_cvar_soft_cl"  "cvar_effect_soft_cl"
            run_stats_pair "A4_mode_adapt_cl"     "A8_ours_full"     "cvar_effect_adaptive_cl"
            run_stats_pair "A2_mode_soft_cl"      "A4_mode_adapt_cl" "fusion_effect_no_cvar_cl"
            run_stats_pair "A6_cvar_soft_cl"      "A8_ours_full"     "fusion_effect_cvar_cl"
            run_stats_pair "A7_cvar_adapt_no_cl"  "A8_ours_full"     "closed_loop_effect_full"
            run_stats_pair "A0_mean_pred_cl"      "A2_mode_soft_cl"  "multi_intent_vs_mean"
        fi
    else
        echo "  no summaries produced"
    fi
}

write_design_csv
echo "${WORLD_FILE}" > "${OUT_DIR}/world_file.txt"
echo "${PREDICTOR_PARAM_FILE}" > "${OUT_DIR}/predictor_param_file.txt"

TOTAL=$((${#CONFIGS[@]} * SEEDS))
COUNT=0
T_START=$(date +%s)
EST_SEC=$(( TOTAL * (DURATION + 40) ))
EST_HMS=$(printf '%02dh%02dm' $((EST_SEC/3600)) $(((EST_SEC%3600)/60)))

echo
echo "================================================================"
echo "  35-pedestrian factorial ablation"
echo "  configs: ${#CONFIGS[@]} (${CONFIG_SET})  seeds/config: ${SEEDS}"
echo "  total runs: ${TOTAL}"
echo "  world: ${WORLD_FILE}"
echo "  planner launch: ${IM2_MPPI_LAUNCH}"
echo "  timeout/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  gazebo gui: ${GAZEBO_GUI}  rviz: ${ENABLE_RVIZ}"
echo "  estimated wall clock: ${EST_HMS}"
echo "  results: ${OUT_DIR}"
echo "================================================================"

for SEED_OFFSET in $(seq 0 $((SEEDS - 1))); do
    SEED=$((SEED_START + SEED_OFFSET))
    for CONFIG in "${CONFIGS[@]}"; do
        IFS='|' read -r NAME METHOD FUSION CL <<< "${CONFIG}"
        COUNT=$((COUNT + 1))

        if [[ -f "${OUT_DIR}/${NAME}_seed${SEED}_summary.json" ]]; then
            echo "[skip] ${NAME}_seed${SEED} already exists"
            continue
        fi

        ELAPSED=$(( $(date +%s) - T_START ))
        REMAINING=$(( (TOTAL - COUNT + 1) * (DURATION + 40) ))
        printf "\n>>> [%d/%d] seed=%d config=%s elapsed=%ds est_remaining=%ds\n" \
            "${COUNT}" "${TOTAL}" "${SEED}" "${NAME}" "${ELAPSED}" "${REMAINING}"

        run_one "${NAME}" "${METHOD}" "${FUSION}" "${CL}" "${SEED}"
    done
done

aggregate_outputs

echo
echo "================================================================"
echo "  DONE. Results dir: ${OUT_DIR}"
echo "================================================================"
