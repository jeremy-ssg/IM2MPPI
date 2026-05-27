#!/usr/bin/env bash
# ============================================================================
#  run_four_methods_full_lap_bag.sh
# ----------------------------------------------------------------------------
#  Paper-data driver for the default start.launch scene.
#
#  Runs four methods on the start.launch default world:
#    M0_intent_mpc  = original Intent-MPC stack
#    M1_vanilla     = vanilla MPPI
#    M5_dra_mppi    = DRA-MPPI baseline
#    M4_im2_full    = proposed full IM2-MPPI
#
#  Differences from run_all_experiments.sh:
#    - evaluator uses relative full-lap completion: lap_finish_fraction=1.0
#    - no 98% visual cutoff; all timeseries CSV rows are kept until the full
#      relative lap finishes or DURATION is reached
#    - records a compact rosbag per run with only key topics needed for
#      trajectory, obstacle, planning, rollout, and intent visualization
#    - writes plot_input_manifest.csv for runs that really produced summary
#      and timeseries files, plus file_inventory.csv for missing-file checks
#
#  Usage:
#    bash $(rospack find trajectory_planner)/scripts/run_four_methods_full_lap_bag.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    bash .../run_four_methods_full_lap_bag.sh          # 1 seed, 140 s timeout
#    bash .../run_four_methods_full_lap_bag.sh 5 160    # 5 seeds
#    GAZEBO_GUI=true ENABLE_RVIZ=true bash .../run_four_methods_full_lap_bag.sh 1
#
#  Optional environment overrides:
#    OUT_DIR_OVERRIDE=/home/user/IM2MPPI/results/full_lap_resume
#    SEED_START=1
#    WALL_TIMEOUT=1140       # wall-clock guard per run; default DURATION*6+300
#    GAZEBO_GUI=false
#    ENABLE_RVIZ=false
#    RECORD_BAG=true
#    BAG_COMPRESSION=lz4       # lz4 | bz2 | none
#    WORLD_FILE=/abs/path.world  # otherwise start.launch default world is used
# ============================================================================

set -u

SEEDS=${1:-1}
DURATION=${2:-140}
GOAL_RADIUS=${3:-0.8}
SEED_START=${SEED_START:-1}
LAP_FINISH_FRACTION=${LAP_FINISH_FRACTION:-1.0}
LAP_MIN_PATH_FRACTION=${LAP_MIN_PATH_FRACTION:-0.98}
WALL_TIMEOUT=${WALL_TIMEOUT:-$((DURATION * 6 + 300))}
GAZEBO_GUI="${GAZEBO_GUI:-false}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
RECORD_BAG="${RECORD_BAG:-true}"
BAG_COMPRESSION="${BAG_COMPRESSION:-lz4}"
WORLD_FILE="${WORLD_FILE:-}"

INTENT_MPC_LAUNCH="${INTENT_MPC_LAUNCH:-autonomous_flight intent_mpc_demo.launch}"
IM2_MPPI_LAUNCH="${IM2_MPPI_LAUNCH:-autonomous_flight im2_mppi_demo.launch enable_rviz:=${ENABLE_RVIZ}}"

if [[ -n "${OUT_DIR_OVERRIDE:-}" ]]; then
    OUT_DIR="${OUT_DIR_OVERRIDE}"
    STAMP="$(basename "${OUT_DIR}")"
    echo "[resume] using existing OUT_DIR = ${OUT_DIR}"
else
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_DIR="${HOME}/IM2MPPI/results/full_lap_bag_${STAMP}"
fi

LOG_DIR="${OUT_DIR}/logs"
BAG_DIR="${OUT_DIR}/bags"
mkdir -p "${LOG_DIR}" "${BAG_DIR}"

YAML_PLANNER="$(rospack find trajectory_planner)/cfg/im2_mppi.yaml"
SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
REF_TRAJ="$(rospack find autonomous_flight)/cfg/mpc_navigation/ref_trajectory.txt"

# Compact but sufficient bag topic regex.
# Dynamic obstacles: /onboard_detector/GT_obstacle_bbox and /gazebo/model_states
# Static obstacles/map: /dynamic_map/inflated_voxel_map, /dynamic_map/voxel_map
# Ego: odom/pose/vel/acc and target_state
# Planned trajectories: Intent-MPC and IM2-MPPI trajectory topics
# MPPI samples/intent: sampled_rollouts and dynamic_obstacle_predictions
BAG_TOPIC_REGEX='^(/clock|/tf|/tf_static|/gazebo/model_states|/move_base_simple/goal|/CERLAB/quadcopter/(odom|pose|vel|acc)|/autonomous_flight/target_state|/onboard_detector/(GT_obstacle_bbox|history_trajectories|dynamic_bboxes|tracked_bboxes|velocity_visualizaton)|/dynamic_map/(inflated_voxel_map|voxel_map|explored_voxel_map|2D_occupancy_map)|/im2mppi/(best_trajectory|sampled_rollouts|reference_path|dynamic_obstacle_predictions|goal|plan_time_ms)|/mpcNavigation/(mpc_trajectory|input_trajectory|pwl_trajectory|poly_traj|rrt_path|goal|plan_time_ms))$'

CONFIGS=(
  "M0_intent_mpc|Intent-MPC|${INTENT_MPC_LAUNCH}|N/A|N/A|N/A|intent_mpc"
  "M1_vanilla|MPPI|${IM2_MPPI_LAUNCH}|vanilla_mppi|soft|true|im2_mppi"
  "M5_dra_mppi|DRA-MPPI|${IM2_MPPI_LAUNCH}|dra_mppi|soft|false|im2_mppi"
  "M4_im2_full|Ours|${IM2_MPPI_LAUNCH}|cvar_mppi|adaptive|true|im2_mppi"
)

cleanup_all() {
    for SIG in 15 9; do
        pkill -${SIG} -f rosbag                  2>/dev/null || true
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
    echo "[full-lap-bag] restoring yaml"
    if [[ -f "${YAML_PLANNER}.fulllapbak" ]]; then
        cp "${YAML_PLANNER}.fulllapbak" "${YAML_PLANNER}"
        rm -f "${YAML_PLANNER}.fulllapbak"
    fi
    cleanup_all
    exit
}

cp "${YAML_PLANNER}" "${YAML_PLANNER}.fulllapbak"
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

bag_compression_args() {
    case "${BAG_COMPRESSION}" in
        lz4) echo "--lz4" ;;
        bz2) echo "--bz2" ;;
        none|"") echo "" ;;
        *)
            echo "[warn] unknown BAG_COMPRESSION=${BAG_COMPRESSION}; using no compression" >&2
            echo ""
            ;;
    esac
}

start_bag() {
    local tag=$1
    BAG_PID=""
    BAG_NODE="/${tag}_bag"
    BAG_FILE="${BAG_DIR}/${tag}.bag"

    if [[ "${RECORD_BAG}" != "true" ]]; then
        return
    fi

    local comp
    comp="$(bag_compression_args)"
    echo "    [bag] recording ${BAG_FILE}"
    # shellcheck disable=SC2086
    rosbag record ${comp} --regex \
        --duration="$((WALL_TIMEOUT + 60))" \
        -O "${BAG_FILE}" \
        "${BAG_TOPIC_REGEX}" \
        __name:="${BAG_NODE#/}" \
        > "${LOG_DIR}/${tag}_bag.log" 2>&1 &
    BAG_PID=$!
    sleep 2
}

stop_bag() {
    if [[ "${RECORD_BAG}" != "true" ]]; then
        return
    fi
    if [[ -n "${BAG_NODE:-}" ]]; then
        rosnode kill "${BAG_NODE}" >/dev/null 2>&1 || true
    fi
    if [[ -n "${BAG_PID:-}" ]]; then
        wait "${BAG_PID}" >/dev/null 2>&1 || true
    fi
    sleep 1
}

archive_one() {
    local tmp_out=$1
    local tag=$2
    local eval_algo=$3

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
        return 0
    else
        echo "    [FAIL] no summary produced; see ${LOG_DIR}/${tag}_*.log"
        return 1
    fi
}

write_manifest() {
    local manifest="${OUT_DIR}/plot_input_manifest.csv"
    echo "config,method,seed,summary_json,timeseries_csv,bag,reference_path,lap_finish_fraction,planned_trajectory_topic,rollout_topic,intent_prediction_topic,obstacle_topic,static_map_topic" > "${manifest}"
    for CONFIG in "${CONFIGS[@]}"; do
        IFS='|' read -r NAME LABEL _LAUNCH _METHOD _FUSION _CL _EVAL <<< "${CONFIG}"
        for SEED in $(seq "${SEED_START}" $((SEED_START + SEEDS - 1))); do
            local summary="${OUT_DIR}/${NAME}_seed${SEED}_summary.json"
            local timeseries="${OUT_DIR}/${NAME}_seed${SEED}_timeseries.csv"
            local bag="${BAG_DIR}/${NAME}_seed${SEED}.bag"
            if [[ ! -f "${summary}" || ! -f "${timeseries}" ]]; then
                continue
            fi
            echo "${NAME},${LABEL},${SEED},${OUT_DIR}/${NAME}_seed${SEED}_summary.json,${OUT_DIR}/${NAME}_seed${SEED}_timeseries.csv,${BAG_DIR}/${NAME}_seed${SEED}.bag,${REF_TRAJ},${LAP_FINISH_FRACTION},/im2mppi/best_trajectory;/mpcNavigation/mpc_trajectory,/im2mppi/sampled_rollouts,/im2mppi/dynamic_obstacle_predictions,/onboard_detector/GT_obstacle_bbox,/dynamic_map/inflated_voxel_map" >> "${manifest}"
        done
    done
    echo "  wrote ${manifest}"
}

file_size_or_zero() {
    local path=$1
    if [[ -f "${path}" ]]; then
        stat -c%s "${path}" 2>/dev/null || echo 0
    else
        echo 0
    fi
}

file_yes_no() {
    local path=$1
    if [[ -f "${path}" ]]; then
        echo yes
    else
        echo no
    fi
}

write_inventory() {
    local inventory="${OUT_DIR}/file_inventory.csv"
    echo "config,method,seed,summary_exists,timeseries_exists,path_metrics_exists,plan_time_exists,target_state_exists,cmd_accel_exists,collision_events_exists,diagnostics_exists,bag_exists,summary_bytes,timeseries_bytes,bag_bytes" > "${inventory}"
    for CONFIG in "${CONFIGS[@]}"; do
        IFS='|' read -r NAME LABEL _LAUNCH _METHOD _FUSION _CL _EVAL <<< "${CONFIG}"
        for SEED in $(seq "${SEED_START}" $((SEED_START + SEEDS - 1))); do
            local tag="${NAME}_seed${SEED}"
            local summary="${OUT_DIR}/${tag}_summary.json"
            local timeseries="${OUT_DIR}/${tag}_timeseries.csv"
            local path_metrics="${OUT_DIR}/${tag}_path_metrics.csv"
            local plan_time="${OUT_DIR}/${tag}_plan_time.csv"
            local target_state="${OUT_DIR}/${tag}_target_state.csv"
            local cmd_accel="${OUT_DIR}/${tag}_cmd_accel.csv"
            local collision="${OUT_DIR}/${tag}_collision_events.csv"
            local diagnostics="${OUT_DIR}/${tag}_diagnostics.json"
            local bag="${BAG_DIR}/${tag}.bag"
            echo "${NAME},${LABEL},${SEED},$(file_yes_no "${summary}"),$(file_yes_no "${timeseries}"),$(file_yes_no "${path_metrics}"),$(file_yes_no "${plan_time}"),$(file_yes_no "${target_state}"),$(file_yes_no "${cmd_accel}"),$(file_yes_no "${collision}"),$(file_yes_no "${diagnostics}"),$(file_yes_no "${bag}"),$(file_size_or_zero "${summary}"),$(file_size_or_zero "${timeseries}"),$(file_size_or_zero "${bag}")" >> "${inventory}"
        done
    done
    echo "  wrote ${inventory}"
}

run_one() {
    local name=$1
    local label=$2
    local launch=$3
    local method=$4
    local fusion=$5
    local cl=$6
    local eval_algo=$7
    local seed=$8
    local tag="${name}_seed${seed}"
    local plan_time_topic="/im2mppi/plan_time_ms"
    if [[ "${eval_algo}" == "intent_mpc" ]]; then
        plan_time_topic="/mpcNavigation/plan_time_ms"
    fi

    echo
    echo "================================================================"
    echo "[$(date +%T)] ${tag}  ${label}"
    echo "    method=${method} fusion=${fusion} closed_loop=${cl}"
    echo "================================================================"

    cleanup_all

    if [[ "${method}" != "N/A" ]]; then
        set_yaml_str "method_type" "${method}"
        set_yaml_str "fusion_mode" "${fusion}"
        set_yaml_num "random_seed" "${seed}"
    fi

    roscore > "${LOG_DIR}/${tag}_roscore.log" 2>&1 &
    sleep 3

    if [[ "${cl}" != "N/A" ]]; then
        rosparam set /autonomous_flight/closed_loop_intent_enabled "${cl}" >/dev/null 2>&1 || true
    fi

    local sim_extra=""
    if [[ -n "${WORLD_FILE}" ]]; then
        sim_extra="world_name:=${WORLD_FILE}"
    fi

    OBS_BRANCH_SEED="${seed}" \
    timeout --kill-after=30 "${WALL_TIMEOUT}" \
        roslaunch uav_simulator start.launch gui:="${GAZEBO_GUI}" ${sim_extra} \
        > "${LOG_DIR}/${tag}_sim.log" 2>&1 &
    sleep 10

    pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
    pkill -9 -f keyboard_control      2>/dev/null || true
    pkill -9 -f key_teleop            2>/dev/null || true
    pkill -9 -f keyboardCtrl          2>/dev/null || true

    start_bag "${tag}"

    local tmp_out="${OUT_DIR}/_tmp_${tag}"
    mkdir -p "${tmp_out}"

    # Start the evaluator before the planner stack so the raw CSV covers the
    # whole lap instead of missing the first segment during takeoff/init.
    # We pass the reference path explicitly because /autonomous_flight params
    # are not guaranteed to exist before the planner launch comes up.
    timeout --kill-after=30 "${WALL_TIMEOUT}" \
        roslaunch trajectory_planner evaluate_planner.launch \
            algorithm:="${eval_algo}" \
            duration:="${DURATION}" \
            completion_mode:="lap" \
            shutdown_on_success:="true" \
            goal_radius:="${GOAL_RADIUS}" \
            lap_completion_radius:="${GOAL_RADIUS}" \
            lap_reference_path:="${REF_TRAJ}" \
            lap_finish_fraction:="${LAP_FINISH_FRACTION}" \
            lap_min_path_fraction:="${LAP_MIN_PATH_FRACTION}" \
            plan_time_topic:="${plan_time_topic}" \
            output_dir:="${tmp_out}" \
        > "${LOG_DIR}/${tag}_eval.log" 2>&1 &
    EVAL_PID=$!
    sleep 2

    timeout --kill-after=30 "${WALL_TIMEOUT}" \
        roslaunch ${launch} \
        > "${LOG_DIR}/${tag}_stack.log" 2>&1 &

    wait "${EVAL_PID}" >/dev/null 2>&1 || true

    stop_bag
    if archive_one "${tmp_out}" "${tag}" "${eval_algo}"; then
        rm -rf "${tmp_out}"
    else
        mv "${tmp_out}" "${OUT_DIR}/${tag}_failed_tmp" 2>/dev/null || true
    fi

    cleanup_all
}

TOTAL=$((${#CONFIGS[@]} * SEEDS))
COUNT=0
T_START=$(date +%s)
EST_SEC=$(( TOTAL * (DURATION + 55) ))
EST_HMS=$(printf '%02dh%02dm' $((EST_SEC/3600)) $(((EST_SEC%3600)/60)))
EST_END=$(date -d "+${EST_SEC} seconds" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || \
          date -v+${EST_SEC}S '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo "?")

echo
echo "================================================================"
echo "  Four-method full-lap bag driver"
echo "  methods: Intent-MPC, MPPI, DRA-MPPI, Ours"
echo "  world: ${WORLD_FILE:-<start.launch default>}"
echo "  seeds: ${SEEDS}  seed_start: ${SEED_START}  total runs: ${TOTAL}"
echo "  timeout/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  wall-time guard/run: ${WALL_TIMEOUT}s"
echo "  lap_finish_fraction: ${LAP_FINISH_FRACTION}  lap_min_path_fraction: ${LAP_MIN_PATH_FRACTION}"
echo "  gazebo gui: ${GAZEBO_GUI}  rviz: ${ENABLE_RVIZ}  record_bag: ${RECORD_BAG}"
echo "  bag compression: ${BAG_COMPRESSION}"
echo "  estimated wall clock: ${EST_HMS}  (done around ${EST_END})"
echo "  results: ${OUT_DIR}"
echo "================================================================"

for SEED in $(seq "${SEED_START}" $((SEED_START + SEEDS - 1))); do
    for CONFIG in "${CONFIGS[@]}"; do
        IFS='|' read -r NAME LABEL LAUNCH METHOD FUSION CL EVAL_ALGO <<< "${CONFIG}"
        COUNT=$((COUNT + 1))

        if [[ -f "${OUT_DIR}/${NAME}_seed${SEED}_summary.json" ]]; then
            printf "\n>>> [%d/%d] %s seed=%d already done, skipped\n" \
                "${COUNT}" "${TOTAL}" "${NAME}" "${SEED}"
            continue
        fi

        ELAPSED=$(( $(date +%s) - T_START ))
        REMAINING=$(( (TOTAL - COUNT + 1) * (DURATION + 55) ))
        printf "\n>>> [%d/%d] seed=%d config=%s elapsed=%ds est_remaining=%ds\n" \
            "${COUNT}" "${TOTAL}" "${SEED}" "${NAME}" "${ELAPSED}" "${REMAINING}"
        run_one "${NAME}" "${LABEL}" "${LAUNCH}" "${METHOD}" "${FUSION}" "${CL}" "${EVAL_ALGO}" "${SEED}"
    done
done

echo
echo "================================================================"
echo "  All full-lap runs done. Aggregating..."
echo "================================================================"

if compgen -G "${OUT_DIR}/*_summary.json" > /dev/null; then
    python3 "${SCRIPT_DIR}/compare_planner_eval.py" \
        "${OUT_DIR}/compare_all.csv" "${OUT_DIR}"/*_summary.json \
        > "${LOG_DIR}/aggregate.log" 2>&1 || true
    echo "  wrote ${OUT_DIR}/compare_all.csv"
    python3 "${SCRIPT_DIR}/print_summary_table.py" "${OUT_DIR}" || true
else
    echo "  no summaries produced"
fi

write_manifest
write_inventory

echo
echo "================================================================"
echo "  DONE. Results dir: ${OUT_DIR}"
echo "  Bags dir:    ${BAG_DIR}"
echo "  Plot input:  ${OUT_DIR}/plot_input_manifest.csv"
echo "  Inventory:   ${OUT_DIR}/file_inventory.csv"
echo "================================================================"
