#!/usr/bin/env bash
# ============================================================================
#  run_dra_mppi_intent_ped_counts.sh
# ----------------------------------------------------------------------------
#  Run only the DRA-MPPI baseline on the intent-uncertain pedestrian worlds:
#    * 35 pedestrians
#    * 25 pedestrians
#
#  The worlds are generated from intent_branch_3.world by keeping the first N
#  pedestrian models.  The intent-uncertain launch is used, so the static
#  prebuilt map is disabled just like the earlier pedestrian experiments.
#
#  Usage:
#    ./run_dra_mppi_intent_ped_counts.sh [RUNS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_dra_mppi_intent_ped_counts.sh 5 90 0.8
#    GAZEBO_GUI=false ENABLE_RVIZ=false \
#        ./run_dra_mppi_intent_ped_counts.sh 50 90 0.8
#
#  Optional environment overrides:
#    OUT_ROOT_OVERRIDE=/home/user/IM2MPPI/results/dra_intent_ped_resume
#    SEED_START=1
#    GAZEBO_GUI=true
#    ENABLE_RVIZ=true
#    PREDICTOR_PARAM_FILE=/abs/path/to/predictor.yaml
# ============================================================================

set -euo pipefail

RUNS=${1:-1}
DURATION=${2:-90}
GOAL_RADIUS=${3:-0.8}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v rospack >/dev/null 2>&1; then
    UAV_SIM_DIR="$(rospack find uav_simulator 2>/dev/null || true)"
    AUTO_FLIGHT_DIR="$(rospack find autonomous_flight 2>/dev/null || true)"
else
    UAV_SIM_DIR=""
    AUTO_FLIGHT_DIR=""
fi

if [[ -z "${UAV_SIM_DIR}" ]]; then
    UAV_SIM_DIR="$(cd "${SCRIPT_DIR}/../../uav_simulator" 2>/dev/null && pwd || true)"
fi
if [[ -z "${AUTO_FLIGHT_DIR}" ]]; then
    AUTO_FLIGHT_DIR="$(cd "${SCRIPT_DIR}/../../autonomous_flight" 2>/dev/null && pwd || true)"
fi
if [[ -z "${UAV_SIM_DIR}" || ! -d "${UAV_SIM_DIR}" ]]; then
    echo "[dra-intent] could not locate uav_simulator"
    echo "  run from the repository, or source the catkin workspace first."
    exit 1
fi
if [[ -z "${AUTO_FLIGHT_DIR}" || ! -d "${AUTO_FLIGHT_DIR}" ]]; then
    echo "[dra-intent] could not locate autonomous_flight"
    echo "  run from the repository, or source the catkin workspace first."
    exit 1
fi

WORLD_DIR="${UAV_SIM_DIR}/worlds/intent_uncertain"
BASE_WORLD="${WORLD_DIR}/intent_branch_3.world"
PREDICTOR_PARAM_FILE="${PREDICTOR_PARAM_FILE:-${AUTO_FLIGHT_DIR}/cfg/mpc_navigation/predictor_param.yaml}"

if [[ ! -f "${BASE_WORLD}" ]]; then
    echo "[dra-intent] base world not found: ${BASE_WORLD}"
    exit 1
fi

python3 "${SCRIPT_DIR}/generate_intent_ped_count_worlds.py" \
    --base "${BASE_WORLD}" \
    --out-dir "${WORLD_DIR}" \
    --counts 35 25

if [[ -z "${OUT_ROOT_OVERRIDE:-}" ]]; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_ROOT_OVERRIDE="${HOME}/IM2MPPI/results/dra_intent_ped_counts_${STAMP}"
fi
mkdir -p "${OUT_ROOT_OVERRIDE}"

export GAZEBO_GUI="${GAZEBO_GUI:-true}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
export IM2_MPPI_LAUNCH="autonomous_flight im2_mppi_intent_uncertain.launch enable_rviz:=${ENABLE_RVIZ} predictor_param_file:=${PREDICTOR_PARAM_FILE}"

echo "================================================================"
echo "  DRA-MPPI intent pedestrian-count benchmark"
echo "  counts: 35, 25"
echo "  runs/count: ${RUNS}"
echo "  timeout/run: ${DURATION}s  goal_radius: ${GOAL_RADIUS}m"
echo "  gazebo gui: ${GAZEBO_GUI}  rviz: ${ENABLE_RVIZ}"
echo "  predictor yaml: ${PREDICTOR_PARAM_FILE}"
echo "  results root: ${OUT_ROOT_OVERRIDE}"
echo "================================================================"

for COUNT in 35 25; do
    WORLD_FILE="${WORLD_DIR}/intent_branch_${COUNT}.world"
    OUT_DIR="${OUT_ROOT_OVERRIDE}/ped${COUNT}"

    echo
    echo "================================================================"
    echo "  Running DRA-MPPI on ${COUNT} pedestrians"
    echo "  world: ${WORLD_FILE}"
    echo "  results: ${OUT_DIR}"
    echo "================================================================"

    WORLD_FILE="${WORLD_FILE}" \
    OUT_DIR_OVERRIDE="${OUT_DIR}" \
    bash "${SCRIPT_DIR}/run_dra_mppi_experiments.sh" \
        "${RUNS}" "${DURATION}" "${GOAL_RADIUS}"
done

echo
echo "================================================================"
echo "  DONE. Results root: ${OUT_ROOT_OVERRIDE}"
echo "================================================================"
