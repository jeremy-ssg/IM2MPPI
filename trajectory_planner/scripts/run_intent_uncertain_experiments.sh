#!/usr/bin/env bash
# ============================================================================
#  run_intent_uncertain_experiments.sh
# ----------------------------------------------------------------------------
#  Batch driver for the intent-uncertain pedestrian-only scene.
#
#  It reuses run_all_experiments.sh, but overrides:
#    * Gazebo world: intent_uncertain/intent_branch_3.world
#    * Planner launch files: no-static-map variants
#    * RViz/Gazebo GUI: enabled by default so the new scene can be inspected
#
#  Usage:
#    ./run_intent_uncertain_experiments.sh [SEEDS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_intent_uncertain_experiments.sh 5 90
#    ./run_intent_uncertain_experiments.sh 50 90 0.8
#
#  Optional overrides:
#    WORLD_FILE=/abs/path/to/world.world
#    PREDICTOR_PARAM_FILE=/abs/path/to/predictor.yaml
#    GAZEBO_GUI=false
#    ENABLE_RVIZ=false
#    OUT_DIR_OVERRIDE=/home/user/IM2MPPI/results/intent_uncertain_resume
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
WORLD_DEFAULT="$(rospack find uav_simulator)/worlds/intent_uncertain/intent_branch_3.world"
PREDICTOR_PARAM_DEFAULT="$(rospack find autonomous_flight)/cfg/mpc_navigation/predictor_param.yaml"

export WORLD_FILE="${WORLD_FILE:-${WORLD_DEFAULT}}"
export PREDICTOR_PARAM_FILE="${PREDICTOR_PARAM_FILE:-${PREDICTOR_PARAM_DEFAULT}}"
export GAZEBO_GUI="${GAZEBO_GUI:-true}"
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"

export INTENT_MPC_LAUNCH="${INTENT_MPC_LAUNCH:-autonomous_flight intent_mpc_intent_uncertain.launch enable_rviz:=${ENABLE_RVIZ} predictor_param_file:=${PREDICTOR_PARAM_FILE}}"
export IM2_MPPI_LAUNCH="${IM2_MPPI_LAUNCH:-autonomous_flight im2_mppi_intent_uncertain.launch enable_rviz:=${ENABLE_RVIZ} predictor_param_file:=${PREDICTOR_PARAM_FILE}}"

if [[ -z "${OUT_DIR_OVERRIDE:-}" ]]; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    export OUT_DIR_OVERRIDE="${HOME}/IM2MPPI/results/intent_uncertain_${STAMP}"
fi

echo "================================================================"
echo "  Intent-uncertain pedestrian benchmark"
echo "  world:           ${WORLD_FILE}"
echo "  predictor yaml:  ${PREDICTOR_PARAM_FILE}"
echo "  static map:      disabled by *_intent_uncertain.launch"
echo "  gazebo gui:      ${GAZEBO_GUI}"
echo "  rviz:            ${ENABLE_RVIZ}"
echo "  results:         ${OUT_DIR_OVERRIDE}"
echo "================================================================"

exec bash "${SCRIPT_DIR}/run_all_experiments.sh" "$@"
