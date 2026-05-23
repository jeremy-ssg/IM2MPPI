#!/usr/bin/env bash
# ============================================================================
#  run_intent_cvar_stress_experiments.sh
# ----------------------------------------------------------------------------
#  Explicit opt-in runner for the CVaR intent stress scene. The standard
#  run_intent_uncertain_experiments.sh remains on the default 45-person
#  benchmark by default.
# ============================================================================

set -euo pipefail

export WORLD_FILE="${WORLD_FILE:-$(rospack find uav_simulator)/worlds/intent_uncertain/intent_branch_cvar_stress.world}"
export PREDICTOR_PARAM_FILE="${PREDICTOR_PARAM_FILE:-$(rospack find autonomous_flight)/cfg/mpc_navigation/predictor_param_intent_uncertain.yaml}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
exec bash "${SCRIPT_DIR}/run_intent_uncertain_experiments.sh" "$@"
