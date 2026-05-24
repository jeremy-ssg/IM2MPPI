#!/usr/bin/env bash
# ============================================================================
#  run_dra_mppi_sparse_medium.sh
# ----------------------------------------------------------------------------
#  Single-method DRA-MPPI batch on the new Sparse-Medium scenario
#  (generated_sparse_medium.world: 25 dynamic obstacles @ 0.4-0.6 m/s).
#
#  This sits between Sparse-Slow (20 obs / 0.3-0.5 m/s) and Medium-Normal
#  (40 obs / 0.5-0.8 m/s) so DRA-MPPI can be probed along an obstacle-
#  density gradient.
#
#  Usage:
#    ./run_dra_mppi_sparse_medium.sh [RUNS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_dra_mppi_sparse_medium.sh                 # 1 run, 90 s timeout
#    ./run_dra_mppi_sparse_medium.sh 10 90 0.8       # 10 seeds, 90 s each
#    GAZEBO_GUI=false ENABLE_RVIZ=false \
#        ./run_dra_mppi_sparse_medium.sh 50 90       # headless 50-seed sweep
#
#  Pass-through env vars (see run_dra_mppi_experiments.sh):
#    OUT_DIR_OVERRIDE, SEED_START, GAZEBO_GUI, ENABLE_RVIZ
# ============================================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_DIR="$(cd "${SCRIPT_DIR}/../../uav_simulator/worlds/generated_env" 2>/dev/null && pwd || true)"
if [[ -z "${WORLD_DIR}" ]] && command -v rospack >/dev/null 2>&1; then
    WORLD_DIR="$(rospack find uav_simulator 2>/dev/null)/worlds/generated_env"
fi
if [[ -z "${WORLD_DIR}" || ! -d "${WORLD_DIR}" ]]; then
    echo "[sparse-medium] could not locate uav_simulator/worlds/generated_env"
    echo "  did you 'source ~/catkin_ws/devel/setup.bash' first?"
    exit 1
fi
WORLD="${WORLD_DIR}/generated_sparse_medium.world"

if [[ ! -f "${WORLD}" ]]; then
    echo "[sparse-medium] generating scenario worlds (one-off; idempotent)"
    python3 "${SCRIPT_DIR}/generate_generated_env_scenarios.py" \
        --base "${WORLD_DIR}/generated_env.world" \
        --out-dir "${WORLD_DIR}"
fi

if [[ -z "${OUT_DIR_OVERRIDE:-}" ]]; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    export OUT_DIR_OVERRIDE="${HOME}/IM2MPPI/results/dra_sparse_medium_${STAMP}"
fi

export WORLD_FILE="${WORLD}"

exec bash "${SCRIPT_DIR}/run_dra_mppi_experiments.sh" "$@"
