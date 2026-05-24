#!/usr/bin/env bash
# ============================================================================
#  run_dra_mppi_sparse_slow.sh
# ----------------------------------------------------------------------------
#  Single-method DRA-MPPI batch on the Sparse-Slow scenario
#  (generated_sparse_slow.world: 20 dynamic obstacles @ 0.3-0.5 m/s).
#
#  Usage:
#    ./run_dra_mppi_sparse_slow.sh [RUNS] [DURATION_SEC] [GOAL_RADIUS]
#
#  Examples:
#    ./run_dra_mppi_sparse_slow.sh                 # 1 run, 90 s timeout
#    ./run_dra_mppi_sparse_slow.sh 10 90 0.8       # 10 seeds, 90 s each
#    GAZEBO_GUI=false ENABLE_RVIZ=false \
#        ./run_dra_mppi_sparse_slow.sh 50 90       # headless 50-seed sweep
#
#  Pass-through env vars (see run_dra_mppi_experiments.sh):
#    OUT_DIR_OVERRIDE, SEED_START, GAZEBO_GUI, ENABLE_RVIZ
# ============================================================================

set -u

# Self-locate so this wrapper works even if the catkin workspace hasn't
# been sourced yet (rospack find would return empty string in that case,
# silently producing "no such file /scripts/...").
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_DIR="$(cd "${SCRIPT_DIR}/../../uav_simulator/worlds/generated_env" 2>/dev/null && pwd || true)"
if [[ -z "${WORLD_DIR}" ]] && command -v rospack >/dev/null 2>&1; then
    WORLD_DIR="$(rospack find uav_simulator 2>/dev/null)/worlds/generated_env"
fi
if [[ -z "${WORLD_DIR}" || ! -d "${WORLD_DIR}" ]]; then
    echo "[sparse-slow] could not locate uav_simulator/worlds/generated_env"
    echo "  did you 'source ~/catkin_ws/devel/setup.bash' first?"
    exit 1
fi
WORLD="${WORLD_DIR}/generated_sparse_slow.world"

if [[ ! -f "${WORLD}" ]]; then
    echo "[sparse-slow] generating scenario worlds (one-off; idempotent)"
    python3 "${SCRIPT_DIR}/generate_generated_env_scenarios.py" \
        --base "${WORLD_DIR}/generated_env.world" \
        --out-dir "${WORLD_DIR}"
fi

# Tag the per-run results directory so it doesn't collide with the generic
# DRA batch (which writes to dra_mppi_<stamp>).
if [[ -z "${OUT_DIR_OVERRIDE:-}" ]]; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    export OUT_DIR_OVERRIDE="${HOME}/IM2MPPI/results/dra_sparse_slow_${STAMP}"
fi

export WORLD_FILE="${WORLD}"

exec bash "${SCRIPT_DIR}/run_dra_mppi_experiments.sh" "$@"
