#!/usr/bin/env bash
# Replay selected benchmark seeds in the existing IM2-MPPI RViz layout.
#
# Usage:
#   bash $(rospack find trajectory_planner)/scripts/show_selected_seed_trajectories_rviz.sh [RESULTS_DIR] [SEEDS]
#
# Example:
#   bash $(rospack find trajectory_planner)/scripts/show_selected_seed_trajectories_rviz.sh \
#     ~/IM2MPPI/results/full_lap_bag_20260528_080853 \
#     12,19,21,24,25,28,29

set -u

DEFAULT_RESULTS="${HOME}/IM2MPPI/results/full_lap_bag_20260528_080853"
REQUESTED_RESULTS_DIR="${1:-${RESULTS_DIR:-${DEFAULT_RESULTS}}}"
SEEDS="${2:-${SEEDS:-12,19,21,24,25,28,29}}"
PLAYBACK_SPEED="${PLAYBACK_SPEED:-1.0}"
PLAYBACK_RATE="${PLAYBACK_RATE:-20.0}"
PAUSE_BETWEEN_SEEDS="${PAUSE_BETWEEN_SEEDS:-2.0}"
LOOP_PLAYBACK="${LOOP_PLAYBACK:-true}"

SCRIPT_DIR="$(rospack find trajectory_planner)/scripts"
RVIZ_CONFIG="$(rospack find autonomous_flight)/cfg/im2_mppi_navigation.rviz"
REPLAY_NODE="${SCRIPT_DIR}/visualize_seed_trajectories_rviz.py"

has_timeseries_csv() {
    compgen -G "$1/*_timeseries.csv" >/dev/null 2>&1
}

normalize_results_dir() {
    local dir="$1"
    local base
    base="$(basename "${dir}")"

    if [[ ! -d "${dir}" ]]; then
        return 1
    fi

    if has_timeseries_csv "${dir}"; then
        printf '%s\n' "${dir}"
        return 0
    fi

    if [[ -d "${dir}/${base}" ]] && has_timeseries_csv "${dir}/${base}"; then
        printf '%s\n' "${dir}/${base}"
        return 0
    fi

    printf '%s\n' "${dir}"
    return 0
}

resolve_results_dir() {
    local requested="$1"
    local base
    local candidate
    local resolved

    base="$(basename "${requested}")"
    if resolved="$(normalize_results_dir "${requested}")"; then
        printf '%s\n' "${resolved}"
        return 0
    fi

    local candidates=(
        "${HOME}/IM2MPPI/results/${base}"
        "${HOME}/MOTIONPLANNING/IM2MPPI/results/${base}"
        "${HOME}/MOTIONPLANNING/IM2MPPI/src/IM2MPPI/results/${base}"
        "/mnt/c/Users/WIN11/IM2MPPI/analysis_results/${base}"
        "/mnt/c/Users/WIN11/IM2MPPI/analysis_results/${base}/${base}"
    )

    for candidate in "${candidates[@]}"; do
        if resolved="$(normalize_results_dir "${candidate}")"; then
            printf '%s\n' "${resolved}"
            return 0
        fi
    done

    while IFS= read -r candidate; do
        if resolved="$(normalize_results_dir "${candidate}")"; then
            printf '%s\n' "${resolved}"
            return 0
        fi
    done < <(find "${HOME}" -maxdepth 5 -type d -name "${base}" 2>/dev/null | head -n 20)

    return 1
}

if ! RESULTS_DIR="$(resolve_results_dir "${REQUESTED_RESULTS_DIR}")"; then
    echo "[seed-rviz] results directory not found: ${REQUESTED_RESULTS_DIR}" >&2
    echo "[seed-rviz] I also checked common WSL/Windows analysis-result locations." >&2
    echo "[seed-rviz] pass it explicitly as the first argument." >&2
    echo "[seed-rviz] for example:" >&2
    echo "  bash \$(rospack find trajectory_planner)/scripts/show_selected_seed_trajectories_rviz.sh /actual/path/to/full_lap_bag_20260528_080853 12,19,21,24,25,28,29" >&2
    exit 1
fi

if [[ "${RESULTS_DIR}" != "${REQUESTED_RESULTS_DIR}" ]]; then
    echo "[seed-rviz] requested results path was not directly usable:"
    echo "[seed-rviz]   ${REQUESTED_RESULTS_DIR}"
    echo "[seed-rviz] using resolved path:"
    echo "[seed-rviz]   ${RESULTS_DIR}"
fi

if ! has_timeseries_csv "${RESULTS_DIR}"; then
    echo "[seed-rviz] warning: no *_timeseries.csv files found directly under ${RESULTS_DIR}" >&2
fi

ROSCORE_PID=""
REPLAY_PID=""
RVIZ_PID=""

cleanup() {
    if [[ -n "${REPLAY_PID}" ]]; then
        kill "${REPLAY_PID}" >/dev/null 2>&1 || true
        wait "${REPLAY_PID}" >/dev/null 2>&1 || true
    fi
    if [[ -n "${RVIZ_PID}" ]]; then
        kill "${RVIZ_PID}" >/dev/null 2>&1 || true
        wait "${RVIZ_PID}" >/dev/null 2>&1 || true
    fi
    if [[ -n "${ROSCORE_PID}" ]]; then
        kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
        wait "${ROSCORE_PID}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

if ! rostopic list >/dev/null 2>&1; then
    echo "[seed-rviz] starting roscore"
    roscore >/tmp/seed_traj_rviz_roscore.log 2>&1 &
    ROSCORE_PID=$!
    sleep 3
fi

echo "[seed-rviz] results : ${RESULTS_DIR}"
echo "[seed-rviz] seeds   : ${SEEDS}"
echo "[seed-rviz] topic   : /seed_trajectory_viz/markers"
echo "[seed-rviz] rviz    : ${RVIZ_CONFIG}"

python3 "${REPLAY_NODE}" \
    _results_dir:="${RESULTS_DIR}" \
    _seeds:="${SEEDS}" \
    _playback_speed:="${PLAYBACK_SPEED}" \
    _rate:="${PLAYBACK_RATE}" \
    _pause_between_seeds:="${PAUSE_BETWEEN_SEEDS}" \
    _loop:="${LOOP_PLAYBACK}" &
REPLAY_PID=$!

sleep 1
rviz -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!

wait "${RVIZ_PID}"
