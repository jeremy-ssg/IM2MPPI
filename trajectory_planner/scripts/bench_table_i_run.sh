#!/usr/bin/env bash
# bench_table_i_run.sh
# --------------------
# Run the Table I per-stage timing benchmark end-to-end and print the
# LaTeX block ready to paste into the paper.
#
# Drives bench_table_i.launch, which composes exactly the same stack
# run_all_experiments.sh uses for IM2-MPPI runs:
#
#     roslaunch uav_simulator      start.launch
#     roslaunch autonomous_flight  im2_mppi_demo.launch
#
# i.e. the production RViz config (im2_mppi_navigation.rviz) that
# renders only the detector's FOV-filtered dynamic-obstacle bboxes —
# NOT every world obstacle. This is the lightweight visualisation
# behaviour the paper benchmarks have always used.
#
# Prerequisites:
#   1) The C++ instrumentation from bench_table_i_instrument.md must be
#      applied to im2_mppi_planner.{h,cpp} and the workspace rebuilt.
#      Without it, no timing records will be produced.
#   2) ROS environment sourced (devel/setup.bash).
#
# Usage:
#   ./bench_table_i_run.sh                # 90 s
#   ./bench_table_i_run.sh 180            # 180 s
#   WORLD=path/to/sceneB.world ./bench_table_i_run.sh 120
#   GUI=false RVIZ=false ./bench_table_i_run.sh 90   # headless
#
# Outputs:
#   - $STAGE_LOG (default /tmp/im2_stage_<ts>.jsonl) — raw per-tick log
#   - $LATEX_OUT (default /tmp/table_i_<ts>.tex)    — LaTeX block
#   - prints LaTeX to stdout

set -uo pipefail

DURATION="${1:-90}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TS="$(date +%s)"
STAGE_LOG="${STAGE_LOG:-/tmp/im2_stage_${TS}.jsonl}"
LATEX_OUT="${LATEX_OUT:-/tmp/table_i_${TS}.tex}"
LAUNCH_LOG="${LAUNCH_LOG:-/tmp/im2_bench_launch_${TS}.log}"
YAML_PATH="${YAML_PATH:-${SCRIPT_DIR}/../cfg/im2_mppi.yaml}"

WORLD="${WORLD:-}"
GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"

echo ">> stage timing log : $STAGE_LOG"
echo ">> latex output     : $LATEX_OUT"
echo ">> launch log       : $LAUNCH_LOG"
echo ">> duration         : ${DURATION}s"
echo ">> gui=${GUI}  rviz=${RVIZ}"
[[ -n "$WORLD" ]] && echo ">> world override   : $WORLD"

# Wipe / create the log so we only collect fresh ticks.
: > "$STAGE_LOG"

if ! command -v roslaunch >/dev/null 2>&1; then
    echo "!! roslaunch not on PATH — did you source devel/setup.bash?" >&2
    exit 4
fi
if rostopic list >/dev/null 2>&1; then
    echo "!! a roscore is already running. Stop it (or unset ROS_MASTER_URI)" >&2
    echo "!! and re-run — the benchmark needs a clean master." >&2
    exit 5
fi

LAUNCH_PID=""

cleanup() {
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo ">> terminating roslaunch (pid $LAUNCH_PID)..."
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -TERM "$LAUNCH_PID" 2>/dev/null || true
        sleep 1
        kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi
    # Match run_all_experiments.sh cleanup_all() ordering.
    for SIG in 15 9; do
        pkill -${SIG} -f im2_mppi_navigation_node    2>/dev/null || true
        pkill -${SIG} -f tracking_controller_node    2>/dev/null || true
        pkill -${SIG} -f onboard_detector            2>/dev/null || true
        pkill -${SIG} -f dynamic_predictor           2>/dev/null || true
        pkill -${SIG} -f teleop_twist_keyboard       2>/dev/null || true
        pkill -${SIG} -f keyboard_control            2>/dev/null || true
        pkill -${SIG} -f key_teleop                  2>/dev/null || true
        pkill -${SIG} -f keyboardCtrl                2>/dev/null || true
        pkill -${SIG} -f gzclient                    2>/dev/null || true
        pkill -${SIG} -f gzserver                    2>/dev/null || true
        pkill -${SIG} -f rviz                        2>/dev/null || true
        pkill -${SIG} -f roslaunch                   2>/dev/null || true
        pkill -${SIG} -f rosmaster                   2>/dev/null || true
        pkill -${SIG} -f rosout                      2>/dev/null || true
        [[ "${SIG}" == "15" ]] && sleep 3
    done
    sleep 2
}
trap cleanup EXIT INT TERM

# Compose roslaunch args. stage_timing_log is a <param> inside
# bench_table_i.launch, so it's on the master before any node starts.
LAUNCH_ARGS=("trajectory_planner" "bench_table_i.launch"
             "stage_timing_log:=$STAGE_LOG"
             "gui:=$GUI"
             "enable_rviz:=$RVIZ")
[[ -n "$WORLD" ]] && LAUNCH_ARGS+=("world_name:=$WORLD")

echo ">> roslaunch ${LAUNCH_ARGS[*]}"
( timeout --kill-after=15 "$((DURATION + 30))" \
      roslaunch "${LAUNCH_ARGS[@]}" \
      > "$LAUNCH_LOG" 2>&1 ) &
LAUNCH_PID=$!

# Match run_all_experiments.sh: after the simulator/planner stack
# starts, kill any teleop nodes start.launch spawns so they don't
# steal /cmd_vel and the planner has full control.
echo ">> waiting 10s for sim + stack init, then killing teleop..."
sleep 10
pkill -9 -f teleop_twist_keyboard 2>/dev/null || true
pkill -9 -f keyboard_control      2>/dev/null || true
pkill -9 -f key_teleop            2>/dev/null || true
pkill -9 -f keyboardCtrl          2>/dev/null || true

# Stream tick counts every 5s so the user can see progress live.
echo ">> running for ${DURATION}s..."
SLEPT=10                # already waited 10s above
LAST_LINES=0
while [[ "$SLEPT" -lt "$DURATION" ]]; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "!! roslaunch died early (after ${SLEPT}s). See: $LAUNCH_LOG" >&2
        break
    fi
    sleep 5
    SLEPT=$((SLEPT + 5))
    LINES="$(wc -l < "$STAGE_LOG" 2>/dev/null || echo 0)"
    if [[ "$LINES" -ne "$LAST_LINES" ]]; then
        echo "   [${SLEPT}s] $LINES tick(s) recorded"
        LAST_LINES="$LINES"
    fi
done

cleanup
trap - EXIT INT TERM

N_TICKS=0
if [[ -s "$STAGE_LOG" ]]; then
    N_TICKS="$(wc -l < "$STAGE_LOG")"
fi
echo ">> collected $N_TICKS plan() ticks"

if [[ "$N_TICKS" -eq 0 ]]; then
    cat >&2 <<EOF
!! Zero ticks recorded. The most likely cause is that the C++
!! instrumentation in bench_table_i_instrument.md has not been
!! applied (or the build has not picked it up). Specifically:
!!   - im2_mppi_planner.{h,cpp} must read im2_mppi/stage_timing_log
!!     in loadParams() and call writeStageTimingRecord() at the end of
!!     planCPU()/planGPU().
!!   - catkin_make must be re-run after the edits.
!!
!! See: ${SCRIPT_DIR}/bench_table_i_instrument.md
!! See launch log for runtime errors: $LAUNCH_LOG
EOF
    exit 8
fi

if [[ "$N_TICKS" -lt 50 ]]; then
    echo "!! WARNING: fewer than 50 ticks; results will be noisy." >&2
fi

python3 "${SCRIPT_DIR}/bench_table_i.py" "$STAGE_LOG" \
        --yaml "$YAML_PATH" \
        --latex \
    | tee "$LATEX_OUT"

echo
echo ">> wrote LaTeX block to $LATEX_OUT"
echo ">> raw timing log retained at $STAGE_LOG"
echo ">> roslaunch console log:    $LAUNCH_LOG"
