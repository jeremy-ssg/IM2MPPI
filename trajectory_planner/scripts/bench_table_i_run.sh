#!/usr/bin/env bash
# bench_table_i_run.sh
# --------------------
# Run the Table I per-stage timing benchmark end-to-end and print the
# LaTeX block ready to paste into the paper.
#
# Prerequisites:
#   1) The C++ instrumentation from bench_table_i_instrument.md must be
#      applied to im2_mppi_planner.{h,cpp} and the workspace rebuilt.
#      Without it, the planner never writes timing records and this
#      script will report zero ticks at the end.
#   2) ROS environment sourced (devel/setup.bash).
#   3) A benchmark launch script capable of driving plan() at full rate.
#      Defaults to run_im2_full_debug.sh; override via $BENCH_LAUNCH.
#
# Usage:
#   ./bench_table_i_run.sh                # 90 s default
#   ./bench_table_i_run.sh 180            # 180 s
#   BENCH_LAUNCH=./run_dra_mppi_experiments.sh ./bench_table_i_run.sh
#
# Outputs:
#   - $STAGE_LOG (default /tmp/im2_stage_<ts>.jsonl) — raw per-tick log
#   - $LATEX_OUT (default /tmp/table_i_<ts>.tex)    — LaTeX block
#   - prints LaTeX to stdout

set -uo pipefail

DURATION="${1:-90}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_LAUNCH="${BENCH_LAUNCH:-${SCRIPT_DIR}/run_im2_full_debug.sh}"

TS="$(date +%s)"
STAGE_LOG="${STAGE_LOG:-/tmp/im2_stage_${TS}.jsonl}"
LATEX_OUT="${LATEX_OUT:-/tmp/table_i_${TS}.tex}"
YAML_PATH="${YAML_PATH:-${SCRIPT_DIR}/../cfg/im2_mppi.yaml}"

# Window we'll wait for roscore to come up after launching the bench.
ROSCORE_WAIT_MAX="${ROSCORE_WAIT_MAX:-30}"

echo ">> stage timing log : $STAGE_LOG"
echo ">> latex output     : $LATEX_OUT"
echo ">> bench launcher   : $BENCH_LAUNCH"
echo ">> duration         : ${DURATION}s"

# Wipe / create the log so we only collect fresh ticks.
: > "$STAGE_LOG"

# Check ROS is sourced before we even start.
if ! command -v rosparam >/dev/null 2>&1; then
    echo "!! ROS not on PATH — did you source devel/setup.bash?" >&2
    exit 4
fi

# Refuse to clobber an existing roscore the user might be using for
# something else; we want full control of /im2_mppi/stage_timing_log.
if rostopic list >/dev/null 2>&1; then
    echo "!! a roscore is already running. Stop it (or unset ROS_MASTER_URI)" >&2
    echo "!! and re-run — the bench launcher needs to start its own." >&2
    exit 5
fi

BENCH_PID=""

cleanup() {
    if [[ -n "$BENCH_PID" ]] && kill -0 "$BENCH_PID" 2>/dev/null; then
        echo ">> terminating bench launcher (pid $BENCH_PID)..."
        kill "$BENCH_PID" 2>/dev/null || true
    fi
    sleep 1
    # Mop up anything the launcher's own cleanup may have missed.
    pkill -f roslaunch  2>/dev/null || true
    pkill -f rosmaster  2>/dev/null || true
    pkill -f rosout     2>/dev/null || true
    pkill -f gzserver   2>/dev/null || true
    pkill -f gzclient   2>/dev/null || true
    pkill -f rviz       2>/dev/null || true
    sleep 1
}
trap cleanup EXIT INT TERM

# 1) Start the bench launcher in the background — it will spin up its
#    own roscore and full planner stack.
echo ">> starting bench launcher..."
"$BENCH_LAUNCH" 1 "$DURATION" &
BENCH_PID=$!

# 2) Wait for roscore to come up so we can set the timing-log parameter
#    BEFORE the planner constructs and reads its rosparams.
echo ">> waiting for roscore (up to ${ROSCORE_WAIT_MAX}s)..."
SLEPT=0
while ! rosparam list >/dev/null 2>&1; do
    sleep 1
    SLEPT=$((SLEPT + 1))
    if [[ "$SLEPT" -ge "$ROSCORE_WAIT_MAX" ]]; then
        echo "!! roscore did not come up within ${ROSCORE_WAIT_MAX}s." >&2
        echo "!! Inspect bench launcher log; aborting." >&2
        exit 6
    fi
done
echo ">> roscore up after ${SLEPT}s; setting stage timing log path"

# 3) Set the parameter the instrumented planner reads at construction.
#    run_im2_full_debug.sh has a ~13 s gap between roscore startup and
#    the im2_mppi_demo.launch fire, so we comfortably win the race.
if ! rosparam set /im2_mppi/stage_timing_log "$STAGE_LOG"; then
    echo "!! failed to set /im2_mppi/stage_timing_log on the master" >&2
    exit 7
fi
echo ">> /im2_mppi/stage_timing_log = $STAGE_LOG"

# 4) Let the planner run. The bench launcher will exit on its own when
#    DURATION elapses (it has its own timeout); we wait for it.
wait "$BENCH_PID" || true
BENCH_PID=""

# 5) Aggregate.
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
