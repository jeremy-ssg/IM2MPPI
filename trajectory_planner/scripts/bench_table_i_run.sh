#!/usr/bin/env bash
# bench_table_i_run.sh
# --------------------
# Run the Table I per-stage timing benchmark end-to-end and print the
# LaTeX block ready to paste into the paper.
#
# Prerequisites:
#   1) The C++ instrumentation from bench_table_i_instrument.md must be
#      applied to im2_mppi_planner.{h,cpp} and the workspace rebuilt.
#   2) ROS environment sourced (devel/setup.bash).
#   3) A benchmark launch file capable of driving plan() at full rate.
#      Defaults to run_im2_full_debug.sh; override via $BENCH_LAUNCH.
#
# Usage:
#   ./bench_table_i_run.sh                # 90 s, default scenario
#   ./bench_table_i_run.sh 180            # 180 s
#   BENCH_LAUNCH=./run_dra_mppi_experiments.sh ./bench_table_i_run.sh
#
# Output:
#   - $STAGE_LOG (default /tmp/im2_stage_<ts>.jsonl) — raw per-tick log
#   - $LATEX_OUT (default /tmp/table_i_<ts>.tex)    — LaTeX block
#   - prints LaTeX to stdout as well

set -euo pipefail

DURATION="${1:-90}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_LAUNCH="${BENCH_LAUNCH:-${SCRIPT_DIR}/run_im2_full_debug.sh}"

TS="$(date +%s)"
STAGE_LOG="${STAGE_LOG:-/tmp/im2_stage_${TS}.jsonl}"
LATEX_OUT="${LATEX_OUT:-/tmp/table_i_${TS}.tex}"
YAML_PATH="${YAML_PATH:-${SCRIPT_DIR}/../cfg/im2_mppi.yaml}"

echo ">> stage timing log : $STAGE_LOG"
echo ">> latex output     : $LATEX_OUT"
echo ">> bench launcher   : $BENCH_LAUNCH"
echo ">> duration         : ${DURATION}s"

# Wipe / create the log so we collect only fresh ticks.
: > "$STAGE_LOG"

# Tell the planner where to write per-tick stage timings.
rosparam set /im2_mppi/stage_timing_log "$STAGE_LOG"

# Start the benchmark in the background.
"$BENCH_LAUNCH" &
BENCH_PID=$!

# Trap on exit so a Ctrl-C still kills the launcher.
cleanup() {
    if kill -0 "$BENCH_PID" 2>/dev/null; then
        kill "$BENCH_PID" || true
    fi
    sleep 1
    # Make sure any orphan rosnodes from the launch file are gone.
    rosnode kill -a 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Let the planner run for the requested duration.
sleep "$DURATION"

cleanup
trap - EXIT INT TERM

N_TICKS="$(wc -l < "$STAGE_LOG" || echo 0)"
echo ">> collected $N_TICKS plan() ticks"

if [[ "$N_TICKS" -lt 50 ]]; then
    echo "!! WARNING: fewer than 50 ticks recorded; results will be noisy." >&2
    echo "!! Check that the C++ instrumentation has been applied and that" >&2
    echo "!! /im2_mppi/stage_timing_log is being read by the planner." >&2
fi

# Aggregate + emit LaTeX.
python3 "${SCRIPT_DIR}/bench_table_i.py" "$STAGE_LOG" \
        --yaml "$YAML_PATH" \
        --latex \
    | tee "$LATEX_OUT"

echo
echo ">> wrote LaTeX block to $LATEX_OUT"
echo ">> raw timing log retained at $STAGE_LOG"
