#!/usr/bin/env bash
# Run a ROS node under gdb when used as a roslaunch launch-prefix.
# The wrapped node command is received as "$@" from roslaunch.

set +e

LOG="${IM2_MPPI_GDB_LOG:-/tmp/im2_mppi_gdb_backtrace.log}"
mkdir -p "$(dirname "${LOG}")"

{
    echo
    echo "================================================================"
    echo "[gdb_roslaunch_prefix] $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Command: $*"
    echo "Log: ${LOG}"
    echo "================================================================"
} >> "${LOG}"

if ! command -v gdb >/dev/null 2>&1; then
    echo "[gdb_roslaunch_prefix] ERROR: gdb not found." | tee -a "${LOG}"
    exec "$@"
fi

gdb -q -batch \
    -ex "set pagination off" \
    -ex "set print thread-events off" \
    -ex "handle SIGPIPE nostop noprint pass" \
    -ex "run" \
    -ex "echo \n===== info threads =====\n" \
    -ex "info threads" \
    -ex "echo \n===== thread apply all bt full =====\n" \
    -ex "thread apply all bt full" \
    -ex "echo \n===== bt full =====\n" \
    -ex "bt full" \
    -ex "quit" \
    --args "$@" 2>&1 | tee -a "${LOG}"

exit "${PIPESTATUS[0]}"
