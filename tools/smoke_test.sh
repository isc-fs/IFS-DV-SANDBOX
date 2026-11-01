#!/usr/bin/env bash
# Integration test: start the pipeline on the mock sim, drive for a while,
# check that every stage publishes, the car moves and no cones are hit.
#
# Run inside the sandbox shell (that's what `sb-check` does) or in CI.
# Extra arguments go to sb-launch, so you can test YOUR stages:
#
#   tools/smoke_test.sh                                   # baseline
#   tools/smoke_test.sh control:=alice_pipeline/control   # with my control
#   SMOKE_DURATION=90 tools/smoke_test.sh track:=track_20260404_013721.csv
set -uo pipefail

DURATION="${SMOKE_DURATION:-40}"
LOG="${SMOKE_LOG:-/tmp/smoke_launch.log}"

echo "smoke test: ${DURATION} s of sim time, launch args: sim:=mock $*"
# Own process group (setsid) so we can stop the launch AND all its nodes.
setsid ros2 launch sandbox_bringup sandbox.launch.py sim:=mock foxglove:=false "$@" > "$LOG" 2>&1 &
LAUNCH_PID=$!
stop_launch() {
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fastdds shm clean >/dev/null 2>&1 || true   # segments left by killed nodes
}
trap stop_launch EXIT

ros2 run sandbox_tools smoke_check --ros-args -p use_sim_time:=true -p duration:="${DURATION}"
RC=$?

stop_launch

if grep -qE "Traceback|\[ERROR\]|process has died" "$LOG"; then
    echo; echo "--- errors in the launch log ($LOG) ---"
    grep -E -A8 "Traceback|\[ERROR\]|process has died" "$LOG" | head -60
    RC=1
fi
if [ "$RC" -ne 0 ]; then
    echo; echo "--- last 30 lines of the launch log ---"; tail -30 "$LOG"
fi
exit "$RC"
