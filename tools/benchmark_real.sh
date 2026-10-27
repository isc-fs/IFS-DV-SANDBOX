#!/usr/bin/env bash
# Benchmark the REAL pipeline (IFS09-DV-PIPELINE) in the sandbox scenarios,
# to produce the reference rows of the leaderboard. For the heads, not the
# new members. Runs on the host (Linux / Mac / Git Bash).
#
#   tools/benchmark_real.sh mock-A mock-B mock-C
#   tools/benchmark_real.sh ifssim-A            # IFSSIM must be running
#   REAL_IMAGE=ifssim-dv_pipeline_stack:latest tools/benchmark_real.sh mock-A
#
# Needs the pipeline image built by the IFSSIM repo (docker compose build
# dv_pipeline_stack there). Results land in results/real-pipeline/; copy
# the ones you want to keep into benchmark/reference/ and commit them.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
IMAGE="${REAL_IMAGE:-ifssim-dv_pipeline_stack:latest}"
LAPS="${LAPS:-1}"
SANDBOX="$(docker compose ps -q sandbox)"
[[ -n "$SANDBOX" ]] || { echo "start the sandbox first (tools/shell.sh)"; exit 1; }
[[ $# -gt 0 ]] || set -- mock-A

ROS_SETUP='source /opt/ros/humble/setup.bash && source /dv_pipeline_stack_ws/install/setup.bash'
start_real() {
    docker rm -f ifs-real >/dev/null 2>&1 || true
    # Same network + shared memory as the sandbox container, so DDS just works.
    docker run -d --name ifs-real --network host --ipc "container:$SANDBOX" \
        -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e NUMBA_CACHE_DIR=/numba_cache -v ifs-real-numba:/numba_cache \
        --entrypoint bash "$IMAGE" -c "$ROS_SETUP && ros2 launch bringup sim_pipeline.launch.py use_sim_time:=true" >/dev/null
}
real_cli() { docker exec ifs-real bash -c "$ROS_SETUP && timeout 200 ros2 run sim_supervisor supervisor_cli $*" || true; }
# Without a simulator the mission system never reaches "ready", so bring the
# autonomy nodes up directly through mode_manager (prepare, then activate).
activate_direct() {
    docker exec ifs-real bash -c "$ROS_SETUP && \
        ros2 service call /activate_mode dv_msgs/srv/ActivateMode '{mission: trackdrive, activate: false}' >/dev/null && \
        ros2 service call /activate_mode dv_msgs/srv/ActivateMode '{mission: trackdrive, activate: true}' >/dev/null" \
        && echo "-> real pipeline active" || echo "!! could not activate the real pipeline"
}

# First ever run: Numba compiles the planner for ~2 min, longer than the
# pipeline's own prepare timeout. Warm the cache once.
if ! docker run --rm -v ifs-real-numba:/c --entrypoint sh ifs-sandbox:humble -c 'ls /c | grep -q .' 2>/dev/null; then
    echo "-> warming the real pipeline's Numba cache (once, ~2-3 min)"
    start_real; sleep 5; real_cli set_mission 1; sleep 150
    docker rm -f ifs-real >/dev/null
fi

for scenario in "$@"; do
    echo "=== real pipeline on $scenario"
    start_real
    sleep 8
    if [[ "$scenario" == real-* ]]; then
        # Real car bag: activate the stack first (it waits for data), then
        # replay the bag and save its outputs as the reference.
        activate_direct
        docker compose exec -T sandbox /entrypoint.sh python3 /sandbox/tools/benchmark.py \
            --real --record-reference --label real-pipeline --scenario "$scenario" || true
    else
        # Sandbox side: sim (mock or IFSSIM) + adapter + scorer, in the background.
        docker compose exec -T sandbox /entrypoint.sh python3 /sandbox/tools/benchmark.py \
            --real --label real-pipeline --laps "$LAPS" --scenario "$scenario" &
        BENCH=$!
        sleep 12
        real_cli run 1                     # SetMission(trackdrive) + RuntimeControl
        wait "$BENCH" || true
    fi
    docker logs ifs-real > "results/real-pipeline-$scenario.log" 2>&1 || true
    docker rm -f ifs-real >/dev/null
done
