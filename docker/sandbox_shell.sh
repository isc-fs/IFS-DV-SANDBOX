# Loaded by every interactive shell in the sandbox container.

source /opt/ros/humble/setup.bash
source /opt/ifs_ws/install/setup.bash
if [ -f /sandbox/ros2_ws/install/local_setup.bash ]; then
    source /sandbox/ros2_ws/install/local_setup.bash
fi

# Build your workspace and re-source it. Extra args go to colcon,
# e.g. `sb-build --packages-select sandbox_pipeline`.
sb-build() {
    (cd /sandbox/ros2_ws && colcon build --symlink-install "$@") \
        && source /sandbox/ros2_ws/install/local_setup.bash \
        && echo "sb-build: done, workspace sourced"
}

# Start the sandbox. Args are launch arguments, e.g.
#   sb-launch perception:=gt
#   sb-launch sim:=mock control:=sandbox_cpp/control_cpp
sb-launch() {
    fastdds shm clean >/dev/null 2>&1   # leftovers from a previous Ctrl+C
    ros2 launch sandbox_bringup sandbox.launch.py "$@"
}

# Integration test on the mock sim (same as CI). Args go to sb-launch:
#   sb-check                                   # baseline pipeline
#   sb-check planning:=alice_pipeline/planning # with your planner
sb-check() {
    /sandbox/tools/smoke_test.sh "$@"
}

# Benchmark + leaderboard (docs/LEADERBOARD.md). Args go to tools/benchmark.py:
#   sb-score control:=alice_pipeline/control
#   sb-score --scenario mock-all
sb-score() {
    python3 /sandbox/tools/benchmark.py "$@"
}
alias sb-leaderboard='python3 /sandbox/tools/leaderboard.py'

# Run every package's unit tests (same as CI).
sb-test() {
    (cd /sandbox/ros2_ws/src && python3 -m pytest -q -p no:cacheprovider */test "$@")
}

# Talk to the simulator (load a track, start, stop, reset). `sim --help`.
alias sim='python3 /sandbox/tools/sim.py'

if [ -z "$SANDBOX_QUIET" ] && [ -n "$PS1" ]; then
    echo "IFS-DV-SANDBOX shell. Commands: sb-build, sb-launch, sb-new, sb-test, sb-check, sb-score, sim --help"
    if [ ! -f /sandbox/ros2_ws/install/local_setup.bash ]; then
        echo "First time here? Run: sb-build"
    fi
fi

# Create your own package from the template: sb-new alice -> src/alice_pipeline
sb-new() {
    local name="$1"
    if [[ ! "$name" =~ ^[a-z][a-z0-9_]*$ ]]; then
        echo "usage: sb-new <name>   (lowercase letters, digits, _), e.g. sb-new alice"; return 1
    fi
    local pkg="${name}_pipeline" dst="/sandbox/ros2_ws/src/${name}_pipeline"
    if [[ -e "$dst" ]]; then echo "$dst already exists"; return 1; fi
    cp -r /sandbox/ros2_ws/templates/my_pipeline "$dst"
    mv "$dst/my_pipeline" "$dst/$pkg"
    mv "$dst/resource/my_pipeline" "$dst/resource/$pkg"
    # Unique test file name: pytest refuses two test files with the same name.
    mv "$dst/test/test_my_pipeline.py" "$dst/test/test_$pkg.py"
    grep -rl my_pipeline "$dst" | xargs sed -i "s/my_pipeline/$pkg/g"
    chown -R --reference=/sandbox "$dst"     # owned by you, not root (Linux hosts)
    echo "Created $dst"
    echo "Next: sb-build, then e.g.  sb-launch control:=$pkg/control"
}
