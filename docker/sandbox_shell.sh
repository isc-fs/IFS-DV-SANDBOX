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
#   sb-new alice --from ifs09   start perception from the real IFS09 code: copies
#                               it into src/alice_pipeline/alice_cone_detection and
#                               makes perception.py run it. Works on a package you
#                               already have too (your old perception.py is kept
#                               as perception_before_ifs09.py).
sb-new() {
    local name="$1" from=""
    if [[ "$2" == "--from" ]]; then from="$3"; fi
    if [[ ! "$name" =~ ^[a-z][a-z0-9_]*$ ]] || [[ -n "$2" && "$from" != "ifs09" ]]; then
        echo "usage: sb-new <name> [--from ifs09]   (name: lowercase letters, digits, _), e.g. sb-new alice"
        return 1
    fi
    local pkg="${name}_pipeline" dst="/sandbox/ros2_ws/src/${name}_pipeline"
    if [[ -e "$dst" && -z "$from" ]]; then echo "$dst already exists"; return 1; fi
    local existed=yes
    if [[ ! -e "$dst" ]]; then
        existed=no
        cp -r /sandbox/ros2_ws/templates/my_pipeline "$dst"
        mv "$dst/my_pipeline" "$dst/$pkg"
        mv "$dst/resource/my_pipeline" "$dst/resource/$pkg"
        # Unique test file name: pytest refuses two test files with the same name.
        mv "$dst/test/test_my_pipeline.py" "$dst/test/test_$pkg.py"
        grep -rl my_pipeline "$dst" | xargs sed -i "s/my_pipeline/$pkg/g"
        echo "Created $dst"
    fi
    if [[ "$from" == "ifs09" ]]; then
        _sb_add_ifs09 "$name" "$existed" || return 1
    fi
    chown -R --reference=/sandbox "$dst"     # owned by you, not root (Linux hosts)
    echo "Next: sb-build, then e.g.  sb-launch control:=$pkg/control"
}

# Copy the vendored IFS09 perception into a student package (see sb-new).
_sb_add_ifs09() {
    local name="$1" pkg="${1}_pipeline" code="${1}_cone_detection"
    local dst="/sandbox/ros2_ws/src/${1}_pipeline" src=/sandbox/vendor/ifs09_cone_detection
    local tpl=/sandbox/ros2_ws/templates/ifs09
    if [[ -e "$dst/$code" ]]; then echo "$dst/$code already exists"; return 1; fi
    # Own copy, renamed so it can't clash with the vendored one
    # (perception:=ifs09 keeps running the original, to compare with).
    cp -r "$src/cone_detection" "$dst/$code"
    cp "$src/LICENSE" "$dst/$code/LICENSE"
    cp "$tpl/my_strategy.py" "$dst/$code/strategies/my_strategy.py"
    find "$dst/$code" -name '*.py' -exec sed -i -E \
        "s/^([[:space:]]*)(from|import) cone_detection([. ])/\1\2 ${code}\3/" {} +
    if [[ "$2" == "yes" && -f "$dst/$pkg/perception.py" ]]; then   # keep the student's own
        mv "$dst/$pkg/perception.py" "$dst/$pkg/perception_before_ifs09.py"
        echo "Your old perception.py is now $pkg/perception_before_ifs09.py"
    fi
    sed -e "s/my_cone_detection/$code/g" -e "s/my_pipeline/$pkg/g" "$tpl/perception.py" > "$dst/$pkg/perception.py"
    sed -i -e "s/my_cone_detection/$code/g" "$dst/$code/strategies/my_strategy.py"
    # Packages made before --from existed only list one Python package.
    sed -i -e 's/^from setuptools import setup$/from setuptools import find_packages, setup/' \
           -e 's/packages=\[package_name\],/packages=find_packages(exclude=["test", "test.*"]),/' "$dst/setup.py"
    echo "IFS09 perception copied to $dst/$code (start in $pkg/perception.py)"
}
