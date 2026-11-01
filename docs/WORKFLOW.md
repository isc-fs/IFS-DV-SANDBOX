# Workflow

## Commands cheat sheet

On your computer (Windows: PowerShell or cmd, Linux: terminal):

| | |
|---|---|
| `tools\shell.cmd` / `tools/shell.sh` | open a shell inside the sandbox (run again for more shells) |
| `docker compose down` | stop the container (your files are safe, they live in the repo) |
| `docker compose down -v` | also delete build results (then `sb-build` again) |

Inside the sandbox shell:

| | |
|---|---|
| `sb-build` | build the workspace. Needed after adding files/nodes/packages, not after editing |
| `sb-launch [stage:=...]` | start the pipeline, see below |
| `sb-new <name>` | create `ros2_ws/src/<name>_pipeline` from the template |
| `sb-new <name> --from ifs09` | start your perception from the real pipeline's code ([Level 3](LEVELS.md#level-3-perception--12-weeks)) |
| `sb-test` | run every package's unit tests (what CI runs) |
| `sb-check [stage:=...]` | smoke test: the whole loop drives on the mock (what CI runs) |
| `sb-score [stage:=...] [--scenario ...]` | benchmark + [leaderboard](LEADERBOARD.md) |
| `sim load/start/stop/reset/status` | control IFSSIM |
| `ros2 run sandbox_tools teleop` | drive with the keyboard (launch with `control:=off`) |

## `sb-launch` arguments

| Argument | Values | Default |
|---|---|---|
| `sim` | `ifssim`, `mock`, `none` (when replaying a bag) | `ifssim` |
| `track` | CSV in `tracks/` (mock only, IFSSIM uses `sim load`) | `track_20260404_013723.csv` |
| `odometry` | `baseline`, `gt`, `off`, `pkg/exec` | `baseline` |
| `perception` | `baseline`, `gt`, `ifs09` (the real pipeline's code), `off`, `pkg/exec` | `baseline` |
| `slam`, `planning`, `control` | `baseline`, `off`, `pkg/exec` | `baseline` |
| `params` | path to a parameters YAML | none |
| `skew` | `true` → start `lidar_skewer` | `false` |
| `foxglove` | `false` → no visualisation bridge | `true` |

## Your own package

```bash
sb-new alice
sb-build
sb-launch control:=alice_pipeline/control
```

- To add a node: create `alice_pipeline/alice_pipeline/my_node.py` with a
  `main()`, add `"my_node = alice_pipeline.my_node:main"` to
  `entry_points` in `setup.py`, run `sb-build`, then
  `sb-launch planning:=alice_pipeline/my_node`.
- Need a Python library? `pip install <lib>` inside the container works
  until the container is recreated. If others need it too, ask to add it
  to `docker/Dockerfile`.
- C++ node: create an `ament_cmake` package next to yours (`ros2 pkg create
  --build-type ament_cmake --dependencies rclcpp ...`) and launch it the
  same way.

### Tests for your package

`sb-new` also creates `alice_pipeline/test/test_alice_pipeline.py`.
`sb-test` runs it, and so does CI on every PR. The tests call your plain
functions (`compute`, `plan`, `find_cones`) with hand-made inputs, so they
need no simulator and take milliseconds.

- The tests that are already there check the contract (shapes, ranges).
  They must keep passing.
- One behaviour test starts skipped. Delete its `@pytest.mark.skip` line
  once your control steers.
- Add your own in the same style: a small input where you know the right
  answer. For example: cones only on one side, a hairpin, a scan with no
  cones. When you fix a bug, first add a test that shows it, then fix it.

Run only yours: `sb-test -k alice` (or `cd /sandbox/ros2_ws/src && pytest alice_pipeline/test`).

## Parameters

Every node declares its parameters at the top of its file with a
default. Change them:
- live: `ros2 param set /control target_speed 7.0` (the baselines read
  parameters every cycle),
- for a run: write a YAML (see `sandbox_bringup/config/example_params.yaml`)
  and `sb-launch params:=/sandbox/my_params.yaml`.

## Your fork, branches and CI

**Everyone works in their own fork** of `isc-fs/IFS-DV-SANDBOX`. The main repo
only holds the sandbox itself; your code, branches, issues and PRs live in
your fork. Inside your fork you use the same workflow as the real pipeline
(IFS09-DV-PIPELINE README), so what you practise here is what you'll do there.

### Set up your fork (once)

1. On GitHub, open `isc-fs/IFS-DV-SANDBOX` → **Fork**. **Untick "Copy the
   `main` branch only"**: you need `dev` too.
2. In your fork: **Settings → General → Features → tick Issues** (forks
   start with issues off).
3. In your fork: **Actions tab → "I understand my workflows, enable them"**
   (forks start with Actions off). Without this there's no CI and no
   automatic issues.
4. **Settings → Collaborators → add the heads**, so they can review your PRs.
5. **Settings → Branches**: protect `dev` (require a PR + the CI check).
6. Clone **your fork** (not the main repo):
   ```bash
   git clone https://github.com/<your-user>/IFS-DV-SANDBOX.git
   cd IFS-DV-SANDBOX
   git remote add upstream https://github.com/isc-fs/IFS-DV-SANDBOX.git
   ```

### Working

```
main  ──────────────●───────────────────●──▶   after each race (merge dev → main)
                    ↑                   ↑
dev   ──●───●───●───●───●───●───●───●───●──▶   your work, always green
        ↑   ↑       ↑   ↑       ↑   ↑
     feat/1 fix/1 feat/2 ...  (one per task, deleted after merge)
```

1. **Branch from `dev`** with the next free number of its type:
   ```bash
   git checkout dev && git pull
   git checkout -b feat/3-pure-pursuit      # or fix/2-planner-crash
   git push -u origin feat/3-pure-pursuit
   ```
   Next number = last closed issue with that label + 1 (Issues → label `feat`).
2. **An issue opens by itself** in your fork when you push the branch,
   titled `[feat/3-pure-pursuit] ...`. Your first commit message fills its
   description. If the number is wrong, the issue tells you.
3. **Commit often, push often**, so the heads can see your progress and help.
4. **Open a Pull Request to `dev` of your fork** (not `main`, and not the main
   repo). The template asks for `Closes #<issue>` and how you tested it.
5. **CI runs on the PR** (Actions tab): build, unit tests (`sb-test`) and the
   smoke test (`sb-check`: the whole pipeline must still drive on the mock).
   Run both locally first, it's faster. If you put a line
   `sb-score: <args>` in the PR description, CI also scores that setup and
   puts the leaderboard in the job summary.
6. **Review**: a head reviews it. Fix the comments, push again, and CI reruns.
7. **Merge** when it's green and approved. Delete the branch; the issue
   closes itself.

### Getting sandbox updates

When the heads fix something or add new reference scores in the main repo:
on GitHub, your fork → **Sync fork** (for `main` and `dev`), then `git pull`.
Or from the terminal:
```bash
git checkout dev && git fetch upstream && git merge upstream/dev && git push
```
Your code is in your own package (`ros2_ws/src/<you>_pipeline`), so updates
never conflict with it.

### Tips

- One sprint ≈ one `feat/` branch ≈ one PR. Small PRs get reviewed faster.
- Don't commit `results/`, `bags/` or `.env` (git ignores them).
- Found a bug in the **sandbox itself** (not your code)? Fix it on a branch and
  open the PR to `isc-fs/IFS-DV-SANDBOX` `dev`, so every fork gets it.

Git runs on your computer (VS Code, GitHub Desktop, terminal), not in
the container.

## Recording and replaying (bags)

```bash
# record while driving
ros2 bag record -o /sandbox/bags/run1 /lidar_points /imu /testing_only/odom /testing_only/track /tf_static

# replay later, without any simulator, only the stages you want to test
ros2 bag play /sandbox/bags/run1 --clock          # shell 1
sb-launch sim:=none slam:=off planning:=off control:=off   # shell 2: perception only
```

Bags land in `bags/` in the repo (git ignores it). Great for comparing
two versions of a node on exactly the same data.
