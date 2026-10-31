# IFS-DV-SANDBOX

A small, complete driverless pipeline you can break, rewrite and race.
Built for new ISC Driverless members, after the onboarding exercises.

```
 wheel speed, IMU ──► ODOMETRY ───────┐
                      smooth, drifts   ▼
 LiDAR ──► PERCEPTION ──────────────► SLAM ──► PLANNING ──► CONTROL ──► car
           cones in car frame         pose +   path to      throttle,
                                      cone map follow       steering, brake
```

The onboarding showed you single pieces of the real pipeline. Here you
see **the whole loop**: how the stages feed each other, what each one
needs from the previous one, and what happens downstream when one of
them is bad.

## The idea

- **Everything already works.** Each of the 5 stages has a simple *baseline* node
  (~100–200 lines of Python), and the car laps the track out of the box.
- **Every baseline is naive on purpose.** Each one has clear weak spots.
  Your job is to replace them.
- **Swap one stage at a time.** Run your node for one stage and the baseline
  (or ground truth) for the others, so you always have a car that drives:
  ```bash
  sb-launch planning:=alice_pipeline/planning      # my planner, everything else baseline
  sb-launch perception:=gt                          # skip perception: perfect cones from the sim
  ```
- **The only rule is the [topic contract](docs/TOPICS.md).** Between stages
  the topics, message types and frames are fixed. Inside your node you can
  do anything you like: your own algorithm, any library, C++, a neural net.
  You don't have to copy how the real pipeline does it.

## Quick start

You need **Docker Desktop** (Windows/Mac) or **Docker** (Linux). You don't
need to install ROS on your computer. Full guide with
troubleshooting: **[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md)**.

**Fork this repo first** (one fork per person, setup in
[WORKFLOW.md](docs/WORKFLOW.md#set-up-your-fork-once)), then:

```bash
git clone https://github.com/<your-user>/IFS-DV-SANDBOX.git
cd IFS-DV-SANDBOX

tools\shell.cmd          # Windows (PowerShell or cmd)
tools/shell.sh           # Linux / Mac
```

The first time, this builds the image (about 5 min) and opens a shell
inside the container. Inside it:

```bash
sb-build                 # build the workspace (again after adding files or packages)
sb-launch sim:=mock      # whole pipeline on a tiny built-in simulator, no Unreal needed
```

Within seconds the car drives a lap. The `monitor` prints lap times
and cone hits. To see it, connect Lichtblick to `ws://localhost:8765`
(see the guide).

With the real simulator: start `IFSSIM.exe` by double-clicking it in its own
folder, then open **two terminals on your computer, both in the repo folder**
(`IFS-DV-SANDBOX`), and run `tools\shell.cmd` (Linux/Mac: `tools/shell.sh`) in
each. Both are now shells inside the same container:

```bash
# terminal 1 (sandbox shell)
sim status                             # check it reaches IFSSIM
sim load track_20260404_013721.csv     # load a track

# terminal 2 (sandbox shell)
sb-launch                              # start the pipeline, leave it running

# terminal 1 again, once the nodes are up
sim start                              # release the brakes
```

Step by step, with what to expect: [GETTING_STARTED.md, step 5](docs/GETTING_STARTED.md#5-the-real-simulator-ifssim).

## Where to go next

| | |
|---|---|
| [docs/PIPELINE.md](docs/PIPELINE.md) | **Read first.** Tour of the pipeline: what each stage does, how they connect, frames and timing |
| [docs/LEVELS.md](docs/LEVELS.md) | **What to do.** Learning track: replace the stages one by one until the whole pipeline is yours |
| [docs/SPRINTS.md](docs/SPRINTS.md) | **When.** Weekly sprints, dates, demos and races |
| [docs/LEADERBOARD.md](docs/LEADERBOARD.md) | **How good is it?** Scores for each stage + overall, vs the real pipeline |
| [docs/TOPICS.md](docs/TOPICS.md) | The topic contract (reference) |
| [docs/WORKFLOW.md](docs/WORKFLOW.md) | Commands cheat sheet, your own package, **your fork, branches + CI** (same as the real pipeline), bags |

## What's in the repo

| Path | What | Edit it? |
|---|---|---|
| `ros2_ws/src/sandbox_pipeline/` | Baseline node of each stage | Read it, copy from it |
| `ros2_ws/src/<you>_pipeline/` | **Your** nodes, created with `sb-new <you>` | Yes! |
| `ros2_ws/src/sandbox_tools/` | Helpers (contract, geometry, point clouds) + tools: ground truth, mock sim, visualizer, monitor, teleop | Not needed |
| `ros2_ws/src/sandbox_msgs/` | `Cone` / `ConeArray` messages | No (it's the contract) |
| `ros2_ws/src/sandbox_bringup/` | `sb-launch` (the launch file that wires stages together) | If you want |
| `vendor/` | IFSSIM bridge + `fs_msgs`, copied from the team repos | No |
| `tools/` | `shell.*` (enter the container), `sim.py` (control the simulator), benchmark + leaderboard | No |
| `benchmark/reference/` | Reference scores (real pipeline, baseline) | Heads only |
| `.github/` | CI + branch/issue automation, PR template (same as the real pipeline) | No |
| `tracks/` | Track CSVs | Add your own |
| `lichtblick/sandbox.json` | Visualisation layout | If you want |

---

*ISC Racing Team, Driverless*
