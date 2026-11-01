# Levels

A track from "I can run it" to "the car drives on code I wrote, and I
know why". At each level you replace one stage with your own node. The
rest stays baseline or ground truth, so the car keeps driving and you
can see the effect of your change.

**When:** you do control, planning and one of perception or localisation
(the stage your idea belongs to), in the order you choose. Dates and goals
are in **[SPRINTS.md](SPRINTS.md)**.

**Measure every level with `sb-score`.** It gives each stage a score and
compares it with the baseline and the real team pipeline. The command for
each level is in [LEADERBOARD.md](LEADERBOARD.md#which-test-for-which-level).

**Two tracks.** "In IFSSIM" below means **track B**
(`track_20260404_013723.csv`, `ifssim-B`): the default, where you test
and iterate. **Track A** (`track_20260404_013721.csv`, `ifssim-A`) is the
hard one: a tight hairpin on a 3 m wide track. Go for it when B works.

There's no single right solution. Every level has a **"done when"** so
you know when you can move on, and **experiments** that show how your
stage affects the others. The experiments teach more than the "done
when" does, so don't skip them.

Before you start: read [PIPELINE.md](PIPELINE.md) and create your package:

```bash
sb-new alice          # -> ros2_ws/src/alice_pipeline with 5 skeleton nodes
sb-build
```

Each skeleton already subscribes and publishes the right topics. You
fill in one function (`find_cones`, `plan`, `compute`...). Write it from
scratch, or copy the baseline and improve it: both are fine.

---

## Level 0: drive it, look at it, break it  *(½ day)*

1. `sb-launch sim:=mock`. Watch a full lap in Foxglove with the
   `sandbox.json` layout.
2. Find each stage's output in the 3D panel: LiDAR points, detections
   (tall cones), map (short cones), ground truth (grey), path, lookahead
   point.
3. `ros2 node list`, `ros2 topic hz` on every contract topic. Write down
   the rate of each one. Which is the fastest? Why?
4. Drive yourself: `sb-launch sim:=mock control:=off`, then in another
   shell `ros2 run sandbox_tools teleop`.
5. **Break things** while it drives (each in a fresh launch). For each,
   predict what will happen *before* you try it:
   - kill planning: `sb-launch sim:=mock planning:=off`
   - perfect vs real odometry: `sb-launch sim:=mock odometry:=gt`, then the
     baseline. Compare `/sandbox/monitor/pose_error`
   - make perception noisy: `sb-launch sim:=mock perception:=gt`, then
     `ros2 param set /gt_cones position_noise 0.3`
   - swap colours: `ros2 param set /gt_cones swap_rate 0.2`
   - go faster: `ros2 param set /control target_speed 9.0`

**Done when** you can draw the loop from memory, with the topic name and
frame on every arrow, and explain one of the failures above stage by stage.

## Level 1: control  *(1 week)*

Your node: `alice_pipeline/control.py` → `compute(path_body, speed)`.

```bash
sb-launch sim:=mock perception:=gt control:=alice_pipeline/control
```

Start with "drive straight slowly", then steering towards the path
(Pure Pursuit, Stanley, your own idea...), then speed.

**Done when:** in IFSSIM, 3 laps in a row with 0 cones hit, faster than the
baseline. Iterate on the mock (`sim:=mock`), validate in IFSSIM.

**Experiments**
- Lookahead vs speed: try short and long lookahead at 4 m/s and at 8 m/s.
  When does it oscillate, when does it cut corners?
- Make steering and speed work *together*: slow down where the path
  bends. How do you know the path bends? (The team calls this the
  curvature of the path.)
- Turn on `position_noise` in `gt_cones`. Noisy cones → noisy path →
  what does your steering do? How can control be robust to it?

## Level 2: planning  *(1 week)*

Your node: `alice_pipeline/planning.py` → `plan(cones_body, colors)`.

```bash
sb-launch sim:=mock perception:=gt planning:=alice_pipeline/planning
```

**Done when:** 3 clean laps in IFSSIM with ground-truth perception made
realistic (`sb-score ... --gt-noise`: `position_noise 0.1`, `miss_rate 0.2`,
`swap_rate 0.05`).

**Experiments**
- Plot your path vs the baseline's (Foxglove, both runs, or record
  bags). Which is smoother? Does control behave differently with each?
- Try the other tracks (`sim tracks` lists them,
  `sb-launch sim:=mock track:=...`).
- What does your planner do with `unknown_rate 1.0` (no colours at all)?

## Level 3: perception  *(1–2 weeks)*

Your node: `alice_pipeline/perception.py` → `find_cones(points)`.

```bash
sb-launch sim:=mock perception:=alice_pipeline/perception
```

**Three ways to start**, agree which with the heads:

- **From scratch**, in the skeleton `sb-new` gave you.
- **From the sandbox baseline**: copy `sandbox_pipeline/perception_baseline.py`
  into your `perception.py` and change it. Short and simple, easy to read.
- **From the real pipeline (IFS09)**, if your idea builds on its code:
  ```bash
  sb-launch sim:=mock perception:=ifs09   # the real cone detection, unchanged
  sb-new alice --from ifs09               # your own copy of it, in your package
  sb-build
  sb-launch sim:=mock perception:=alice_pipeline/perception
  ```
  `--from ifs09` works on the package you already have: it copies the
  pipeline's `cone_detection` code to `alice_pipeline/alice_cone_detection/`
  and makes your `perception.py` run it (your old one is kept as
  `perception_before_ifs09.py`). Put your idea in that code, or as a new
  strategy in `strategies/my_strategy.py`: the same interface as the real
  pipeline, so it can go back there next to `base`. `perception:=ifs09`
  keeps running the original, so you can always compare with it.

  The real perception is colour-blind (its SLAM and planner don't need
  colour), but the sandbox planner does: the sandbox adds a colour per
  cone, like the baseline. From intensity when it tells the colours apart
  (the mock); in IFSSIM it doesn't, so it follows the two track edges
  (`sandbox_tools/geometry.py`, `colour_by_side`). The first start after each
  change takes 10–20 s (Numba compiles the code), then ~2 s.

The mock LiDAR is clean, so **move to IFSSIM as soon as it works there**:
the real sim has many more points, noise and dropout. Its intensity
doesn't tell blue from yellow, so colour comes from geometry there.

**Done when:** in IFSSIM, 3 laps with ≤ 1 cone hit using your perception
(everything else baseline), and processing time always under 50 ms per
scan. **Then the real car:** run it on the real bag
(`sb-score --scenario real-bag perception:=...`): it must keep up with
174 k-point scans and agree with the real pipeline on most cones
([LEADERBOARD.md](LEADERBOARD.md#real-car-data-real-bag)).

**Experiments**
- Compare your detections with the grey ground-truth cones. How far can
  you see cones? How many do you miss at 15 m?
- How much does colour accuracy matter? Run your detector with colours
  forced to UNKNOWN and see what planning does.
- Time budget: what happens to the whole loop if perception takes 150 ms?
  (Add a `time.sleep` to try it.)

## Level 4: localisation (odometry + SLAM)  *(1–2 weeks)*

Your nodes: `alice_pipeline/odometry.py` and `alice_pipeline/slam.py`.
This is the hardest level. Do it in steps, each one runs on its own:

1. **Odometry.** Your own dead reckoning from `/motor_rpm` + `/imu`
   (+ `/steering_angle` if you like). Fill `step()` in `odometry.py`.
   Then make it better than the baseline: handle the gyro bias, combine
   sensors with a small Kalman filter...
   ```bash
   sb-score odometry:=alice_pipeline/odometry --scenario ifssim-all
   ```
   Look at the Odometry score (drift %) and `/sandbox/monitor/pose_error`.
2. **Mapping without drift.** Build the map in your `slam.py` using perfect
   odometry: `sb-launch odometry:=gt slam:=alice_pipeline/slam`. Better
   association than the baseline, rejecting ghost cones.
3. **Correct the drift with the cones.** Switch back to real odometry
   (yours or the baseline). When you see a mapped cone again, the difference
   between where it is in the map and where you see it now tells you how much
   the odometry drifted. Update `map → odom`. Now it's SLAM.
   ```bash
   sb-launch odometry:=alice_pipeline/odometry slam:=alice_pipeline/slam
   ```

**Done when:** 3 laps in IFSSIM with your odometry + your SLAM, no ground
truth anywhere (apart from the odometry's start pose), max pose error under
0.5 m per lap (the monitor prints it), and a SLAM pose error clearly lower
than your odometry's error alone. Then run them on the real bag with your
perception: do your trajectories follow the real EKF's and SLAM's?

**Experiments**
- Odometry: change `rpm_to_ms` by 2 %. How far off is the car after one lap?
  What does that do to the map?
- Use the latest pose instead of the pose at the scan's stamp. What happens
  to the map in corners?
- Second lap: does your map stay one map, or does it start a second copy
  of the track?

## Level 5: your pipeline  *(end of split)*

Your own nodes for every stage you built, the baseline for the rest,
**no ground truth anywhere**, in IFSSIM:

```bash
sb-score odometry:=alice_pipeline/odometry perception:=alice_pipeline/perception \
         planning:=alice_pipeline/planning control:=alice_pipeline/control --scenario ifssim-all
```

Then the real goal: **what from your sandbox work deserves to go into the
real pipeline?** Pick your strongest stage (by its leaderboard score vs the
real pipeline's), explain why it's better, and what it would take to bring
it to IFS09-DV-PIPELINE. If the heads agree, that becomes your first task on
the real pipeline next split. The sandbox code itself never gets merged
there: you reimplement the idea properly in the real pipeline.

---

## Want more?

- **C++**: write one of your nodes in C++ (`ament_cmake` package, same
  topics). The launch doesn't care which language it is. The team is
  porting parts of the real pipeline to C++ this season.
- **LiDAR motion distortion**: `sb-launch skew:=true` publishes
  `/lidar_points_skewed` (what a sweeping LiDAR sees on a moving car, with a
  per-point time field `t`). Can your perception handle it? Can you undo it?
  See `sandbox_tools/skew_model.py`.
- **New tracks**: add CSVs to `tracks/` (format: `colour,x,y,...`, see existing ones).
