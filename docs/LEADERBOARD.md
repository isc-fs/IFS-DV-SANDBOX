# Leaderboard

How good is your stage, and how does it compare with the **real team
pipeline** (IFS09-DV-PIPELINE) and the sandbox baseline? `sb-score`
runs a standard test, measures every stage against the simulator's
ground truth, and prints a table.

```bash
sb-score control:=alice_pipeline/control perception:=gt     # one test (mock-A)
sb-score control:=alice_pipeline/control perception:=gt --scenario mock-all
sb-leaderboard                                              # show the table again
```

Your runs are saved in `results/<label>/` (default label: `me`, change it
with `--label alice`). The table is also written to
`results/LEADERBOARD.md`: open it in VS Code with `Ctrl+Shift+V`.

## What a row looks like

```
| # | Who            | Setup (O / P / S / Pl / C)       | Odometry | Perception | SLAM | Planning | Control | Laps (s) | Cones | Time  | Overall |
| 1 | ★ real-pipeline | real / real / real / real / real |    92    |   71       |  88  |    90    |   84    | 64.2     | 0     | 64.2  | **100** |
| 2 | me             | base / GT / base / base / alice  |    97    |   GT       |  90  |    91    |   86    | 80.3     | 1     | 82.3  | **78**  |
| 3 | ★ baseline      | base / base / base / base / base |    97    |   86       |  85  |    92    |   79    | 111.2    | 0     | 111.2 | **58**  |
```
*(made-up numbers)*

- **★** = reference run, committed in `benchmark/reference/`.
- **Setup**: what ran as each stage (odometry / perception / SLAM / planning / control). `base` = sandbox baseline, `GT` =
  ground truth, `GT~` = noisy ground truth, `real` = the team pipeline,
  anything else = a package.
- **Stage scores** (0–100) measure how good each stage's output is. A
  stage that was ground truth shows `GT` instead of a score.
- **Time** = lap time + 2 s per cone hit (Formula Student rule).
  DNF (didn't finish) = car stopped, left the track or ran out of time.
- **Overall** = 100 × reference time / your time. The reference is the real
  pipeline when it has a run on that test (otherwise the baseline). So 100 =
  as fast as the real pipeline, more than 100 = faster.
  On the mock, Overall only measures lap time on a simplified car with
  unlimited grip, so the baseline beats the real pipeline there. IFSSIM and
  the car bag are the tests that count.

## Which test for which level

**IFSSIM is the test that counts.** Use the mock (`mock-*`) to iterate
quickly; the scores you report and compare are on IFSSIM (`ifssim-*`,
IFSSIM must be running: `sb-score` loads the track and starts and stops
the car for you) and, for perception and SLAM, on the real car bag.

Compare like with like: run the same setup the references used.

**Your computer changes some scores.** Your code runs at the speed of your
PC, so latency, % of scans answered and (a bit) lap times depend on it.
Accuracy scores (detections, position error, drift, map, path) mostly
don't. The ★ rows were measured on the heads' PC, so:
- for a fair comparison with the baseline, also score it on your PC:
  run the same `sb-score` command without your `<you>/...` stages (keep
  `perception:=gt` and the like) and add `--label baseline-mine`, e.g.
  `sb-score perception:=gt --label baseline-mine`;
- the real pipeline can't easily run on your PC: compare with its
  accuracy scores, and read its latency and times as "on the heads' PC".

| Level | Command (`--scenario ifssim-all`, or `mock-all` while iterating) | Compare your stage score with |
|---|---|---|
| 1 Control | `sb-score perception:=gt control:=<you>/control` | ★ baseline `GT/base/base/base` and the real pipeline's Control |
| 2 Planning | `sb-score perception:=gt planning:=<you>/planning control:=<you>/control --gt-noise` | ★ baseline `GT~/base/base/base` and the real pipeline's Planning |
| 3 Perception | `sb-score perception:=<you>/perception`, **and** `sb-score --scenario real-bag perception:=<you>/perception` | ★ baseline and the real pipeline's Perception. Started from the IFS09 code? Also run `perception:=ifs09` on your PC: same code before your change, same computer |
| 4 Localisation | `sb-score odometry:=<you>/odometry`, then `sb-score odometry:=<you>/odometry slam:=<you>/slam`, **and** `sb-score --scenario real-bag odometry:=<you>/odometry perception:=<you>/perception slam:=<you>/slam` | the real pipeline's Odometry (its EKF) and SLAM |
| 5 Everything | `sb-score perception:=... slam:=... planning:=... control:=...` | the real pipeline's Overall: can you beat 100? |

The real pipeline's stage scores come from its full run (its own
perception feeding its own SLAM, and so on), so they're the target
for each stage.

## Real car data: `real-bag`

`bags/manual_20260920_154527_indexed.mcap`: 200 s of the IFS-08 driven by
hand. Real Hesai LiDAR (174 k points per scan, with per-point `ring` and
`timestamp` fields), IMU, motor RPM, steering.
It's 7.4 GB, so it's not in git: **download it from the team drive into
the repo's `bags/` folder** (ask the heads for the link).

```bash
sb-score --scenario real-bag perception:=alice_pipeline/perception
sb-score --scenario real-bag perception:=alice_pipeline/perception slam:=alice_pipeline/slam
```

The car was driven, not your code, so this is **open loop**: only perception
(odometry, and SLAM if you pass it) run, and nothing is scored for planning, control
or lap time. There's no ground truth either, so the bag measures:

| Metric | What it tells you |
|---|---|
| latency p50 / p95 / max | from scan arrival to your cones: can you keep up? A scan arrives every ~115 ms |
| processed fraction | % of scans you answered (dropped scans = blind car) |
| agreement recall / precision | of the cones the real pipeline found, how many you found (and how many of yours it found too), within 0.5 m |
| odometry / SLAM trajectory agreement | mean distance between your odometry and the real EKF's, and between your SLAM pose and the real SLAM's |

Perception score on the bag = 100 × agreement F1 × speed factor, where the
speed factor goes from 0.5 (too slow, drops scans) to 1 (≤ 50 ms and every
scan answered). Finding the cones comes first: fast and empty scores 0.
The real pipeline agrees with itself by definition, so its score only shows
its speed. Match its detections and beat its timing to pass it. Odometry and SLAM scores on the bag = 100 at 0 m of trajectory disagreement, 0 at 2 m.
**Agreement isn't truth**: the real pipeline also makes mistakes, so look
at your detections in Lichtblick on top of the point cloud too.

**The real ground is not where the sim taught you.** The TF says the LiDAR
is 0.913 m up, but in the data the ground is ~0.5 m below the sensor next
to the car and rises with distance: the LiDAR is pitched. A fixed ground
height (like the baseline's) finds almost nothing here. Fit the ground
plane each scan (RANSAC, onboarding exercise 5), as the real pipeline does.

"latency" on the bag is measured from the moment the scan arrives. The Hesai
stamps each scan at its start, about one sweep (~115 ms) before it arrives,
and nobody can remove that part.

## What each score measures

Everything is measured against the simulator's ground truth while the car
drives. The formulas are in `sandbox_tools/scoring.py`.

| Stage | Metrics (all in the JSON of each run) | Score |
|---|---|---|
| **Perception** | *recall*: % of real cones closer than 15 m that you detect · *precision*: % of your detections that are real cones (within 0.5 m) · *position error* · *latency* (scan time → your message) · colour accuracy | 35 % recall + 25 % precision + 20 % position (0 at 0.3 m) + 20 % latency (full up to 50 ms, 0 at 150 ms). Position and latency are multiplied by recall |
| **Odometry** | *error* vs ground truth (mean, max) · yaw error · *drift %* = max error / distance driven | 100 at 0 % drift, 0 at 3 % |
| **SLAM** | *pose error* (mean, max) · yaw error · *map recall*: % of the cones the car passed that are in your map · *map precision* · *map error* | 40 % pose (0 at 0.5 m mean) + 20 % map recall + 20 % map precision + 20 % map position (0 at 0.3 m) |
| **Planning** | *centre offset*: how far your path points (first 15 m) are from the middle of the track · *outside*: % of path points off the track · *length*: how far ahead the path goes | 50 % centring (0 at 0.75 m) + 30 % inside the track + 20 % length (full at 15 m) |
| **Control** | *centre offset* of the car · cones hit · *steering rate* RMS (how jerky) · mean speed | 50 % tracking (0 at 0.75 m) + 30 % cones (0 at 5) + 20 % smoothness (0 at 3 /s) |
| **Overall** | lap time(s), cones hit | 100 × reference time / your corrected time |

Notes:
- **Colour accuracy isn't in any score.** The real pipeline doesn't
  classify colours (its planner doesn't need them), so it would be unfair.
  It's still in the JSON if you want to track it.
- Stage scores reward quality, not speed. Speed is what Overall is
  about. A careful slow car gets good stage scores and a low Overall.
- Upstream stages affect downstream scores: bad perception hurts the
  SLAM and planning scores too. That's the point of comparing with the
  same setup.

## Where the reference rows come from

- **Baseline:** run with `sb-score --label baseline` for each setup in the
  table above.
- **Real pipeline:** `tools/benchmark_real.sh <scenarios>` (for the heads), e.g.
  `tools/benchmark_real.sh ifssim-A ifssim-B real-bag`.
  It runs the real IFS09 stack in its Docker image next to the sandbox,
  and `real_adapter` republishes its outputs on the sandbox topics so the
  same scorer measures it.

On the bag, the real pipeline's outputs are saved as
`benchmark/reference/real-bag-outputs.json`: that's what "agreement" is
measured against.

The heads update the references when the real pipeline changes. Copy new
JSONs from `results/` into `benchmark/reference/` and open a PR.
