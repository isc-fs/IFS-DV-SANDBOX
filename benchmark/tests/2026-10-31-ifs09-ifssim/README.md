# IFS09 perception on IFSSIM: test of 2026-10-31

What was checked, on the heads' PC (Windows 10, GTX 1660 SUPER, Docker
Desktop), branch `feat/3-ifs09-perception`, IFSSIM local build (`dev`,
`Saved/StagedBuilds/Windows`), tracks A and B (`ifssim-all`):

| Question | Answer |
|---|---|
| Does the car drive with `perception:=ifs09`? | **No, but IFS09 isn't the reason.** The car leaves the track on the first tight bends, and it does the same with baseline perception and with ground-truth cones. The sandbox baseline control can't do an IFSSIM lap yet (see below). |
| Is its score close to the real pipeline's? | **Detection: yes. Latency: no.** Recall, precision and position error match the real pipeline. The perception score is 75 against 84, and the whole gap is latency (p95 100 ms against 33 ms). |
| Is it fast enough (< 50 ms processing)? | **Yes on IFSSIM:** 22 ms mean, 26 ms p95 per scan (93 k points). **No on the real bag:** 87 ms mean, 108 ms p95 (174 k points). |
| Does a student copy (`sb-new <you> --from ifs09`) behave the same? | **Yes.** On the same scans it finds the same cones, and it differs from the vendored code no more than the vendored code differs from itself. It also scores the same in the full loop. |

## 1. Perception scores vs the real pipeline

`sb-score --scenario ifssim-all perception:=ifs09` (everything else baseline):

| Track | Who | Recall | Precision | Position error | Latency p95 | Perception score |
|---|---|---|---|---|---|---|
| A | ifs09 in the sandbox | 0.83 | 0.99 | 0.08 m | 100 ms | 75 |
| A | ★ real pipeline | 0.82 | 1.00 | 0.05 m | 33 ms | 84 |
| B | ifs09 in the sandbox | 0.83 | 0.99 | 0.06 m | 100 ms | 75 |
| B | ★ real pipeline | 0.82 | 1.00 | 0.05 m | 33 ms | 84 |

- The detection numbers are the real pipeline's, so the vendored code and the
  sandbox glue (`sandbox_tools/ifs09.py`) are right.
- The latency is not. The node's own processing is ~22 ms, but scan → cones
  measures 100 ms at p95, against 33 ms for the real pipeline (★ reference of 28 Oct).
  The missing ~70 ms is outside the detector: receiving the ~1.4 MB cloud,
  queueing, or the other sandbox nodes competing for CPU. **Not investigated
  yet.** At ≤ 50 ms the sandbox would score ~84.
- The colour comes from intensity, and IFSSIM sends intensity 0 on every point.
  So colour falls back to the side (left = blue, right = yellow): colour
  accuracy is 0.62. That's not scored, but the sandbox planner pairs blue with
  yellow, so on bends it gets wrong colours.
- Clustering: the log says `cluster_backend=pypi unavailable ... falling
  back to grid/sklearn`. The real pipeline image (`ifs09-dv-pipeline:dev-f58d9e5`)
  doesn't have `dbscan` either, so both run the same fallback. Harmless.

## 2. Why the car doesn't drive (not IFS09)

Track A, same command with different perceptions:

| Perception | Result | Planning | Control |
|---|---|---|---|
| `ifs09` | DNF, left the track, 2 cones | 41 | 18 |
| baseline | DNF, car stopped, 1 cone | 60 | 24 |
| ground truth (perfect cones) | DNF, car stopped, 6 cones | 80 | 0 |

With ground truth, odometry and SLAM track the true pose to ~0.1 m and there's
always a path. At 5 m/s the baseline control saturates the steering (+1.0)
on the first tight bend, then swings to −1.0, and clips the cones. At
`target_speed: 3.0` it gets about a minute in (1 cone), then fails on a
later hairpin. The steering geometry matches the sim (22.4°, 1.627 m). The
mock car has unlimited grip, which is why this never showed up there. There
are no baseline IFSSIM reference rows either: the baseline stack has not
finished an IFSSIM lap so far.

**To do:** tune or fix the baseline control (and check the planner's path in
hairpins) on IFSSIM, then rerun `sb-score --scenario ifssim-all perception:=ifs09`.
Until then, students can't get a lap time on IFSSIM with any perception.

## 3. Speed (processing per scan)

`compare_copy.py`, the detector alone on recorded scans:

| Data | Points / scan | Mean | p95 | Max |
|---|---|---|---|---|
| IFSSIM, 200 scans while driving | 93 k | 22 ms | 26 ms | 55 ms |
| Real car bag, 80 scans | 174 k | 87 ms | 108 ms | 145 ms |

Under 50 ms on IFSSIM, but not on the real-car data, where the point clouds
are almost twice as dense.

## 4. Student copy (`sb-new <you> --from ifs09`)

- `sb-new zztest --from ifs09` → `sb-build` → `sb-test`: builds, 10 passed / 1 skipped.
- The copied code is identical to `vendor/ifs09_cone_detection` except
  for the renamed imports.
- The same scans through the vendored detector twice, and through the student copy.
  The detector uses random RANSAC, so not even the vendored code matches itself bit for bit:

| Data | | Cones | Unmatched (> 10 cm) | Max position difference |
|---|---|---|---|---|
| IFSSIM (100 scans) | vendored vs vendored | 1191 | 0 | 0.7 cm |
| IFSSIM (100 scans) | vendored vs student | 1191 | 0 | 0.5 cm |
| real bag (40 scans) | vendored vs vendored | 387 | 137 | 9.8 cm |
| real bag (40 scans) | vendored vs student | 389 | 106 | 9.3 cm |

- In the full loop (`sb-score --scenario ifssim-all perception:=zztest_pipeline/perception`):
  track A recall 0.83, precision 0.99, score 72 (ifs09: 75), with the same DNF.
  Track B ended earlier at a different point (score 85), so it isn't a
  like-for-like comparison.
- The bag shows that the IFS09 detector is not deterministic run to run on real
  data: about 1 in 3 cones changes between two runs on the same scan. Worth
  knowing when a student compares "before / after my change": run it more
  than once.

## Gotcha for whoever runs IFSSIM next

Start `IFSSIM.exe` **from its own folder** (double-click it, or
`Start-Process ... -WorkingDirectory <folder>`). Started from another folder,
it doesn't find `settings.json` and silently uses a default LiDAR: ~560 flat
points per scan, mount (0, 0, 0). The car then sees no cones and never moves.

## Files

- `runs/`: the JSON of every `sb-score` run above (`ifs09-`, `baseline-`, `gt-`, `student-copy-`).
- `compare_copy.py`: the student-copy comparison and timing.
