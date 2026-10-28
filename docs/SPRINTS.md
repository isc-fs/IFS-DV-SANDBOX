# Sprints

One-week sprints, from **Saturday to Friday**. Each sprint covers one of the
[levels](LEVELS.md) and ends with a short demo at the weekend meeting.

**Every sprint ends with a Pull Request to `dev` in your fork** (green CI,
reviewed by a head) and your row in the [leaderboard](LEADERBOARD.md), the same way the
team works on the real pipeline ([WORKFLOW.md](WORKFLOW.md#your-fork-branches-and-ci)).

**Workload:** about **5–6 hours a week** during term. Every sprint has a
**minimum**, which is enough to keep going, and a **stretch** for anyone with time
left over. If you only reach the minimum, that's fine: the baselines mean
you never block the rest of the pipeline.

## Calendar

| Sprint | Dates | Level | Demo at the end |
|---|---|---|---|
| **S1** | Sat 31 Oct – Fri 6 Nov | Setup + Level 0: drive it, break it | Explain the loop and one failure |
| **S2** | Sat 7 – Fri 13 Nov | Level 1: control | Your control laps the mock |
| **S3** | Sat 14 – Fri 20 Nov | Level 2: planning | Your planning + control lap with noisy cones |
| **S4** | Sat 21 – Fri 27 Nov | IFSSIM race prep *(light week)* | **Race 1, Sat 28 Nov** |
| | 30 Nov – 13 Dec | **Exams: no sprint** | |
| **S5** | Mon 14 – Sun 20 Dec | Catch-up *(optional)* | Pick your holiday stage |
| **H1–H2** | 21 Dec – 6 Jan | Level 3 **or** 4 (localisation), self-paced | Short async update each week |
| **S6** | Sat 9 – Fri 15 Jan | Level 5: your full pipeline | **Final race + pitch, Sat 16 Jan** |

After the final race the sandbox is done, and the second split moves on to
projects on the real pipeline.

---

## S1: setup + Level 0  *(31 Oct – 6 Nov)*

Get the sandbox working on your laptop and understand the loop.

- **Minimum:** [GETTING_STARTED](GETTING_STARTED.md) works on your laptop
  (mock lap seen in Lichtblick). Read [PIPELINE.md](PIPELINE.md). Do the
  "break things" experiments of Level 0.
- **Your fork + first PR (practise the git flow):** set up your fork
  ([WORKFLOW.md](WORKFLOW.md#set-up-your-fork-once)), then branch
  `feat/1-<you>-package`, `sb-new <you>`, `sb-build`, `sb-test`, `sb-check`,
  push. Watch the issue open itself. Open the PR to `dev` of your fork and get CI green.
- **First score:** `sb-score --label <you>` (baseline) to see the leaderboard.
- **Stretch:** IFSSIM running on your laptop with the baseline, driving with
  teleop.
- **Demo (5 min):** draw the loop with topics and frames. Explain one
  failure you caused, stage by stage.

> Most of the trouble in this sprint is installation. If something doesn't
> work, ask in the chat on day 1, not day 6.

## S2: control  *(7 – 13 Nov)*

[Level 1](LEVELS.md#level-1-control--1-week).

- **Minimum:** your `control.py` laps IFSSIM with `perception:=gt`
  without hitting cones (any speed). Iterate on the mock, check in IFSSIM.
- **Stretch:** faster than the baseline (~111 s), with speed that adapts
  to the bends.
- **Demo:** lap in IFSSIM + your Control score next to the baseline and the
  real pipeline (`sb-score perception:=gt control:=<you>/control --scenario ifssim-all`).
  Show one parameter you tuned and what happened.
- **PR:** your control merged into `dev`.

## S3: planning  *(14 – 20 Nov)*

[Level 2](LEVELS.md#level-2-planning--1-week).

- **Minimum:** your `planning.py` + your control lap IFSSIM with
  `perception:=gt --gt-noise`.
- **Stretch:** the full "done when" (`miss_rate 0.2`, `swap_rate 0.05`).
- **Demo:** your path vs the baseline path in Lichtblick, and your Planning
  score (Level 2 command in [LEADERBOARD.md](LEADERBOARD.md)). Which path helps
  control more, and why?
- **PR:** your planning merged into `dev`.

## S4: IFSSIM race prep  *(21 – 27 Nov, light week before exams)*

No new level. Polish what you have in IFSSIM, at higher speed.

- **Minimum:** your planning + control lap IFSSIM with the baseline
  perception and SLAM (not ground truth). Fix what breaks: real LiDAR,
  real latency, noisier cones.
- **Stretch:** start looking at Level 3 or 4 (whichever you'll take over
  the holidays).
- **Race 1 (Sat 28 Nov):** IFSSIM, on a track you haven't seen before, your
  planning + control and baseline everything else, scored with `sb-score`.
  Target: the real pipeline's Overall (100). It's also a check that your setup
  works before the break.

## Exams  *(30 Nov – 13 Dec)*

Nothing is expected. Don't touch the sandbox unless you want a break
from studying.

## S5: catch-up  *(14 – 20 Dec, optional)*

- Finish anything left over from S1–S4.
- **Choose your holiday stage:** perception (Level 3) or localisation,
  odometry + SLAM (Level 4), whichever you like most.
- Short call with the heads to agree your holiday goal.

## H1–H2: winter holidays  *(21 Dec – 6 Jan, self-paced)*

**Pick one**: Level 3 perception or Level 4 localisation, the one you chose
in S5. Not both. Spread the work out as you like over the two weeks;
plan on roughly two normal sprints' worth of work. Take real days off too.

- **Minimum, perception:** your perception laps IFSSIM with processing under
  50 ms, and runs on the real car bag (`--scenario real-bag`) without dropping scans.
- **Minimum, localisation:** your own odometry (step 1 of Level 4) scored
  in IFSSIM, plus your own mapping on perfect odometry (step 2). Try both on
  the real bag too.
- **Stretch:** the full "done when" of your level, and get close to the real
  pipeline's score for that stage.
- **PR:** open before the end of the holidays. Reviews may be slower.
- **Check-in:** one message a week in the chat (what works, what's stuck,
  a screenshot). The heads answer asynchronously, and there will be one
  optional call.

## S6: your full pipeline + what goes to the real one  *(9 – 15 Jan)*

[Level 5](LEVELS.md#level-5-your-pipeline--end-of-split). You work on your
own: your nodes for every stage you built, the baseline for the rest,
**no ground truth anywhere**.

```bash
sb-score odometry:=alice_pipeline/odometry perception:=alice_pipeline/perception \
         planning:=alice_pipeline/planning control:=alice_pipeline/control --scenario ifssim-all
```

- **Minimum:** one clean lap in IFSSIM, no ground truth.
- **Stretch:** speed, then an Overall score over 100 (faster than the real pipeline).
- **Final race (Sat 16 Jan):** unseen track, your full pipeline vs the real
  pipeline on the leaderboard.
- **Pitch (5 min, same day):** the stage where you did best against the real
  pipeline (leaderboard scores), why it's better, and what it would take to
  bring the idea to IFS09-DV-PIPELINE. If the heads agree, open an issue in
  the pipeline repo: that's your first real-pipeline task next split. The
  sandbox code is not merged there; the idea is reimplemented properly.

---

## How a sprint runs

| When | What | Time |
|---|---|---|
| Saturday meeting | Demos of the sprint that ends (5 min each, with the leaderboard) + plan the next one | ~1 h |
| During the week | Work on your `feat/` branch, push at least twice | ~5 h |
| Wednesday | One message in the chat: progress + what's blocking you | 2 min |

- **Stuck for more than an hour?** Ask. Being blocked is the only real way to
  fall behind, because everything else has a baseline to fall back on.
- **Behind?** Do the minimum and skip the stretch. If you missed a minimum,
  keep going with the next sprint and use the baseline for the stage you
  didn't finish. Catch up in S5.
- **Ahead?** Do the stretch goals, then the "Want more?" ideas at the end
  of [LEVELS.md](LEVELS.md), or help someone who's stuck.
- **Deliverable** = a PR to `dev` of your fork with green CI by Friday night, plus
  your `sb-score` row. Merged after review.
