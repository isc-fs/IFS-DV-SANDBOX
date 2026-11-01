# Sprints

Sprints run from **Saturday to Friday** and end with a short demo at the
weekend meeting.

**Every block ends with a Pull Request to `dev` in your fork** (green CI,
reviewed by a head) and your row in the [leaderboard](LEADERBOARD.md), the same way the
team works on the real pipeline ([WORKFLOW.md](WORKFLOW.md#your-fork-branches-and-ci)).

**Workload:** about **5–6 hours a week** during term. Every block has a
**minimum**, which is enough to keep going, and a **stretch** for anyone with time
left over. If you only reach the minimum, that's fine: the baselines mean
you never block the rest of the pipeline.

## Your stage and your order

**Your stage** is the one your algorithm idea belongs to: perception
([Level 3](LEVELS.md#level-3-perception--12-weeks)) or localisation
([Level 4](LEVELS.md#level-4-localisation-odometry--slam--12-weeks)).
You build your idea into that stage. The other stage stays baseline (it's
in "Ahead?" below if you have time left).

Over the split you do three [blocks](#the-blocks): **control**, **planning**
and **your idea**. Every stage has a baseline, so the car keeps driving
whatever order you do them in. You choose one of two orders at the setup
meeting:

| Sprint | Dates | **Ideas first** | **Basics first** |
|---|---|---|---|
| Setup | Sat 31 Oct, at the meeting | Install + first drive + present your idea | same |
| **S1** | Sat 31 Oct – Fri 6 Nov | **Your idea** *(week 1 of 2)* | Control |
| **S2** | Sat 7 – Fri 13 Nov | **Your idea** *(week 2 of 2)* | Planning |
| | Sat 14 Nov, at the meeting | No-ground-truth run, together | same |
| **S3** | Sat 14 – Fri 20 Nov | Control | **Your idea** *(week 1 of 2)* |
| **S4** | Sat 21 – Fri 27 Nov | Planning | **Your idea** *(week 2 of 2)* |
| | Sat 28 Nov, at the meeting | Demos + holiday plan, last meeting before exams | same |
| | 30 Nov – 20 Dec | **Exams: no sprint** | |
| **H1–H2** | 21 Dec – 6 Jan | Optional: improve your idea, or catch up | same |
| **S5** | Sat 9 – Fri 15 Jan | Your full pipeline | same |
| | Sat 16 Jan | **Final race + pitch of your idea** | same |

**Ideas first:** you start on what you presented, while it's fresh, and
have more time after it to iterate before the pitch. The start is steeper:
your first PR is perception or localisation.

**Basics first:** you learn the loop on the easier stages first, then do
your idea with that experience.

Both orders finish everything before exams, so the holidays are free.

After the final race the sandbox is done, and the second split moves on to
projects on the real pipeline.

---

## Setup  *(Sat 31 Oct, at the meeting)*

We do this together, so problems get fixed in the room.

- **Present your algorithm idea** and get feedback from the heads. Agree:
  - your stage;
  - where you start: the real pipeline's code (`sb-new <you> --from ifs09`),
    a copy of the sandbox baseline, or your own node from scratch (see
    [Your idea](#your-idea--2-sprints));
  - what will prove it works: a lap in IFSSIM (closed loop) or the real
    car bag (real LiDAR, but open loop);
  - your order.
- [GETTING_STARTED](GETTING_STARTED.md) steps 1–5: the mock lap in
  Foxglove **and** the baseline lapping IFSSIM. Don't stop after the mock:
  every block is checked in IFSSIM.
- Set up your fork ([WORKFLOW.md](WORKFLOW.md#set-up-your-fork-once)) and
  create your package: `sb-new <you>`, `sb-build`, `sb-test`, `sb-check`.
- First score: `sb-score --label <you>` (baseline) to see the leaderboard.
- **Start the real car bag download** ([step 6](GETTING_STARTED.md#6-real-car-data-needed-from-level-3),
  7.4 GB). Ideas first: you need it in week 1.
- At home, in the first days: read [PIPELINE.md](PIPELINE.md) and do the
  "break things" experiments of Level 0 in [LEVELS.md](LEVELS.md).
  They show you how each stage affects the others.

Your first block is also your first PR, so you practise the git flow:
branch `feat/1-<you>-<stage>` (e.g. `feat/1-alice-control` or
`feat/1-alice-perception`), push, watch the issue open itself, open the PR
to `dev` of your fork and get CI green.

> If something still doesn't work after the meeting, ask in the chat
> straight away, not at the end of the week.

## Sat 14 Nov: no-ground-truth run  *(at the meeting, after the demos)*

We do this together, like the setup day. Run everything you've built so
far with the baseline for the rest, **no ground truth**:

```bash
# basics first: your planning + control on baseline perception and SLAM
sb-score planning:=<you>/planning control:=<you>/control --scenario ifssim-all
# ideas first: your stage drives the car, e.g.
sb-score perception:=<you>/perception --scenario ifssim-all
```

Fix what breaks: real LiDAR, real latency, noisier cones. From now on
that's the setup you build on. Basics first: also check the real car bag
works (`sb-score --scenario real-bag`).

## Sat 28 Nov: demos + holiday plan  *(last meeting before exams)*

- **Demos** of the blocks that end in S4 (5 min each, with the leaderboard).
  Ideas first: your planning. Basics first: your idea.
- **Holiday plan (optional):** if you want to work over the holidays, agree
  with the heads what you'll do and what counts as done, since feedback
  is slower then.
- **Check your setup works on its own:** your pipeline builds and runs on
  your laptop, and your S4 PR is open.

## Exams  *(30 Nov – 20 Dec)*

Nothing is expected. Don't touch the sandbox unless you want a break
from studying.

## H1–H2: winter holidays  *(21 Dec – 6 Jan, optional)*

Nothing is required: take the break. If you want to work, pick one:

- **Improve your idea:** more experiments, better scores for the pitch.
- **Catch up:** finish a minimum you missed in S1–S4.
- **Go further:** the other stage (Level 3 or 4), or "Want more?" in
  [LEVELS.md](LEVELS.md).

Reviews are slower. One optional call with the heads, and you can post in
the chat whenever you're stuck.

## S5: your full pipeline + your idea  *(9 – 15 Jan)*

[Level 5](LEVELS.md#level-5-your-pipeline--end-of-split). You work on your
own: your control, your planning and your stage (with your idea), the
baseline for the rest, **no ground truth anywhere**. For example, with
perception as your stage:

```bash
sb-score perception:=alice_pipeline/perception planning:=alice_pipeline/planning \
         control:=alice_pipeline/control --scenario ifssim-all
```

- **Minimum:** one clean lap of track B in IFSSIM, no ground truth. Then track A.
- **Stretch:** speed, then an Overall score over 100 (faster than the real pipeline).
- **Final race (Sat 16 Jan):** unseen track, your full pipeline vs the real
  pipeline on the leaderboard.
- **Pitch (5 min, same day):** your idea. What it does, how it scored
  against the baseline and the real pipeline, and what it would take to
  bring it to IFS09-DV-PIPELINE. If the heads agree, open an issue in the
  pipeline repo: that's your first real-pipeline task next split. The
  sandbox code is not merged there; the idea is reimplemented properly.

---

## The blocks

Each block's minimum, stretch and demo, whichever sprints you do it in.

### Control  *(1 sprint)*

[Level 1](LEVELS.md#level-1-control--1-week).

- **Minimum:** your `control.py` laps track B in IFSSIM with `perception:=gt`
  without hitting cones (any speed). Iterate on the mock, check in IFSSIM.
- **Stretch:** faster than the baseline on track B (118.8 s), with speed
  that adapts to the bends. Then track A.
- **Demo:** lap in IFSSIM + your Control score next to the baseline and the
  real pipeline (`sb-score perception:=gt control:=<you>/control --scenario ifssim-B`).
  Show one parameter you tuned and what happened.
- **PR:** your control merged into `dev`.

### Planning  *(1 sprint, after control)*

[Level 2](LEVELS.md#level-2-planning--1-week).

- **Minimum:** your `planning.py` + your control lap track B in IFSSIM with
  `perception:=gt --gt-noise`.
- **Stretch:** the full "done when" (`miss_rate 0.2`, `swap_rate 0.05`).
- **Demo:** your path vs the baseline path in Foxglove, and your Planning
  score (Level 2 command in [LEADERBOARD.md](LEADERBOARD.md)). Which path helps
  control more, and why?
- **PR:** your planning merged into `dev`.

### Your idea  *(2 sprints)*

Your idea, built into your stage and measured.

**Where you start** (agreed at setup):
- **Your idea builds on the real pipeline's code** (perception, for now):
  `sb-new <you> --from ifs09` copies IFS09's `cone_detection` into your
  package, and your perception runs it. Change the part your idea is
  about, or add a strategy. `perception:=ifs09` keeps the original to
  compare with ([Level 3](LEVELS.md#level-3-perception--12-weeks)).
- **Your idea changes one part of the simple version:** copy the sandbox
  baseline node for your stage into your package and change that part.
- **Your idea replaces the whole stage:** write your own node. Until it
  works, the baseline keeps the car driving.

- **Minimum:** your stage, with your idea in it, meets the level's minimum,
  and is scored with `sb-score` next to the baseline and the real pipeline
  on whichever test proves it (IFSSIM, the real bag, or both):
  - **perception:** laps IFSSIM with processing under 50 ms, and runs on
    the real car bag (`--scenario real-bag`) without dropping scans;
  - **localisation:** your odometry (step 1 of Level 4) scored in IFSSIM,
    plus your mapping on perfect odometry (step 2). Try both on the real
    bag too.
- **Stretch:** the full "done when" of your level, and beat the real
  pipeline's score for your stage.
- An idea that doesn't beat the baseline is still a good result if you can
  show why with the scores. That's worth pitching too.
- **After the first sprint:** short progress update at the meeting (no demo).
- **Demo:** your scores next to the baseline and the real pipeline. Which
  part of your idea made the difference?
- **PR:** your idea merged into `dev`.

---

## How a sprint runs

| When | What | Time |
|---|---|---|
| Saturday meeting | Demos of the blocks that end (5 min each, with the leaderboard) + plan the next one | ~1 h |
| During the week | Work on your `feat/` branch, push at least twice | ~5 h |
| Wednesday | One message in the chat: progress + what's blocking you | 2 min |

- **Stuck for more than an hour?** Ask. Being blocked is the only real way to
  fall behind, because everything else has a baseline to fall back on.
- **Behind?** Do the minimum and skip the stretch. If you missed a minimum,
  move on to your next block and use the baseline for the stage you
  didn't finish. Don't try to do two blocks at once: catch up over the
  holidays if you want, and the baseline covers you either way.
- **Ahead?** Do the stretch goals, then the other stage (Level 3 or 4) or
  the "Want more?" ideas at the end of [LEVELS.md](LEVELS.md), or help
  someone who's stuck.
- **Deliverable** = a PR to `dev` of your fork with green CI by Friday night
  at the end of each block, plus your `sb-score` row. Merged after review.
