#!/usr/bin/env python3
"""Run benchmark scenarios, score every stage, compare with the references.

Inside the sandbox shell this is `sb-score`:

    sb-score                                     # baseline on mock-A
    sb-score control:=alice_pipeline/control     # your control, rest baseline
    sb-score perception:=gt planning:=alice_pipeline/planning --gt-noise
    sb-score --scenario mock-all                 # every mock track
    sb-score --scenario ifssim-A                 # IFSSIM must be running (loads + starts it for you)
    sb-score --list                              # show scenarios

Each run is saved in results/<label>/ and the leaderboard is printed
(and written to results/LEADERBOARD.md). See docs/LEADERBOARD.md.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")
STAGES = ("odometry", "perception", "slam", "planning", "control")

SCENARIOS = {
    "mock-A": {"sim": "mock", "track": "track_20260404_013721.csv"},
    "mock-B": {"sim": "mock", "track": "track_20260404_013723.csv"},
    "mock-C": {"sim": "mock", "track": "track_20260404_013725.csv"},
    "ifssim-A": {"sim": "ifssim", "track": "track_20260404_013721.csv"},
    "ifssim-B": {"sim": "ifssim", "track": "track_20260404_013723.csv"},
    "ifssim-C": {"sim": "ifssim", "track": "track_20260404_013725.csv"},
    # Real car data (no ground truth): perception (+ SLAM) vs the real pipeline.
    "real-bag": {"sim": "bag", "bag": "bags/manual_20260920_154527_indexed.mcap"},
}
BAG_TOPICS = ["/lidar_points", "/imu", "/motor_rpm", "/steering_angle", "/tf_static"]
GROUPS = {
    "real": ["real-bag"],
    "mock-all": ["mock-A", "mock-B", "mock-C"],
    "ifssim-all": ["ifssim-A", "ifssim-B", "ifssim-C"],
}

# "Realistic" ground-truth perception used by --gt-noise (Level 2).
GT_NOISE = {"position_noise": 0.10, "miss_rate": 0.20, "swap_rate": 0.05}


def own(path: str):
    """Give files we create the owner of the repo (not root) on Linux hosts."""
    try:
        st = os.stat(ROOT)
        os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


def clean_shm():
    """Remove shared-memory segments left by killed ROS processes. They pile
    up in /dev/shm and, once it's full, DDS gets slow and drops messages."""
    subprocess.run(["bash", "-c", "fastdds shm clean"], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=False)


def sim(*args):
    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "sim.py"), *args], check=False)


def write_params(path: str, params: dict):
    with open(path, "w") as f:
        for node, values in params.items():
            f.write(f"{node}:\n  ros__parameters:\n")
            for k, v in values.items():
                f.write(f"    {k}: {v}\n")


def run_bag(name: str, label: str, stage_args: list[str], config: dict, real: bool, record: bool) -> dict:
    sc = SCENARIOS[name]
    bag = os.path.join(ROOT, sc["bag"])
    if not os.path.exists(bag):
        sys.exit(f"bag not found: {sc['bag']}\nDownload it into the repo's bags/ folder "
                 "(see docs/LEADERBOARD.md, 'Real car data').")
    os.makedirs(os.path.join(RESULTS, label), exist_ok=True)
    own(RESULTS)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = os.path.join(RESULTS, label, f"{name}-{stamp}.json")
    reference = os.path.join(ROOT, "benchmark", "reference", f"{name}-outputs.json")
    params_file = os.path.join(tempfile.gettempdir(), f"{name}-params.yaml")
    write_params(params_file, sc.get("params", {}))
    log = os.path.join(tempfile.gettempdir(), f"bench-{name}.log")

    # Open loop: nothing can drive a recorded car, so planning/control are off.
    stages = {s: config[s] for s in STAGES}
    launch = ["ros2", "launch", "sandbox_bringup", "sandbox.launch.py", "sim:=none", "foxglove:=false",
              f"params:={params_file}", "planning:=off", "control:=off"]
    if real:
        launch += ["odometry:=off", "perception:=off", "slam:=off", "real:=true"]
    else:
        launch += [f"odometry:={stages['odometry']}", f"perception:={stages['perception']}",
                   f"slam:={stages['slam']}"]

    print(f"\n=== {name}  ({label})  {' '.join(stage_args) or 'baseline'}  log: {log}", flush=True)
    with open(log, "w") as f:
        proc = subprocess.Popen(launch, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(6.0)
        player = subprocess.Popen(["ros2", "bag", "play", bag, "--clock", "100", "--topics", *BAG_TOPICS],
                                  stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        subprocess.run(["ros2", "run", "sandbox_tools", "bag_scorer", "--ros-args",
                        "-p", "use_sim_time:=true", "-p", f"output:={out}", "-p", f"label:={label}",
                        "-p", f"scenario:={name}", "-p", f"reference:={reference}",
                        "-p", f"record:={'true' if record else 'false'}",
                        "-p", f"config:='{json.dumps(config)}'"], check=False)
    finally:
        for p in (player, proc):
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=15)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
    clean_shm()
    if not os.path.exists(out):
        sys.exit(f"the scorer didn't write a result; see the launch log {log}")
    own(os.path.join(RESULTS, label))
    own(out)
    if record:
        own(reference)
    with open(out) as f:
        return json.load(f)


def run_scenario(name: str, label: str, stage_args: list[str], config: dict, laps: int,
                 params_file: str | None, real: bool) -> dict:
    sc = SCENARIOS[name]
    os.makedirs(os.path.join(RESULTS, label), exist_ok=True)
    own(RESULTS)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = os.path.join(RESULTS, label, f"{name}-{stamp}.json")
    log = os.path.join(tempfile.gettempdir(), f"bench-{name}.log")

    launch = ["ros2", "launch", "sandbox_bringup", "sandbox.launch.py", f"sim:={sc['sim']}",
              "foxglove:=false", *stage_args]
    if sc["sim"] == "mock":
        launch.append(f"track:={sc['track']}")
    if params_file:
        launch.append(f"params:={params_file}")
    if real:
        launch += [f"{s}:=off" for s in STAGES] + ["real:=true"]

    if sc["sim"] == "ifssim":
        sim("load", sc["track"])

    print(f"\n=== {name}  ({label})  {' '.join(stage_args) or 'baseline'}  log: {log}", flush=True)
    with open(log, "w") as f:
        proc = subprocess.Popen(launch, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        if sc["sim"] == "ifssim":
            time.sleep(8.0)                      # let the nodes come up before releasing the car
            sim("start", "--laps", str(laps + 1))
        subprocess.run(["ros2", "run", "sandbox_tools", "scorer", "--ros-args",
                        "-p", "use_sim_time:=true", "-p", f"laps:={laps}", "-p", f"output:={out}",
                        "-p", f"label:={label}", "-p", f"scenario:={name}",
                        "-p", f"config:='{json.dumps(config)}'",
                        "-p", f"start_timeout:={300.0 if real else 120.0}"], check=False)
    finally:
        if sc["sim"] == "ifssim":
            sim("stop")
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        if sc["sim"] == "ifssim":
            sim("reset")
        clean_shm()
    if not os.path.exists(out):
        sys.exit(f"the scorer didn't write a result; see the launch log {log}")
    own(os.path.join(RESULTS, label))
    own(out)
    with open(out) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage_args", nargs="*", help="launch arguments, e.g. control:=alice_pipeline/control")
    ap.add_argument("--scenario", default="mock-A", help="scenario name or group (see --list)")
    ap.add_argument("--label", default=os.environ.get("SANDBOX_USER", "me"), help="your name in the leaderboard")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--gt-noise", action="store_true", help="with perception:=gt, make it realistic (noise, misses, swaps)")
    ap.add_argument("--real", action="store_true", help="score the REAL pipeline (started separately, see tools/benchmark_real.sh)")
    ap.add_argument("--record-reference", action="store_true",
                    help="(heads) with --real on a bag scenario: save the real pipeline's outputs as the reference")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    if a.list:
        for k, v in SCENARIOS.items():
            print(f"{k:10s} {v['sim']:7s} {v.get('track') or v.get('bag')}")
        for k, v in GROUPS.items():
            print(f"{k:10s} = {' '.join(v)}")
        return

    names = GROUPS.get(a.scenario, [a.scenario])
    for n in names:
        if n not in SCENARIOS:
            sys.exit(f"unknown scenario {n!r}; try --list")

    config = {s: "baseline" for s in STAGES}
    for arg in a.stage_args:
        k, _, v = arg.partition(":=")
        if k in config:
            config[k] = v
        if k == "params":
            sys.exit("params:= is not allowed in sb-score (scores must be comparable); use --gt-noise for noisy GT")
    if a.real:
        config = {s: "real" for s in STAGES}
    if a.gt_noise:
        if config["perception"] != "gt":
            sys.exit("--gt-noise only makes sense with perception:=gt")
        config["perception"] = "gt-noisy"
    params_file = None
    if a.gt_noise:
        params_file = os.path.join(tempfile.gettempdir(), "gt_noise.yaml")
        with open(params_file, "w") as f:
            f.write("gt_cones:\n  ros__parameters:\n")
            for k, v in GT_NOISE.items():
                f.write(f"    {k}: {v}\n")

    for n in names:
        if SCENARIOS[n]["sim"] == "bag":
            cfg = dict(config)
            if not a.real:   # open loop: planning/control can't run; SLAM only if you ask for it
                cfg["planning"] = cfg["control"] = "off"
                if not any(arg.startswith("slam:=") for arg in a.stage_args):
                    cfg["slam"] = "off"
            res = run_bag(n, a.label, a.stage_args, cfg, a.real, a.record_reference)
            print(f"--> {n}: {res['status']}, perception {res['perception']['score']}")
            continue
        res = run_scenario(n, a.label, a.stage_args, config, a.laps, params_file, a.real)
        print(f"--> {n}: {res['status']}, time {res['time_s']}, cones hit {res['cones_hit']}")

    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "leaderboard.py"),
                    "--scenario", *names], check=False)


if __name__ == "__main__":
    main()
