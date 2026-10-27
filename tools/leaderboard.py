#!/usr/bin/env python3
"""Print the leaderboard: your benchmark runs vs the reference runs.

    python3 tools/leaderboard.py                    # every scenario with results
    python3 tools/leaderboard.py --scenario mock-A  # just one

References (committed):  benchmark/reference/*.json   (real pipeline, baseline)
Your runs (local):       results/<label>/*.json        (written by sb-score)

Also writes results/LEADERBOARD.md (open it in VS Code with Ctrl+Shift+V).
Only the standard library is needed, so it runs on your computer too.
"""
from __future__ import annotations

import argparse
import glob
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGES = ("odometry", "perception", "slam", "planning", "control")
SHORT = {"baseline": "base", "gt": "GT", "gt-noisy": "GT~", "real": "real", "off": "off"}


def load(pattern: str, reference: bool):
    runs = []
    for p in sorted(glob.glob(pattern)):
        try:
            with open(p) as f:
                r = json.load(f)
        except (OSError, ValueError):
            continue
        if not isinstance(r, dict) or "scenario" not in r:
            continue            # e.g. *-outputs.json: reference data, not a run
        r["_reference"] = reference
        runs.append(r)
    return runs


def setup_str(cfg: dict) -> str:
    parts = []
    for s in STAGES:
        v = cfg.get(s, "baseline")
        v = SHORT.get(v, v.split("/")[0].replace("_pipeline", ""))
        parts.append(v)
    return " / ".join(parts)


def stage_cell(run: dict, stage: str) -> str:
    v = run["config"].get(stage, "baseline")
    if v in ("gt", "gt-noisy"):
        return "GT" if v == "gt" else "GT~"
    score = run.get(stage, {}).get("score")
    if score is None:
        return "-"
    return f"{score:.0f}"


def time_key(r):
    return r["time_s"] if r.get("time_s") is not None else float("inf")


def table(scenario: str, laps: int, runs: list[dict]) -> list[str]:
    # best run per (label, setup)
    best = {}
    for r in runs:
        key = (r["label"], setup_str(r["config"]), r["_reference"])
        if key not in best or time_key(r) < time_key(best[key]):
            best[key] = r
    rows = sorted(best.values(), key=time_key)

    refs = [r for r in rows if r["_reference"] and r.get("time_s") is not None]
    real = [r for r in refs if r["config"].get("perception") == "real"]
    ref_time = min((r["time_s"] for r in (real or refs)), default=None)
    ref_name = "real pipeline" if real else "baseline"

    out = [f"### {scenario}  ({laps} lap{'s' if laps > 1 else ''})", "",
           f"Overall = 100 × reference time / your time (reference: {ref_name}"
           + (f", {ref_time:.1f} s" if ref_time else "") + "). Stage scores 0–100.", "",
           "| # | Who | Setup (O / P / S / Pl / C) | Odometry | Perception | SLAM | Planning | Control | Laps (s) | Cones | Time | Overall |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(rows, 1):
        who = ("★ " if r["_reference"] else "") + r["label"]
        laps_s = ", ".join(f"{t:.1f}" for t in r.get("lap_times_s", [])) or "-"
        if r.get("time_s") is None:
            t, overall = r["status"].replace("dnf: ", "DNF: "), "0"
        else:
            t = f"{r['time_s']:.1f}"
            overall = f"{100 * ref_time / r['time_s']:.0f}" if ref_time else "-"
        out.append(f"| {i} | {who} | {setup_str(r['config'])} | "
                   + " | ".join(stage_cell(r, s) for s in STAGES)
                   + f" | {laps_s} | {r.get('cones_hit', '-')} | {t} | **{overall}** |")
    out.append("")
    return out


def bag_table(scenario: str, runs: list[dict]) -> list[str]:
    """Real-car bag: open loop, no ground truth -> perception/SLAM only."""
    best = {}
    for r in runs:
        key = (r["label"], setup_str(r["config"]), r["_reference"])
        score = r["perception"].get("score") or 0
        if key not in best or score > (best[key]["perception"].get("score") or 0):
            best[key] = r
    rows = sorted(best.values(), key=lambda r: -(r["perception"].get("score") or 0))
    fmt = lambda v, f="{:.0f}": "-" if v is None else f.format(v)  # noqa: E731
    out = [f"### {scenario}  (real car data, open loop)", "",
           "Agreement = vs what the real pipeline detects on the same bag (not truth). "
           "A scan arrives every ~115 ms.", "",
           "| # | Who | Setup (O / P / S) | Perception | latency p95 (ms) | scans answered | agree recall | agree precision | Odometry | odom diff (m) | SLAM | traj. diff (m) |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(rows, 1):
        p, sl, od = r["perception"], r.get("slam", {}), r.get("odometry", {})
        who = ("★ " if r["_reference"] else "") + r["label"]
        setup = " / ".join(setup_str(r["config"]).split(" / ")[:3])
        out.append(f"| {i} | {who} | {setup} | **{fmt(p.get('score'))}** | {fmt(p.get('latency_p95_ms'))} | "
                   f"{fmt(None if p.get('processed_fraction') is None else 100 * p['processed_fraction'], '{:.0f} %')} | "
                   f"{fmt(p.get('agreement_recall'), '{:.2f}')} | {fmt(p.get('agreement_precision'), '{:.2f}')} | "
                   f"{fmt(od.get('score'))} | {fmt(od.get('trajectory_agreement_m'), '{:.2f}')} | "
                   f"{fmt(sl.get('score'))} | {fmt(sl.get('trajectory_agreement_m'), '{:.2f}')} |")
    out.append("")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", nargs="*", help="only these scenarios")
    a = ap.parse_args()

    runs = load(os.path.join(ROOT, "benchmark", "reference", "*.json"), True)
    runs += load(os.path.join(ROOT, "results", "*", "*.json"), False)
    groups = {}
    for r in runs:
        groups.setdefault((r["scenario"], r.get("laps_required", 1)), []).append(r)

    lines = ["# Leaderboard", "",
             "★ = reference run. Setup: what ran as each stage (base = sandbox baseline, "
             "GT = ground truth, GT~ = noisy ground truth, real = IFS09 pipeline, "
             "other = your package).",
             ""]
    for (sc, laps) in sorted(groups):
        if a.scenario and sc not in a.scenario:
            continue
        if laps == 0:
            lines += bag_table(sc, groups[(sc, laps)])
        else:
            lines += table(sc, laps, groups[(sc, laps)])
    text = "\n".join(lines)
    print("\n" + text)
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    path = os.path.join(ROOT, "results", "LEADERBOARD.md")
    with open(path, "w") as f:
        f.write(text + "\n")
    try:  # owned by the repo owner, not root (Linux hosts)
        st = os.stat(ROOT)
        os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


if __name__ == "__main__":
    main()
