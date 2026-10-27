#!/usr/bin/env python3
"""Control the IFSSIM simulator from the command line (replaces Mission Control).

Inside the sandbox shell this is the `sim` command:

    sim status                 is the sim reachable? which track, laps, ...
    sim tracks                 list the tracks in tracks/
    sim load <track.csv>       load a track (the car is placed at the start line)
    sim start [--laps N]       release the brakes: from now on /control_command drives
    sim stop                   emergency brake (EBS). `sim start` releases it again
    sim reset                  car back to the start line, braked
    sim pause | resume         freeze / unfreeze the simulation

Typical session:   sim load skidpad.csv  ->  sb-launch (other terminal)  ->  sim start

Only needs the Python standard library.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRACKS_DIR = os.path.join(ROOT, "tracks")


class Sim:
    """One-line-per-command text protocol over TCP (same as Mission Control uses)."""

    def __init__(self, host: str, port: int):
        try:
            self.sock = socket.create_connection((host, port), timeout=3.0)
        except OSError as e:
            sys.exit(f"Cannot reach the simulator at {host}:{port} ({e}).\n"
                     "Is IFSSIM running? On Windows/Mac IFSSIM_HOST should be host.docker.internal "
                     "(see .env).")
        self.buf = b""

    def cmd(self, line: str, timeout: float = 5.0) -> str:
        self.sock.settimeout(timeout)
        self.sock.sendall((line + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("simulator closed the connection")
            self.buf += chunk
        out, _, self.buf = self.buf.partition(b"\n")
        return out.decode().strip()

    def json(self, line: str, timeout: float = 5.0):
        resp = self.cmd(line, timeout)
        try:
            return json.loads(resp)
        except ValueError:
            return {"raw": resp}


def host_track_path(name: str) -> str:
    """Path of a track as the SIMULATOR (on the host) sees it."""
    if os.path.isabs(name) or ":" in name:
        return name
    base = os.environ.get("HOST_TRACKS_DIR") or TRACKS_DIR
    sep = "\\" if "\\" in base else "/"
    return base.rstrip("/\\") + sep + name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["status", "tracks", "load", "start", "stop", "reset", "pause", "resume"])
    ap.add_argument("track", nargs="?", help="track CSV name (for load)")
    ap.add_argument("--laps", type=int, default=10)
    ap.add_argument("--event", default="trackdrive", choices=["trackdrive", "autocross", "acceleration", "skidpad"])
    ap.add_argument("--host", default=os.environ.get("IFSSIM_HOST", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("IFSSIM_PORT", "41451")))
    a = ap.parse_args()

    if a.command == "tracks":
        for f in sorted(os.listdir(TRACKS_DIR)):
            if f.endswith(".csv"):
                print(f)
        return

    sim = Sim(a.host, a.port)
    if a.command == "status":
        print("ping:", sim.cmd("ping"))
        ref = sim.json("getRefereeState")
        ref.pop("cone_positions", None)
        print(json.dumps(ref, indent=2))
    elif a.command == "load":
        if not a.track:
            sys.exit("usage: sim load <track.csv>   (see `sim tracks`)")
        path = host_track_path(a.track)
        print("loading", path, "...")
        print(sim.json(f"loadTrack {path}", timeout=30.0))
        sim.cmd("activateEbs")          # parked until `sim start`
        print("Track loaded, car braked. Start your pipeline, then: sim start")
    elif a.command == "start":
        print(sim.json(f"setEventType {a.event} {a.laps}"))
        sim.cmd("simResume")
        sim.cmd("releaseEbs")
        sim.cmd("enableApiControl 1")
        print(f"GO: {a.event}, {a.laps} laps. The car now follows /control_command.")
    elif a.command == "stop":
        sim.cmd("activateEbs")
        print("EBS on: car braked and commands ignored. `sim start` to continue.")
    elif a.command == "reset":
        sim.cmd("activateEbs")
        g = sim.json("getStartGatePose")
        if "x" not in g:
            sys.exit("No start pose known: load a track first (sim load ...).")
        sim.cmd(f"simSetVehiclePose {g['x']} {g['y']} {g['z']} {g['qw']} {g['qx']} {g['qy']} {g['qz']}")
        print("Car back at the start line, braked. Restart your pipeline (fresh map!), then: sim start")
    elif a.command == "pause":
        sim.cmd("simPause")
    elif a.command == "resume":
        sim.cmd("simResume")


if __name__ == "__main__":
    main()
