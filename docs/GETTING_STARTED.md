# Getting started

Goal of this page: the car driving a lap on your computer, first on the
built-in mock simulator, then on IFSSIM. Plan for about 30 minutes,
most of it waiting for downloads.

Everything ROS-related runs inside one Docker container. On your
computer you only need Docker, Git, a visualiser and (later) the IFSSIM
binary.

---

## 1. Install

| What | Windows | Linux |
|---|---|---|
| **Git** | [git-scm.com](https://git-scm.com/download/win) (default options) | `sudo apt install git` |
| **Docker** | [Docker Desktop](https://www.docker.com/products/docker-desktop/) ≥ 4.34, WSL 2 backend | [Docker Engine](https://docs.docker.com/engine/install/) + compose plugin |
| **Foxglove** (visualiser, the one the real pipeline uses) | Desktop app from [foxglove.dev/download](https://foxglove.dev/download), sign in with a free account | Same (`.deb`) |
| **IFSSIM** (step 5) | Windows zip from [IFSSIM releases](https://github.com/isc-fs/IFSSIM/releases/latest) | Releases are Windows/Mac only: use the mock sim, or ask the heads |

**Docker Desktop settings (Windows):**
- *Settings → Resources → Network → **Enable host networking***. The
  sandbox needs it to receive the simulator's sensor data, so if your
  version has this checkbox, turn it on and restart Docker Desktop.
- *Settings → Resources*: give Docker at least 4 GB of memory.

> Already did the onboarding? Same requirements, so you're set. Stop the
> onboarding stack before using the sandbox (`docker compose down` in the
> onboarding repo): both talk to the same simulator ports.

## 2. Fork, clone and open the sandbox shell

```powershell
# fork isc-fs/IFS-DV-SANDBOX on GitHub first (docs/WORKFLOW.md), then:
git clone https://github.com/<your-user>/IFS-DV-SANDBOX.git
cd IFS-DV-SANDBOX
tools\shell.cmd                 # Linux/Mac: tools/shell.sh
```

The first run does three things:
1. writes `.env` with the settings for your machine (simulator address,
   path to `tracks/`),
2. builds the Docker image (~5 min, only once),
3. opens a **bash shell inside the container**. The prompt changes to
   `root@sandbox`.

The repo folder is shared with the container (it's `/sandbox` inside),
so you edit files with VS Code on Windows as usual, and the container
sees the changes immediately.

Open as many sandbox shells as you want: run `tools\shell.cmd` again in
another terminal.

## 3. Build

Inside the sandbox shell:

```bash
sb-build
```

Run it again when you **add** a file, a node or a package. Editing an
existing Python file doesn't need a rebuild: restart the launch.

## 4. First drive: the mock simulator

```bash
sb-launch sim:=mock
```

The mock is a tiny 2D simulator inside the container (no Unreal, no GPU),
so you can always test something in seconds. After a few seconds you should see:

```
[monitor]: Started lap timing
[perception]: processing: mean 4.7 ms, max 9.8 ms per scan
...
[monitor]: LAP 1   111.15 s   avg 4.7 m/s   max 4.7 m/s   cones hit: 0
```

**See it:** open Foxglove → *Open connection* → *Foxglove WebSocket* →
`ws://localhost:8765`. Then *Layout → Import from file* →
`foxglove/sandbox.json`. You should see the LiDAR points, the
detected cones (tall), the map (short), the path (green line) and plots
of speed and steering.

`Ctrl+C` stops everything.

## 5. The real simulator (IFSSIM)

1. Unzip the IFSSIM release anywhere and start `IFSSIM.exe`. Leave it running.
2. In a sandbox shell:
   ```bash
   sim status                             # prints a ping reply and the referee state
   sim load track_20260404_013721.csv     # `sim tracks` lists them
   ```
3. In a second sandbox shell:
   ```bash
   sb-launch
   ```
   Wait until the nodes are up (a few seconds).
4. Back in the first shell:
   ```bash
   sim start
   ```

Other `sim` commands: `stop` (emergency brake), `reset` (back to the
start line), `pause`, `resume`. After a `reset`, restart `sb-launch` so
the map starts fresh.

## 6. Real car data (needed from Level 3)

Download `manual_20260920_154527_indexed.mcap` (7.4 GB, 200 s of the IFS-08
driven by hand) from the team drive (ask the heads for the link), and put it
in the repo's `bags/` folder. Git ignores that folder. Check it works:

```bash
sb-score --scenario real-bag          # baseline perception on the real car data
```

> Windows: the repo folder is shared with Docker, and reading 7 GB through
> it can be slow. If the replay stutters, clone the repo inside WSL
> (`\\wsl$\Ubuntu\home\...`) instead of `C:\Users\...`.

---

## Troubleshooting

**`sim status`: "Cannot reach the simulator"**
- Is IFSSIM running?
- Windows: `.env` must say `IFSSIM_HOST=host.docker.internal`.
  Linux: `127.0.0.1`. After editing `.env`, run
  `docker compose up -d --force-recreate sandbox` and open a new shell.

**Sim connects but no LiDAR / the car doesn't react.** Check
`ros2 topic hz /lidar_points`. If nothing arrives, host networking
isn't working: enable it in Docker Desktop (step 1) and restart Docker Desktop.

**`sim load` says the file doesn't exist.** The simulator runs on
Windows and needs a Windows path. Open `.env` and check that
`HOST_TRACKS_DIR` is the full path of the repo's `tracks` folder
(e.g. `C:/Users/you/IFS-DV-SANDBOX/tracks`, forward slashes).

**`\r: command not found` or `bad interpreter` errors.** Git converted line
endings. The repo has a `.gitattributes` against this, but if you cloned
before it existed: `git rm --cached -r . && git reset --hard`.

**`package 'xyz' not found`.** You added a package or an executable:
run `sb-build`.

**Foxglove shows nothing.** Check the connection is `ws://localhost:8765`
and that `sb-launch` is running. In the 3D panel, *Display frame* should
be `map` or `base_link`.

**Everything is slow.** Docker Desktop may be low on memory or CPU
(Settings → Resources). The mock sim is much lighter than IFSSIM.

**Start from a clean build.** `docker compose down -v` deletes the build
volumes. Then reopen the shell and run `sb-build`.

**Can't install the Foxglove app?** Use the web version,
[app.foxglove.dev](https://app.foxglove.dev), in Chrome or Edge: same
connection (`ws://localhost:8765`) and the same layout.
