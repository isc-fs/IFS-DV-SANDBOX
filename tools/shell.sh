#!/usr/bin/env bash
# Open a shell inside the sandbox container (Linux / Mac / Git Bash).
# Windows PowerShell or cmd: use tools\shell.cmd instead.
#
#   tools/shell.sh              # interactive shell
#   tools/shell.sh <cmd...>     # run one command with ROS sourced
#
# Open as many as you like: one for sb-launch, one for `sim`, one for
# `ros2 topic echo ...`.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# First run: write .env with the right settings for this machine.
if [[ ! -f .env ]]; then
    case "$(uname -s)" in
        Linux*)  host=127.0.0.1;            tracks="$ROOT/tracks" ;;
        Darwin*) host=host.docker.internal; tracks="$ROOT/tracks" ;;
        *)       host=host.docker.internal; tracks="$(cygpath -m "$ROOT/tracks" 2>/dev/null || echo "$ROOT/tracks")" ;;
    esac
    printf 'IFSSIM_HOST=%s\nIFSSIM_PORT=41451\nHOST_TRACKS_DIR=%s\n' "$host" "$tracks" > .env
    echo "-> wrote .env (IFSSIM_HOST=$host)"
    docker compose up -d --force-recreate sandbox   # (re)create with the new settings
fi

if ! docker compose ps --status running --services 2>/dev/null | grep -qx sandbox; then
    echo "-> starting the sandbox container"
    docker compose up -d sandbox
fi

if [[ $# -eq 0 ]]; then
    exec docker compose exec sandbox bash
fi
exec docker compose exec sandbox /entrypoint.sh "$@"
