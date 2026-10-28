# Open a shell inside the sandbox container (Windows PowerShell).
# Easiest: double-click tools\shell.cmd, or run it from a terminal.
#
#   tools\shell.cmd             # interactive shell
#   tools\shell.cmd <cmd...>    # run one command with ROS sourced

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
# docker-compose.yml uses ${PWD}, which bash/zsh set but PowerShell doesn't.
$env:PWD = $Root

# First run: write .env with the right settings for this machine.
$EnvFile = Join-Path $Root ".env"
if (-not (Test-Path $EnvFile)) {
    $Tracks = (Join-Path $Root "tracks") -replace '\\', '/'
    "IFSSIM_HOST=host.docker.internal`nIFSSIM_PORT=41451`nHOST_TRACKS_DIR=$Tracks`n" |
        Set-Content -NoNewline -Encoding ascii $EnvFile
    Write-Host "-> wrote .env (HOST_TRACKS_DIR=$Tracks)"
    docker compose up -d --force-recreate sandbox   # (re)create with the new settings
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# Windows PowerShell turns redirected stderr lines (even a harmless Docker
# warning) into errors, and "Stop" would abort the script: relax it here.
$running = & { $ErrorActionPreference = "Continue"; docker compose ps --status running --services 2>$null }
if (-not ($running -contains "sandbox")) {
    Write-Host "-> starting the sandbox container"
    docker compose up -d sandbox
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if ($args.Count -eq 0) {
    docker compose exec sandbox bash
} else {
    docker compose exec sandbox /entrypoint.sh @args
}
exit $LASTEXITCODE
