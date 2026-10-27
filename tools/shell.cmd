@echo off
rem Windows entry point: opens a shell inside the sandbox container.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0shell.ps1" %*
