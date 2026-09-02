# One-time setup for the ArduSoar dev environment on Windows.
#
# ArduPilot SITL is a Linux toolchain — on Windows it runs under WSL2 (Ubuntu).
# This script checks WSL, then runs scripts/setup.sh INSIDE WSL against this repo.
#
# Run in PowerShell:   powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
$ErrorActionPreference = "Stop"

# 1. WSL present?
if (-not (Get-Command wsl -ErrorAction SilentlyContinue)) {
    Write-Host "WSL is not installed." -ForegroundColor Yellow
    Write-Host "In an ADMIN PowerShell run:  wsl --install -d Ubuntu"
    Write-Host "then reboot and re-run this script."
    exit 1
}

# 2. A Linux distro installed? (`wsl -l -q` lists them)
$distros = (wsl -l -q) -join "`n"
if ([string]::IsNullOrWhiteSpace($distros)) {
    Write-Host "WSL has no Linux distro installed." -ForegroundColor Yellow
    Write-Host "Run:  wsl --install -d Ubuntu   (then reboot, set a username/password)."
    exit 1
}
Write-Host "WSL distro(s) found:`n$distros"

# 3. Run the Linux setup inside WSL, pointed at this repo.
$repoWin = (Resolve-Path "$PSScriptRoot\..").Path
$repoWsl = (wsl wslpath -a "$repoWin").Trim()
Write-Host "Repo (WSL path): $repoWsl"
Write-Host "Running scripts/setup.sh inside WSL..." -ForegroundColor Cyan

# NOTE: building on /mnt/c (Windows drive) works but is slow. For best speed,
# clone this repo INSIDE the WSL filesystem (e.g. ~/ArduSoar) and run setup there.
wsl bash -lc "cd '$repoWsl' && bash scripts/setup.sh"

Write-Host "`nDone. Use the demos from inside WSL:" -ForegroundColor Green
Write-Host "    wsl"
Write-Host "    cd $repoWsl && sitl/run_demo.sh"
