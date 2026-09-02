#!/usr/bin/env bash
# One-time setup for the ArduSoar dev environment (macOS, Linux, or WSL2 on Windows).
#
# Installs ArduPilot SITL, applies our thermal-scenario patch, and builds the Python
# venv — by default as SIBLINGS of this repo, which is what the run_*.sh scripts
# expect. Override with env vars:
#     ARDUPILOT_DIR=/path/to/ardupilot  SOAR_VENV=/path/to/venv  scripts/setup.sh
#
# Re-runnable: skips work that's already done.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$REPO/../ardupilot}"
SOAR_VENV="${SOAR_VENV:-$REPO/../soar-venv}"
OS="$(uname -s)"

# Pick a Python >= 3.10 (ArduPilot's autotest needs it).
PY=""
for c in python3.12 python3.11 python3.10 python3; do
    if command -v "$c" >/dev/null 2>&1; then
        v="$("$c" -c 'import sys;print(sys.version_info[0]*100+sys.version_info[1])')"
        if [ "$v" -ge 310 ]; then PY="$c"; break; fi
    fi
done
if [ -z "$PY" ]; then
    echo "ERROR: need Python >= 3.10."
    [ "$OS" = "Darwin" ] && echo "  macOS:  brew install python@3.12" \
                         || echo "  Linux:  sudo apt install python3.12 python3.12-venv"
    exit 1
fi

echo "Repo:      $REPO"
echo "ArduPilot: $ARDUPILOT_DIR"
echo "venv:      $SOAR_VENV   (python: $PY)"
echo

# 1. ArduPilot source + prereqs ------------------------------------------------
if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
    echo ">> cloning ArduPilot (this is large)…"
    git clone --recurse-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
fi
cd "$ARDUPILOT_DIR"
echo ">> installing ArduPilot prerequisites (may prompt / take a while)…"
if [ "$OS" = "Darwin" ]; then
    [ -x Tools/environment_install/install-prereqs-mac.sh ] && Tools/environment_install/install-prereqs-mac.sh -y || true
else
    [ -x Tools/environment_install/install-prereqs-ubuntu.sh ] && Tools/environment_install/install-prereqs-ubuntu.sh -y || true
fi

# 2. Our SITL thermal-scenario patch (idempotent) ------------------------------
PATCH="$REPO/sitl/sitl_thermals_scenario5.patch"
if git apply --reverse --check "$PATCH" >/dev/null 2>&1; then
    echo ">> scenario-5 patch already applied"
elif git apply --check "$PATCH" >/dev/null 2>&1; then
    git apply "$PATCH" && echo ">> applied scenario-5 patch"
else
    echo ">> WARNING: scenario-5 patch did not apply cleanly (weather-truth demo may not work)"
fi

# 3. Build SITL ----------------------------------------------------------------
echo ">> building ArduPlane SITL…"
./waf configure --board sitl
./waf plane

# 4. Python venv ---------------------------------------------------------------
cd "$REPO"
if [ ! -x "$SOAR_VENV/bin/python" ]; then
    echo ">> creating venv with $PY…"
    "$PY" -m venv "$SOAR_VENV"
fi
echo ">> installing Python deps…"
"$SOAR_VENV/bin/pip" install --quiet --upgrade pip
"$SOAR_VENV/bin/pip" install --quiet pymavlink "empy==3.3.4" pexpect future numpy MAVProxy matplotlib

echo
echo "Done. Try a demo:"
echo "    sitl/run_demo.sh                 # ArduSoar thermalling"
echo "    sitl/run_weather_truth_demo.sh   # real weather -> route -> fly (needs the patch)"
