# Setup — running the SITL / companion demos

The Python code (weather, planner, companion) is in this repo. **ArduPilot is not**
— it's large and lives outside the repo. The setup scripts install it (plus our
SITL thermal patch and a Python venv) as **siblings of this repo**, which is what
the `run_*.sh` demos expect by default.

```
<parent>/
  ArduSoar/      <- this repo
  ardupilot/     <- created by setup (ArduPilot SITL)
  soar-venv/     <- created by setup (Python venv)
```

## macOS / Linux

```bash
scripts/setup.sh
```

## Windows

ArduPilot SITL is a Linux toolchain, so on Windows it runs under **WSL2**:

```powershell
# one-time, in an ADMIN PowerShell (then reboot, set an Ubuntu username):
wsl --install -d Ubuntu

# then, in a normal PowerShell from the repo:
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
```

`setup.ps1` just runs `scripts/setup.sh` inside WSL. For best build speed, clone
this repo **inside** the WSL filesystem (`~/ArduSoar`) rather than on `C:\` (`/mnt/c`).

## What setup does

1. clones ArduPilot (`--recurse-submodules`) if missing
2. installs ArduPilot prerequisites (Homebrew on macOS / apt on Linux)
3. applies `sitl/sitl_thermals_scenario5.patch` (thermals at arbitrary positions)
4. builds ArduPlane SITL (`./waf configure --board sitl && ./waf plane`)
5. creates the Python venv and installs `pymavlink empy pexpect future numpy MAVProxy matplotlib`

It needs **Python ≥ 3.10** (ArduPilot's autotest). On macOS that's
`brew install python@3.12`; on Linux `apt install python3.12 python3.12-venv`.

## Custom locations

The scripts default to siblings of the repo. Override with env vars (used by both
`setup.sh` and the `run_*.sh` demos):

```bash
export ARDUPILOT_DIR=/my/ardupilot
export SOAR_VENV=/my/soar-venv
scripts/setup.sh
sitl/run_demo.sh
```

## Run the demos

```bash
sitl/run_demo.sh                  # reproduce ArduSoar thermalling
companion/run_companion_demo.sh   # weather hotspot -> fly there -> hand off to ArduSoar
companion/run_xc_demo.sh          # cross-country relay between forecast hotspots
sitl/run_route_demo.sh            # fly a planner-generated route
sitl/run_weather_truth_demo.sh    # real weather -> route -> thermals there -> fly (needs the patch)
```

Ground-only tools (no ArduPilot needed, just the venv or system Python):

```bash
python -m planner.route_planner --source soaringmeteo --lat 43.47 --lon -80.54 --region-km 150
python -m planner.replan --prior <prior.json> --demo
python -m dashboard.app           # http://127.0.0.1:8050  (needs: pip install dash plotly)
```

## On the Raspberry Pi 5 (real aircraft)

The Pi 5 runs the **companion MAVLink layer** (`companion/`), not SITL. Install just
the Python deps (`pip install pymavlink`) and point the connection at the flight
controller's serial port instead of SITL's TCP, e.g.
`--conn /dev/serial0` (add the baud, 921600). The planner/weather/replan stay on the
ground. See `companion/README.md`.
