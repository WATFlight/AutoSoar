#!/usr/bin/env bash
# Connect line A (real weather -> route) with line B (SITL flight): plan a route
# from REAL weather, place simulated thermals AT those forecast positions
# (SITL scenario 5), start SITL at the route origin, and fly it — so the aircraft
# actually catches lift at the forecast hotspots.
#
# Usage: sitl/run_weather_truth_demo.sh [lat] [lon]
set -uo pipefail

SOAR_VENV="${SOAR_VENV:-$(cd "$(dirname "$0")/.." && pwd)/../soar-venv}"
VENV="$SOAR_VENV/bin/python"
SYS=python3
ARDUPILOT="${ARDUPILOT_DIR:-$(cd "$(dirname "$0")/.." && pwd)/../ardupilot}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LAT="${1:-43.47}"
LON="${2:--80.54}"
THERMALS=/tmp/sitl_thermals.txt

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
sleep 2

cd "$REPO"
# Real-weather cross-country route (Open-Meteo W*, 50 km box sampled at 10 km so
# thermals are a flyable distance apart), and write the thermals at those positions.
$SYS -m planner.route_planner --source openmeteo --lat "$LAT" --lon "$LON" \
     --region-km 50 --region-step-km 10 --w-min 1.0 --max-waypoints 5 \
     --ceiling-alt 850 --thermal-radius 500 \
     --sitl-thermals "$THERMALS" 2>&1 | grep -vE "NotOpenSSL|warnings.warn"

ROUTE_JSON="$(ls -t "$REPO"/planner/routes/route_openmeteo-region_*.json | head -1)"
echo "[weather-truth] route: $ROUTE_JSON"
echo "[weather-truth] thermals placed at forecast positions (rel. to first hotspot):"; cat "$THERMALS"
# SITL home = the first hotspot, so the aircraft climbs out on a real thermal.
ORIGIN="$($SYS -c "import json;d=json.load(open('$ROUTE_JSON'));w=d['waypoints'][0];print(f\"{w['lat']},{w['lon']},0,0\")")"
echo "[weather-truth] SITL home (first hotspot) = $ORIGIN"

cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 30 --custom-location="$ORIGIN" \
    > /tmp/sitl_wt.log 2>&1 &
for i in $(seq 1 60); do
    lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1 && break
    sleep 1
done
sleep 5

cd "$REPO"
$VENV sitl/fly_weather_truth.py --conn tcp:127.0.0.1:5760 --route "$ROUTE_JSON" --timeout 600
RC=$?

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
echo "[weather-truth] exit $RC"
exit $RC
