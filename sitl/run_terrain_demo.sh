#!/usr/bin/env bash
# End-to-end terrain validation: build a TERRAIN-trigger prior (sun-slope + ridge +
# wind over real DEM, scaled by live GFS W*), plan a route through those terrain
# hotspots, place simulated thermals AT them (SITL scenario 5), and fly it — so the
# aircraft catches lift at the terrain-predicted positions.
#
# Usage: sitl/run_terrain_demo.sh [lat] [lon]   (default: Golden, CO foothills)
set -uo pipefail

SOAR_VENV="${SOAR_VENV:-$(cd "$(dirname "$0")/.." && pwd)/../soar-venv}"
VENV="$SOAR_VENV/bin/python"
SYS=python3
ARDUPILOT="${ARDUPILOT_DIR:-$(cd "$(dirname "$0")/.." && pwd)/../ardupilot}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LAT="${1:-39.70}"; LON="${2:--105.30}"
THERMALS=/tmp/sitl_thermals.txt
PRIOR=/tmp/terrain_prior.json

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
sleep 2

cd "$REPO"
echo "[terrain] building terrain-trigger prior (DEM + sun + live GFS)…"
$SYS -c "import json; from weather.terrain_prior import build_prior; \
p=build_prior($LAT,$LON,size_km=25.0,n=24); p.pop('_grid',None); \
json.dump(p, open('$PRIOR','w')); print('  W*=%s  %d hotspots'%(p['thermal_strength_ms'],p['thermal_count']))" \
  2>&1 | grep -vE "NotOpenSSL|warnings.warn"
$SYS -m planner.route_planner --prior "$PRIOR" --takeoff-alt 120 --ceiling-alt 600 \
     --thermal-radius 500 --max-waypoints 4 --sitl-thermals "$THERMALS" 2>&1 | grep -vE "NotOpenSSL|warnings.warn"

ROUTE_JSON="$(ls -t "$REPO"/planner/routes/route_terrain-trigger_*.json | head -1)"
echo "[terrain] route: $ROUTE_JSON"
echo "[terrain] thermals at terrain hotspots (rel. to first):"; cat "$THERMALS"
ORIGIN="$($SYS -c "import json; d=json.load(open('$ROUTE_JSON')); w=d['waypoints'][0]; print('%s,%s,0,0'%(w['lat'],w['lon']))")"
echo "[terrain] SITL home (first hotspot) = $ORIGIN"

cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 30 --custom-location="$ORIGIN" \
    > /tmp/sitl_terrain.log 2>&1 &
for i in $(seq 1 60); do lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1 && break; sleep 1; done
sleep 5

cd "$REPO"
$VENV sitl/fly_weather_truth.py --conn tcp:127.0.0.1:5760 --route "$ROUTE_JSON" --timeout 600
RC=$?

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
echo "[terrain] exit $RC"
exit $RC
