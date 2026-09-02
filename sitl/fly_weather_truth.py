#!/usr/bin/env python3
"""Fly a real-weather planner route in SITL with thermals placed at the forecast
positions (SITL scenario 5) — connecting "real weather -> route" (line A) with
"aircraft actually catches lift" (line B).

Requires: SITL built with the scenario-5 patch, started at the route origin, and
/tmp/sitl_thermals.txt written by the planner (--sitl-thermals). This script sets
SIM_THML_SCENARI=5, uploads the route mission, flies it, and reports — for each
planned forecast hotspot — whether ArduSoar found lift there and how much it climbed.

Usage: fly_weather_truth.py --route planner/routes/route_*.json
"""
import argparse
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from companion import mav, geo  # noqa: E402


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--route", required=True, help="route_*.json (sibling .waypoints is uploaded)")
    ap.add_argument("--timeout", type=float, default=600)
    ap.add_argument("--match-m", type=float, default=1800, help="how near a hotspot a thermal episode counts")
    args = ap.parse_args()

    with open(args.route) as f:
        route = json.load(f)
    wps = route["waypoints"]
    waypoints_file = args.route.replace(".json", ".waypoints")

    m = mav.connect(args.conn)
    log(f"Connected (sys {m.target_system})")
    if not mav.wait_gps_fix(m):
        log("FAILED: no GPS fix")
        return 2

    mav.set_param(m, "SIM_THML_SCENARI", 5)    # thermals from /tmp/sitl_thermals.txt
    mav.set_param(m, "SOAR_VSPEED", 0.55)
    mav.set_param(m, "SOAR_ENABLE", 1)

    ok, n = mav.upload_qgc_file(m, waypoints_file)
    if not ok:
        log("FAILED: mission rejected")
        return 1
    log(f"Mission ACCEPTED ({n} items); {len(wps)} forecast hotspots, ceiling {route['ceiling_alt_m']} m")

    mav.set_mode(m, "AUTO")
    if not mav.arm(m):
        log("FAILED: could not arm")
        return 1
    log("Armed; climbing out")
    airborne = time.time() + 120
    while time.time() < airborne:
        pos = mav.vehicle_position(m)
        if pos and pos[2] >= 90:
            break
    mav.set_soaring_switch(m, 2)
    log("Airborne; ArduSoar enabled, flying the real-weather route")

    # monitor -------------------------------------------------------------
    trace = []
    episodes = []          # (lat, lon, entry_alt, peak_alt)
    in_thermal = False
    ep = None
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        try:
            msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT"],
                               blocking=True, timeout=1)
        except ConnectionError:
            log("Connection to SITL lost"); break
        if msg is None:
            continue
        if msg.get_type() != "GLOBAL_POSITION_INT":
            continue
        lat, lon, alt = msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
        mode = m.flightmode
        trace.append((round(time.time() - t0, 1), lat, lon, round(alt, 1), mode))
        if mode == "THERMAL" and not in_thermal:
            in_thermal, ep = True, [lat, lon, alt, alt]
        elif mode == "THERMAL" and in_thermal:
            ep[3] = max(ep[3], alt)
        elif mode != "THERMAL" and in_thermal:
            in_thermal = False
            episodes.append(tuple(ep))
        if mode == "RTL":
            log("Mission complete (RTL)"); break
    if in_thermal and ep:
        episodes.append(tuple(ep))

    # match thermal episodes to planned hotspots --------------------------
    csv_path = os.path.join(os.path.dirname(__file__), "weather_truth_trace.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "lat", "lon", "alt_m", "mode"]); w.writerows(trace)

    log("=" * 64)
    log(f"Real-weather route flown ({route['source']}). Per forecast hotspot:")
    hit = 0
    for i, wp in enumerate(wps):
        best = None
        for (elat, elon, ea, pa) in episodes:
            d = geo.haversine_m(wp["lat"], wp["lon"], elat, elon)
            if d <= args.match_m and (best is None or (pa - ea) > best):
                best = pa - ea
        if best is not None and best >= 5:
            hit += 1
            log(f"  hotspot {i+1} ({wp['lat']:.4f},{wp['lon']:.4f}, W*={wp['w_star']}): "
                f"LIFT FOUND, climbed +{best:.0f} m")
        else:
            log(f"  hotspot {i+1} ({wp['lat']:.4f},{wp['lon']:.4f}, W*={wp['w_star']}): no lift")
    log(f"  -> {hit}/{len(wps)} forecast hotspots produced real lift in the air")
    log(f"  trace -> {csv_path}")
    log("=" * 64)
    return 0 if hit >= 2 else 1


if __name__ == "__main__":
    sys.exit(main())
