#!/usr/bin/env python3
"""Plot the weather-truth flight: ground track through the forecast thermals and
altitude vs time (THERMAL circling shaded).

Usage: plot_weather_truth.py [route_*.json] [weather_truth_trace.csv] [out.png]
"""
import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(__file__)
route_path = sys.argv[1] if len(sys.argv) > 1 else None
if route_path is None:
    import glob
    route_path = sorted(glob.glob(os.path.join(HERE, "..", "planner", "routes",
                                               "route_openmeteo-region_*.json")))[-1]
csv_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "weather_truth_trace.csv")
out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "weather_truth_demo.png")

route = json.load(open(route_path))
wps = route["waypoints"]

t, lat, lon, alt, mode = [], [], [], [], []
with open(csv_path) as f:
    for r in csv.DictReader(f):
        t.append(float(r["t_s"])); lat.append(float(r["lat"])); lon.append(float(r["lon"]))
        alt.append(float(r["alt_m"])); mode.append(r["mode"])

fig, (axm, axa) = plt.subplots(1, 2, figsize=(13, 5.2))

# ground track
axm.plot(lon, lat, color="#1f4e79", lw=1.0, alpha=0.8, label="flight path")
th = [(w["lon"], w["lat"], w["w_star"]) for w in wps]
axm.scatter([p[0] for p in th], [p[1] for p in th], s=[140 for _ in th],
            c="#e08a1f", edgecolor="#7a4a10", zorder=5, label="forecast thermal")
for i, p in enumerate(th):
    axm.annotate(f"{i+1}\nW*={p[2]}", (p[0], p[1]), ha="center", va="center", fontsize=7)
axm.set_xlabel("lon"); axm.set_ylabel("lat")
axm.set_title("Ground track through forecast thermals")
axm.legend(loc="best", fontsize=9); axm.grid(True, alpha=0.3)
axm.set_aspect("auto")

# altitude vs time, THERMAL shaded
axa.plot(t, alt, color="#1f4e79", lw=1.0)
in_th = False; start = 0.0
for i, mo in enumerate(mode):
    if mo == "THERMAL" and not in_th:
        in_th, start = True, t[i]
    elif mo != "THERMAL" and in_th:
        axa.axvspan(start, t[i], color="#e08a1f", alpha=0.25); in_th = False
if in_th:
    axa.axvspan(start, t[-1], color="#e08a1f", alpha=0.25)
axa.set_xlabel("time since arm (s, wall @ speedup 30)")
axa.set_ylabel("relative altitude (m)")
axa.set_title("Climb at each forecast hotspot, glide between")
axa.legend(handles=[plt.Line2D([], [], color="#1f4e79", lw=1.2, label="altitude"),
                    Patch(facecolor="#e08a1f", alpha=0.25, label="THERMAL (ArduSoar circling)")],
           loc="upper left", fontsize=9)
axa.grid(True, alpha=0.3)

fig.suptitle("Weather-truth: real Open-Meteo route flown, ArduSoar climbs at every forecast thermal",
             fontsize=12)
fig.tight_layout()
fig.savefig(out, dpi=130)
print(f"wrote {out}")
