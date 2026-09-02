#!/usr/bin/env python3
"""Animate a SITL flight trace as a video — watch the aircraft fly the route,
catching lift at each forecast thermal.

Reads a trace CSV (t,lat,lon,alt,mode) and a route JSON (thermal positions), and
renders a 2-panel animation: ground track with a moving aircraft (orange while
ArduSoar is circling a THERMAL), and the altitude profile with a moving cursor.
Saves a GIF (no ffmpeg needed).

Usage:
    python sitl/playback.py
    python sitl/playback.py --trace sitl/weather_truth_trace.csv \
        --route planner/routes/route_openmeteo-region_43.47_-80.54.json \
        --out sitl/weather_truth_playback.gif --frames 250
"""
import argparse
import csv
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(__file__)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", default=os.path.join(HERE, "weather_truth_trace.csv"))
    ap.add_argument("--route", default=None)
    ap.add_argument("--out", default=os.path.join(HERE, "weather_truth_playback.gif"))
    ap.add_argument("--frames", type=int, default=250)
    ap.add_argument("--fps", type=int, default=20)
    args = ap.parse_args()

    route_path = args.route or sorted(glob.glob(os.path.join(
        HERE, "..", "planner", "routes", "route_openmeteo-region_*.json")))[-1]
    wps = json.load(open(route_path))["waypoints"]

    t, lat, lon, alt, mode = [], [], [], [], []
    with open(args.trace) as f:
        for r in csv.DictReader(f):
            t.append(float(r["t_s"])); lat.append(float(r["lat"])); lon.append(float(r["lon"]))
            alt.append(float(r["alt_m"])); mode.append(r["mode"])
    n = len(t)
    idx = [int(round(i * (n - 1) / (args.frames - 1))) for i in range(args.frames)]

    fig, (axm, axa) = plt.subplots(1, 2, figsize=(12, 5))
    th_lon = [w["lon"] for w in wps]; th_lat = [w["lat"] for w in wps]
    axm.scatter(th_lon, th_lat, s=160, c="#e08a1f", edgecolor="#7a4a10", zorder=3)
    for i, w in enumerate(wps):
        axm.annotate(f"{i+1}", (w["lon"], w["lat"]), ha="center", va="center", fontsize=8, zorder=4)
    axm.set_xlim(min(lon + th_lon) - 0.01, max(lon + th_lon) + 0.01)
    axm.set_ylim(min(lat + th_lat) - 0.01, max(lat + th_lat) + 0.01)
    axm.set_xlabel("lon"); axm.set_ylabel("lat"); axm.grid(True, alpha=0.3)
    axm.set_title("Flight track (orange dots = forecast thermals)")
    (trail,) = axm.plot([], [], color="#1f4e79", lw=1.0, alpha=0.7)
    (craft,) = axm.plot([], [], "o", ms=9, color="#1f4e79")

    axa.set_xlim(0, t[-1]); axa.set_ylim(min(alt) - 20, max(alt) + 30)
    axa.set_xlabel("time since arm (s)"); axa.set_ylabel("relative altitude (m)")
    axa.grid(True, alpha=0.3); axa.set_title("Altitude")
    (acurve,) = axa.plot([], [], color="#1f4e79", lw=1.0)
    (acursor,) = axa.plot([], [], "o", ms=8, color="#1f4e79")
    txt = axa.text(0.02, 0.95, "", transform=axa.transAxes, va="top", fontsize=10,
                   family="monospace")

    def frame(k):
        j = idx[k]
        col = "#e08a1f" if mode[j] == "THERMAL" else "#1f4e79"
        trail.set_data(lon[:j + 1], lat[:j + 1])
        craft.set_data([lon[j]], [lat[j]]); craft.set_color(col)
        acurve.set_data(t[:j + 1], alt[:j + 1])
        acursor.set_data([t[j]], [alt[j]]); acursor.set_color(col)
        txt.set_text(f"t={t[j]:5.0f}s\nalt={alt[j]:5.0f} m\nmode={mode[j]}")
        return trail, craft, acurve, acursor, txt

    fig.suptitle("Weather-truth flight: real Open-Meteo route, ArduSoar climbs at each forecast thermal")
    fig.tight_layout()
    anim = FuncAnimation(fig, frame, frames=len(idx), interval=1000 / args.fps, blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps))
    print(f"wrote {args.out} ({len(idx)} frames @ {args.fps} fps)")


if __name__ == "__main__":
    main()
