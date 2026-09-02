#!/usr/bin/env python3
"""Visualise the terrain-trigger prior: terrain + the trigger heatmap + the
hotspots, with the sun and wind directions. Usage: plot_terrain.py [lat] [lon]
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from weather.terrain_prior import build_prior

lat = float(sys.argv[1]) if len(sys.argv) > 1 else 39.70
lon = float(sys.argv[2]) if len(sys.argv) > 2 else -105.30
out = os.path.join(os.path.dirname(__file__), "..", "output", "terrain_trigger.png")

p = build_prior(lat, lon)
g = p["_grid"]
elev = np.array(g["elev"]); score = np.array(g["score"])
ext = p["bounds"][1]
extent = (-ext, ext, -ext, ext)
hx = [c[0] for c in p["candidates"]]; hy = [c[1] for c in p["candidates"]]
az = math.radians(p["sun"]["azimuth_deg"]); el = p["sun"]["elevation_deg"]
wind = p["wind"]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.4))
for ax, data, cmap, title in [
        (ax1, elev, "terrain", "Terrain (m) + triggered hotspots"),
        (ax2, score / (score.max() + 1e-9), "hot", "Trigger score = sun-slope x ridge x windward")]:
    im = ax.imshow(data, extent=extent, origin="upper", cmap=cmap, aspect="equal")
    ax.scatter(hx, hy, s=90, facecolors="none", edgecolors="#0a3d91", linewidths=2, zorder=5)
    for i, (x, y) in enumerate(zip(hx, hy)):
        ax.annotate(str(i + 1), (x, y), color="#0a3d91", fontsize=8, ha="center", va="center")
    fig.colorbar(im, ax=ax, shrink=0.8)
    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)"); ax.set_title(title)
    # sun arrow (toward the sun) + wind arrow (toward)
    L = ext * 0.6
    ax.annotate("", xy=(L * math.sin(az), L * math.cos(az)), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="#f4a300", lw=2))
    ax.text(L * math.sin(az) * 1.05, L * math.cos(az) * 1.05, f"sun {el:.0f}°",
            color="#b37400", fontsize=8)
    wn = math.hypot(*wind) + 1e-9
    ax.annotate("", xy=(wind[0] / wn * L * 0.7, wind[1] / wn * L * 0.7), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="#1f7a1f", lw=2))
    ax.text(wind[0] / wn * L * 0.75, wind[1] / wn * L * 0.75, "wind", color="#1f7a1f", fontsize=8)

fig.suptitle(f"Terrain-trigger prior @ {lat},{lon}  W*={p['thermal_strength_ms']} m/s  "
             f"(GFS strength x terrain placement)")
fig.tight_layout()
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=130)
print(f"wrote {out}")
