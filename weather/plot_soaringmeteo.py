"""Plot the SoaringMeteo table (separate from the grabber).

Reads the CSV written by weather/soaringmeteo.py and draws the three variables
over time: thermal velocity (W*), soaring-layer top, and boundary-layer wind.

    python -m weather.plot_soaringmeteo                 # default Oklahoma CSV
    python -m weather.plot_soaringmeteo <path-to.csv>
"""

from __future__ import annotations

import csv
import os
import sys
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

from weather.soaringmeteo import DATA_DIR

_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "output")


def load_csv(path: str):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def plot(path: str = None, out: str = None) -> str:
    path = path or os.path.join(DATA_DIR, "soaringmeteo_36.687_-97.137.csv")
    rows = load_csv(path)
    t = [datetime.strptime(r["time"], "%Y-%m-%dT%H:%M:%SZ") for r in rows]
    tv = [float(r["thermal_velocity_ms"]) for r in rows]
    blt = [float(r["soaring_layer_top_m"]) if r["soaring_layer_top_m"] else 0.0 for r in rows]
    wind = [float(r["wind_bl_speed_kmh"]) for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, tv, color="#0F6E56", marker="o", ms=3)
    axes[0].set_ylabel("W* (m/s)")
    axes[0].set_title("thermal velocity (thermal strength)")
    axes[1].plot(t, blt, color="#185FA5", marker="o", ms=3)
    axes[1].set_ylabel("top (m)")
    axes[1].set_title("soaring-layer top (thermal ceiling)")
    axes[2].plot(t, wind, color="#C75D2C", marker="o", ms=3)
    axes[2].set_ylabel("wind (km/h)")
    axes[2].set_title("boundary-layer wind (thermal drift)")
    for ax in axes:
        ax.grid(alpha=0.25)
    axes[2].xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %Hz"))
    fig.autofmt_xdate()
    fig.suptitle(f"SoaringMeteo GFS — {os.path.basename(path)}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = out or os.path.join(_OUTPUT_DIR, "soaringmeteo_forecast.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else None
    print("saved chart ->", plot(p))
