"""2-D map of SoaringMeteo over a region at ONE time.

Samples a lat/lon box (native ~0.25 deg grid, or coarser) and draws, for a chosen
forecast hour:
  * thermal velocity (W*)        -> colour field
  * boundary-layer wind          -> arrows
  * soaring-layer top            -> contour lines

    python -m weather.plot_soaringmeteo_map                 # default box around Oklahoma
    python -m weather.plot_soaringmeteo_map --time 2026-06-21T18:00:00Z

Resolution note: GFS is ~0.25 deg (~28 km); you can subsample coarser with
--step but not finer (that data does not exist). Use the WRF model for km-scale.
"""

from __future__ import annotations

import argparse
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from weather.soaringmeteo import fetch_region, DATA_DIR

_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "output")


def _grids(recs):
    lats = sorted({r["lat"] for r in recs})
    lons = sorted({r["lon"] for r in recs})
    li = {v: i for i, v in enumerate(lats)}
    lj = {v: j for j, v in enumerate(lons)}
    shape = (len(lats), len(lons))
    tv = np.full(shape, np.nan)
    blt = np.full(shape, np.nan)
    u = np.full(shape, np.nan)
    v = np.full(shape, np.nan)
    for r in recs:
        i, j = li[r["lat"]], lj[r["lon"]]
        tv[i, j] = r["thermal_velocity_ms"]
        blt[i, j] = r["soaring_layer_top_m"] if r["soaring_layer_top_m"] is not None else np.nan
        u[i, j] = r["wind_u_kmh"]
        v[i, j] = r["wind_v_kmh"]
    return np.array(lons), np.array(lats), tv, blt, u, v


def write_csv(meta, recs, path=None):
    os.makedirs(DATA_DIR, exist_ok=True)
    path = path or os.path.join(DATA_DIR, f"soaringmeteo_region_{meta['time'].replace(':', '')}.csv")
    cols = ["lat", "lon", "thermal_velocity_ms", "soaring_layer_top_m", "wind_u_kmh", "wind_v_kmh"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in recs:
            w.writerow({c: r[c] for c in cols})
    return path


def plot_map(meta, recs, out=None):
    lons, lats, tv, blt, u, v = _grids(recs)
    LON, LAT = np.meshgrid(lons, lats)

    fig, ax = plt.subplots(figsize=(11, 8))
    mesh = ax.pcolormesh(lons, lats, tv, cmap="YlOrRd", shading="nearest", vmin=0)
    cb = fig.colorbar(mesh, ax=ax, shrink=0.85)
    cb.set_label("thermal velocity W* (m/s)")

    if np.isfinite(blt).any():
        cs = ax.contour(LON, LAT, blt, colors="#185FA5", linewidths=0.8, alpha=0.7)
        ax.clabel(cs, inline=True, fontsize=7, fmt="%.0f m")

    ax.quiver(LON, LAT, u, v, color="#333", scale=400, width=0.003, alpha=0.8)

    ax.set_xlabel("longitude")
    ax.set_ylabel("latitude")
    ax.set_title(f"{meta.get('source', 'SoaringMeteo GFS')} — {meta['time']}  "
                 f"(run {meta['run']}, {meta['zone']})\n"
                 f"colour = thermal velocity, arrows = BL wind, contours = soaring-layer top "
                 f"[~{meta['step_deg']*111:.0f} km grid]", fontsize=10)
    ax.set_aspect(1.0 / np.cos(np.radians(np.mean(lats))))
    ax.grid(alpha=0.2)

    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = out or os.path.join(_OUTPUT_DIR, "soaringmeteo_map.png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat0", type=float, default=34.0)
    ap.add_argument("--lat1", type=float, default=39.0)
    ap.add_argument("--lon0", type=float, default=-100.0)
    ap.add_argument("--lon1", type=float, default=-94.0)
    ap.add_argument("--time", default=None, help="ISO UTC, e.g. 2026-06-21T18:00:00Z")
    ap.add_argument("--step", type=float, default=None, help="sample step in degrees (>= 0.25)")
    a = ap.parse_args()

    meta, recs = fetch_region(a.lat0, a.lat1, a.lon0, a.lon1, at_time=a.time, step_deg=a.step)
    print(f"region {meta['bbox']}  time={meta['time']}  run={meta['run']}")
    print(f"  {meta['n']} grid points, {meta['n_requests']} block requests, "
          f"step ~{meta['step_deg']*111:.0f} km")
    print(f"  available times: {meta['available_times'][:6]} ...")
    print("  saved table ->", write_csv(meta, recs))
    print("  saved map   ->", plot_map(meta, recs))


if __name__ == "__main__":
    main()
