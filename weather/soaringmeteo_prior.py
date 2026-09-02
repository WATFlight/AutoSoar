"""Build a companion-ready thermal prior from a live SoaringMeteo forecast.

SoaringMeteo gives the *bulk* soaring conditions for the area directly — thermal
velocity W\*, soaring-layer top (our cloud base), and boundary-layer wind — at
~0.25 deg (~28 km) resolution, i.e. one value for a local flight area. We turn
that into the same `(x, y, strength, prob)` prior schema the companion consumes
(see `weather/processor.py`): the W\* sets the candidate strength, the BL top sets
the ceiling, the BL wind sets the drift, and candidate *positions* are sampled
consistently with the day's strength (a forecast, not ground truth).

So the strategic layer now runs on real weather: "today is a 2 m/s day, base
~1800 m, wind from the SW" rather than a synthetic constant.
"""
from __future__ import annotations

import json
import os

import numpy as np

from weather import soaringmeteo


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_prior(lat, lon, at_time=None, bounds=(-2000.0, 2000.0, -2000.0, 2000.0),
                seed=0):
    """Fetch SoaringMeteo at (lat, lon) and return a companion-schema prior dict."""
    run, zone, point = soaringmeteo.fetch_point(lat, lon)
    target = at_time or soaringmeteo.peak_time(point)
    hour = soaringmeteo._hour_at(point, target)
    if hour is None:
        raise RuntimeError(f"SoaringMeteo has no hour {target} for ({lat},{lon})")

    bl = hour.get("bl", {})
    W0 = round(hour.get("v", 0) / 10.0, 2)             # thermal velocity W* (m/s)
    cloud_base = bl.get("h")                            # soaring-layer top (m)
    # BL wind components (km/h, u=east v=north) -> drift vector air blows TOWARD (m/s)
    wx = round(bl.get("u", 0.0) / 3.6, 2)
    wy = round(bl.get("v", 0.0) / 3.6, 2)

    # how many candidates / how trustworthy, scaled by the day's strength
    n = int(_clamp(6 + W0 * 3.0, 6, 24))
    base_prob = _clamp(0.45 + 0.08 * W0, 0.2, 0.85)

    rng = np.random.default_rng(seed)
    x0, x1, y0, y1 = bounds
    cands = []
    for _ in range(n):
        x = float(rng.uniform(x0, x1))
        y = float(rng.uniform(y0, y1))
        s = float(_clamp(W0 + rng.normal(0, 0.3), 0.8, 6.0))
        p = float(_clamp(base_prob + rng.normal(0, 0.08), 0.15, 0.95))
        cands.append([round(x, 1), round(y, 1), round(s, 2), round(p, 2)])

    return {
        "generated_at": run["path"],
        "location": {"lat": lat, "lon": lon, "time": target},
        "bounds": list(bounds),
        "wind": [wx, wy],
        "cloud_base_m": round(cloud_base) if cloud_base else None,
        "thermal_strength_ms": W0,
        "thermal_count": n,
        "candidates": cands,
        "inputs": {"thermal_velocity_ms": W0, "soaring_layer_top_m": cloud_base,
                   "wind_bl_u_kmh": bl.get("u"), "wind_bl_v_kmh": bl.get("v")},
        "source": "soaringmeteo",
        "zone": zone["id"], "run": run["path"],
    }


def main(lat=43.47, lon=-80.54):
    prior = build_prior(lat, lon)
    out = os.path.join(os.path.dirname(__file__), "data",
                       f"soaringmeteo_prior_{lat}_{lon}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(prior, f, indent=2)
    print(f"SoaringMeteo prior @ ({lat}, {lon})  zone={prior['zone']}  run={prior['run']}")
    print(f"  W* = {prior['thermal_strength_ms']} m/s   cloud_base = {prior['cloud_base_m']} m"
          f"   wind(toward,m/s) = {prior['wind']}")
    print(f"  {prior['thermal_count']} candidates sampled for time {prior['location']['time']}")
    print(f"  -> {out}")
    return prior


if __name__ == "__main__":
    main()
