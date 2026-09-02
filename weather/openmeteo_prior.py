"""Build a companion-ready thermal prior from Open-Meteo's Deardorff W\* pipeline.

Parallel to `weather/soaringmeteo_prior.py`, but the bulk W\* comes from
`weather/openmeteo_thermal.py` (the Deardorff convective-velocity formula on
Open-Meteo GFS inputs) instead of SoaringMeteo. Both sources now feed the SAME
`(x, y, strength, prob)` prior entry point the companion and dashboard consume.
"""
from __future__ import annotations

import datetime as _dt
import json
import os

import numpy as np

from weather import openmeteo_thermal


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _default_time():
    """Today at 18:00 UTC — a representative peak-convection hour."""
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT18:00:00Z")


def build_prior(lat, lon, at_time=None, bounds=(-2000.0, 2000.0, -2000.0, 2000.0),
                seed=0):
    """Fetch Open-Meteo W\* near (lat, lon) and return a companion-schema prior."""
    at_time = at_time or _default_time()
    # one GFS cell or two around the point; take the strongest as the area's W*
    dlat, dlon = 0.12, 0.12
    meta, recs = openmeteo_thermal.fetch_region(
        lat - dlat, lat + dlat, lon - dlon, lon + dlon, at_time)
    if not recs:
        raise RuntimeError(f"Open-Meteo returned no W* for ({lat},{lon}) at {at_time}")
    best = max(recs, key=lambda r: r["thermal_velocity_ms"])
    W0 = best["thermal_velocity_ms"]
    cloud_base = best["soaring_layer_top_m"]
    wx = round(best["wind_u_kmh"] / 3.6, 2)            # km/h (toward) -> m/s
    wy = round(best["wind_v_kmh"] / 3.6, 2)

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
        "generated_at": at_time,
        "location": {"lat": lat, "lon": lon, "time": at_time},
        "bounds": list(bounds),
        "wind": [wx, wy],
        "cloud_base_m": round(cloud_base) if cloud_base else None,
        "thermal_strength_ms": W0,
        "thermal_count": n,
        "candidates": cands,
        "inputs": {"thermal_velocity_ms": W0, "soaring_layer_top_m": cloud_base,
                   "wind_u_kmh": best["wind_u_kmh"], "wind_v_kmh": best["wind_v_kmh"]},
        "source": "open-meteo-wstar",
    }


def main(lat=43.47, lon=-80.54):
    prior = build_prior(lat, lon)
    out = os.path.join(os.path.dirname(__file__), "data",
                       f"openmeteo_prior_{lat}_{lon}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(prior, f, indent=2)
    print(f"Open-Meteo W* prior @ ({lat}, {lon})  time={prior['location']['time']}")
    print(f"  W* = {prior['thermal_strength_ms']} m/s   cloud_base = {prior['cloud_base_m']} m"
          f"   wind(toward,m/s) = {prior['wind']}")
    print(f"  {prior['thermal_count']} candidates  ->  {out}")
    return prior


if __name__ == "__main__":
    main()
