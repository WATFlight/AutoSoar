"""Software B — weather -> thermal prior.

Turns a weather snapshot into the glider's pre-flight map, using standard soaring
meteorology:

  * thermal STRENGTH  ~ surface heating (shortwave radiation) + instability (CAPE)
                        + dryness (temperature - dewpoint spread)
  * thermal CEILING   (= our cloud_base) ~ boundary-layer height, else the lifting
                        condensation level  LCL ~ 125 * (T - Td)
  * DRIFT (wind)      ~ boundary-layer wind, as a vector the air blows toward
  * DENSITY / COUNT   ~ how strongly the day is working

The candidate LOCATIONS are a sampled forecast (a stochastic field consistent with
those bulk parameters) — NOT the simulator's ground truth. That is the whole point:
the map is now an independent guess from real weather, so it is realistically
imperfect rather than copied from the answer.

Output is exactly the (x, y, strength, prob) list that build_prior() consumes, plus
wind + cloud_base, written to prior_latest.json (the "data link" the glider reloads).
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

OUT = os.path.join(os.path.dirname(__file__), "prior_latest.json")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_prior(weather: dict, bounds=(-2000.0, 2000.0, -2000.0, 2000.0),
               seed: int = 0, generated_at=None) -> dict:
    T = weather.get("temperature_c")
    T = 15.0 if T is None else T
    Td = weather.get("dewpoint_c")
    Td = T - 5.0 if Td is None else Td
    rad = weather.get("radiation_wm2") or 0.0
    cape = weather.get("cape_jkg") or 0.0
    blh = weather.get("blh_m")
    cloud = weather.get("cloud_pct") or 0.0
    ws = weather.get("wind_speed_ms") or 0.0
    wd = weather.get("wind_dir_deg") or 0.0

    # wind as a vector the air blows TOWARD, in (east=x, north=y) m/s
    drad = math.radians(wd)
    wx = -ws * math.sin(drad)
    wy = -ws * math.cos(drad)

    # cloud base: boundary-layer height if we have it, else the LCL
    spread = max(T - Td, 0.0)
    cloud_base = blh if blh else 125.0 * spread
    cloud_base = _clamp(cloud_base, 500.0, 2500.0)

    # thermal strength (m/s), bounded to our world's 1.5..5 range
    heat = rad / 250.0
    inst = cape / 400.0
    dry = _clamp(spread / 12.0, 0.0, 1.0)
    W0 = _clamp(1.2 + heat + inst + 0.8 * dry, 1.0, 5.0)

    # how many thermals over the area, and how trustworthy (overcast -> less)
    n = int(_clamp(6 + rad / 110.0 + cape / 130.0, 6, 26))
    base_prob = _clamp(0.75 - cloud / 250.0, 0.2, 0.85)

    # sampled forecast locations (NOT the sim truth)
    rng = np.random.default_rng(seed)
    x0, x1, y0, y1 = bounds
    cands = []
    for _ in range(n):
        x = float(rng.uniform(x0, x1))
        y = float(rng.uniform(y0, y1))
        s = float(_clamp(W0 + rng.normal(0, 0.4), 1.0, 5.0))
        p = float(_clamp(base_prob + rng.normal(0, 0.08), 0.15, 0.95))
        cands.append([round(x, 1), round(y, 1), round(s, 2), round(p, 2)])

    return {
        "generated_at": generated_at,
        "location": {"lat": weather.get("lat"), "lon": weather.get("lon"),
                     "time": weather.get("time")},
        "bounds": list(bounds),
        "wind": [round(wx, 2), round(wy, 2)],
        "cloud_base_m": round(cloud_base),
        "thermal_strength_ms": round(W0, 2),
        "thermal_count": n,
        "candidates": cands,
        "inputs": {"radiation_wm2": rad, "cape_jkg": cape, "blh_m": blh,
                   "cloud_pct": cloud, "temp_c": T, "dewpoint_c": Td,
                   "wind_speed_ms": ws, "wind_dir_deg": wd},
        "source": "open-meteo",
    }


def write_prior(prior: dict, path: str = OUT) -> str:
    with open(path, "w") as f:
        json.dump(prior, f, indent=2)
    return path


def load_prior(path: str = OUT) -> dict:
    with open(path) as f:
        return json.load(f)
