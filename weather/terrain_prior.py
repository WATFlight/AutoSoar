"""Terrain-driven thermal trigger prior — upgrade the forecast from "regional
strength" to "point precision" using the shape of the ground.

GFS tells us how strong thermals are today and how high (the regional W* / cloud
base / wind). The GROUND decides WHERE they trigger. We compute, per DEM cell:

  (1) Sun-slope heating  — insolation = how square-on the slope faces the sun at
      the forecast time (sun-facing slopes heat the air most -> trigger).
  (2) Terrain release     — Topographic Position Index: ridges / upper slopes /
      knolls release thermals; valleys don't.
  (3) Wind coupling       — windward (upwind-facing) slopes get a convergence
      bonus, and the triggered thermal is drifted slightly downwind.

trigger = insolation x (1 + a*ridge) x (1 + b*windward)

Local maxima of that field become candidate thermals, with strength = regional W*
scaled by the trigger score. Output is the same (x, y, strength, prob) prior the
planner / companion / dashboard already consume — so terrain placement drops
straight into the pipeline. No heavy GIS deps: elevation via the OpenTopoData free
API, solar geometry computed in closed form.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import os
import time
import urllib.request

import numpy as np

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")
DEM_DATASET = "srtm30m"
_BATCH = 100  # OpenTopoData free API: 100 locations / request, 1 req / s


# ---- solar geometry (NOAA, closed form) ----------------------------------
def solar_position(lat, lon, when_utc):
    """Return (elevation_deg, azimuth_deg) of the sun. azimuth clockwise from N."""
    doy = when_utc.timetuple().tm_yday
    hour = when_utc.hour + when_utc.minute / 60 + when_utc.second / 3600
    gamma = 2 * math.pi / 365 * (doy - 1 + (hour - 12) / 24)
    eqtime = 229.18 * (0.000075 + 0.001868 * math.cos(gamma) - 0.032077 * math.sin(gamma)
                       - 0.014615 * math.cos(2 * gamma) - 0.040849 * math.sin(2 * gamma))
    decl = (0.006918 - 0.399912 * math.cos(gamma) + 0.070257 * math.sin(gamma)
            - 0.006758 * math.cos(2 * gamma) + 0.000907 * math.sin(2 * gamma)
            - 0.002697 * math.cos(3 * gamma) + 0.00148 * math.sin(3 * gamma))
    tst = (hour * 60) + eqtime + 4 * lon            # true solar time (min), UTC
    ha = math.radians(tst / 4 - 180)                 # hour angle
    la = math.radians(lat)
    el = math.asin(math.sin(la) * math.sin(decl) + math.cos(la) * math.cos(decl) * math.cos(ha))
    az = math.atan2(-math.cos(decl) * math.sin(ha),
                    math.cos(la) * math.sin(decl) - math.sin(la) * math.cos(decl) * math.cos(ha))
    return math.degrees(el), math.degrees(az) % 360


def _default_time():
    return _dt.datetime.now(_dt.timezone.utc).replace(hour=18, minute=0, second=0, microsecond=0)


# ---- DEM fetch (OpenTopoData, cached) ------------------------------------
def fetch_dem(lat, lon, size_km, n):
    """Return (lats[n], lons[n], elev[n,n]) over a size_km box. Row 0 = north."""
    os.makedirs(DATA_DIR, exist_ok=True)
    cache = os.path.join(DATA_DIR, f"dem_{lat}_{lon}_{size_km}_{n}.json")
    half_lat = (size_km / 2) / 110.574
    half_lon = (size_km / 2) / (111.320 * math.cos(math.radians(lat)))
    lats = [lat + half_lat - 2 * half_lat * r / (n - 1) for r in range(n)]   # north->south
    lons = [lon - half_lon + 2 * half_lon * c / (n - 1) for c in range(n)]
    if os.path.exists(cache):
        elev = np.array(json.load(open(cache)))
        return lats, lons, elev
    pts = [(la, lo) for la in lats for lo in lons]
    elevs = []
    for i in range(0, len(pts), _BATCH):
        chunk = pts[i:i + _BATCH]
        q = "|".join(f"{a},{o}" for a, o in chunk)
        url = f"https://api.opentopodata.org/v1/{DEM_DATASET}?locations={q}"
        d = json.loads(urllib.request.urlopen(url, timeout=60).read())
        elevs += [r["elevation"] if r["elevation"] is not None else 0.0 for r in d["results"]]
        if i + _BATCH < len(pts):
            time.sleep(1.0)
    elev = np.array(elevs, dtype=float).reshape(n, n)
    json.dump(elev.tolist(), open(cache, "w"))
    return lats, lons, elev


# ---- terrain analysis ----------------------------------------------------
def _box_mean(z, rad):
    n = z.shape[0]
    out = np.zeros_like(z)
    for r in range(n):
        for c in range(n):
            r0, r1 = max(0, r - rad), min(n, r + rad + 1)
            c0, c1 = max(0, c - rad), min(n, c + rad + 1)
            out[r, c] = z[r0:r1, c0:c1].mean()
    return out


def trigger_field(lats, lons, elev, sun_el, sun_az, wind_toward, a_ridge=0.6, b_wind=0.5):
    """Return (score[n,n], dx_m, dy_m). score = sun-slope x ridge x windward."""
    n = elev.shape[0]
    lat0 = lats[n // 2]
    dx = (lons[1] - lons[0]) * 111320 * math.cos(math.radians(lat0))   # east spacing (m)
    dy = (lats[0] - lats[1]) * 110574                                  # north spacing (m), row0=N
    # gradients: d_east (cols increase east), d_north (rows increase south -> negate)
    dzde = np.gradient(elev, axis=1) / dx
    dzdn = -np.gradient(elev, axis=0) / dy
    # surface normal (east, north, up), normalised
    nz = np.ones_like(elev)
    nmag = np.sqrt(dzde**2 + dzdn**2 + 1.0)
    ne, nn, nu = -dzde / nmag, -dzdn / nmag, nz / nmag
    # (1) insolation = max(0, normal . sun) ; 0 if sun below horizon
    if sun_el <= 0:
        ins = np.zeros_like(elev)
    else:
        se = math.cos(math.radians(sun_el)) * math.sin(math.radians(sun_az))
        sn = math.cos(math.radians(sun_el)) * math.cos(math.radians(sun_az))
        su = math.sin(math.radians(sun_el))
        ins = np.clip(ne * se + nn * sn + nu * su, 0, None)
    # (2) ridge / release: topographic position index (normalised, ridges>0)
    tpi = elev - _box_mean(elev, rad=max(2, n // 8))
    ridge = np.clip(tpi / (np.abs(tpi).max() + 1e-9), 0, None)
    # (3) windward: slope face direction . wind-from
    wt = np.hypot(*wind_toward)
    if wt > 1e-6:
        wfe, wfn = -wind_toward[0] / wt, -wind_toward[1] / wt          # wind-from unit
        fmag = np.hypot(dzde, dzdn) + 1e-9
        face_e, face_n = -dzde / fmag, -dzdn / fmag                    # downhill (slope faces)
        windward = np.clip(face_e * wfe + face_n * wfn, 0, None)
    else:
        windward = np.zeros_like(elev)
    score = ins * (1 + a_ridge * ridge) * (1 + b_wind * windward)
    return score, dx, dy


def _hotspots(score, dx, dy, lat0, top_n=6, min_sep_cells=3):
    n = score.shape[0]
    order = sorted(((score[r, c], r, c) for r in range(n) for c in range(n)
                    if 0 < r < n - 1 and 0 < c < n - 1), reverse=True)
    smax = order[0][0] if order else 0.0
    picks = []
    for s, r, c in order:
        if s < 0.25 * smax:
            break
        if all(abs(r - pr) + abs(c - pc) > min_sep_cells for _, pr, pc in picks):
            picks.append((s, r, c))
        if len(picks) >= top_n:
            break
    return picks, smax


# ---- prior assembly ------------------------------------------------------
def build_prior(lat, lon, when_utc=None, size_km=8.0, n=24, regional=None,
                top_n=6, drift_s=60.0):
    """Build a terrain-trigger prior. ``regional`` = {w_star, wind:[e,n] m/s,
    cloud_base_m}; if None, pulled live from Open-Meteo W* for (lat, lon)."""
    when_utc = when_utc or _default_time()
    if regional is None:
        from weather.openmeteo_prior import build_prior as om
        p = om(lat, lon)
        regional = {"w_star": p["thermal_strength_ms"], "wind": p["wind"],
                    "cloud_base_m": p["cloud_base_m"]}
    w_reg = regional["w_star"]
    wind = regional.get("wind", [0.0, 0.0])

    el, az = solar_position(lat, lon, when_utc)
    lats, lons, elev = fetch_dem(lat, lon, size_km, n)
    score, dx, dy = trigger_field(lats, lons, elev, el, az, wind)
    picks, smax = _hotspots(score, dx, dy, lat, top_n=top_n)

    cands = []
    cx, cy = n // 2, n // 2
    for s, r, c in picks:
        east = (c - cx) * dx + wind[0] * drift_s     # drift the thermal downwind
        north = (cy - r) * dy + wind[1] * drift_s
        sc = s / (smax + 1e-9)
        strength = round(w_reg * (0.4 + 0.6 * sc), 2)
        prob = round(min(0.95, 0.3 + 0.6 * sc), 2)
        cands.append([round(east, 1), round(north, 1), strength, prob])

    ext = size_km * 1000 / 2
    return {
        "generated_at": when_utc.strftime("%Y-%m-%dT%H:%MZ"),
        "location": {"lat": lat, "lon": lon, "time": when_utc.strftime("%Y-%m-%dT%H:%MZ")},
        "bounds": [-ext, ext, -ext, ext],
        "wind": wind,
        "cloud_base_m": regional.get("cloud_base_m"),
        "thermal_strength_ms": w_reg,
        "thermal_count": len(cands),
        "candidates": cands,
        "sun": {"elevation_deg": round(el, 1), "azimuth_deg": round(az, 1)},
        "source": "terrain-trigger",
        "_grid": {"lats": lats, "lons": lons, "elev": elev.tolist(),
                  "score": score.tolist(), "dx": dx, "dy": dy},
    }


def main(lat=39.70, lon=-105.30, size_km=8.0, n=24):
    prior = build_prior(lat, lon, size_km=size_km, n=n)
    s = prior["sun"]
    print(f"Terrain-trigger prior @ {lat},{lon}  {prior['location']['time']}")
    print(f"  sun: elev {s['elevation_deg']}deg az {s['azimuth_deg']}deg | "
          f"regional W*={prior['thermal_strength_ms']} m/s wind {prior['wind']}")
    print(f"  {prior['thermal_count']} terrain-triggered hotspots (ENU m, strongest first):")
    for c in sorted(prior["candidates"], key=lambda c: -c[3]):
        print(f"    ({c[0]:7.0f},{c[1]:7.0f})  W*={c[2]:.1f}  p={c[3]:.2f}")
    out = os.path.join(DATA_DIR, f"terrain_prior_{lat}_{lon}.json")
    slim = {k: v for k, v in prior.items() if k != "_grid"}
    json.dump(slim, open(out, "w"), indent=2)
    print(f"  -> {out}")
    return prior


if __name__ == "__main__":
    main()
