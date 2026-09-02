"""Grab a SoaringMeteo (GFS) point forecast: thermal velocity, soaring-layer top,
and boundary-layer wind — hourly.

SoaringMeteo's GFS grid is ~0.25 deg (~27 km), so a 1 x 1 km area is far smaller
than one grid cell: we fetch the single grid cell that COVERS the point. The
endpoint layout (reverse-engineered from the v2 web app) is:

    {ROOT}/forecast.json                         # index: runs + zones
    {ROOT}/{run}/{zone}/locations/{c}-{u}.json   # a 4x4 block of grid points
    point = block[col % 4][row % 4]              # the cell, with hourly forecasts

Field mapping is taken verbatim from the app's own parser:
    thermalVelocity = v / 10   (m/s)
    soaringLayerTop = bl.h     (m)
    boundaryLayerWind = (bl.u, bl.v)   (km/h, u=east, v=north)
"""

from __future__ import annotations

import json
import math
import os
import urllib.request

ROOT = "https://soaringmeteo.org/v2/data/7/gfs"
BLOCK = 4
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


def _get(url: str, timeout: float = 20.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode())


def load_index():
    return _get(f"{ROOT}/forecast.json")


def zone_for(lat: float, lon: float, index: dict) -> dict:
    for z in index["zones"]:
        x0, y0, x1, y1 = z["raster"]["extent"]
        if x0 <= lon <= x1 and y0 <= lat <= y1:
            return z
    raise ValueError(f"({lat}, {lon}) is not covered by any SoaringMeteo zone")


def grid_index(lat: float, lon: float, zone: dict):
    """(lon, lat) -> (col, row) in the zone's raster (same math as the web app)."""
    x0, y0, x1, y1 = zone["raster"]["extent"]
    res = zone["raster"]["resolution"]
    col = round((lon - x0) / res - 0.5)
    row = round((y1 - lat) / res - 0.5)
    return col, row


def fetch_point(lat: float, lon: float, index: dict = None):
    """Return (run, zone, point_dict). Tries the latest run, falls back to the
    previous one if the latest isn't fully published yet."""
    index = index or load_index()
    zone = zone_for(lat, lon, index)
    col, row = grid_index(lat, lon, zone)
    runs = [f for f in index["forecasts"] if zone["id"] in f["zones"]]
    if not runs:
        raise ValueError(f"no runs cover zone {zone['id']}")
    last_err = None
    for run in reversed(runs):                       # newest first
        url = f"{ROOT}/{run['path']}/{zone['id']}/locations/{col // BLOCK}-{row // BLOCK}.json"
        try:
            block = _get(url)
            return run, zone, block[col % BLOCK][row % BLOCK]
        except Exception as e:                       # not published yet -> older run
            last_err = e
    raise RuntimeError(f"could not fetch any run for {zone['id']}: {last_err}")


def extract_hourly(point: dict) -> list:
    """Hourly rows of the three requested variables (+ wind components)."""
    rows = []
    for day in point.get("d", []):
        for h in day.get("h", []):
            bl = h.get("bl", {})
            u, v = bl.get("u", 0.0), bl.get("v", 0.0)
            rows.append({
                "time": h.get("t"),
                "thermal_velocity_ms": round(h.get("v", 0) / 10.0, 2),
                "soaring_layer_top_m": bl.get("h"),
                "wind_bl_u_kmh": u,
                "wind_bl_v_kmh": v,
                "wind_bl_speed_kmh": round(math.hypot(u, v), 1),
                "wind_bl_from_deg": round((math.degrees(math.atan2(-u, -v)) + 360) % 360),
            })
    return rows


def point_lonlat(col: int, row: int, zone: dict):
    """Grid (col, row) -> (lon, lat) of the cell centre (inverse of grid_index)."""
    x0, y0, x1, y1 = zone["raster"]["extent"]
    res = zone["raster"]["resolution"]
    return (col + 0.5) * res + x0, y1 - (row + 0.5) * res


def _hours(point: dict) -> list:
    out = []
    for day in point.get("d", []):
        out.extend(day.get("h", []))
    return out


def _hour_at(point: dict, target_iso: str):
    for h in _hours(point):
        if h.get("t") == target_iso:
            return h
    return None


def available_times(point: dict) -> list:
    return [h.get("t") for h in _hours(point)]


def peak_time(point: dict):
    """The hour with the strongest thermal — a good default time for a map."""
    hrs = _hours(point)
    return max(hrs, key=lambda h: h.get("v", 0)).get("t") if hrs else None


def fetch_region(lat0, lat1, lon0, lon1, at_time=None, step_deg=None, index=None):
    """Sample a lat/lon box at one time. Returns (meta, records) where each record
    is {lat, lon, thermal_velocity_ms, soaring_layer_top_m, wind_u_kmh, wind_v_kmh}.

    Sampling resolution is the native grid (~0.25 deg); step_deg coarser than that
    subsamples. Grid points are fetched 16 at a time (one request per 4x4 block)."""
    index = index or load_index()
    clat, clon = (lat0 + lat1) / 2.0, (lon0 + lon1) / 2.0
    zone = zone_for(clat, clon, index)
    res = zone["raster"]["resolution"]
    stride = max(1, round((step_deg or res) / res))
    c0 = grid_index(clat, lon0, zone)[0]
    c1 = grid_index(clat, lon1, zone)[0]
    r0 = grid_index(lat1, clon, zone)[1]
    r1 = grid_index(lat0, clon, zone)[1]
    cols = list(range(min(c0, c1), max(c0, c1) + 1, stride))
    rows = list(range(min(r0, r1), max(r0, r1) + 1, stride))
    runs = [f for f in index["forecasts"] if zone["id"] in f["zones"]]
    cc, cr = cols[len(cols) // 2], rows[len(rows) // 2]

    for run in reversed(runs):
        base = f"{ROOT}/{run['path']}/{zone['id']}/locations"
        blocks = {}

        def block(c, r):
            key = (c // BLOCK, r // BLOCK)
            if key not in blocks:
                try:
                    blocks[key] = _get(f"{base}/{key[0]}-{key[1]}.json")
                except Exception:
                    blocks[key] = None
            return blocks[key]

        if block(cc, cr) is None:                 # run not published yet -> older run
            continue
        sample = block(cc, cr)[cc % BLOCK][cr % BLOCK]
        target = at_time or peak_time(sample)
        recs = []
        for c in cols:
            for r in rows:
                blk = block(c, r)
                if blk is None:
                    continue
                h = _hour_at(blk[c % BLOCK][r % BLOCK], target)
                if h is None:
                    continue
                bl = h.get("bl", {})
                lon, lat = point_lonlat(c, r, zone)
                recs.append({"lat": round(lat, 3), "lon": round(lon, 3),
                             "thermal_velocity_ms": round(h.get("v", 0) / 10.0, 2),
                             "soaring_layer_top_m": bl.get("h"),
                             "wind_u_kmh": bl.get("u", 0), "wind_v_kmh": bl.get("v", 0)})
        meta = {"zone": zone["id"], "run": run["path"], "time": target,
                "available_times": available_times(sample), "step_deg": stride * res,
                "n": len(recs), "bbox": [lat0, lat1, lon0, lon1],
                "n_requests": len(blocks)}
        return meta, recs
    raise RuntimeError("no published run covers this region")


def fetch_table(lat: float, lon: float):
    run, zone, point = fetch_point(lat, lon)
    rows = extract_hourly(point)
    meta = {"lat": lat, "lon": lon, "zone": zone["id"], "run": run["path"],
            "elevation_m": point.get("h"), "grid_res_deg": zone["raster"]["resolution"],
            "n_hours": len(rows)}
    return meta, rows


def write_csv(meta: dict, rows: list, path: str = None) -> str:
    os.makedirs(DATA_DIR, exist_ok=True)
    path = path or os.path.join(DATA_DIR, f"soaringmeteo_{meta['lat']}_{meta['lon']}.csv")
    cols = ["time", "thermal_velocity_ms", "soaring_layer_top_m",
            "wind_bl_speed_kmh", "wind_bl_from_deg", "wind_bl_u_kmh", "wind_bl_v_kmh"]
    with open(path, "w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(c, "")) for c in cols) + "\n")
    return path


def main(lat=36.687, lon=-97.137):
    meta, rows = fetch_table(lat, lon)
    grid_km = meta["grid_res_deg"] * 111.0
    print(f"SoaringMeteo GFS @ ({lat}, {lon})  zone={meta['zone']}  run={meta['run']}  "
          f"elev={meta['elevation_m']} m")
    print(f"  grid cell ~{grid_km:.0f} x {grid_km:.0f} km  (a 1x1 km area falls inside ONE cell)")
    print(f"  {meta['n_hours']} hourly steps. First daytime hours:")
    hdr = f"  {'time (UTC)':17} {'W* m/s':>7} {'BLtop m':>8} {'BLwind km/h':>11} {'from':>5}"
    print(hdr)
    shown = [r for r in rows if r["thermal_velocity_ms"] > 0][:12] or rows[:12]
    for r in shown:
        print(f"  {r['time']:17} {r['thermal_velocity_ms']:7.1f} {str(r['soaring_layer_top_m']):>8} "
              f"{r['wind_bl_speed_kmh']:11.1f} {r['wind_bl_from_deg']:5.0f}")
    path = write_csv(meta, rows)
    print(f"saved table -> {path}")
    return meta, rows


if __name__ == "__main__":
    main()
