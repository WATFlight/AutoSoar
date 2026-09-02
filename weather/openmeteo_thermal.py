"""Same structure as weather/soaringmeteo.py, but the SOURCE is Open-Meteo and the
thermal velocity is COMPUTED (not read), using the convective-velocity-scale w*
algorithm that SoaringMeteo / RASP use.

w* (Deardorff convective velocity scale):

    w* = ( g / T  *  Q0  *  z_i ) ** (1/3)
    Q0 = H / (rho * cp)      # surface kinematic heat flux  (K m/s)

where
    g   = 9.81 m/s^2
    T   = near-surface temperature (K)
    H   = surface sensible heat flux (W/m^2)      <- Open-Meteo sensible_heat_flux
    rho = air density = P / (Rd * T)              <- from surface_pressure, T
    cp  = 1004 J/(kg K),  Rd = 287.05 J/(kg K)
    z_i = convective boundary-layer height (m)    <- Open-Meteo boundary_layer_height

When the surface flux is downward (stable / night) or z_i <= 0, w* = 0 (no thermals).

The other two map variables come straight from Open-Meteo:
    soaring_layer_top_m = boundary_layer_height
    boundary-layer wind = the ~925 hPa wind

Records are returned in the SAME shape as soaringmeteo.fetch_region, so the map
script (weather/plot_soaringmeteo_map.py) plots them unchanged — for direct
comparison against the SoaringMeteo map.
"""

from __future__ import annotations

import datetime as _dt
import math
import time
import urllib.parse
import urllib.request
import urllib.error
import json

GFS = "https://api.open-meteo.com/v1/gfs"
GFS_RES = 0.25                      # GFS native grid (deg)
HOURLY = ["boundary_layer_height", "sensible_heat_flux", "temperature_2m",
          "surface_pressure", "wind_speed_925hPa", "wind_direction_925hPa"]
G, RD, CP = 9.81, 287.05, 1004.0
BATCH = 100                         # locations per Open-Meteo request


def _default_time():
    """Today at 18:00 UTC — a sensible mid-afternoon thermal sample for the US.
    Used so the maps default to the current run instead of a stale hard-coded date."""
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT18:00:00Z")


def compute_wstar(heat_flux_wm2, blh_m, temp_c, pressure_hpa):
    """Deardorff convective velocity scale w* (m/s). 0 when not convective."""
    if heat_flux_wm2 is None or blh_m is None or heat_flux_wm2 <= 0 or blh_m <= 0:
        return 0.0
    t_k = (temp_c if temp_c is not None else 15.0) + 273.15
    p_pa = (pressure_hpa if pressure_hpa is not None else 1013.25) * 100.0
    rho = p_pa / (RD * t_k)
    q0 = heat_flux_wm2 / (rho * CP)                 # kinematic heat flux (K m/s)
    cube = (G / t_k) * q0 * blh_m
    return round(cube ** (1.0 / 3.0), 2) if cube > 0 else 0.0


def _wind_uv(speed_kmh, dir_from_deg):
    """Met wind (speed, FROM-direction) -> (u, v) the air blows TOWARD (km/h)."""
    if speed_kmh is None or dir_from_deg is None:
        return 0.0, 0.0
    d = math.radians(dir_from_deg)
    return round(-speed_kmh * math.sin(d), 1), round(-speed_kmh * math.cos(d), 1)


def _axis(a, b, step):
    """A regular grid from a..b at the requested step (any step, incl. < 0.25)."""
    lo, hi = min(a, b), max(a, b)
    n = max(1, int(round((hi - lo) / step)))
    return [round(lo + i * step, 5) for i in range(n + 1)]


def _get(lats, lons, retries=4):
    q = urllib.parse.urlencode({
        "latitude": ",".join(f"{x:.4f}" for x in lats),
        "longitude": ",".join(f"{x:.4f}" for x in lons),
        "hourly": ",".join(HOURLY), "timezone": "GMT", "forecast_days": 7,
    })
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(f"{GFS}?{q}", timeout=30) as r:
                d = json.loads(r.read())
            return d if isinstance(d, list) else [d]
        except urllib.error.HTTPError as e:
            if e.code == 429 and attempt < retries - 1:
                time.sleep(8 * (attempt + 1))      # back off on rate limit
                continue
            raise


def fetch_region(lat0, lat1, lon0, lon1, at_time, step_deg=GFS_RES):
    """Sample a box at one UTC time; compute w*; return (meta, records) in the
    same shape as soaringmeteo.fetch_region. ``at_time`` like 2026-06-21T18:00:00Z."""
    lats = _axis(lat0, lat1, step_deg)
    lons = _axis(lon0, lon1, step_deg)
    points = [(la, lo) for la in lats for lo in lons]
    key = at_time[:16]                                  # 'YYYY-MM-DDThh:mm'
    recs = []
    for i in range(0, len(points), BATCH):
        chunk = points[i:i + BATCH]
        results = _get([p[0] for p in chunk], [p[1] for p in chunk])
        for (la, lo), res in zip(chunk, results):
            h = res["hourly"]
            try:
                j = next(k for k, t in enumerate(h["time"]) if t[:16] == key)
            except StopIteration:
                continue
            blh = h["boundary_layer_height"][j]
            w = compute_wstar(h["sensible_heat_flux"][j], blh,
                              h["temperature_2m"][j], h["surface_pressure"][j])
            u, v = _wind_uv(h["wind_speed_925hPa"][j], h["wind_direction_925hPa"][j])
            # label by the REQUESTED 0.25-deg grid point (Open-Meteo returns the
            # nearest model-cell centre, which is jittered and would break the grid)
            recs.append({"lat": round(la, 3), "lon": round(lo, 3),
                         "thermal_velocity_ms": w, "soaring_layer_top_m": blh,
                         "wind_u_kmh": u, "wind_v_kmh": v})
    meta = {"source": "Open-Meteo GFS (w* computed)", "zone": "open-meteo",
            "run": "latest", "time": at_time, "step_deg": step_deg,
            "n": len(recs), "bbox": [lat0, lat1, lon0, lon1], "n_requests": (len(points) + BATCH - 1) // BATCH}
    return meta, recs


def map_box(lat=36.687, lon=-97.137, size_km=20.0, step_km=2.0,
            at_time=None, out_name="openmeteo_thermal_20km.png"):
    """A finely-sampled map over a size_km x size_km box centred on (lat, lon).
    NOTE: the w* INPUTS are GFS (~25 km) in the US, so within a 20 km box the field
    is essentially one GFS cell — the fine sampling just renders it smoothly."""
    import os
    at_time = at_time or _default_time()
    from weather.plot_soaringmeteo_map import plot_map, write_csv, _OUTPUT_DIR
    half = size_km / 2.0
    dlat = half / 111.0
    dlon = half / (111.0 * math.cos(math.radians(lat)))
    step = step_km / 111.0
    meta, recs = fetch_region(lat - dlat, lat + dlat, lon - dlon, lon + dlon, at_time, step_deg=step)
    meta["source"] = f"Open-Meteo w* (sampled ~{step_km:.0f} km; GFS inputs)"
    out = os.path.join(_OUTPUT_DIR, out_name)
    spread = [r["thermal_velocity_ms"] for r in recs]
    print(f"Open-Meteo w* @ {at_time}: {size_km:.0f}x{size_km:.0f} km box, "
          f"{meta['n']} points (~{step_km:.0f} km), {meta['n_requests']} requests")
    print(f"  W* over the box: min {min(spread):.2f}  max {max(spread):.2f} m/s")
    print("  saved table ->", write_csv(meta, recs))
    print("  saved map   ->", plot_map(meta, recs, out=out))
    return meta, recs


def main(at_time=None):
    import os
    at_time = at_time or _default_time()
    from weather.plot_soaringmeteo_map import plot_map, write_csv, _OUTPUT_DIR
    meta, recs = fetch_region(34.0, 39.0, -100.0, -94.0, at_time)
    out = os.path.join(_OUTPUT_DIR, "openmeteo_thermal_map.png")
    print(f"Open-Meteo GFS w* @ {at_time}: {meta['n']} points, {meta['n_requests']} requests")
    print("  saved table ->", write_csv(meta, recs))
    print("  saved map   ->", plot_map(meta, recs, out=out))
    return meta, recs


if __name__ == "__main__":
    main()
