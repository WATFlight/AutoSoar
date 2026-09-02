"""Software A — weather collector.

Pulls real weather for a location from the free Open-Meteo API (no key) and saves
a raw dump. Returns a compact snapshot for the day's best soaring window: we pick
the hour of PEAK shortwave radiation in the forecast (representative of when
thermals actually work), so fetching at night still yields a daytime profile.

    from weather.collector import fetch_weather
    w = fetch_weather(43.47, -80.54)        # Waterloo

Run a collector on a timer (cron / scheduler) every few minutes to keep it fresh.
"""

from __future__ import annotations

import json
import os
import urllib.parse
import urllib.request
from datetime import datetime, timezone

OPEN_METEO = "https://api.open-meteo.com/v1/forecast"
HOURLY = [
    "shortwave_radiation", "cape", "boundary_layer_height",
    "temperature_2m", "dew_point_2m", "cloud_cover",
    "wind_speed_925hPa", "wind_direction_925hPa",
    "wind_speed_10m", "wind_direction_10m",
]
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


def fetch_raw(lat: float, lon: float, timeout: float = 15.0) -> dict:
    q = urllib.parse.urlencode({
        "latitude": lat, "longitude": lon,
        "hourly": ",".join(HOURLY),
        "wind_speed_unit": "ms", "forecast_days": 1, "timezone": "auto",
    })
    with urllib.request.urlopen(f"{OPEN_METEO}?{q}", timeout=timeout) as r:
        return json.loads(r.read().decode())


def _peak_hour(hourly: dict) -> int:
    rad = hourly.get("shortwave_radiation") or []
    if not rad:
        return 0
    return max(range(len(rad)), key=lambda i: rad[i] if rad[i] is not None else -1)


def fetch_weather(lat: float, lon: float, save: bool = True, timeout: float = 15.0) -> dict:
    """Real weather snapshot at the day's peak-radiation hour."""
    raw = fetch_raw(lat, lon, timeout)
    h = raw["hourly"]
    i = _peak_hour(h)

    def g(key, default=None):
        v = h.get(key)
        return v[i] if (v and i < len(v) and v[i] is not None) else default

    ws, wd = g("wind_speed_925hPa"), g("wind_direction_925hPa")     # boundary-layer wind
    if ws is None:                                                  # fall back to surface
        ws, wd = g("wind_speed_10m", 0.0), g("wind_direction_10m", 0.0)

    snap = {
        "lat": lat, "lon": lon, "time": h["time"][i],
        "temperature_c": g("temperature_2m"), "dewpoint_c": g("dew_point_2m"),
        "radiation_wm2": g("shortwave_radiation", 0.0), "cape_jkg": g("cape", 0.0),
        "blh_m": g("boundary_layer_height"), "cloud_pct": g("cloud_cover", 0.0),
        "wind_speed_ms": ws or 0.0, "wind_dir_deg": wd or 0.0,
    }
    if save:
        os.makedirs(DATA_DIR, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        fn = os.path.join(DATA_DIR, f"weather_{stamp}.json")
        with open(fn, "w") as f:
            json.dump({"snapshot": snap, "raw": raw}, f, indent=2)
        snap["_saved"] = fn
    return snap


if __name__ == "__main__":
    w = fetch_weather(43.47, -80.54)
    print(f"weather @ ({w['lat']}, {w['lon']}) peak hour {w['time']}:")
    print(f"  radiation {w['radiation_wm2']:.0f} W/m2 | CAPE {w['cape_jkg']:.0f} J/kg | "
          f"BLH {w['blh_m']} m | cloud {w['cloud_pct']:.0f}% | "
          f"T {w['temperature_c']}/{w['dewpoint_c']} C | "
          f"wind {w['wind_speed_ms']:.1f} m/s @ {w['wind_dir_deg']:.0f} deg")
    if w.get("_saved"):
        print("  saved", w["_saved"])
