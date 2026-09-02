"""Unit tests for the terrain-trigger prior (pure logic, no network/DEM fetch)."""
import datetime as dt

import numpy as np

from weather import terrain_prior as tp


def test_solar_high_at_noon_below_horizon_at_night():
    noon = dt.datetime(2026, 6, 21, 12, 0, tzinfo=dt.timezone.utc)   # local noon at lon 0
    midnight = dt.datetime(2026, 6, 21, 0, 0, tzinfo=dt.timezone.utc)
    el_noon, _ = tp.solar_position(0.0, 0.0, noon)
    el_night, _ = tp.solar_position(0.0, 0.0, midnight)
    assert el_noon > 40           # sun high
    assert el_night < 0           # below the horizon


def _planar_grid(face_south):
    """8x8 grid; a uniform slope facing south (or north). Returns lats, lons, elev."""
    n = 8
    lat0, lon0 = 39.7, -105.3
    half = 0.02
    lats = [lat0 + half - 2 * half * r / (n - 1) for r in range(n)]   # north -> south
    lons = [lon0 - half + 2 * half * c / (n - 1) for c in range(n)]
    rows = np.arange(n).reshape(n, 1)
    # south-facing: high in the north (row 0), low in the south -> elevation decreases southward
    elev = (n - 1 - rows) * 10.0 if face_south else rows * 10.0
    return lats, lons, np.repeat(elev, n, axis=1).astype(float)


def test_sun_facing_slope_scores_higher():
    # southern sun (az 180), moderate elevation, no wind to isolate insolation
    lats, lons, es = _planar_grid(face_south=True)
    s_south, _, _ = tp.trigger_field(lats, lons, es, sun_el=30, sun_az=180, wind_toward=(0, 0))
    lats, lons, en = _planar_grid(face_south=False)
    s_north, _, _ = tp.trigger_field(lats, lons, en, sun_el=30, sun_az=180, wind_toward=(0, 0))
    assert s_south.mean() > s_north.mean()


def test_no_sun_no_trigger():
    lats, lons, e = _planar_grid(face_south=True)
    s, _, _ = tp.trigger_field(lats, lons, e, sun_el=-5, sun_az=180, wind_toward=(0, 0))
    assert s.max() == 0.0
