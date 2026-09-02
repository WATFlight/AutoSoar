"""Coordinate-transform tests — the seam most likely to hide a silent bug when
several people edit the planner / companion / SITL frame mappings.

Covers:
  * companion.geo ENU <-> lat/lon round-trips and sign conventions
  * the DUPLICATE enu/latlon implementation in planner.route_planner stays
    numerically identical to companion.geo (catches a future divergence between
    the two copies)
  * planner.write_sitl_thermals ENU(east, north) -> SITL "x_north y_east" mapping
    (SIM_Aircraft scenario-5 frame; an easy axis-swap bug)
  * plan_route survives an empty candidate set (regression guard for the
    weak-weather max([]) crash fixed in 4d8352e)
"""
import math

import pytest

from companion import geo
from planner import route_planner as rp

ORIGIN = (43.47, -80.54)   # the project's usual test field


# ---- companion.geo round-trips -------------------------------------------
def test_enu_latlon_roundtrip():
    east, north = 1500.0, -800.0
    lat, lon = geo.enu_to_latlon(*ORIGIN, east, north)
    e2, n2 = geo.latlon_to_enu(*ORIGIN, lat, lon)
    assert e2 == pytest.approx(east, abs=1e-6)
    assert n2 == pytest.approx(north, abs=1e-6)


def test_latlon_enu_roundtrip():
    lat, lon = 43.485, -80.515
    e, n = geo.latlon_to_enu(*ORIGIN, lat, lon)
    la2, lo2 = geo.enu_to_latlon(*ORIGIN, e, n)
    assert la2 == pytest.approx(lat, abs=1e-9)
    assert lo2 == pytest.approx(lon, abs=1e-9)


def test_sign_conventions():
    # north of origin -> lat up (lon unchanged); east of origin -> lon up (lat unchanged)
    lat_n, lon_n = geo.enu_to_latlon(*ORIGIN, 0.0, 1000.0)
    assert lat_n > ORIGIN[0]
    assert lon_n == pytest.approx(ORIGIN[1], abs=1e-9)
    lat_e, lon_e = geo.enu_to_latlon(*ORIGIN, 1000.0, 0.0)
    assert lon_e > ORIGIN[1]
    assert lat_e == pytest.approx(ORIGIN[0], abs=1e-9)


# ---- haversine ------------------------------------------------------------
def test_one_degree_north_is_about_111km():
    d = geo.haversine_m(43.0, -80.0, 44.0, -80.0)
    assert d == pytest.approx(math.pi * 6378137.0 / 180.0, rel=1e-6)  # ~111195 m


def test_haversine_zero_and_symmetric():
    assert geo.haversine_m(43.47, -80.54, 43.47, -80.54) == pytest.approx(0.0, abs=1e-9)
    a = geo.haversine_m(43.0, -80.0, 43.5, -80.7)
    b = geo.haversine_m(43.5, -80.7, 43.0, -80.0)
    assert a == pytest.approx(b, abs=1e-6)


# ---- the two duplicate implementations must agree -------------------------
def test_planner_geo_matches_companion_geo():
    for east, north in [(0.0, 0.0), (1500.0, -800.0), (-2300.0, 1599.0), (500.0, 3000.0)]:
        assert rp.enu_to_latlon(*ORIGIN, east, north) == pytest.approx(
            geo.enu_to_latlon(*ORIGIN, east, north))
    for lat, lon in [(43.49, -80.51), (43.40, -80.62)]:
        assert rp.latlon_to_enu(*ORIGIN, lat, lon) == pytest.approx(
            geo.latlon_to_enu(*ORIGIN, lat, lon))


# ---- ENU -> SITL frame swap (scenario 5) ----------------------------------
def test_sitl_thermals_frame_swap(tmp_path):
    # ENU east=100, north=200 must be written as "x_north y_east" = "200 100"
    route = [{"enu_x": 100.0, "enu_y": 200.0, "w_star": 3.0}]
    p = tmp_path / "th.txt"
    rp.write_sitl_thermals(route, str(p), radius=500.0, ref_enu=(0.0, 0.0))
    f = p.read_text().split()
    assert float(f[0]) == pytest.approx(200.0)   # x_north <- enu_y
    assert float(f[1]) == pytest.approx(100.0)    # y_east  <- enu_x
    assert float(f[2]) == pytest.approx(3.0)      # strength
    assert float(f[3]) == pytest.approx(500.0)    # radius


def test_sitl_thermals_relative_to_ref():
    import tempfile, os
    route = [{"enu_x": 100.0, "enu_y": 200.0, "w_star": 3.0}]
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "th.txt")
        rp.write_sitl_thermals(route, p, radius=500.0, ref_enu=(100.0, 200.0))
        f = open(p).read().split()
    assert float(f[0]) == pytest.approx(0.0)      # ref is the waypoint itself
    assert float(f[1]) == pytest.approx(0.0)


# ---- empty-candidate guard (regression for the weak-weather crash) --------
def test_plan_route_empty_candidates_no_crash():
    route, goal = rp.plan_route({"candidates": []})
    assert route == []
