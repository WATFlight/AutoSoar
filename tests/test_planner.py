"""Unit tests for the ground path planner (pure logic, no network)."""
import math
import os
import tempfile

from navigation.thermal_prior import BeliefMap, CandidatePoint
from planner import route_planner as rp


def test_enu_latlon_roundtrip():
    olat, olon = 43.47, -80.54
    for east, north in [(0, 0), (1500, -2300), (-800, 600)]:
        lat, lon = rp.enu_to_latlon(olat, olon, east, north)
        e2, n2 = rp.latlon_to_enu(olat, olon, lat, lon)
        assert abs(e2 - east) < 1.0 and abs(n2 - north) < 1.0


def test_plan_route_picks_strongest_by_default():
    prior = {"candidates": [[1000, 0, 3.0, 0.8], [2000, 0, 2.0, 0.5], [500, 500, 1.5, 0.4]]}
    route, goal = rp.plan_route(prior, plan_alt=1500)
    assert route, "expected a route"
    last = route[-1]
    assert abs(last["enu_x"] - 1000) < 1 and abs(last["enu_y"]) < 1   # the strongest


def test_plan_route_respects_reachability():
    prior = {"candidates": [[5000, 0, 3.0, 0.9]]}      # 5 km away
    route, _ = rp.plan_route(prior, plan_alt=100)       # reach ~ (100-80)*22 = 440 m
    assert route == []


def test_plan_route_empty_prior():
    route, _ = rp.plan_route({"candidates": []})
    assert route == []


def test_write_qgc_is_soaring_mission():
    route_ll = [{"seq": 1, "enu_x": 0, "enu_y": 0, "w_star": 2.0, "prob": 0.8,
                 "lat": 40.0, "lon": -80.0}]
    with tempfile.TemporaryDirectory() as d:
        p = rp.write_qgc(route_ll, (40.0, -80.0), os.path.join(d, "r.waypoints"),
                         takeoff_alt=120, ceiling_alt=300)
        lines = [l for l in open(p).read().splitlines() if l]
    assert lines[0] == "QGC WPL 110"
    rows = [l.split("\t") for l in lines[1:]]
    assert len(rows) == 4                       # home, takeoff, 1 hotspot, RTL
    assert rows[1][3] == "22"                    # NAV_TAKEOFF
    assert rows[2][3] == "31"                    # NAV_LOITER_TO_ALT (soaring)
    assert rows[3][3] == "20"                    # RETURN_TO_LAUNCH


def test_headwind_blocks_and_tailwind_extends():
    # Flying north to a candidate at y=4000.  alt=300, reserve=80 → usable=220m,
    # still-air range=220*22=4840m.
    # Headwind convention: wind vector points in the direction the air MOVES.
    #   Flying north (positive y), wind=(0,-6) blows south → opposes travel → headwind.
    #   wind_along = -6 → eff_rng = 4840*(12-6)/12 = 2420m < 4000m → blocked.
    #   wind=(0,+6) blows north → assists travel → tailwind.
    #   wind_along = +6 → eff_rng = 4840*(12+6)/12 = 7260m > 4000m → reachable.
    blocked = BeliefMap([CandidatePoint(x=0, y=4000, prob=0.9, strength_guess=3.0)])
    reached = BeliefMap([CandidatePoint(x=0, y=4000, prob=0.9, strength_guess=3.0)])
    assert blocked.best_target(0, 0, 300, (0, 5000),
                               wind=(0.0, -6.0), airspeed=12.0) is None
    assert reached.best_target(0, 0, 300, (0, 5000),
                               wind=(0.0, 6.0), airspeed=12.0) is not None


def test_lookahead_beats_greedy():
    # Lookahead picks B (leads to strong C) instead of A (dead end), covering more waypoints.
    #
    # Geometry (no wind, goal=(10000,1200), plan_alt=150 → usable=70m → range=1540m):
    #   A (1400,    0): W*=2.5, prob=0.9 — high single-step score; A can't reach B or C
    #   B (   0, 1200): W*=1.5, prob=0.8 — lower score; B can reach C
    #   C (1400, 1600): W*=3.5, prob=0.9 — reachable from B (1456m), not from start (2126m) or A (1600m)
    #
    # dist(start→A)=1400  dist(start→B)=1200  dist(start→C)=2126  (all < or > 1540 as needed)
    # dist(A→B)=1844 ✗   dist(A→C)=1600 ✗   dist(B→C)=1456 ✓
    #
    # Greedy: score_A=2.32 > score_B=1.20 → picks A → A is dead end → route=[A] (1 wp)
    # Lookahead: B+C total=3.46 > A+nothing=2.32 → picks B → then C → route=[B,C] (2 wp)
    prior = {
        "candidates": [
            [1400,    0, 2.5, 0.9],   # A
            [   0, 1200, 1.5, 0.8],   # B
            [1400, 1600, 3.5, 0.9],   # C
        ],
        "wind": [0.0, 0.0],
    }
    goal = (10000, 1200)

    route_greedy, _ = rp.plan_route(prior, goal_enu=goal, plan_alt=150,
                                    airspeed=12.0, lookahead=1)
    route_ahead,  _ = rp.plan_route(prior, goal_enu=goal, plan_alt=150,
                                    airspeed=12.0, lookahead=2)

    assert route_greedy, "greedy should find at least one waypoint"
    assert route_ahead,  "lookahead should find at least one waypoint"
    assert abs(route_greedy[0]["enu_x"] - 1400) < 1 and abs(route_greedy[0]["enu_y"]) < 1, \
        f"greedy should pick A(1400,0), got {route_greedy[0]}"
    assert abs(route_ahead[0]["enu_x"]) < 1 and abs(route_ahead[0]["enu_y"] - 1200) < 1, \
        f"lookahead should pick B(0,1200), got {route_ahead[0]}"
    assert len(route_ahead) > len(route_greedy), \
        f"lookahead route should be longer: {len(route_ahead)} vs {len(route_greedy)}"


def test_write_sitl_thermals_relative_to_first():
    route_ll = [{"enu_x": 100, "enu_y": 200, "w_star": 2.0},
                {"enu_x": 100, "enu_y": 5200, "w_star": 2.5}]
    with tempfile.TemporaryDirectory() as d:
        p = rp.write_sitl_thermals(route_ll, os.path.join(d, "t.txt"), radius=300,
                                   ref_enu=(route_ll[0]["enu_x"], route_ll[0]["enu_y"]))
        lines = open(p).read().splitlines()
    # first thermal at the SITL home (0,0); second 5000 m north (x_north col first)
    assert lines[0].split()[:2] == ["0.0", "0.0"]
    assert abs(float(lines[1].split()[0]) - 5000) < 1
