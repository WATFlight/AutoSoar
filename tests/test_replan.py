"""Unit tests for the vision-feedback re-planner (pure logic, no network)."""
from navigation.thermal_prior import CandidatePoint
from planner import replan as rp
from planner.route_planner import enu_to_latlon


ORIGIN = (43.47, -80.54)


def _cands():
    return [CandidatePoint(x=1000, y=0, prob=0.5, strength_guess=2.0),
            CandidatePoint(x=-1000, y=0, prob=0.5, strength_guess=2.0)]


def _obs(enu_x, enu_y, kind, w=2.5):
    lat, lon = enu_to_latlon(ORIGIN[0], ORIGIN[1], enu_x, enu_y)
    o = {"lat": lat, "lon": lon, "kind": kind}
    if kind in ("lift", "cloud"):
        o["w_star"] = w
    return o


def test_lift_raises_confidence():
    c = _cands()
    rp.apply_vision(c, ORIGIN, {"observations": [_obs(1000, 0, "lift")]})
    assert c[0].prob > 0.5


def test_empty_lowers_confidence():
    c = _cands()
    rp.apply_vision(c, ORIGIN, {"observations": [_obs(-1000, 0, "empty")]})
    assert c[1].prob < 0.1


def test_cloud_adds_new_candidate():
    c = _cands()
    n0 = len(c)
    # far from any existing candidate -> added as a new (forecast-missed) thermal
    rp.apply_vision(c, ORIGIN, {"observations": [_obs(8000, 8000, "cloud", w=3.4)]})
    assert len(c) == n0 + 1
    assert any(abs(x.strength_guess - 3.4) < 0.01 for x in c)


def test_replan_routes_to_updated_belief():
    prior = {"location": {"lat": ORIGIN[0], "lon": ORIGIN[1]},
             "candidates": [[1000, 0, 2.0, 0.5], [-1000, 0, 2.0, 0.5]]}
    report = {"aircraft": {"lat": ORIGIN[0], "lon": ORIGIN[1], "alt_m": 600},
              "observations": [_obs(1000, 0, "lift", w=3.0), _obs(-1000, 0, "empty")]}
    route_ll, goal_ll, origin, start, vlog = rp.replan(prior, report)
    assert route_ll, "expected a route after re-plan"
    # should head to the confirmed (east) candidate, not the disconfirmed (west) one
    assert route_ll[-1]["enu_x"] > 0
