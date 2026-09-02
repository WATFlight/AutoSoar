"""planner.vision_link (B5): the runtime trigger that re-plans when the Pi drops
a new vision report. Offline (temp files, no transport)."""
import json

from planner import vision_link as vl
from planner.route_planner import enu_to_latlon

ORIGIN = (43.47, -80.54)


def _report(enu_x, enu_y, observations):
    la, lo = enu_to_latlon(*ORIGIN, enu_x, enu_y)
    return {"aircraft": {"lat": la, "lon": lo, "alt_m": 600}, "observations": observations}


def test_report_mtime_missing_is_none(tmp_path):
    assert vl.report_mtime(str(tmp_path / "nope.json")) is None


def test_run_once_emits_route_files(tmp_path):
    prior = {"location": {"lat": ORIGIN[0], "lon": ORIGIN[1]},
             "candidates": [[0.0, 1000.0, 2.0, 0.5], [0.0, 2000.0, 3.0, 0.6]]}
    route, vlog = vl.run_once(prior, _report(0.0, 100.0, []), str(tmp_path))
    assert len(route) >= 1
    assert (tmp_path / "route_replanned.json").exists()
    assert (tmp_path / "route_replanned.waypoints").exists()


def test_watch_triggers_once_on_present_report(tmp_path):
    prior = {"location": {"lat": ORIGIN[0], "lon": ORIGIN[1]},
             "candidates": [[0.0, 1000.0, 2.0, 0.5]]}
    pp = tmp_path / "prior.json"
    pp.write_text(json.dumps(prior))
    rep = tmp_path / "vision.json"
    rep.write_text(json.dumps(_report(0.0, 100.0, [])))
    out = tmp_path / "out"
    n = vl.watch(str(pp), str(rep), str(out), poll_s=0.0, once=True)
    assert n == 1
    assert (out / "route_replanned.waypoints").exists()


def test_watch_max_iters_without_report_does_nothing(tmp_path):
    prior = {"location": {"lat": ORIGIN[0], "lon": ORIGIN[1]},
             "candidates": [[0.0, 1000.0, 2.0, 0.5]]}
    pp = tmp_path / "prior.json"
    pp.write_text(json.dumps(prior))
    n = vl.watch(str(pp), str(tmp_path / "never.json"), str(tmp_path / "out"),
                 poll_s=0.0, max_iters=3)
    assert n == 0


def test_run_once_applies_vision_then_routes(tmp_path):
    # an "empty" obs on the only candidate drops it below min_prob -> no route
    prior = {"location": {"lat": ORIGIN[0], "lon": ORIGIN[1]},
             "candidates": [[0.0, 1000.0, 2.0, 0.5]]}
    report = _report(0.0, 100.0, [{**_obs(0.0, 1000.0), "kind": "empty"}])
    route, vlog = vl.run_once(prior, report, str(tmp_path))
    assert route == []
    assert any("disconfirm" in line for line in vlog)


def _obs(enu_x, enu_y):
    la, lo = enu_to_latlon(*ORIGIN, enu_x, enu_y)
    return {"lat": la, "lon": lo}
