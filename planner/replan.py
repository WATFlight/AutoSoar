"""Re-plan the route from in-flight vision/vario feedback.

The Pi 5 reports what it actually saw while flying the uploaded route; we fold that
into the belief map and emit an updated route + mission. This closes the loop:
ground forecast -> fly -> observe -> re-plan -> upload the next leg.

Vision-report schema (what the Pi 5 sends back — our format):

    {
      "time": "2026-06-21T18:30:00Z",
      "aircraft": {"lat": 43.34, "lon": -80.49, "alt_m": 620},   # current position
      "observations": [
        {"lat": 43.34, "lon": -80.31, "kind": "lift",  "w_star": 2.6},  # confirmed lift ahead
        {"lat": 43.30, "lon": -80.40, "kind": "empty"},                  # searched, nothing
        {"lat": 43.28, "lon": -80.35, "kind": "cloud", "w_star": 3.2}    # cumulus the forecast missed
      ]
    }

`kind`: "lift"/"cloud" raise confidence (or add a new candidate); "empty" lowers it.
Re-planning then runs from the aircraft's current position.

CLI:
    python -m planner.replan --prior <prior.json> --vision <report.json>
    python -m planner.replan --prior <prior.json> --demo
"""
from __future__ import annotations

import argparse
import json
import math
import os

from navigation.thermal_prior import BeliefMap, CandidatePoint
from planner.route_planner import (plan_route, to_latlon_route, enu_to_latlon,
                                   latlon_to_enu, write_json, write_qgc, _clamp)


def _nearest(cands, x, y, max_m):
    best, bd = None, max_m
    for c in cands:
        d = math.hypot(c.x - x, c.y - y)
        if d <= bd:
            best, bd = c, d
    return best


def apply_vision(cands, origin, report, match_m=2500.0):
    """Fold a vision report into the candidate set (in place). Returns a log."""
    log = []
    for ob in report.get("observations", []):
        ex, ey = latlon_to_enu(origin[0], origin[1], ob["lat"], ob["lon"])
        kind = ob.get("kind")
        near = _nearest(cands, ex, ey, match_m)
        if kind in ("lift", "cloud"):
            w = ob.get("w_star", 2.5)
            if near is not None:
                near.prob = _clamp(near.prob + 0.4, 0.0, 1.0)
                near.x, near.y, near.strength_guess = ex, ey, w  # refine to the real fix
                log.append(f"confirmed lift near forecast pt -> prob {near.prob:.2f}")
            else:
                cands.append(CandidatePoint(x=ex, y=ey, prob=0.85, strength_guess=w))
                log.append(f"added vision-found thermal (not in forecast) W*={w}")
        elif kind == "empty":
            if near is not None:
                near.prob *= 0.1
                log.append(f"disconfirmed forecast pt -> prob {near.prob:.2f}")
    return log


def replan(prior, report):
    """Return (updated_route_ll, goal_ll, origin, start_enu) after vision feedback."""
    origin = (prior["location"]["lat"], prior["location"]["lon"])
    cands = [CandidatePoint(x=c[0], y=c[1], prob=c[3], strength_guess=c[2])
             for c in prior["candidates"]]
    vlog = apply_vision(cands, origin, report)

    ac = report.get("aircraft")
    start = latlon_to_enu(origin[0], origin[1], ac["lat"], ac["lon"]) if ac else (0.0, 0.0)

    belief = BeliefMap(cands)
    active = belief.active()
    if not active:
        return [], None, origin, start, vlog
    goal = max(active, key=lambda c: c.prob * c.strength_guess)
    route, goal_enu = plan_route_from(cands, (goal.x, goal.y), start)
    route_ll = to_latlon_route(route, origin)
    goal_ll = enu_to_latlon(origin[0], origin[1], goal_enu[0], goal_enu[1])
    return route_ll, goal_ll, origin, start, vlog


def plan_route_from(cands, goal_enu, start_enu, plan_alt=700.0, max_waypoints=8):
    """plan_route but on an already-built candidate list (post-vision)."""
    prior_like = {"candidates": [[c.x, c.y, c.strength_guess, c.prob] for c in cands]}
    return plan_route(prior_like, goal_enu=goal_enu, start_enu=start_enu,
                      plan_alt=plan_alt, max_waypoints=max_waypoints)


def _demo_report(prior):
    """Synthesize a plausible vision report against a prior (for offline testing):
    confirm the first candidate, empty the second, and spot a new cloud."""
    origin = (prior["location"]["lat"], prior["location"]["lon"])
    cs = prior["candidates"]
    obs = []
    if len(cs) >= 1:
        la, lo = enu_to_latlon(origin[0], origin[1], cs[0][0], cs[0][1])
        obs.append({"lat": la, "lon": lo, "kind": "lift", "w_star": cs[0][2]})
    if len(cs) >= 2:
        la, lo = enu_to_latlon(origin[0], origin[1], cs[1][0], cs[1][1])
        obs.append({"lat": la, "lon": lo, "kind": "empty"})
    la, lo = enu_to_latlon(origin[0], origin[1], 4000.0, 4000.0)  # outside the forecast box
    obs.append({"lat": la, "lon": lo, "kind": "cloud", "w_star": 3.4})
    start_lat, start_lon = enu_to_latlon(origin[0], origin[1], 200.0, 200.0)
    return {"time": "demo", "aircraft": {"lat": start_lat, "lon": start_lon, "alt_m": 600},
            "observations": obs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prior", required=True, help="the original prior JSON (full candidate set)")
    ap.add_argument("--vision", help="a Pi 5 vision report JSON")
    ap.add_argument("--demo", action="store_true", help="synthesize a sample vision report")
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "routes"))
    ap.add_argument("--takeoff-alt", type=float, default=120.0)
    ap.add_argument("--ceiling-alt", type=float, default=700.0)
    args = ap.parse_args()

    with open(args.prior) as f:
        prior = json.load(f)
    if args.demo:
        report = _demo_report(prior)
    else:
        with open(args.vision) as f:
            report = json.load(f)

    route_ll, goal_ll, origin, start, vlog = replan(prior, report)
    print("Vision feedback applied:")
    for line in vlog:
        print(f"  - {line}")
    if not route_ll:
        print("No active candidates left after feedback.")
        return
    ac = report.get("aircraft") or {}
    print(f"Re-planned from aircraft {ac.get('lat', '?')},{ac.get('lon', '?')}"
          f"  ->  {len(route_ll)} waypoints:")
    for wp in route_ll:
        print(f"   {wp['seq']:>2}. {wp['lat']:.5f},{wp['lon']:.5f}  W*={wp['w_star']:.1f} p={wp['prob']:.2f}")

    os.makedirs(args.out_dir, exist_ok=True)
    jp = write_json(route_ll, prior, origin, goal_ll,
                    os.path.join(args.out_dir, "route_replanned.json"),
                    args.takeoff_alt, round(args.ceiling_alt))
    wp = write_qgc(route_ll, origin, os.path.join(args.out_dir, "route_replanned.waypoints"),
                   args.takeoff_alt, round(args.ceiling_alt))
    print(f"  -> {jp}\n  -> {wp}")


if __name__ == "__main__":
    main()
