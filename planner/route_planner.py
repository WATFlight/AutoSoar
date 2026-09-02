"""Plan a ground-side soaring route from a weather prior.

Greedy strategic chain: starting from home, repeatedly pick the best reachable
thermal candidate toward the goal (reusing `navigation.thermal_prior.BeliefMap`,
the same scoring the dashboard and companion use), step to it, and continue until
the goal vicinity is reached or candidates run out. The result is an ordered list
of waypoints we hand off as an uploadable path; the Pi 5 + flight controller fly
it.

Output formats:
  * route JSON   — our rich format (lat/lon + ENU + forecast W* + probability)
  * QGC .waypoints — standard Mission Planner / QGC route the Pi 5 can upload

CLI:
    python -m planner.route_planner --source soaringmeteo --lat 43.47 --lon -80.54
    python -m planner.route_planner --prior weather/data/soaringmeteo_prior_...json
"""
from __future__ import annotations

import argparse
import json
import math
import os

from companion.geo import enu_to_latlon, latlon_to_enu
from navigation.thermal_prior import BeliefMap, CandidatePoint, prob_gaussian


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_region_prior(source, lat, lon, size_km, at_time=None, w_min=0.8,
                       step_km=None):
    """Build a cross-country prior whose candidates are REAL W* grid cells over a
    size_km box (not a small-box sampled field). Each reachable cell with
    W* >= w_min becomes a candidate at its true ENU position relative to (lat,lon).

    step_km (Open-Meteo only) samples finer than the native ~28 km GFS grid — the
    W* values stay real (per-point Deardorff on GFS inputs) but positions are
    denser, so the route is flyable thermal-to-thermal. SoaringMeteo is fixed-grid.
    """
    half_lat = (size_km / 2.0) / 111.0
    half_lon = (size_km / 2.0) / (111.0 * math.cos(math.radians(lat)))
    if source == "soaringmeteo":
        from weather.soaringmeteo import fetch_region
        meta, recs = fetch_region(lat - half_lat, lat + half_lat,
                                  lon - half_lon, lon + half_lon)
    else:
        import datetime as _dt
        from weather.openmeteo_thermal import fetch_region, GFS_RES
        at_time = at_time or _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT18:00:00Z")
        step_deg = (step_km / 111.0) if step_km else GFS_RES
        meta, recs = fetch_region(lat - half_lat, lat + half_lat,
                                  lon - half_lon, lon + half_lon, at_time, step_deg=step_deg)
    cands, winds, base = [], [], []
    for r in recs:
        w = r["thermal_velocity_ms"]
        if w < w_min:
            continue
        e, n = latlon_to_enu(lat, lon, r["lat"], r["lon"])
        p = prob_gaussian(w)   # 1−Φ((w_z_min−W*)/σ); AutoSOAR §3.4.1
        cands.append([round(e, 1), round(n, 1), round(w, 2), round(p, 2)])
        winds.append((r.get("wind_u_kmh", 0.0), r.get("wind_v_kmh", 0.0)))
        if r.get("soaring_layer_top_m"):
            base.append(r["soaring_layer_top_m"])
    wx = round(sum(w[0] for w in winds) / max(len(winds), 1) / 3.6, 2) if winds else 0.0
    wy = round(sum(w[1] for w in winds) / max(len(winds), 1) / 3.6, 2) if winds else 0.0
    extent = size_km * 1000.0 / 2.0
    return {
        "generated_at": meta.get("run", "latest"),
        "location": {"lat": lat, "lon": lon, "time": meta.get("time")},
        "bounds": [-extent, extent, -extent, extent],
        "wind": [wx, wy],
        "cloud_base_m": round(sum(base) / len(base)) if base else None,
        "thermal_strength_ms": round(max((c[2] for c in cands), default=0.0), 2),
        "thermal_count": len(cands),
        "candidates": cands,
        "source": f"{source}-region",
    }


def plan_route(prior, goal_enu=None, start_enu=(0.0, 0.0), plan_alt=1500.0,
               max_waypoints=8, arrive_m=120.0, airspeed=12.0, lookahead=2,
               energy=None):
    """Return an ordered list of waypoints (dicts) from a weather prior.

    Each waypoint: {seq, enu_x, enu_y, w_star, prob}.

    airspeed: cruise airspeed in m/s (must match AIRSPEED_CRUISE on the FC).
    lookahead: 1 = greedy single-step; >=2 = depth-N chain scoring for flying far.
    Wind is read from prior["wind"] and used to correct glide range and scoring.
    energy (optional EnergyModel): skip waypoints the aircraft can't return home from.
    """
    cands = [CandidatePoint(x=c[0], y=c[1], prob=c[3], strength_guess=c[2])
             for c in prior["candidates"] if len(c) >= 4]
    if not cands:
        return [], (goal_enu or start_enu)
    wind = tuple(prior.get("wind") or [0.0, 0.0])
    belief = BeliefMap(cands)
    if goal_enu is None:
        g = max(cands, key=lambda c: c.prob * c.strength_guess)
        goal_enu = (g.x, g.y)

    route = []
    cur = start_enu
    # Score thermals as if the aircraft is 200 m below the thermal ceiling.
    # plan_alt is the ceiling (reachability); scoring_alt sets Q_ij's altitude gap.
    scoring_alt = max(plan_alt - 200.0, 0.0)
    while len(route) < max_waypoints:
        if lookahead >= 2:
            target = belief.plan_chain(cur[0], cur[1], scoring_alt, goal_enu, plan_alt,
                                       wind=wind, airspeed=airspeed)
        else:
            target = belief.best_target(cur[0], cur[1], scoring_alt, goal_enu,
                                        wind=wind, airspeed=airspeed,
                                        plan_alt=plan_alt)
        if target is None:
            break
        if energy is not None and not energy.affordable(target.x, target.y, plan_alt):
            target.confirmed = True
            continue
        wp = {"seq": len(route) + 1, "enu_x": round(target.x, 1),
              "enu_y": round(target.y, 1),
              "w_star": round(target.strength_guess, 2),
              "prob": round(target.prob, 2)}
        if energy is not None:
            wp["return_home_wh"] = round(energy.return_home_wh(target.x, target.y, plan_alt), 1)
        route.append(wp)
        target.confirmed = True
        cur = (target.x, target.y)
        if math.hypot(goal_enu[0] - cur[0], goal_enu[1] - cur[1]) <= arrive_m:
            break
    return route, goal_enu


def to_latlon_route(route, origin):
    """Attach lat/lon to each ENU waypoint."""
    out = []
    for wp in route:
        lat, lon = enu_to_latlon(origin[0], origin[1], wp["enu_x"], wp["enu_y"])
        out.append({**wp, "lat": round(lat, 7), "lon": round(lon, 7)})
    return out


def write_json(route_ll, prior, origin, goal_ll, path, takeoff_alt, ceiling_alt):
    doc = {
        "source": prior.get("source"),
        "generated_for_time": prior.get("location", {}).get("time"),
        "origin": {"lat": origin[0], "lon": origin[1]},
        "goal": {"lat": goal_ll[0], "lon": goal_ll[1]},
        "takeoff_alt_m": takeoff_alt,
        "ceiling_alt_m": ceiling_alt,
        "thermal_strength_ms": prior.get("thermal_strength_ms"),
        "cloud_base_m": prior.get("cloud_base_m"),
        "wind": prior.get("wind"),
        # each waypoint is a thermal hotspot to soar at (climb to ceiling, then go)
        "waypoints": [{**wp, "role": "thermal_loiter_to_alt"} for wp in route_ll],
        # what the Pi 5 should do with this path (FC is ArduPilot/ArduSoar)
        "handoff": {
            "autopilot": "ArduPilot (ArduSoar)",
            "upload": "the sibling .waypoints file is a native ArduPilot mission; upload as-is",
            "before_auto": ["set SOAR_ENABLE=1",
                            "enable soaring: MAV_CMD_DO_AUX_FUNCTION(88, HIGH)",
                            "set mode AUTO and arm"],
            "per_hotspot": "NAV_LOITER_TO_ALT lets ArduSoar circle and climb to ceiling_alt_m, then it glides to the next",
            "vision_return": "if the Pi reports vision-confirmed thermal positions, feed them back to re-plan (BeliefMap.confirm/disconfirm) and re-emit a route",
        },
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
    return path


def write_sitl_thermals(route_ll, path, radius=400.0, ref_enu=(0.0, 0.0)):
    """Write a SITL thermal-truth file (scenario 5): one thermal per route
    waypoint at its real forecast position, in metres relative to ``ref_enu``
    (the SITL home). Line: "x_north y_east w r", matching SIM_Aircraft.cpp's local
    frame (x=North, y=East). The planner's ENU is x=East, y=North.
    """
    rx, ry = ref_enu
    with open(path, "w") as f:
        for wp in route_ll:
            f.write(f"{wp['enu_y'] - ry:.1f} {wp['enu_x'] - rx:.1f} "
                    f"{wp['w_star']:.2f} {radius:.1f}\n")
    return path


def write_qgc(route_ll, origin, path, takeoff_alt, ceiling_alt, home_alt=0.0,
              plain=False):
    """Write a native ArduPilot mission (QGC WPL 110).

    Soaring-aware (default): home, NAV_TAKEOFF, then each hotspot is a
    NAV_LOITER_TO_ALT that circles/soars up to ``ceiling_alt`` before gliding to
    the next, and a final NAV_RETURN_TO_LAUNCH. With SOAR_ENABLE=1 the loiters are
    where ArduSoar works the thermal; if there's no lift it climbs on the motor, so
    it never hangs. ``plain=True`` emits simple NAV_WAYPOINTs instead.

    Frames: 0=global (home), 3=global-relative-alt. Commands: 16 WAYPOINT,
    22 TAKEOFF, 31 LOITER_TO_ALT, 20 RETURN_TO_LAUNCH.
    """
    rows = ["QGC WPL 110"]

    def row(seq, cur, frame, cmd, p1, p2, lat, lon, alt):
        return (f"{seq}\t{cur}\t{frame}\t{cmd}\t{p1:.6f}\t{p2:.6f}\t0.000000\t"
                f"0.000000\t{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1")

    rows.append(row(0, 1, 0, 16, 0, 0, origin[0], origin[1], home_alt))
    rows.append(row(1, 0, 3, 22, 15.0, 0, origin[0], origin[1], takeoff_alt))
    seq = 2
    for wp in route_ll:
        if plain:
            rows.append(row(seq, 0, 3, 16, 0, 0, wp["lat"], wp["lon"], ceiling_alt))
        else:
            # NAV_LOITER_TO_ALT: p1=heading-required, p2=loiter radius (0=default)
            rows.append(row(seq, 0, 3, 31, 1.0, 0, wp["lat"], wp["lon"], ceiling_alt))
        seq += 1
    rows.append(row(seq, 0, 2, 20, 0, 0, 0.0, 0.0, 0.0))     # RTL
    with open(path, "w") as f:
        f.write("\n".join(rows) + "\n")
    return path


def _load_prior(args):
    if args.prior:
        with open(args.prior) as f:
            return json.load(f)
    if args.source == "soaringmeteo":
        from weather.soaringmeteo_prior import build_prior
        return build_prior(args.lat, args.lon)
    if args.source == "openmeteo":
        from weather.openmeteo_prior import build_prior
        return build_prior(args.lat, args.lon)
    if args.source == "terrain":
        from weather.terrain_prior import build_prior
        return build_prior(args.lat, args.lon)
    raise SystemExit("provide --prior FILE or --source {soaringmeteo,openmeteo,terrain} --lat --lon")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prior", help="a prior JSON file (overrides --source)")
    ap.add_argument("--source", choices=["soaringmeteo", "openmeteo", "terrain"],
                    default="terrain",
                    help="terrain: DEM-based hotspots (best placement, needs internet for DEM + Open-Meteo); "
                         "openmeteo/soaringmeteo: random placement within regional W* bounds")
    ap.add_argument("--lat", type=float, default=43.47)
    ap.add_argument("--lon", type=float, default=-80.54)
    ap.add_argument("--region-km", type=float, default=None,
                    help="plan cross-country over a real W* grid of this size (km) instead of a local box")
    ap.add_argument("--region-step-km", type=float, default=None,
                    help="Open-Meteo sampling step (km) within the region (default: native ~28 km grid)")
    ap.add_argument("--w-min", type=float, default=0.8, help="min W* (m/s) for a region cell to be a candidate")
    ap.add_argument("--goal-lat", type=float, default=None)
    ap.add_argument("--goal-lon", type=float, default=None)
    ap.add_argument("--takeoff-alt", type=float, default=120.0)
    ap.add_argument("--ceiling-alt", type=float, default=None,
                    help="climb-to altitude at each thermal (default: cloud_base - 200 m)")
    ap.add_argument("--plain", action="store_true",
                    help="emit plain NAV_WAYPOINTs instead of soaring NAV_LOITER_TO_ALT")
    ap.add_argument("--max-waypoints", type=int, default=8)
    ap.add_argument("--sitl-thermals", default=None,
                    help="also write a SITL scenario-5 thermal-truth file at the route positions")
    ap.add_argument("--thermal-radius", type=float, default=400.0)
    ap.add_argument("--battery-wh", type=float, default=None,
                    help="usable battery (Wh); enables the motor-energy budget so the route "
                         "won't commit past where the aircraft could still motor home")
    ap.add_argument("--motor-power-w", type=float, default=600.0)
    ap.add_argument("--reserve-wh", type=float, default=8.0)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "routes"))
    args = ap.parse_args()

    energy = None
    if args.battery_wh is not None:
        from planner.energy import EnergyModel
        energy = EnergyModel(battery_wh=args.battery_wh, motor_power_w=args.motor_power_w,
                             reserve_wh=args.reserve_wh)

    if args.region_km and not args.prior:
        prior = build_region_prior(args.source, args.lat, args.lon, args.region_km,
                                   w_min=args.w_min, step_km=args.region_step_km)
    else:
        prior = _load_prior(args)
    if not prior.get("candidates"):
        raise SystemExit(
            f"[planner] no forecast cell clears --w-min {args.w_min} m/s — weak day. "
            f"Lower --w-min, or pick a stronger time/location.")
    loc = prior["location"]
    origin = (loc["lat"], loc["lon"])

    goal_enu = None
    if args.goal_lat is not None and args.goal_lon is not None:
        goal_enu = latlon_to_enu(origin[0], origin[1], args.goal_lat, args.goal_lon)

    # soaring ceiling: climb to just under cloud base at each thermal
    if args.ceiling_alt is not None:
        ceiling = args.ceiling_alt
    else:
        base = prior.get("cloud_base_m") or 600.0
        ceiling = _clamp(base - 200.0, args.takeoff_alt + 50.0, 3000.0)

    # plan reachability from the ceiling's glide range, so the route only hops as
    # far as the aircraft can actually glide between thermals.
    route, goal_enu = plan_route(prior, goal_enu=goal_enu, plan_alt=ceiling,
                                 max_waypoints=args.max_waypoints, energy=energy)
    route_ll = to_latlon_route(route, origin)
    goal_ll = enu_to_latlon(origin[0], origin[1], goal_enu[0], goal_enu[1])

    os.makedirs(args.out_dir, exist_ok=True)
    tag = f"{prior.get('source','route')}_{args.lat}_{args.lon}"
    jpath = write_json(route_ll, prior, origin, goal_ll,
                       os.path.join(args.out_dir, f"route_{tag}.json"),
                       args.takeoff_alt, round(ceiling))
    wpath = write_qgc(route_ll, origin,
                      os.path.join(args.out_dir, f"route_{tag}.waypoints"),
                      args.takeoff_alt, round(ceiling), plain=args.plain)

    if args.sitl_thermals:
        # express thermals relative to the first waypoint (the SITL home), so the
        # aircraft starts at a thermal and climbs out before gliding to the next.
        ref = (route_ll[0]["enu_x"], route_ll[0]["enu_y"]) if route_ll else (0.0, 0.0)
        write_sitl_thermals(route_ll, args.sitl_thermals, radius=args.thermal_radius, ref_enu=ref)
        print(f"  SITL home (first hotspot): {route_ll[0]['lat']:.6f},{route_ll[0]['lon']:.6f}")

    print(f"Ground route from {prior.get('source')}  origin {origin[0]:.5f},{origin[1]:.5f}  "
          f"W*~{prior.get('thermal_strength_ms')} m/s  ceiling {round(ceiling)} m")
    print(f"  goal: {goal_ll[0]:.5f},{goal_ll[1]:.5f}   {len(route_ll)} waypoints:")
    for wp in route_ll:
        print(f"   {wp['seq']:>2}. {wp['lat']:.5f},{wp['lon']:.5f}  "
              f"W*={wp['w_star']:.1f} p={wp['prob']:.2f}  (ENU {wp['enu_x']:.0f},{wp['enu_y']:.0f})")
    print(f"  -> {jpath}")
    print(f"  -> {wpath}  (QGC .waypoints for the Pi 5 to upload)")
    return route_ll


if __name__ == "__main__":
    main()
