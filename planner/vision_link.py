"""Runtime vision-feedback link — watch for a report from the Pi, re-plan, emit.

``replan.py`` is the CONSUMER (folds a vision report into the belief map and emits
a new route). This module is the runtime glue the roadmap flagged as missing:
*how a report gets from the Pi to the planner, and what triggers a re-plan.*

It is transport-agnostic: the Pi writes a vision-report JSON to ``--report`` (a
path the ground can read), and this watches that path. Pick the transport per
deployment:

  * shared filesystem (sshfs / NFS over the Pi's wifi/LTE) — Pi writes the file;
  * MAVLink FTP or a small companion socket drops the file into the watch path;
  * manual: ``scp`` the report over.

Trigger = "a newer report appeared". On each new report we run ``replan.replan()``
and write ``route_replanned.{json,waypoints}``, then wait for the next one. The
ground station / operator uploads the new ``.waypoints`` to the aircraft (the same
hand-off the planner already documents).

CLI:
    python -m planner.vision_link --prior prior.json --report /mnt/pi/vision.json
    python -m planner.vision_link --prior prior.json --report vision.json --once
"""
from __future__ import annotations

import argparse
import json
import os
import time

from planner.replan import replan
from planner.route_planner import write_json, write_qgc


def report_mtime(path):
    """Modification time of the report file, or None if it isn't there yet."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


def run_once(prior, report, out_dir, takeoff_alt=120.0, ceiling_alt=700.0):
    """Replan from one vision report; write route_replanned.{json,waypoints}.
    Returns (route_ll, vision_log) — route_ll is [] if nothing is left to fly."""
    route_ll, goal_ll, origin, start, vlog = replan(prior, report)
    if not route_ll:
        return [], vlog
    os.makedirs(out_dir, exist_ok=True)
    write_json(route_ll, prior, origin, goal_ll,
               os.path.join(out_dir, "route_replanned.json"), takeoff_alt, round(ceiling_alt))
    write_qgc(route_ll, origin, os.path.join(out_dir, "route_replanned.waypoints"),
              takeoff_alt, round(ceiling_alt))
    return route_ll, vlog


def watch(prior_path, report_path, out_dir, poll_s=2.0, once=False,
          max_iters=None, log=print):
    """Poll ``report_path``; each time it appears/changes, replan and emit a route.
    ``once`` returns after the first report; ``max_iters`` bounds the poll loop
    (testing). Returns the number of replans performed."""
    with open(prior_path) as f:
        prior = json.load(f)
    last = None
    replans, iters = 0, 0
    while True:
        mt = report_mtime(report_path)
        if mt is not None and mt != last:
            last = mt
            with open(report_path) as f:
                report = json.load(f)
            route_ll, vlog = run_once(prior, report, out_dir)
            for line in vlog:
                log(f"  - {line}")
            log(f"re-planned -> {len(route_ll)} waypoints  ({out_dir}/route_replanned.waypoints)")
            replans += 1
            if once:
                return replans
        iters += 1
        if max_iters is not None and iters >= max_iters:
            return replans
        time.sleep(poll_s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prior", required=True, help="the original prior JSON (full candidate set)")
    ap.add_argument("--report", required=True, help="path the Pi writes its vision report to")
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "routes"))
    ap.add_argument("--poll", type=float, default=2.0, help="seconds between checks")
    ap.add_argument("--once", action="store_true", help="exit after the first report")
    args = ap.parse_args()
    print(f"watching {args.report} for vision reports (Ctrl-C to stop)")
    n = watch(args.prior, args.report, args.out_dir, poll_s=args.poll, once=args.once)
    print(f"done; {n} re-plan(s)")


if __name__ == "__main__":
    main()
