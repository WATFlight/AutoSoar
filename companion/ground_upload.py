#!/usr/bin/env python3
"""Pre-flight: upload route + set ArduSoar params to F405 over direct USB/UART.

Connect laptop → USB-UART → F405 before flight. Uploads mission and
confirms GPS fix. Disconnect after "Upload complete" before securing aircraft.

SOAR_POLAR_K formula (Tabor/Guilliard ArduSoar 2018):
    K = 2 × mass_kg × g / (rho × wing_area_m2)  ≈  16 × mass_kg / wing_area_m2
CD0 and SOAR_POLAR_B must be fit to flight data (steady glide at several speeds).

Usage:
    python3 -m companion.ground_upload --conn /dev/ttyUSB0 --route route.waypoints
    python3 -m companion.ground_upload --conn tcp:127.0.0.1:5760 --route route.waypoints
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from companion import mav  # noqa: E402


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="/dev/ttyUSB0",
                    help="Direct FC connection (USB-UART or tcp:... for SITL)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--route", required=True,
                    help=".waypoints file (or .json; .waypoints sibling is used)")
    ap.add_argument("--no-gps-wait", action="store_true",
                    help="Skip GPS fix confirmation (bench test only)")
    ap.add_argument("--soar-alt-min",    type=float, default=80.0,
                    help="SOAR_ALT_MIN — re-enters thermal below this (m)")
    ap.add_argument("--soar-alt-max",    type=float, default=200.0,
                    help="SOAR_ALT_MAX — exits thermal above this (m)")
    ap.add_argument("--soar-alt-cutoff", type=float, default=50.0,
                    help="SOAR_ALT_CUTOFF — glide (throttle-off) begins above this (m)")
    ap.add_argument("--soar-vspeed",        type=float, default=0.7,
                    help="SOAR_VSPEED — vario threshold to trigger thermaling (m/s); "
                         "ArduSoar default 0.7, lower to 0.4 in weak conditions")
    ap.add_argument("--soar-thermal-bank", type=float, default=30.0,
                    help="SOAR_THERMAL_BANK — bank angle during thermaling (deg); "
                         "30 deg per AutoSOAR Depenbusch 2018")
    ap.add_argument("--wp-loiter-rad",     type=float, default=40.0,
                    help="WP_LOITER_RAD — thermaling circle radius (m); "
                         "40 m per AutoSOAR typical thermalling radius")
    ap.add_argument("--mass-kg",           type=float, default=None,
                    help="Aircraft mass (kg) for SOAR_POLAR_K calculation")
    ap.add_argument("--wing-area-m2",      type=float, default=None,
                    help="Wing area (m²) for SOAR_POLAR_K calculation")
    args = ap.parse_args()

    waypoints = args.route
    if waypoints.endswith(".json"):
        waypoints = os.path.splitext(waypoints)[0] + ".waypoints"

    log(f"Connecting to F405 at {args.conn} @ {args.baud}")
    m = mav.connect(args.conn, baud=args.baud)
    log(f"Heartbeat from system {m.target_system}")

    # ArduSoar parameters — set before upload so FC has them before AUTO starts
    mav.set_param(m, "SOAR_ENABLE",        1)
    mav.set_param(m, "SOAR_VSPEED",        args.soar_vspeed)
    mav.set_param(m, "SOAR_ALT_MIN",       args.soar_alt_min)
    mav.set_param(m, "SOAR_ALT_MAX",       args.soar_alt_max)
    mav.set_param(m, "SOAR_ALT_CUTOFF",    args.soar_alt_cutoff)
    mav.set_param(m, "SOAR_THERMAL_BANK",  args.soar_thermal_bank)
    mav.set_param(m, "WP_LOITER_RAD",      args.wp_loiter_rad)

    # SOAR_POLAR_K = 2mg/(rho*A) ≈ 16 * mass / wing_area (Tabor/Guilliard 2018)
    if args.mass_kg and args.wing_area_m2:
        polar_k = 16.0 * args.mass_kg / args.wing_area_m2
        mav.set_param(m, "SOAR_POLAR_K", polar_k)
        log(f"SOAR_POLAR_K = {polar_k:.2f}  (mass={args.mass_kg}kg  "
            f"wing={args.wing_area_m2}m²)  — CD0/B still need flight-data fit")
    else:
        log("SOAR_POLAR_K not set — provide --mass-kg and --wing-area-m2")

    log(f"ArduSoar params: ENABLE=1  VSPEED={args.soar_vspeed}  "
        f"ALT_MIN={args.soar_alt_min}  ALT_MAX={args.soar_alt_max}  "
        f"ALT_CUTOFF={args.soar_alt_cutoff}  BANK={args.soar_thermal_bank}°  "
        f"LOITER_RAD={args.wp_loiter_rad}m")

    # Pre-enable soaring switch (HIGH = automatic mode changes allowed).
    # Without a Pi5 on the aircraft, nothing else will send this command,
    # so we send it here via the direct USB link before disconnecting.
    mav.set_soaring_switch(m, 2)
    log("Soaring switch → HIGH (automatic mode changes enabled)")

    # wait for mission subsystem
    m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=15)

    ok = False
    for attempt in range(4):
        ok, n = mav.upload_qgc_file(m, waypoints)
        if ok:
            break
        log(f"Upload attempt {attempt + 1} rejected, retrying…")
        time.sleep(3)
    if not ok:
        log("FAILED: FC rejected the mission")
        return 1
    log(f"Mission uploaded: {n} items from {os.path.basename(waypoints)}")

    if not args.no_gps_wait:
        log("Waiting for GPS 3D fix…")
        mav.wait_gps_fix(m)
        log("GPS fix confirmed")

    log("=" * 56)
    log("Upload complete. Safe to disconnect USB and secure aircraft.")
    log("Pilot arms via RC → AUTO mode → ArduSoar activates at altitude.")
    log("=" * 56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
