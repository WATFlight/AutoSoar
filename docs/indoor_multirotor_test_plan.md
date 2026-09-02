# Indoor Multirotor Fan Test Plan

**Date:** July 2026  
**Location:** SDC, outside WARG bay  
**Contact:** z95xie@uwaterloo.ca

## Objective

Validate the AtmoMap thermal updraft detection algorithm using a borrowed multirotor drone (WARG Houston class) in a controlled indoor environment. Specifically, verify that AtmoMap can:

1. Detect the presence of an updraft (ŵ builds when airflow is present)
2. Track changes in updraft strength
3. Recognise when the airflow disappears (ŵ decays to baseline)

## Software Involved

Only `companion/ground_monitor.py` runs during this test (on our ground laptop), with `--no-replan` to disable route planning. The embedded `AtmoMap` Kalman filter grid processes `VFR_HUD.climb_rate` from the drone's MAVLink telemetry in real time.

```bash
python3 -m companion.ground_monitor \
    --conn /dev/ttyUSB0 \
    --no-replan
```

All other modules (`route_planner`, `pi5_run`, `BeliefMap`, `GuidanceStateMachine`) are not used in this test.

## Hardware

**From WARG:**
- Houston class multirotor (ArduPilot firmware, ALT_HOLD capable, dataflash logging enabled)
- RC controller + pilot
- Indoor flying space (SDC)

**From AutoGlide team:**
- Ground laptop (Mac, running `ground_monitor.py`)
- USB cable to telemetry ground unit
- Upward-facing fan or blower

**Setup notes:**
- WARG telemetry unit outputs MAVLink over USB; requires driver bundled with Mission Planner
- Dataflash logs can be pulled over MAVLink (RF or USB-C) — no SD card access needed
- `pymavlink` must be installed on the ground laptop (`pip install pymavlink`)

## Experimental Plans

Three plans are available in order of preference. Start with Plan A; fall back if needed.

### Plan A — ALT_HOLD + Fan (Primary)

**Mode:** ALT_HOLD  
**Signal:** `CTUN.ThO` (throttle output) drops when fan is on; `VFR_HUD.climb_rate` briefly spikes then returns to zero (suppressed by altitude controller).

**Procedure:**
1. Arm drone, engage ALT_HOLD. Pilot holds horizontal position via RC.
2. Hover above fan at minimum safe altitude (~0.5 m above fan outlet).
3. Fan OFF — hold 60 s to establish throttle baseline.
4. Fan ON — hold 120 s. `ground_monitor.py` logs AtmoMap ŵ live.
5. Fan OFF — hold 60 s. Confirm ŵ decays to baseline.
6. Repeat steps 3–5 for 3–4 cycles.
7. Post-flight: download `.bin` dataflash log via MAVLink, inspect `CTUN.ThO`.

**If fan signal is too weak:**
- Lower `min_lift` from `0.3` to `0.05` in `ground_monitor.py` before the test
- Fly as close to fan outlet as safely possible
- Increase number of on/off cycles to establish statistical evidence

### Plan B — STABILIZE + Pilot Throttle (Backup, no fan needed)

**Mode:** STABILIZE  
**Signal:** Pilot deliberately increases/decreases throttle to produce sustained `climb_rate`, simulating the signal pattern of a drone inside a thermal. No altitude controller suppression — signal is raw and unfiltered.

**Procedure:**
1. Arm drone, engage STABILIZE. Pilot maintains hover at fixed throttle.
2. Baseline: fixed throttle for 60 s (climb_rate ≈ 0).
3. "Thermal ON": pilot slightly increases throttle for 90–120 s → drone rises → climb_rate > 0 → AtmoMap ŵ builds.
4. "Thermal OFF": pilot returns to hover throttle for 60 s → climb_rate → 0 → ŵ decays.
5. Repeat 3–4 cycles.

**Note:** No extra hardware required beyond the drone itself. Tests the full software pipeline (sensor → MAVLink → AtmoMap) without dependence on airflow equipment.

### Plan C — STABILIZE + Fan (Maximum Signal)

Combine Plans A and B: fan provides real airflow, STABILIZE mode removes altitude controller suppression. Produces the strongest and cleanest `climb_rate` signal. Use if Plan A signal is marginal.

## Pass Criteria

| Criterion | Pass |
|---|---|
| ŵ during fan ON | > 0.1 m/s above baseline |
| ŵ after fan OFF (60 s) | < 0.05 m/s (returns to baseline) |
| On/off pattern reproducible | Consistent across ≥ 3 cycles |
| CTUN.ThO during fan ON (Plan A) | Measurable drop vs baseline |

## Key Parameters (`ground_monitor.py`)

| Parameter | Default | Indoor test |
|---|---|---|
| `min_lift` | 0.3 m/s | 0.05 m/s (lower if signal weak) |
| `kf_R` | 0.5 | unchanged |
| `_ATMO_GRID` | 0.001° (111 m) | unchanged — entire room is one cell |
| `DRIFT_INTERVAL` | 30 s | unchanged |

## Debug Log Reference

`ground_monitor.py` emits the following lines for monitoring AtmoMap in real time:

```
[HH:MM:SS] AtmoMap +obs (lat,lon) climb=+0.42 → ŵ=0.187   # measurement update
[HH:MM:SS] AtmoMap decay dt=30s: 1 cells  max_ŵ=0.143      # KF predict step
```

## Notes

- `_ATMO_GRID = 0.001°` ≈ 111 m: the entire indoor space maps to a single grid cell. Spatial localisation is not a goal of this test.
- ArduSoar (`SOAR_ENABLE`) is not used — this test is on a multirotor running ArduCopter, not ArduPlane.
- The AutoGlide team does not arm the drone, change flight modes, or upload missions. We only read telemetry.
