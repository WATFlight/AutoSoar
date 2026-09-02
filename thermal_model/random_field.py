"""Realistic random thermal world (goals.md goal 1).

  1.1 random 2-D field   - thermals anywhere on the map, not a corridor
  1.2 birth-death        - a space-time Poisson process: thermals spawn at random
                           times/places with random lifespans; the set turns over
  1.3 wind drift         - each thermal drifts downwind over its life (in LifecycleThermal)
  1.4 imperfect map      - the uploaded pre-flight map is a stale snapshot with a
                           global registration offset + per-thermal noise

The whole space-time field is pre-generated (reproducible) and queried as
``field.vertical_velocity(x, y, t)``.
"""

import math

import numpy as np

from thermal_model.lifecycle_thermal import LifecycleThermal, TimeVaryingField


def make_random_world(bounds, t_max, wind=(0.0, 0.0), seed=0,
                      initial_count=10, spawn_rate=0.012,
                      W0_range=(2.8, 4.5), R_range=(50, 70),
                      grow_range=(90, 150), hold_range=(150, 280), decay_range=(150, 250)):
    """Build a churning 2-D thermal field over [0, t_max].

    ``bounds`` = (x_min, x_max, y_min, y_max). ``spawn_rate`` is thermals per
    second (Poisson) born somewhere on the map during the flight; ``initial_count``
    are already alive at takeoff (born in the past, so caught at various ages).
    """
    rng = np.random.default_rng(seed)
    x0, x1, y0, y1 = bounds

    def _make(birth):
        x = float(rng.uniform(x0, x1))
        y = float(rng.uniform(y0, y1))
        # each core wanders on its own: random amplitude/period/direction, so the
        # field meanders rather than translating as one rigid straight line.
        meander = (
            float(rng.uniform(30, 65)) * (1.0 if rng.random() < 0.5 else -1.0),
            2.0 * math.pi / float(rng.uniform(150, 350)),
            float(rng.uniform(30, 65)) * (1.0 if rng.random() < 0.5 else -1.0),
            2.0 * math.pi / float(rng.uniform(150, 350)),
        )
        return LifecycleThermal(
            x, y,
            W0_peak=float(rng.uniform(*W0_range)),
            R=float(rng.uniform(*R_range)),
            birth=birth,
            t_grow=float(rng.uniform(*grow_range)),
            t_hold=float(rng.uniform(*hold_range)),
            t_decay=float(rng.uniform(*decay_range)),
            wind=wind,
            meander=meander,
        )

    thermals = []
    # already running at takeoff (birth in the past -> various ages now)
    for _ in range(initial_count):
        thermals.append(_make(birth=-float(rng.uniform(0, 400))))
    # spawned during the flight (Poisson count, uniform birth times)
    n_spawn = int(rng.poisson(spawn_rate * t_max))
    for _ in range(n_spawn):
        thermals.append(_make(birth=float(rng.uniform(0, t_max))))

    return TimeVaryingField(thermals)


def make_uploaded_map(field, upload_time, seed=0,
                      offset=(0.0, 0.0), rotation_deg=0.0, pos_noise=40.0,
                      map_center=None):
    """A stale, mis-registered snapshot of the field at ``upload_time`` (goal 1.4).

    Returns a list of (x, y, W0) the glider gets pre-flight. Only thermals alive
    at ``upload_time`` are seen; their positions are the drifted positions at that
    time, then corrupted by a global offset + rotation + per-thermal noise. So
    some uploaded thermals will have died/moved by arrival, and thermals born
    after ``upload_time`` are absent.
    """
    rng = np.random.default_rng(seed)
    ang = math.radians(rotation_deg)
    ca, sa = math.cos(ang), math.sin(ang)
    if map_center is None:
        xs = [th.x for th in field.thermals]
        ys = [th.y for th in field.thermals]
        map_center = (sum(xs) / len(xs), sum(ys) / len(ys)) if xs else (0.0, 0.0)
    cx0, cy0 = map_center

    uploaded = []
    for th in field.thermals:
        if not th.alive(upload_time):
            continue
        x, y = th.center(upload_time)
        # rotate about the map centre, then translate, then jitter
        dx, dy = x - cx0, y - cy0
        xr = cx0 + ca * dx - sa * dy + offset[0]
        yr = cy0 + sa * dx + ca * dy + offset[1]
        xr += float(rng.normal(0, pos_noise))
        yr += float(rng.normal(0, pos_noise))
        uploaded.append((xr, yr, th.W0_peak))
    return uploaded
