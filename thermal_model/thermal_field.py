"""A field of several thermals on a finite map.

The world holds a list of ``GaussianThermal`` objects; the lift at a point is the
sum of their contributions (distant thermals contribute ~0, so this behaves like
"whichever thermal you're near"). Helpers build random or route-corridor fields.

The online estimator does NOT need to change for this: it fits a single Gaussian
to a short rolling window, and at any instant the glider is near at most one
thermal, so it always estimates the local one.
"""

import math

import numpy as np

from thermal_model.thermal import GaussianThermal


class ThermalField:
    def __init__(self, thermals: list):
        self.thermals = list(thermals)

    def vertical_velocity(self, x: float, y: float) -> float:
        return sum(th.vertical_velocity(x, y) for th in self.thermals)

    def centers(self):
        return [(th.x_c, th.y_c) for th in self.thermals]


def make_random_field(
    n: int,
    x_range: tuple,
    y_range: tuple,
    strength_range: tuple = (2.5, 4.5),
    radius_range: tuple = (40.0, 70.0),
    min_separation: float = 120.0,
    seed: int = 0,
    max_tries: int = 2000,
) -> ThermalField:
    """Scatter ``n`` thermals uniformly in a box, keeping a minimum spacing."""
    rng = np.random.default_rng(seed)
    thermals, centers = [], []
    tries = 0
    while len(thermals) < n and tries < max_tries:
        tries += 1
        x = float(rng.uniform(*x_range))
        y = float(rng.uniform(*y_range))
        if all(math.hypot(x - cx, y - cy) >= min_separation for cx, cy in centers):
            W0 = float(rng.uniform(*strength_range))
            R = float(rng.uniform(*radius_range))
            thermals.append(GaussianThermal(x, y, W0, R))
            centers.append((x, y))
    return ThermalField(thermals)


def make_corridor_field(
    start: tuple,
    goal: tuple,
    n: int,
    perp_jitter: float = 70.0,
    strength_range: tuple = (2.8, 4.2),
    radius_range: tuple = (45.0, 65.0),
    end_margin: float = 250.0,
    seed: int = 0,
) -> ThermalField:
    """Place ``n`` thermals in a corridor along the start->goal route.

    The figure-8 search only reaches ~2R (~90 m) to each side of the cruise
    line, so for a cross-country test the thermals must sit within that corridor
    to be findable. Each thermal is dropped at a point along the route with a
    small random perpendicular offset (``perp_jitter``).
    """
    rng = np.random.default_rng(seed)
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    dist = math.hypot(dx, dy)
    ux, uy = dx / dist, dy / dist          # along-route unit vector
    px, py = -uy, ux                       # perpendicular unit vector

    # Spread the along-route positions evenly, then jitter each a little.
    alongs = np.linspace(end_margin, dist - end_margin, n)
    thermals = []
    for a in alongs:
        a += float(rng.uniform(-0.5, 0.5)) * (dist / n) * 0.4
        perp = float(rng.uniform(-perp_jitter, perp_jitter))
        cx = sx + a * ux + perp * px
        cy = sy + a * uy + perp * py
        W0 = float(rng.uniform(*strength_range))
        R = float(rng.uniform(*radius_range))
        thermals.append(GaussianThermal(cx, cy, W0, R))
    return ThermalField(thermals)
