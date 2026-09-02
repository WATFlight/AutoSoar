"""Time-varying thermals (lifecycle model).

Real thermals are not static: they form, strengthen, hold, weaken, and die, and
new ones appear. Each thermal has a strength envelope over its life:

    grow (ramp 0->W0) -> hold (W0) -> decay (W0->0) -> dead

So the world is now queried as ``vertical_velocity(x, y, t)``. The radius can
breathe a little with strength too (weaker thermals are tighter).
"""

import math

import numpy as np

from thermal_model.thermal import GaussianThermal


class LifecycleThermal:
    def __init__(self, x, y, W0_peak, R, birth, t_grow, t_hold, t_decay, wind=(0.0, 0.0),
                 meander=None):
        self.x = x  # birth position
        self.y = y
        self.W0_peak = W0_peak
        self.R = max(R, 1.0)
        self.birth = birth
        self.t_grow = t_grow
        self.t_hold = t_hold
        self.t_decay = t_decay
        self.death = birth + t_grow + t_hold + t_decay
        self.wind = wind  # bulk downwind drift over its life (goal 1.3)
        # Per-thermal meander so the field does NOT slide as one rigid straight
        # line: a gentle, smooth wander unique to each core, on top of the wind.
        # (amp_x, omega_x, amp_y, omega_y); sin() so center(birth) == birth pos.
        self.meander = meander or (0.0, 0.0, 0.0, 0.0)

    def center(self, t: float) -> tuple:
        """Drifted position at time t: birth + bulk wind*age + a curving meander."""
        age = t - self.birth
        ax, wx, ay, wy = self.meander
        cx = self.x + self.wind[0] * age + ax * math.sin(wx * age)
        cy = self.y + self.wind[1] * age + ay * math.sin(wy * age)
        return (cx, cy)

    def strength(self, t: float) -> float:
        """Current peak updraft W0(t) along the life envelope."""
        age = t - self.birth
        if age <= 0.0 or age >= (self.t_grow + self.t_hold + self.t_decay):
            return 0.0
        if age < self.t_grow:
            return self.W0_peak * (age / self.t_grow)
        if age < self.t_grow + self.t_hold:
            return self.W0_peak
        return self.W0_peak * (1.0 - (age - self.t_grow - self.t_hold) / self.t_decay)

    def alive(self, t: float) -> bool:
        return self.birth < t < self.death

    def vertical_velocity(self, x: float, y: float, t: float) -> float:
        W = self.strength(t)
        if W <= 0.0:
            return 0.0
        # radius breathes slightly with strength (weaker -> a bit tighter)
        R = self.R * (0.7 + 0.3 * W / self.W0_peak)
        cx, cy = self.center(t)
        r2 = (x - cx) ** 2 + (y - cy) ** 2
        return W * math.exp(-r2 / (R ** 2))


class TimeVaryingField:
    def __init__(self, thermals: list):
        self.thermals = list(thermals)

    def vertical_velocity(self, x: float, y: float, t: float) -> float:
        return sum(th.vertical_velocity(x, y, t) for th in self.thermals)

    def alive_thermals(self, t: float):
        return [th for th in self.thermals if th.alive(t)]


def make_lifecycle_corridor(start, goal, n, seed=0, perp_jitter=55.0,
                            born_fraction=0.6, end_margin=250.0, wind=(0.0, 0.0)):
    """Thermals along the route with staggered lifecycles. A fraction are alive
    at takeoff (they go on the uploaded map); the rest are *born during the
    flight* (new thermals not on the map); some of the early ones die before the
    glider arrives. Returns (field, known_at_takeoff) where known_at_takeoff is
    the (x, y, W0_peak) list for the pre-flight uploaded map."""
    rng = np.random.default_rng(seed)
    sx, sy = start
    gx, gy = goal
    dist = math.hypot(gx - sx, gy - sy)
    ux, uy = (gx - sx) / dist, (gy - sy) / dist
    px, py = -uy, ux

    thermals, known = [], []
    alongs = np.linspace(end_margin, dist - end_margin, n)
    for i, a in enumerate(alongs):
        a += float(rng.uniform(-0.4, 0.4)) * (dist / n)
        perp = float(rng.uniform(-perp_jitter, perp_jitter))
        x = sx + a * ux + perp * px
        y = sy + a * uy + perp * py
        W0 = float(rng.uniform(3.0, 4.5))
        R = float(rng.uniform(50, 65))
        t_grow = float(rng.uniform(90, 150))
        t_hold = float(rng.uniform(150, 280))
        t_decay = float(rng.uniform(150, 250))

        alive_at_takeoff = rng.random() < born_fraction
        if alive_at_takeoff:
            # already running at t=0 (somewhere in its life) -> on the map
            birth = -float(rng.uniform(0, t_grow + t_hold * 0.5))
            known.append((x, y, W0))
        else:
            # born mid-flight -> a "new" thermal not on the uploaded map
            birth = float(rng.uniform(150, dist / 16.0 * 0.7))
        thermals.append(LifecycleThermal(x, y, W0, R, birth, t_grow, t_hold, t_decay, wind=wind))
    return TimeVaryingField(thermals), known
