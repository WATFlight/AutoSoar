"""Prior-guided thermal belief (the "upload a thermal map + wind before flight"
strategy).

Before flight you upload candidate thermal *source* locations (a thermal map)
and the wind. Thermals drift downwind, so the predicted *current* position of
each candidate is the source shifted downwind (proposal 4). Each candidate
carries a probability. In flight the glider flies to the best candidate, and
its own measurements confirm (found lift) or disconfirm (searched, nothing)
each one — a simple Bayesian update over a fixed candidate set.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class CandidatePoint:
    x: float                 # predicted current position (source + wind drift)
    y: float
    prob: float              # belief this is a real, usable thermal (0..1)
    strength_guess: float
    strength_var: float = 1.0  # KF variance on strength_guess (AutoSOAR §3.3 Eq. 13-15)
    visited: bool = False    # we have searched here
    confirmed: bool = False   # we found and used lift here


def prob_gaussian(w_star: float, w_z_min: float = 0.4,
                  sigma: float = 0.8) -> float:
    """P(thermal lift exceeds aircraft sink) = 1 − Φ((w_z_min − W*) / σ).

    AutoSOAR Section 3.4.1: probability from Gaussian CDF over convolved lift.
    w_z_min ≈ sink rate at nominal bank angle (0.4 m/s at 30° bank).
    sigma = forecast uncertainty (0.8 m/s is a reasonable prior for NWP output).
    Replaces the ad-hoc linear formula p = 0.4 + 0.1×W*.
    """
    return 0.5 * math.erfc((w_z_min - w_star) / (sigma * math.sqrt(2.0)))


def build_prior(uploaded_sources, wind, drift_distance: float = 0.0) -> list:
    """Turn uploaded source points into wind-drifted candidates.

    ``uploaded_sources``: list of (x, y, strength_guess[, prob]).
    ``wind``: object with .wx, .wy (or a (wx, wy) tuple); thermals drift the way
    the wind blows. ``drift_distance``: metres to shift downwind.
    """
    wx, wy = (wind.wx, wind.wy) if hasattr(wind, "wx") else (wind[0], wind[1])
    speed = math.hypot(wx, wy)
    ux, uy = (wx / speed, wy / speed) if speed > 1e-6 else (0.0, 0.0)
    cands = []
    for s in uploaded_sources:
        x, y, strength = s[0], s[1], s[2]
        prob = s[3] if len(s) > 3 else 0.6
        cands.append(CandidatePoint(
            x=x + ux * drift_distance,
            y=y + uy * drift_distance,
            prob=prob,
            strength_guess=strength,
        ))
    return cands


class BeliefMap:
    def __init__(self, candidates: list, min_prob: float = 0.12):
        self.candidates = candidates
        self.min_prob = min_prob

    def active(self):
        """Candidates still worth considering (not used up, not ruled out)."""
        return [c for c in self.candidates if c.prob >= self.min_prob and not c.confirmed]

    def _reachable(self, x: float, y: float, usable: float,
                   glide_ratio: float, wind: tuple, airspeed: float) -> list:
        """Candidates reachable from (x, y) with wind-corrected glide range.

        Effective range = usable × L/D × (groundspeed / airspeed).
        Headwind shrinks the circle; tailwind expands it. Clamped at 0 so a
        strong headwind doesn't produce negative range.
        """
        out = []
        for c in self.active():
            dx, dy = c.x - x, c.y - y
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                out.append(c)
                continue
            wind_along = (wind[0] * dx + wind[1] * dy) / dist  # >0 tailwind, <0 headwind
            eff_rng = usable * glide_ratio * max(0.0, 1.0 + wind_along / airspeed)
            if dist <= eff_rng:
                out.append(c)
        return out

    def _score(self, c: CandidatePoint, from_pos: tuple, altitude: float,
               plan_alt: float, wind: tuple, airspeed: float,
               bank_sink: float = 0.7, cruise_sink: float = 0.4) -> float:
        """Expected energy rate of the thermal detour: prob × Q_ij.

        Q_ij = (h_climb − h_lost) / (t_transit + t_climb)   (AutoSOAR Eq. 16–18)

        Headwind enters through t_transit = dist / (airspeed − w_h) — not a
        separate multiplier (Chakrabarty & Langelaan 2011 Eq. 22; AutoSOAR Eq. 27).
        bank_sink: aircraft sink rate at 30° bank angle (AutoSOAR Eq. 17, Table A1).
        cruise_sink: gliding sink rate between thermals.
        """
        dx, dy = c.x - from_pos[0], c.y - from_pos[1]
        dist = math.hypot(dx, dy)
        # headwind component toward candidate (> 0 = headwind)
        w_h = -(wind[0] * dx + wind[1] * dy) / dist if dist > 1e-3 else 0.0
        # net climb rate at thermal bank angle (Eq. 17)
        w_c = max(0.0, c.strength_guess - bank_sink)
        if w_c < 1e-3:
            return 0.0
        v_gnd = max(airspeed - w_h, 1.0)          # effective groundspeed in transit
        t_transit = dist / v_gnd                   # Eq. 27
        h_lost = t_transit * cruise_sink           # altitude lost gliding to thermal
        h_climb = max(0.0, plan_alt - altitude - h_lost)  # altitude to regain at thermal
        if h_climb < 1.0:
            return 0.0
        t_climb = h_climb / w_c
        Q_ij = (h_climb - h_lost) / (t_transit + t_climb)  # Eq. 16
        return c.prob * Q_ij

    def best_target(self, x: float, y: float, altitude: float, goal,
                    glide_ratio: float = 22.0, reserve: float = 80.0,
                    wind: tuple = (0.0, 0.0), airspeed: float = 12.0,
                    plan_alt: float = None) -> CandidatePoint | None:
        """Single-step: pick the best reachable candidate toward the goal."""
        if plan_alt is None:
            plan_alt = altitude + max(reserve, 80.0)
        usable = max(0.0, plan_alt - reserve)   # reachability uses thermal ceiling
        d_goal_now = math.hypot(goal[0] - x, goal[1] - y)
        reach = self._reachable(x, y, usable, glide_ratio, wind, airspeed)
        if not reach:
            return None
        ahead = [c for c in reach if math.hypot(goal[0] - c.x, goal[1] - c.y) < d_goal_now]
        pool = ahead or reach
        return max(pool, key=lambda c: self._score(
            c, (x, y), altitude, plan_alt, wind, airspeed))

    def plan_chain(self, x: float, y: float, altitude: float, goal,
                   plan_alt: float, glide_ratio: float = 22.0,
                   reserve: float = 80.0, wind: tuple = (0.0, 0.0),
                   airspeed: float = 12.0,
                   depth: int = 2, discount: float = 0.7) -> CandidatePoint | None:
        """Multi-step lookahead: pick the next waypoint by scoring depth steps ahead.

        After each thermal visit ArduSoar climbs back to plan_alt. Step 2 score
        is discounted by `discount`. Scoring uses Q_ij (prob × energy rate),
        so headwind is embedded in transit time — not a separate multiplier.
        """
        usable = max(0.0, plan_alt - reserve)  # reachability uses thermal ceiling
        d_goal_now = math.hypot(goal[0] - x, goal[1] - y)
        reach = self._reachable(x, y, usable, glide_ratio, wind, airspeed)
        if not reach:
            return None
        ahead = [c for c in reach if math.hypot(goal[0] - c.x, goal[1] - c.y) < d_goal_now]
        pool = ahead or reach

        if depth <= 1:
            return max(pool, key=lambda c: self._score(
                c, (x, y), altitude, plan_alt, wind, airspeed))

        # After visiting a thermal, aircraft is back at plan_alt; score next step
        # 200 m below ceiling so Q_ij differentiates thermals by strength.
        next_alt = max(plan_alt - 200.0, 0.0)
        best_c, best_total = None, -1e9
        for c in pool:
            s1 = self._score(c, (x, y), altitude, plan_alt, wind, airspeed)
            c.confirmed = True
            nxt = self.plan_chain(c.x, c.y, next_alt, goal, plan_alt,
                                  glide_ratio, reserve, wind, airspeed,
                                  depth - 1, discount)
            c.confirmed = False
            s2 = (self._score(nxt, (c.x, c.y), next_alt, plan_alt, wind, airspeed)
                  if nxt else 0.0)
            total = s1 + discount * s2
            if total > best_total:
                best_total, best_c = total, c
        return best_c

    def confirm(self, c: CandidatePoint, x: float, y: float, strength: float,
                meas_var: float = 0.5) -> None:
        # Occupancy: log-odds update, α_pos=0.0045 (AutoSOAR Eq. 20–21).
        c.confirmed = True
        c.visited = True
        lo = math.log(c.prob / (1.0 - c.prob)) + 0.0045
        c.prob = min(0.95, 1.0 / (1.0 + math.exp(-lo)))
        # Strength: scalar KF measurement update (AutoSOAR §3.3 Eq. 13–15).
        # meas_var = σ²_wz (sensor noise variance ≈ 0.5 (m/s)²).
        K = c.strength_var / (c.strength_var + meas_var)
        c.strength_guess = c.strength_guess + K * (strength - c.strength_guess)
        c.strength_var = (1.0 - K) * c.strength_var
        c.x, c.y = x, y

    def disconfirm(self, c: CandidatePoint) -> None:
        # Negative obs weighted 1/5 of positive (AutoSOAR Eq. 21: α_neg = α_pos/5).
        c.visited = True
        lo = math.log(c.prob / (1.0 - c.prob)) - 0.0009
        c.prob = max(0.05, 1.0 / (1.0 + math.exp(-lo)))

    def drift(self, wind, dt: float) -> None:
        """Advect unconfirmed candidates downwind at 50% of wind speed.

        AutoSOAR Table A1: thermal drift rate = 0.5 × wind speed. Thermals
        are carried by convective eddies that lag the mean flow.
        """
        wx, wy = (wind.wx, wind.wy) if hasattr(wind, "wx") else (wind[0], wind[1])
        for c in self.candidates:
            if not c.confirmed:
                c.x += 0.5 * wx * dt
                c.y += 0.5 * wy * dt

    def decay(self, dt: float, tau: float = 750.0) -> None:
        """Unconfirmed candidates lose probability as the forecast ages.

        tau = z_i / W* (Stull 1988: one eddy turnover time — boundary layer
        depth divided by convective velocity scale). Typical: z_i≈1500 m,
        W*≈2 m/s → tau≈750 s. Was 600 s (custom); now literature-backed.
        """
        f = math.exp(-dt / tau)
        for c in self.candidates:
            if not c.confirmed:
                c.prob *= f
