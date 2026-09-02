"""Simplified Gaussian thermal updraft model (Step 1 of the proposal)."""

import math


class GaussianThermal:
    """A thermal whose vertical air velocity falls off as a Gaussian in radius.

        w(r) = W_0 * exp(-r^2 / R_th^2)

    where ``r`` is the horizontal distance from the thermal core. The model is
    altitude-independent: the core does not drift and the thermal does not move.
    """

    def __init__(self, x_c: float, y_c: float, W_0: float, R_th: float):
        self.x_c = x_c
        self.y_c = y_c
        self.W_0 = W_0
        # Guard against a degenerate (zero) radius blowing up the exponent.
        self.R_th = max(R_th, 1.0)

    def vertical_velocity(self, x: float, y: float) -> float:
        """Return the thermal lift ``w`` (m/s) felt by a glider at ``(x, y)``."""
        r2 = (x - self.x_c) ** 2 + (y - self.y_c) ** 2
        return self.W_0 * math.exp(-r2 / (self.R_th ** 2))
