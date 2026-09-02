"""Battery / motor energy budget for the ground planner.

The planner picks thermal waypoints by GLIDE reachability; on a real electric
glider the binding resource is the BATTERY. This mirrors the dashboard engine's
return-home fuel gauge (``dashboard/engine.py`` ``_energy_to_home_wh`` /
``_spare_energy_wh``) so the ground plan and the in-flight monitor agree: don't
commit to a waypoint you couldn't still motor home from on the remaining battery.

Defaults match the dashboard's ``Params`` (≈2 m electric glider). Override per the
final airframe/battery once the hardware team locks the pack.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class EnergyModel:
    battery_wh: float = 40.0          # usable pack energy (Wh)
    usable_frac: float = 0.85         # plan within this fraction (land with charge)
    motor_power_w: float = 600.0      # motor draw when running
    base_power_w: float = 20.0        # avionics, always on: FC + Pi + GPS + radio
    motor_climb_ms: float = 1.5       # climb rate on the motor
    cruise_speed_ms: float = 16.0
    glide_ratio: float = 22.0
    home_reserve_m: float = 130.0     # altitude kept in hand over home
    reserve_wh: float = 8.0           # never plan below this (land with reserve)
    safety_factor: float = 1.4        # the straight-line estimate is optimistic

    def usable_wh(self) -> float:
        return self.battery_wh * self.usable_frac

    def return_home_wh(self, x: float, y: float, alt: float) -> float:
        """Worst-case Wh to get home from (x, y) at altitude ``alt`` with NO lift:
        avionics for the run home + motor to make up any glide-altitude deficit.
        Mirrors ``dashboard/engine.py``::``_energy_to_home_wh``."""
        d = math.hypot(x, y)
        t_home = d / max(self.cruise_speed_ms, 1.0)
        glide_range = max(0.0, alt - self.home_reserve_m) * self.glide_ratio
        motor_time = 0.0
        if glide_range < d:                                  # altitude alone won't reach home
            deficit = (d / self.glide_ratio + self.home_reserve_m) - alt
            motor_time = max(0.0, deficit) / max(self.motor_climb_ms, 1e-3)
        base_wh = self.base_power_w * (t_home + motor_time) / 3600.0
        motor_wh = self.motor_power_w * motor_time / 3600.0
        return base_wh + motor_wh

    def affordable(self, x: float, y: float, alt: float) -> bool:
        """Can we commit to a waypoint at (x, y) (reached at altitude ``alt``) and
        still return home from it on the motor with the landing reserve intact?"""
        return self.safety_factor * self.return_home_wh(x, y, alt) + self.reserve_wh <= self.usable_wh()
