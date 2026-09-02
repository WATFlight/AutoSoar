"""Thermals that MERGE when they get close (goal: '多个thermal融合').

Wraps a list of LifecycleThermal. At any instant, live thermals whose centres
are within ``merge_dist`` are fused into a single, stronger, larger thermal
(clustered by union-find). The lift is summed over the *fused* thermals, so two
nearby cores read as one bigger updraft rather than a dumbbell.
"""

import math


class MergingField:
    def __init__(self, thermals: list, merge_dist: float = 70.0):
        self.thermals = list(thermals)
        self.merge_dist = merge_dist
        self._eff_t = None      # 1-entry cache: effective() is re-queried at the
        self._eff_cache = None  # same t each tick (vertical_velocity + snapshot)

    # -- clustering ---------------------------------------------------------
    def _clusters(self, t):
        """Cluster the *currently* live thermals by their drifted positions at t.
        Each live thermal is carried as ``(thermal, (cx, cy))`` where (cx, cy) is
        its wind-drifted centre ``center(t)`` — so merging tracks where the cores
        actually are now, not their birth positions."""
        live = [(th, th.center(t)) for th in self.thermals if th.alive(t)]
        n = len(live)
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for i in range(n):
            for j in range(i + 1, n):
                (cix, ciy), (cjx, cjy) = live[i][1], live[j][1]
                if math.hypot(cix - cjx, ciy - cjy) < self.merge_dist:
                    parent[find(i)] = find(j)

        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(live[i])
        return list(groups.values())

    def effective(self, t):
        """Fused thermals at time t as (x, y, W0, R, n_merged), at drifted positions."""
        if t == self._eff_t:
            return self._eff_cache
        out = []
        for cl in self._clusters(t):
            if len(cl) == 1:
                th, (cx, cy) = cl[0]
                out.append((cx, cy, th.strength(t), th.R, 1))
            else:
                ws = [max(th.strength(t), 1e-6) for th, _c in cl]
                W = sum(ws)
                cx = sum(c[0] * w for (_th, c), w in zip(cl, ws)) / W
                cy = sum(c[1] * w for (_th, c), w in zip(cl, ws)) / W
                # combined: full strongest + partial of the rest; a bit wider
                Wm = max(ws) + 0.5 * (W - max(ws))
                Rm = max(th.R for th, _c in cl) * (1.0 + 0.12 * (len(cl) - 1))
                out.append((cx, cy, Wm, Rm, len(cl)))
        self._eff_t, self._eff_cache = t, out
        return out

    def vertical_velocity(self, x: float, y: float, t: float) -> float:
        total = 0.0
        for cx, cy, W, R, _ in self.effective(t):
            if W > 0.0:
                total += W * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (R ** 2))
        return total
