"""Unit tests for AtmoMap Kalman filter (companion/ground_monitor.py)."""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from companion.ground_monitor import AtmoMap


LAT, LON = 43.4715, -80.5441   # arbitrary non-zero indoor test coords


def test_zero_coord_rejected():
    m = AtmoMap()
    m.update(0.0, 0.0, 1.0)
    assert len(m) == 0


def test_min_lift_gate():
    m = AtmoMap(min_lift=0.3)
    m.update(LAT, LON, 0.29)
    assert len(m) == 0
    m.update(LAT, LON, 0.30)
    assert len(m) == 1


def test_update_builds_w():
    m = AtmoMap(min_lift=0.0)
    m.update(LAT, LON, 0.5)
    assert m.lookup(LAT, LON) > 0.0


def test_repeated_updates_converge():
    m = AtmoMap(min_lift=0.0, kf_R=0.5)
    for _ in range(20):
        m.update(LAT, LON, 0.4)
    # after many identical observations ŵ should be close to the signal
    assert abs(m.lookup(LAT, LON) - 0.4) < 0.05


def test_p_decreases_after_updates():
    m = AtmoMap(min_lift=0.0)
    k = m._key(LAT, LON)
    m.update(LAT, LON, 0.4)
    p_after_1 = m._P[k]
    m.update(LAT, LON, 0.4)
    p_after_2 = m._P[k]
    assert p_after_2 < p_after_1


def test_predict_decays_w():
    m = AtmoMap(min_lift=0.0)
    m.update(LAT, LON, 1.0)
    w_before = m.lookup(LAT, LON)
    tau = 150.0
    dt = 30.0
    m.predict_all(dt, tau=tau)
    w_after = m.lookup(LAT, LON)
    expected = w_before * math.exp(-dt / tau)
    assert abs(w_after - expected) < 1e-9


def test_predict_p_grows():
    m = AtmoMap(min_lift=0.0)
    m.update(LAT, LON, 0.4)
    k = m._key(LAT, LON)
    # converge P down first
    for _ in range(10):
        m.update(LAT, LON, 0.4)
    p_before = m._P[k]
    m.predict_all(30.0, tau=150.0)
    assert m._P[k] > p_before


def test_lookup_unvisited_returns_zero():
    m = AtmoMap()
    assert m.lookup(1.0, 1.0) == 0.0


def test_separate_cells():
    m = AtmoMap(min_lift=0.0)
    m.update(43.000, -80.000, 0.5)
    m.update(44.000, -80.000, 1.0)
    assert len(m) == 2
    assert m.lookup(43.000, -80.000) != m.lookup(44.000, -80.000)
