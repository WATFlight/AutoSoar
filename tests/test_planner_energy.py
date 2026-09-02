"""planner.energy + the route_planner energy gate (B4): the route can't commit
past where the aircraft could still motor home on the battery."""
import pytest

from planner import route_planner as rp
from planner.energy import EnergyModel


def test_return_home_wh_zero_at_home_and_grows_with_distance():
    e = EnergyModel()
    assert e.return_home_wh(0.0, 0.0, 1000.0) == pytest.approx(0.0, abs=1e-9)
    near = e.return_home_wh(0.0, 1000.0, 1000.0)
    far = e.return_home_wh(0.0, 30000.0, 1000.0)
    assert 0.0 < near < far


def test_affordable_near_yes_far_no():
    e = EnergyModel(battery_wh=10.0)
    assert e.affordable(0.0, 500.0, 2000.0)         # glide home easily -> cheap
    assert not e.affordable(0.0, 60000.0, 2000.0)   # 60 km -> can't motor home on 10 Wh


def test_plan_route_no_energy_is_unchanged():
    route, _ = rp.plan_route({"candidates": [[0.0, 500.0, 4.0, 0.9]]})
    assert "return_home_wh" not in route[0]


def test_plan_route_energy_annotates_and_gates():
    # near strong candidate (the goal) + a far weak one; a small battery still
    # reaches the near one, and the route carries the return-home cost.
    prior = {"candidates": [[0.0, 500.0, 4.0, 0.9], [0.0, 50000.0, 3.0, 0.5]]}
    e = EnergyModel(battery_wh=20.0)
    route, goal = rp.plan_route(prior, plan_alt=3000.0, energy=e)
    assert len(route) == 1
    assert route[0]["enu_y"] == 500.0
    assert route[0]["return_home_wh"] > 0.0


def test_plan_route_energy_blocks_unaffordable_only_candidate():
    # the single candidate is far enough that returning home would exceed the
    # battery -> the route is empty rather than commit to an unsafe leg.
    prior = {"candidates": [[0.0, 80000.0, 4.0, 0.9]]}
    e = EnergyModel(battery_wh=15.0)
    route, _ = rp.plan_route(prior, plan_alt=3000.0, energy=e)
    assert route == []
