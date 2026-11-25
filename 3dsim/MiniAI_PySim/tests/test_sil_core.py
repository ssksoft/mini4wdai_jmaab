# -*- coding: utf-8 -*-
"""Tests for the Python-control model (no Unreal needed):  pytest 3dsim/MiniAI_PySim/tests"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # .../MiniAI_PySim
sys.path.insert(0, os.path.join(ROOT, "sil"))
import sil_core  # noqa: E402

R_FLAT = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class Const:
    CONTROL_HZ = 500

    def __init__(self, duty=1.0):
        self.duty = duty

    def setup(self, io):
        io.duty = 0.0

    def step(self, io):
        io.duty = self.duty


def run_flat(ctrl, motor="standard", battery="nimh2", seconds=1.0, v0=0.0, dt=1.0 / 240.0):
    car = sil_core.PyCar(ctrl, battery, motor, noise=False)
    v = v0
    for _ in range(int(round(seconds / dt))):
        v += car.step(dt, R_FLAT, [v, 0.0, 0.0], v, True) * dt
    return car, v


# same reference values as tools/check_env.py (speed after 1 s, flat, Ni-MH, duty 1)
@pytest.mark.parametrize("motor,ref", [("standard", 3.385), ("machdash_pro", 5.738)])
def test_full_duty_speed_after_1s(motor, ref):
    _, v = run_flat(Const(1.0), motor)
    assert abs(v - ref) < 0.02


def test_liion_is_faster_than_nimh():
    _, v_nimh = run_flat(Const(1.0), battery="nimh2")
    _, v_li = run_flat(Const(1.0), battery="liion1")
    assert v_li > v_nimh * 1.2


def test_zero_duty_is_short_brake():
    car, v = run_flat(Const(0.0), seconds=0.2, v0=3.0)
    assert v < 3.0 * 0.8                      # brakes, does not coast
    assert car.drv.i < 0                       # current flows backwards (braking)


def test_duty_is_clamped():
    _, v_over = run_flat(Const(1.5), seconds=0.5)
    _, v_one = run_flat(Const(1.0), seconds=0.5)
    assert abs(v_over - v_one) < 1e-9


def test_unset_duty_means_brake():
    class Nothing:
        def step(self, io):
            pass
    car, v = run_flat(Nothing(), seconds=0.2, v0=2.0)
    assert not car.pwm_seen
    assert v < 2.0


def test_io_signals_at_rest():
    seen = {}

    class Probe:
        def step(self, io):
            seen.update(vbat=io.vbat(), temp=io.temp_c(), acc=io.acc, t=io.t)
            io.duty = 0.0
    run_flat(Probe(), seconds=0.05)
    assert abs(seen["vbat"] - 2.75) < 0.01
    assert abs(seen["temp"] - 25.0) < 0.5
    assert abs(seen["acc"][2] - 1.0) < 0.01     # +1 g on the up axis at rest


def test_control_rate():
    calls = []

    class Count:
        CONTROL_HZ = 1000

        def step(self, io):
            calls.append(io.t)
            io.duty = 0.0
    run_flat(Count(), seconds=0.5)
    assert 499 <= len(calls) <= 502


def test_controller_exception_is_reported():
    class Bad:
        def step(self, io):
            raise ZeroDivisionError("boom")
    with pytest.raises(sil_core.ControllerError) as e:
        run_flat(Bad(), seconds=0.01)
    assert "ZeroDivisionError" in str(e.value)


@pytest.mark.parametrize("name", ["lc_brake", "template"])
def test_bundled_controllers_load_and_run(name):
    path = os.path.join(ROOT, "controllers", name + ".py")
    ctrl = sil_core.load_controller(path)
    run_flat(ctrl, seconds=3.0)               # must not raise


def test_params_override():
    ctrl = sil_core.load_controller(os.path.join(ROOT, "controllers", "lc_brake.py"), {"RUN_DUTY": 0.7})
    assert ctrl.RUN_DUTY == 0.7


def test_imu_axes_mapping():
    imu = sil_core.VirtualLSM6DS3(axes="y,-x,z", noise=False)
    imu.update(0.01, R_FLAT, [0.0, 0.0, 0.0])
    ax, ay, az = imu.sensor[:3]
    assert ax == 0.0 and ay == 0.0 and abs(az - 1.0) < 1e-9
    with pytest.raises(ValueError):
        sil_core.parse_axes("x,x,z")
