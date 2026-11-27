# -*- coding: utf-8 -*-
"""Headless (MuJoCo) simulator tests:  pytest 3dsim/MiniAI_PySim/tests
Skipped when MuJoCo is not installed or the course FBX (Git LFS) is not downloaded."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # .../MiniAI_PySim
sys.path.insert(0, os.path.join(ROOT, "sil"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

mujoco = pytest.importorskip("mujoco")
import run_headless as RH  # noqa: E402

if RH.lfs_stubs(RH.COURSE_DIR):
    pytest.skip("course FBX not downloaded (git lfs pull)", allow_module_level=True)


def run(tmp_path, *args):
    return RH.main(list(args) + ["--quiet", "--out", str(tmp_path)])


def test_standard_motor_finishes(tmp_path):
    r = run(tmp_path, "lc_brake", "--laps", "2")
    assert r["outcome"] == "ok"
    assert 2.05 <= r["laps"][0] <= 2.40          # lap 1 includes the standing start
    assert 1.45 <= r["laps"][1] <= 1.85
    for f in ("log.csv", "serial.txt", "summary.json", "replay.npz"):
        assert (tmp_path / f).exists()


def test_machdash_without_brake_goes_off(tmp_path):
    r = run(tmp_path, "lc_brake", "--motor", "machdash_pro", "--param", "BRAKE_S=0", "--param", "EARLY_S=0")
    assert r["outcome"] == "courseout"
    assert r["laps"] == []                       # off in lap 1, like the real car


def test_params_reach_the_controller(tmp_path):
    slow = run(tmp_path / "a", "lc_brake", "--laps", "1", "--param", "RUN_DUTY=0.5")
    fast = run(tmp_path / "b", "lc_brake", "--laps", "1")
    assert slow["outcome"] == fast["outcome"] == "ok"
    assert slow["laps"][0] > fast["laps"][0] + 0.3


def test_controller_error_is_reported(tmp_path):
    bad = tmp_path / "bad.py"
    bad.write_text("class Controller:\n    def step(self, io):\n        io.duty = 1 / 0\n", encoding="utf-8")
    r = run(tmp_path / "out", str(bad))
    assert r["outcome"] == "controller_error"


def test_unknown_controller():
    with pytest.raises(RH.SetupError):
        RH.resolve_py("no_such_controller")


def test_sweep(tmp_path, capsys):
    import sweep_headless
    rc = sweep_headless.main(["lc_brake", "--param", "RUN_DUTY=0.8,1.0", "--laps", "1", "--out", str(tmp_path)])
    assert rc == 0
    lines = (tmp_path / "results.csv").read_text(encoding="utf-8-sig").strip().splitlines()
    assert len(lines) == 3                       # header + 2 runs


def test_replay_record(tmp_path):
    import numpy as np
    import replay_headless as RP
    r = run(tmp_path, "lc_brake", "--laps", "1")
    rec = RP.load(str(tmp_path))
    assert rec["outcome"] == "ok" and rec["qpos"].shape[1] == 7
    assert abs(rec["t"][-1] - r["sim_s"]) < 0.05                 # recorded to the end
    assert 55 < len(rec["t"]) / rec["t"][-1] < 65                # about 60 frames per second
    assert int(rec["lap"][-1]) == 1
    assert RP.resolve_out(str(tmp_path)) == str(tmp_path)
    with pytest.raises(RP.ReplayError):
        RP.resolve_out(str(tmp_path / "nothing"))


def test_video(tmp_path):
    pytest.importorskip("imageio_ffmpeg")
    import replay_headless as RP
    run(tmp_path, "lc_brake", "--laps", "1", "--timeout", "1.0")
    try:
        mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), 32, 32).close()
    except Exception:
        pytest.skip("no OpenGL for offscreen rendering (set MUJOCO_GL=egl on Linux)")
    path = RP.video(str(tmp_path), cam="top", fps=10, size=(320, 180), quiet=True)
    assert os.path.getsize(path) > 1000
