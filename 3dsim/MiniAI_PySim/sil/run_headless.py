# -*- coding: utf-8 -*-
"""Headless run (no Unreal): the same course, car and forces as the UE runtime, rebuilt in MuJoCo,
with your Python controller in the loop.  Good for quick checks and batch runs before going to UE.

    headless.bat lc_brake [--motor machdash_pro] [--battery liion1] [--laps 3] [--param BRAKE_S=0.15] [--plot]
    python 3dsim/MiniAI_PySim/sil/run_headless.py lc_brake ...           (same, without the .bat)
    (controller: a .py file, or a name in MiniAI_PySim/controllers or controllers/local)

Outputs (default folder: 3dsim/MiniAI_PySim/sim_out/<controller name>/):
    log.csv (every step), serial.txt (io.print), summary.json, plot.png (--plot)
Needs: requirements-headless.txt (mujoco, numpy, matplotlib) and the course data in 3dsim/mini4wd_course
(LFS: mini4wd_course/fbx/*.fbx).  See 3dsim/docs/headless_manual.md.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)                                   # .../3dsim/MiniAI_PySim
COURSE_DIR = os.path.join(os.path.dirname(PYSIM), "mini4wd_course")
sys.path.insert(0, HERE)
import sil_core  # noqa: E402

try:                                                            # Japanese output on any Windows console
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass


class SetupError(RuntimeError):
    pass


def lfs_stubs(course_dir):
    """course FBX files that are still Git LFS pointer stubs (not downloaded)"""
    d = os.path.join(course_dir, "fbx")
    out = []
    for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        p = os.path.join(d, f)
        if f.lower().endswith(".fbx") and os.path.getsize(p) < 1024:
            with open(p, "rb") as fh:
                if fh.read(40).startswith(b"version https://git-lfs"):
                    out.append(f)
    return out


def load_backend(course_dir):
    """import MuJoCo and the course model, with messages that say how to fix a broken setup"""
    try:
        import numpy  # noqa: F401
        import mujoco  # noqa: F401
    except ImportError as e:
        raise SetupError("%s が入っていません。3dsim/MiniAI_PySim/setup_headless.bat を実行するか、"
                         "pip install -r 3dsim/MiniAI_PySim/requirements-headless.txt を実行してください" % e.name)
    if not os.path.isfile(os.path.join(course_dir, "mjsim", "m4wd_mj.py")):
        raise SetupError("コースのモデルが見つかりません: %s" % os.path.join(course_dir, "mjsim", "m4wd_mj.py"))
    stubs = lfs_stubs(course_dir)
    if stubs:
        raise SetupError("コース部品の FBX が Git LFS の引換券のままです（%s など %d 個）。"
                         "git lfs pull を実行してください（ZIP ダウンロードでは入手できません）" % (stubs[0], len(stubs)))
    if os.path.join(course_dir, "mjsim") not in sys.path:
        sys.path.insert(0, os.path.join(course_dir, "mjsim"))
    import m4wd_mj
    return m4wd_mj


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lib", help="Python controller: a .py file, or a name in MiniAI_PySim/controllers (or controllers/local)")
    ap.add_argument("--param", action="append", default=[], metavar="KEY=VALUE",
                    help="override a Controller(**params) value (repeatable)")
    ap.add_argument("--battery", default="nimh2", choices=sorted(sil_core.BATTERIES))
    ap.add_argument("--motor", default="standard", choices=sorted(sil_core.MOTORS))
    ap.add_argument("--mass", type=float, default=0.135, help="car mass incl. XIAO + driver board [kg]")
    ap.add_argument("--imu-axes", default="x,y,z", help="sensor x,y,z = body axis (x fwd, y left, z up), e.g. -y,x,z")
    ap.add_argument("--no-noise", action="store_true", help="ideal IMU (no noise / offsets)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=20.0, help="simulation seconds before giving up")
    ap.add_argument("--boot", type=float, default=0.0, help="seconds the car is held still before release")
    ap.add_argument("--course-dir", default=COURSE_DIR)
    ap.add_argument("--out", default=None, help="output folder (default: 3dsim/MiniAI_PySim/sim_out/<controller>)")
    ap.add_argument("--plot", action="store_true", help="also write plot.png (speed, duty, current, gyro)")
    ap.add_argument("--view", action="store_true", help="after the run, replay it in the MuJoCo 3D viewer window")
    ap.add_argument("--video", nargs="?", const="follow", choices=("follow", "top"), default=None,
                    help="after the run, write run.mp4 (camera: follow = behind the car [default], top = whole course)")
    ap.add_argument("--quiet", action="store_true", help="print nothing (for scripts; see the returned dict / summary.json)")
    a = ap.parse_args(argv)
    say = (lambda *x: None) if a.quiet else print

    MJ = load_backend(a.course_dir)
    import numpy as np
    import mujoco

    a.lib = resolve_py(a.lib)
    name = os.path.splitext(os.path.basename(a.lib))[0]
    out = a.out or os.path.join(PYSIM, "sim_out", name)
    os.makedirs(out, exist_ok=True)
    common = dict(mass=a.mass, imu_axes=a.imu_axes, noise=not a.no_noise, seed=a.seed,
                  log_path=os.path.join(out, "log.csv"), serial_path=os.path.join(out, "serial.txt"))
    car = sil_core.PyCar(a.lib, a.battery, a.motor, params=parse_params(a.param), **common)
    seq = []
    if a.boot > 0:
        seq.append((a.boot, None))
    if seq:
        car.boot(seq)
    t_boot = car.t

    m, ds = MJ.build_model(dict(mass=a.mass))
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "car")
    mujoco.mj_forward(m, d)
    C = MJ.course()
    DT, G = MJ.DT, MJ.G
    Ib = m.body_inertia[bid].copy()
    ray_geom = np.zeros(1, np.int32)
    t, t_lap, laps, s_prev = 0.0, None, [], None
    off_t, slow_t, outcome, vpeak, zmax = None, None, None, 0.0, 0.0
    import replay_headless as RP
    rec = RP.Recorder(DT)
    wall0 = time.time()
    try:
        while outcome is None:
            Rb = d.xmat[bid].reshape(3, 3)
            loc = d.xpos[bid].copy(); f, l, u = Rb[:, 0].copy(), Rb[:, 1].copy(), Rb[:, 2].copy()
            vel = d.qvel[0:3].copy(); w = d.qvel[3:6].copy()
            vf = float(np.dot(vel, f))
            start = loc + u * 0.025
            dist = mujoco.mj_ray(m, d, start, -u, None, 1, bid, ray_geom)
            grounded = 0 <= dist <= 0.035
            acc = car.step(DT, Rb.tolist(), vel.tolist(), vf, grounded)
            d.qfrc_applied[:] = 0.0
            if grounded:
                mujoco.mj_applyFT(m, d, f * acc * a.mass, np.zeros(3), d.xipos[bid], bid, d.qfrc_applied)
                fmax = ds["mu_tire"] * a.mass * G * 0.5
                wv = Rb @ w
                for ax in (0.04, -0.04):                          # per-axle tyre side grip (UE formula)
                    p = loc + f * ax + u * 0.012
                    vp = vel + np.cross(wv, p - d.xpos[bid])
                    Fl = max(-fmax, min(fmax, -0.5 * a.mass * float(np.dot(vp, l)) / MJ.DT_UE * 0.8))
                    mujoco.mj_applyFT(m, d, l * Fl, np.zeros(3), p, bid, d.qfrc_applied)
            d.qfrc_applied[3:6] += -ds["ang_damp"] * Ib * w
            rec.add(t, d.qpos[:7], float(np.linalg.norm(vel)), float(car.duty), len(laps))
            mujoco.mj_step(m, d)
            t += DT
            spd = float(np.linalg.norm(vel)); vpeak = max(vpeak, spd); zmax = max(zmax, loc[2])
            if t_lap is None and vf > 0.3:
                t_lap = t                                          # lap timer starts when the car really drives off
            dd = loc - C["fin_p"]
            s, lat = float(np.dot(dd, C["fin_f"])), float(np.dot(dd, C["fin_l"]))
            if t_lap is not None and s_prev is not None and s_prev < 0.0 <= s and abs(lat) < 0.15 and np.dot(vel, C["fin_f"]) > 0:
                if t - t_lap > 0.5:
                    laps.append(t - t_lap)
                    say("  lap %d: %.3f s" % (len(laps), laps[-1]))
                t_lap = t
            s_prev = s
            offtrack = loc[2] < 0.0035
            if u[2] < 0.3:
                off_t = off_t if off_t is not None else t
                offtrack = offtrack or (t - off_t > 0.5)
            else:
                off_t = None
            outside = np.any(loc[:2] < C["lo"][:2] - 0.08) or np.any(loc[:2] > C["hi"][:2] + 0.08)
            slow_t = (slow_t if slow_t is not None else t) if (spd < 0.05 and t_lap is not None and t - t_lap > 1.0) else None
            if offtrack or outside:
                outcome = "courseout"
            elif len(laps) >= a.laps:
                outcome = "ok"
            elif t > a.timeout:
                outcome = "timeout" if t_lap is not None else "never_started"
            elif slow_t is not None and t - slow_t > 1.5:
                outcome = "stuck"
    except sil_core.ControllerError as e:
        outcome = "controller_error"
        say("ERROR:", e)
    finally:
        tail = car.lines[-6:]
        car.close()
    rec.add(t, d.qpos[:7], float(np.linalg.norm(d.qvel[0:3])), float(car.duty), len(laps), force=True)
    rec.save(os.path.join(out, "replay.npz"), a.mass, outcome, a.laps)
    res = dict(controller=a.lib, params=parse_params(a.param), outcome=outcome, laps=[round(x, 4) for x in laps],
               total=round(sum(laps[:a.laps]), 4) if outcome == "ok" else None,
               sim_s=round(t, 3), boot_s=round(t_boot, 3), wall_s=round(time.time() - wall0, 2), vpeak_mps=round(vpeak, 3),
               zmax_m=round(float(zmax), 4), battery=a.battery, motor=a.motor, mass=a.mass, imu_axes=a.imu_axes,
               duty_set=car.pwm_seen, last_print=tail, out=out)
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
    if a.plot:
        plot(os.path.join(out, "log.csv"), os.path.join(out, "plot.png"), name)
    say(json.dumps(res, ensure_ascii=False, indent=1))
    say("結果: %s%s  -> %s" % (OUTCOME_JA.get(outcome, outcome),
                               ("  合計 %.3f s" % res["total"]) if res["total"] else "", out))
    if a.video:
        try:
            res["video"] = RP.video(out, cam=a.video, quiet=a.quiet)
        except RP.ReplayError as e:
            say("動画を作れませんでした:", e)
    if a.view:
        RP.view(out)
    return res


OUTCOME_JA = {"ok": "完走", "courseout": "コースアウト", "stuck": "停止（途中で止まった）", "timeout": "時間切れ",
              "never_started": "一度も走り出さなかった", "controller_error": "制御のエラー"}


def resolve_py(arg):
    """controller path: an existing .py path, or a name in controllers/ or controllers/local/"""
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    fname = os.path.basename(arg) if arg.lower().endswith(".py") else arg + ".py"
    for sub in ("controllers", os.path.join("controllers", "local")):
        cand = os.path.join(PYSIM, sub, fname)
        if os.path.isfile(cand):
            return cand
    raise SetupError("制御ファイルが見つかりません: %s（controllers/ と controllers/local/ を探しました）" % arg)


def parse_params(items):
    out = {}
    for it in items:
        k, v = it.split("=", 1)
        try:
            out[k] = json.loads(v)
        except ValueError:
            out[k] = v
    return out


def plot(csv_path, png, title):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    D = np.genfromtxt(csv_path, delimiter=",", names=True)
    D = D[::4]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(D["t_s"], D["v_mps"], lw=1); ax[0].set_ylabel("speed [m/s]")
    ax[1].plot(D["t_s"], D["duty"], lw=1, color="C1"); ax[1].set_ylabel("duty (PC2)"); ax[1].set_ylim(-0.05, 1.05)
    ax[2].plot(D["t_s"], D["i_motor_A"], lw=1, color="C3"); ax[2].set_ylabel("motor I [A]")
    a2 = ax[2].twinx(); a2.plot(D["t_s"], D["v_bat_V"], lw=1, color="C2"); a2.set_ylabel("V bat [V]", color="C2")
    ax[3].plot(D["t_s"], D["gz_dps"], lw=0.8, label="gyro z"); ax[3].plot(D["t_s"], D["gy_dps"], lw=0.8, label="gyro y")
    ax[3].set_ylabel("[dps]"); ax[3].legend(loc="upper right"); ax[3].set_xlabel("time [s]")
    for x in ax:
        x.grid(alpha=0.3)
    fig.suptitle("Python control: " + title)
    fig.tight_layout(); fig.savefig(png, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    try:
        main()
    except SetupError as e:
        sys.exit("ERROR: %s" % e)
