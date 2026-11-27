# -*- coding: utf-8 -*-
"""3D replay of a headless (MuJoCo) run: an interactive viewer window, or an MP4 video.

run_headless.py records the car pose at 60 Hz into <out>/replay.npz on every run. This module plays it back
on the same MuJoCo model, so nothing is simulated again (the result is exactly the run you got).

    headless view  [name | folder] [--speed 0.5] [--loop]        viewer window (mouse: rotate / zoom / pan)
    headless video [name | folder] [--cam follow|top] [--fps 30] [--size 1280x720] [--speed 1] [-o file.mp4]

name = a controller name (-> sim_out/<name>), or the folder given to --out. Default: lc_brake.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)

REC_HZ = 60.0                                   # pose samples per simulated second (written by run_headless)


# ------------------------------------------------------------------ recording (used by run_headless)
class Recorder:
    """Collects the car pose every 1/REC_HZ s of simulated time."""

    def __init__(self, dt):
        self.every = max(1, int(round(1.0 / (REC_HZ * dt))))
        self.n = 0
        self.t, self.qpos, self.speed, self.duty, self.lap = [], [], [], [], []

    def add(self, t, qpos, speed, duty, lap, force=False):
        if force or self.n % self.every == 0:
            self.t.append(t); self.qpos.append(list(qpos)); self.speed.append(speed)
            self.duty.append(duty); self.lap.append(lap)
        self.n += 1

    def save(self, path, mass, outcome, laps_target):
        import numpy as np
        np.savez_compressed(path, t=np.array(self.t), qpos=np.array(self.qpos), speed=np.array(self.speed),
                            duty=np.array(self.duty), lap=np.array(self.lap, dtype=np.int16),
                            mass=np.array(mass), laps_target=np.array(laps_target), outcome=np.array(outcome))


# ------------------------------------------------------------------ loading
class ReplayError(Exception):
    pass


def resolve_out(arg):
    """controller name or folder -> folder that holds replay.npz"""
    for cand in (arg, os.path.join(PYSIM, "sim_out", arg)):
        if os.path.isfile(os.path.join(cand, "replay.npz")):
            return os.path.abspath(cand)
    raise ReplayError("走行の記録（replay.npz）が見つかりません: %s\n"
                      "  先に走らせてください（例：headless %s）。--out を付けて走らせた場合は、そのフォルダを指定します。"
                      % (arg, os.path.basename(arg.rstrip("\\/")) or "lc_brake"))


def load(out):
    import numpy as np
    z = np.load(os.path.join(out, "replay.npz"))
    rec = {k: z[k] for k in z.files}
    rec["outcome"] = str(rec["outcome"])
    summ = os.path.join(out, "summary.json")
    rec["summary"] = json.load(open(summ, encoding="utf-8")) if os.path.isfile(summ) else {}
    return rec


def model_for(rec):
    sys.path.insert(0, HERE)
    import run_headless as RH
    MJ = RH.load_backend(RH.COURSE_DIR)
    m, _ = MJ.build_model(dict(mass=float(rec["mass"])))
    return m, MJ


OUTCOME_JA = {"ok": "完走", "courseout": "コースアウト", "stuck": "停止", "timeout": "時間切れ",
              "never_started": "走り出さず", "controller_error": "制御のエラー"}


OUTCOME_EN = {"ok": "FINISHED", "courseout": "COURSE OUT", "stuck": "STOPPED", "timeout": "TIME OUT",
              "never_started": "NEVER STARTED", "controller_error": "CONTROLLER ERROR"}


def caption(rec, i):
    """one-line overlay (ASCII only: the viewer font and the video font have no Japanese glyphs)"""
    lap = int(rec["lap"][i]); n = int(rec["laps_target"])
    s = "t %5.2f s   %4.2f m/s   duty %.2f   lap %d/%d" % (rec["t"][i], rec["speed"][i], rec["duty"][i], min(lap + 1, n), n)
    if i == len(rec["t"]) - 1:
        s += "   [%s]" % OUTCOME_EN.get(rec["outcome"], rec["outcome"])
    return s


def frame_at(rec, t_sim):
    import numpy as np
    return int(min(len(rec["t"]) - 1, max(0, np.searchsorted(rec["t"], t_sim))))


# ------------------------------------------------------------------ cameras
def course_view(MJ):
    import numpy as np
    C = MJ.course()
    lo, hi = np.array(C["lo"], float), np.array(C["hi"], float)
    centre = (lo + hi) / 2.0
    span = float(max(hi[0] - lo[0], hi[1] - lo[1]))
    return centre, span


def set_follow(cam, qpos, state):
    """camera behind and above the car, turning with it (smoothed)"""
    import numpy as np
    w, x, y, z = qpos[3:7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    if state.get("yaw") is None:
        state["yaw"] = yaw
    d = (yaw - state["yaw"] + math.pi) % (2 * math.pi) - math.pi
    state["yaw"] += 0.12 * d
    cam.type = 0                                  # mjCAMERA_FREE
    cam.lookat[:] = np.array(qpos[0:3]) + np.array([0, 0, 0.01])
    cam.distance, cam.elevation = 0.38, -22.0
    cam.azimuth = math.degrees(state["yaw"])


def set_top(cam, centre, span):
    cam.type = 0
    cam.lookat[:] = [centre[0], centre[1], 0.0]
    cam.distance, cam.elevation, cam.azimuth = span * 1.05, -62.0, 90.0


# ------------------------------------------------------------------ viewer
def view(out, speed=1.0, loop=False):
    import mujoco
    import mujoco.viewer
    rec = load(out)
    m, MJ = model_for(rec)
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "car")
    T = float(rec["t"][-1])
    state = {"paused": False, "restart": False}

    def key(keycode):
        if keycode == 32:                         # space: pause / resume
            state["paused"] = not state["paused"]
        elif keycode == 82:                       # R: from the start
            state["restart"] = True

    print("ビューア：スペース 一時停止 ／ R 最初から ／ マウス 左ドラッグ 回転・右ドラッグ 移動・ホイール 拡大 ／ 窓を閉じて終了")
    print("記録：%s（%.2f s、%s）" % (out, T, OUTCOME_JA.get(rec["outcome"], rec["outcome"])))
    with mujoco.viewer.launch_passive(m, d, key_callback=key, show_left_ui=False, show_right_ui=False) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = bid
        v.cam.distance, v.cam.elevation = 0.6, -30.0
        t_sim, last, shown = 0.0, time.time(), -1
        while v.is_running():
            now = time.time()
            if not state["paused"]:
                t_sim += (now - last) * speed
            last = now
            if state["restart"]:
                t_sim, state["restart"], shown = 0.0, False, -1
            if t_sim > T + 1.0:
                if loop:
                    t_sim = 0.0
                else:
                    t_sim = T + 1.0
            i = frame_at(rec, t_sim)
            if i != shown or state["paused"] != state.get("shown_paused"):
                state["shown_paused"] = state["paused"]
                d.qpos[:7] = rec["qpos"][i]
                d.qvel[:] = 0.0
                mujoco.mj_forward(m, d)
                shown = i
                v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                             caption(rec, i), ("PAUSED" if state["paused"] else "")))
            v.sync()
            time.sleep(1.0 / 120.0)


# ------------------------------------------------------------------ video
def video(out, path=None, cam="follow", fps=30, size=(1280, 720), speed=1.0, quiet=False):
    import numpy as np
    import mujoco
    try:
        import imageio.v2 as imageio
    except ImportError:
        raise ReplayError("動画の書き出しには imageio と imageio-ffmpeg が必要です。setup_headless.bat をもう一度実行してください"
                          "（または pip install imageio imageio-ffmpeg）。")
    from PIL import Image, ImageDraw, ImageFont
    rec = load(out)
    m, MJ = model_for(rec)
    W, H = size
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, W)
    m.vis.global_.offheight = max(m.vis.global_.offheight, H)
    d = mujoco.MjData(m)
    try:
        r = mujoco.Renderer(m, H, W)
    except Exception as e:
        raise ReplayError("3D の描画を始められませんでした（%s）。Linux で画面がない場合は MUJOCO_GL=egl を設定してください。" % e)
    camera = mujoco.MjvCamera()
    centre, span = course_view(MJ)
    font = _font(max(14, H // 34))
    path = path or os.path.join(out, "run.mp4")
    T = float(rec["t"][-1])
    n = int(math.ceil((T / speed + 1.0) * fps))        # +1 s still frame at the end
    state = {}
    wr = imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=8,
                            ffmpeg_log_level="error", pixelformat="yuv420p")
    try:
        for k in range(n):
            i = frame_at(rec, min(T, k * speed / fps))
            d.qpos[:7] = rec["qpos"][i]
            mujoco.mj_forward(m, d)
            if cam == "top":
                set_top(camera, centre, span)
            else:
                set_follow(camera, rec["qpos"][i], state)
            r.update_scene(d, camera)
            img = Image.fromarray(r.render())
            dr = ImageDraw.Draw(img)
            txt = caption(rec, i)
            dr.rectangle([0, 0, W, font.size + 14], fill=(0, 0, 0))
            dr.text((10, 6), txt, fill=(255, 255, 255), font=font)
            wr.append_data(np.asarray(img))
    finally:
        wr.close()
        r.close()
    if not quiet:
        print("動画 -> %s（%d コマ、%.1f s）" % (path, n, n / fps))
    return path


def _font(px):
    from PIL import ImageFont
    for f in ("consola.ttf", "DejaVuSansMono.ttf", "Menlo.ttc", "LiberationMono-Regular.ttf"):
        try:
            return ImageFont.truetype(f, px)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=px)
    except TypeError:
        return ImageFont.load_default()


# ------------------------------------------------------------------ CLI
def main_view(argv=None):
    ap = argparse.ArgumentParser(prog="headless view", description="3D ビューアで走行を再生します（シミュレーションはやり直しません）")
    ap.add_argument("run", nargs="?", default="lc_brake", help="制御の名前（sim_out/<名前>）か、--out で指定したフォルダ")
    ap.add_argument("--speed", type=float, default=1.0, help="再生の速さ（0.25 = 1/4 倍速）")
    ap.add_argument("--loop", action="store_true", help="最後まで行ったら最初から繰り返す")
    a = ap.parse_args(argv)
    try:
        view(resolve_out(a.run), a.speed, a.loop)
    except ReplayError as e:
        print("エラー:", e); return 1
    return 0


def main_video(argv=None):
    ap = argparse.ArgumentParser(prog="headless video", description="走行を MP4 動画にします（画面は出ません）")
    ap.add_argument("run", nargs="?", default="lc_brake", help="制御の名前（sim_out/<名前>）か、--out で指定したフォルダ")
    ap.add_argument("--cam", choices=("follow", "top"), default="follow", help="follow = 車の後ろから追う ／ top = コース全体")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", default="1280x720", help="幅x高さ")
    ap.add_argument("--speed", type=float, default=1.0, help="再生の速さ（0.5 = スロー）")
    ap.add_argument("-o", "--output", default=None, help="出力ファイル（既定：<フォルダ>/run.mp4）")
    a = ap.parse_args(argv)
    try:
        w, h = (int(x) for x in a.size.lower().split("x"))
        video(resolve_out(a.run), a.output, a.cam, a.fps, (w, h), a.speed)
    except ReplayError as e:
        print("エラー:", e); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main_view())
