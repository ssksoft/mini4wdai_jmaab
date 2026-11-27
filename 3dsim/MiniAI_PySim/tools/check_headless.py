# -*- coding: utf-8 -*-
"""Environment check for the headless (MuJoCo) simulator:   headless check

Prints OK / NG for every item and writes the same report to 3dsim/MiniAI_PySim/sim_out/check_headless_report.txt
(attach it when you report a problem).  Exit code 0 = all OK.
"""
import os
import platform
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PYSIM, "sil"))

# reference: lc_brake, standard motor, Ni-MH, 135 g, seed 0, 1 lap (includes the 2 s brake-hold and the standing start)
REF_LAP1 = (2.05, 2.40)


def main(argv=None):
    lines, ng = [], [0]

    def item(name, ok, detail=""):
        if not ok:
            ng[0] += 1
        lines.append("[%s] %s%s" % ("OK" if ok else "NG", name, ("  - " + detail) if detail else ""))
        print(lines[-1])

    def info(text):
        lines.append("     " + text)
        print(lines[-1])

    print("== ヘッドレス版 環境チェック")
    v = sys.version_info
    info("Python %s (%s) / %s" % (platform.python_version(), sys.executable, platform.platform()))
    item("Python 3.11 以上", v[:2] >= (3, 11), "" if v[:2] >= (3, 11) else "Python 3.12 を入れてください（MuJoCo 4.0.1 は 3.11〜3.13 用）")
    if v[:2] > (3, 13):
        info("注意: Python %d.%d は未確認です。MuJoCo が入らない場合は Python 3.12 を使ってください" % v[:2])

    mods = {}
    for name in ("mujoco", "numpy", "matplotlib"):
        try:
            m = __import__(name)
            mods[name] = m
            ver = getattr(m, "__version__", "?")
            item("パッケージ %s" % name, True, ver)
            if name == "mujoco" and ver != "4.0.1":
                info("注意: 確認済みの MuJoCo は 4.0.1 です（%s では、ぎりぎりの走行の結果が変わることがあります）" % ver)
        except ImportError:
            item("パッケージ %s" % name, False, "setup_headless.bat を実行（または pip install -r requirements-headless.txt）")

    try:
        import imageio, imageio_ffmpeg  # noqa: F401
        item("パッケージ imageio / imageio-ffmpeg（3D 動画用）", True, getattr(imageio, "__version__", "?"))
    except ImportError:
        item("パッケージ imageio / imageio-ffmpeg（3D 動画用）", False,
             "headless video を使うときに必要。setup_headless.bat をもう一度実行（走行・ビューアには不要）")

    import run_headless as RH
    course = RH.COURSE_DIR
    for rel in ("mjsim/m4wd_mj.py", "generator/layout_preview.py", "generator/fbxread.py",
                "ue/m4wd_ctrl.py", "fbx/parts_manifest.json"):
        p = os.path.join(course, rel)
        item("ファイル mini4wd_course/%s" % rel, os.path.isfile(p), "" if os.path.isfile(p) else "見つかりません: " + p)
    stubs = RH.lfs_stubs(course)
    item("コース部品の FBX（Git LFS）が取得済み", not stubs,
         "" if not stubs else "%d 個が引換券のまま（例: %s）。git lfs pull を実行" % (len(stubs), stubs[0]))

    import sil_core
    for name in ("lc_brake", "template"):
        try:
            sil_core.load_controller(os.path.join(PYSIM, "controllers", name + ".py"))
            item("制御ファイル %s.py を読み込める" % name, True)
        except Exception as e:
            item("制御ファイル %s.py を読み込める" % name, False, repr(e))

    if ng[0] == 0:
        try:
            t0 = time.time()
            MJ = RH.load_backend(course)
            m, _ = MJ.build_model({})
            item("コースのモデルを作れる", True, "当たり判定 %d 個, %.1f s" % (len(MJ.course()["hulls"]), time.time() - t0))
            res = RH.main(["lc_brake", "--laps", "1", "--quiet", "--out", os.path.join(PYSIM, "sim_out", "_check")])
            lap = res["laps"][0] if res["laps"] else None
            ok = res["outcome"] == "ok" and lap is not None and REF_LAP1[0] <= lap <= REF_LAP1[1]
            item("試走（lc_brake, 標準モーター, 1周）", ok,
                 "結果 %s, 1周 %s s（目安 %.2f〜%.2f s）, 計算 %.1f s" % (
                     res["outcome"], "%.3f" % lap if lap else "-", REF_LAP1[0], REF_LAP1[1], res["wall_s"]))
        except Exception as e:
            item("試走", False, "%s: %s" % (type(e).__name__, e))
            info(traceback.format_exc().replace("\n", "\n     "))
    else:
        info("NG があるため、試走はしていません")

    lines.append("")
    lines.append("結果: %s（NG %d 件）" % ("すべて OK" if ng[0] == 0 else "要確認", ng[0]))
    print(lines[-1])
    out = os.path.join(PYSIM, "sim_out")
    os.makedirs(out, exist_ok=True)
    rep = os.path.join(out, "check_headless_report.txt")
    with open(rep, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("レポート: " + rep)
    return 0 if ng[0] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
