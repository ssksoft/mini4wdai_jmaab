# -*- coding: utf-8 -*-
"""Environment check for the Python-control simulator.  Run inside Unreal Editor (level open, NOT playing):

    Output Log (Cmd):  py "<repo>/3dsim/MiniAI_PySim/tools/check_env.py"

Prints OK / NG for every item and writes the same report to
    MiniYonkuSim/Saved/PySim/check_env_report.txt
Send that file back when you report a problem.
"""
import os
import sys
import platform
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)                                    # .../3dsim/MiniAI_PySim
DSIM = os.path.dirname(PYSIM)                                     # .../3dsim
UE_DIR = os.path.join(DSIM, "mini4wd_course", "ue")

lines = []
n_ng = [0]


def item(name, ok, detail=""):
    tag = "OK" if ok else "NG"
    if not ok:
        n_ng[0] += 1
    lines.append("[%s] %s%s" % (tag, name, ("  - " + detail) if detail else ""))


def info(text):
    lines.append("     " + text)


# ---------------------------------------------------------------- 1. Unreal / Python
try:
    import unreal
    have_ue = True
except ImportError:
    have_ue = False
item("Unreal Editor の Python で実行している", have_ue, "" if have_ue else "UE の出力ログから py \"...check_env.py\" で実行してください")
info("Python %s / %s" % (platform.python_version(), platform.platform()))
if have_ue:
    try:
        info("Unreal Engine %s" % unreal.SystemLibrary.get_engine_version())
    except Exception:
        pass

# ---------------------------------------------------------------- 2. files
for rel in ("controllers/lc_brake.py", "controllers/template.py", "sil/sil_core.py"):
    p = os.path.join(PYSIM, rel)
    item("ファイル MiniAI_PySim/" + rel, os.path.exists(p), "" if os.path.exists(p) else "見つかりません: " + p)
for rel in ("m4wd_drive.py", "m4wd_ctrl.py", "car_setup.py", "setup_in_ue.py"):
    p = os.path.join(UE_DIR, rel)
    item("ファイル mini4wd_course/ue/" + rel, os.path.exists(p), "" if os.path.exists(p) else "見つかりません: " + p)

# ---------------------------------------------------------------- 2b. LFS files really downloaded (not pointer stubs)
def _lfs_stubs(folder, exts=(".uasset", ".umap", ".fbx")):
    stubs, total = [], 0
    for root, _dirs, files in os.walk(folder):
        for f in files:
            if not f.lower().endswith(exts):
                continue
            total += 1
            p = os.path.join(root, f)
            try:
                if os.path.getsize(p) < 1024:
                    with open(p, "rb") as fh:
                        if fh.read(40).startswith(b"version https://git-lfs"):
                            stubs.append(os.path.relpath(p, DSIM))
            except OSError:
                pass
    return stubs, total


for rel in ("MiniYonkuSim/Content", "mini4wd_course/fbx"):
    folder = os.path.join(DSIM, rel)
    if not os.path.isdir(folder):
        continue
    stubs, total = _lfs_stubs(folder)
    item("LFS の本体がある（%s, %d ファイル）" % (rel, total), not stubs,
         "" if not stubs else "%d 個が中身のない引換券のままです（例: %s）。ZIP で入手した場合は git clone し直し、clone した場合は git lfs pull を実行"
         % (len(stubs), stubs[0]))

# ---------------------------------------------------------------- 3. model self-test (no UE needed)
try:
    sys.path.insert(0, os.path.join(PYSIM, "sil"))
    import importlib
    import sil_core
    importlib.reload(sil_core)

    class _Full:
        CONTROL_HZ = 500

        def setup(self, io):
            io.duty = 0.0

        def step(self, io):
            io.duty = 1.0

    REF = {"standard": 3.385, "machdash_pro": 5.738}             # speed after 1 s, flat, Ni-MH, duty 1 [m/s]
    for motor, ref in REF.items():
        car = sil_core.PyCar(_Full(), "nimh2", motor, noise=False)
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        v, dt = 0.0, 1.0 / 240.0
        for _ in range(240):
            v += car.step(dt, R, [v, 0.0, 0.0], v, True) * dt
        item("モデル自己テスト（%s, 1 秒後の速度）" % motor, abs(v - ref) < 0.02, "%.3f m/s（基準 %.3f）" % (v, ref))
    for name in ("lc_brake", "template"):
        try:
            sil_core.load_controller(os.path.join(PYSIM, "controllers", name + ".py"))
            item("制御ファイル %s.py を読み込める" % name, True)
        except Exception as e:
            item("制御ファイル %s.py を読み込める" % name, False, repr(e))
except Exception as e:
    item("sil_core の読み込み", False, repr(e))
    info(traceback.format_exc().replace("\n", "\n     "))

# ---------------------------------------------------------------- 4. runtime registered (init_unreal.py)
if have_ue:
    drv = sys.modules.get("m4wd_drive")
    item("m4wd_drive が読み込まれている（init_unreal.py が動いた）", drv is not None,
         "" if drv else "Content/Python/init_unreal.py が無いか、プロジェクトの隣に mini4wd_course が無い")
    if drv is not None:
        src = getattr(drv, "__file__", "")
        same = os.path.normcase(os.path.abspath(src)) == os.path.normcase(os.path.join(UE_DIR, "m4wd_drive.py"))
        item("読み込まれた m4wd_drive.py がこのパッケージのもの", same,
             src if same else src + "  ← 別の場所のプロジェクトを開いています。<リポジトリ>/3dsim/MiniYonkuSim/MiniYonkuSim.uproject を開き直す")
        item("m4wd_drive に start_py がある（新しい版）", hasattr(drv, "start_py"),
             "" if hasattr(drv, "start_py") else "古い版です。UE を閉じて git pull で最新にし、エディタを再起動")
        root = getattr(drv, "PY_ROOT", None)
        item("m4wd_drive から MiniAI_PySim が見える", bool(root) and os.path.isdir(root), str(root))
        import builtins
        item("ランタイムが登録済み（Play で動く状態）", getattr(builtins, "_m4wd_tick_handle", None) is not None,
             "" if getattr(builtins, "_m4wd_tick_handle", None) is not None else
             "py import m4wd_drive; m4wd_drive.register() を実行")
    try:
        proj = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
        info("プロジェクト: " + proj)
        rel_ok = os.path.normcase(os.path.normpath(os.path.join(proj, ".."))) == os.path.normcase(os.path.normpath(DSIM))
        item("MiniYonkuSim と MiniAI_PySim が同じ 3dsim フォルダにある", rel_ok,
             "3dsim = " + DSIM + ("" if rel_ok else "  ← 開いているのは別の場所の MiniYonkuSim です（Epic ランチャーの一覧から開いた場合など）"))
        ip = os.path.join(proj, "Content", "Python", "init_unreal.py")
        item("Content/Python/init_unreal.py がある", os.path.exists(ip), ip)
    except Exception as e:
        item("プロジェクトの場所", False, repr(e))

# ---------------------------------------------------------------- 5. level contents
if have_ue:
    try:
        world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        info("開いているレベル: %s" % world.get_name())

        def tagged(tag):
            return list(unreal.GameplayStatics.get_all_actors_with_tag(world, tag))

        for tag, what in (("M4WD_CAR", "車"), ("M4WD_CAM", "カメラ"), ("M4WD_FINISH", "ゴールライン（ラップ計測）"),
                          ("M4WD_START_0", "スタート位置（外側）")):
            n = len(tagged(tag))
            item("レベルに %s（タグ %s）がある" % (what, tag), n >= 1,
                 "" if n else "mini4wd_course/ue/car_setup.py を実行して車・カメラ・スタート位置を置く")
        if not tagged("M4WD_START_1"):
            info("参考: 内側スタート位置（M4WD_START_1）はこのレベルにありません。lane=1 は使えません（lane=0 は問題なし）")
        n = len(tagged("M4WD"))
        item("レベルにコース部品がある（タグ M4WD）", n >= 10,
             "%d 個%s" % (n, "" if n >= 10 else "。mini4wd_course/ue/setup_in_ue.py でレベルを作る"))
        lc = 0
        for a in tagged("M4WD"):
            try:
                if a.static_mesh_component.static_mesh.get_name().startswith("SM_LaneChange"):
                    lc += 1
            except Exception:
                pass
        item("立体レーンチェンジがある", lc >= 1, "%d 個" % lc)
        cars = tagged("M4WD_CAR")
        if cars:
            comp = cars[0].static_mesh_component
            item("車が物理シミュレーション有効", bool(comp.is_simulating_physics()), "")
            info("車の質量（上書き前）: %.4f kg" % comp.get_mass())
    except Exception as e:
        item("レベルの確認", False, repr(e))

# ---------------------------------------------------------------- report
lines.append("")
lines.append("結果: %s（NG %d 件）" % ("すべて OK" if n_ng[0] == 0 else "要確認", n_ng[0]))
report = "\n".join(lines)
out_path = None
if have_ue:
    try:
        d = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()), "PySim")
        os.makedirs(d, exist_ok=True)
        out_path = os.path.join(d, "check_env_report.txt")
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(report + "\n")
    except Exception:
        out_path = None
for ln in lines:
    (unreal.log if have_ue else print)("[M4WD check] " + ln)
if out_path:
    unreal.log("[M4WD check] report -> " + out_path)
