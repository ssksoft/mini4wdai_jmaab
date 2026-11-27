# -*- coding: utf-8 -*-
"""Set up the headless (MuJoCo) simulator: create 3dsim/MiniAI_PySim/.venv, install requirements-headless.txt,
then run the environment check.   setup_headless.bat calls this with the system Python (py -3 or python).

    python setup_headless.py            create / update the .venv and check
    python setup_headless.py --recreate delete the .venv first and build it again
"""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)
VENV = os.path.join(PYSIM, ".venv")
REQ = os.path.join(PYSIM, "requirements-headless.txt")


def venv_python():
    if os.name == "nt":
        return os.path.join(VENV, "Scripts", "python.exe")
    return os.path.join(VENV, "bin", "python")


def run(cmd):
    print("> " + " ".join('"%s"' % c if " " in c else c for c in cmd))
    return subprocess.call(cmd)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recreate", action="store_true")
    a = ap.parse_args(argv)
    v = sys.version_info
    print("== ヘッドレス版のセットアップ")
    print("Python %d.%d.%d: %s" % (v[0], v[1], v[2], sys.executable))
    if v[:2] < (3, 11):
        print("ERROR: Python 3.11 以上が必要です。https://www.python.org/ から Python 3.12 を入れてください")
        return 1
    if a.recreate and os.path.isdir(VENV):
        print("既存の .venv を削除します: " + VENV)
        shutil.rmtree(VENV)
    if not os.path.isfile(venv_python()):
        print("仮想環境を作ります: " + VENV)
        if run([sys.executable, "-m", "venv", VENV]) != 0:
            print("ERROR: 仮想環境を作れませんでした")
            return 1
    py = venv_python()
    if run([py, "-m", "pip", "install", "--upgrade", "pip", "-q"]) != 0:
        print("ERROR: pip を更新できませんでした（ネットワーク・プロキシを確認）")
        return 1
    if run([py, "-m", "pip", "install", "-r", REQ]) != 0:
        print("ERROR: パッケージを入れられませんでした（ネットワーク・プロキシ、Python の版を確認）")
        return 1
    print("")
    rc = run([py, os.path.join(HERE, "check_headless.py")])
    print("")
    if rc == 0:
        print("セットアップ完了。次のように走らせます（3dsim\\MiniAI_PySim で）:")
        print("  headless lc_brake --plot")
    else:
        print("環境チェックで NG がありました。表示された内容を確認してください（docs/headless_manual.md 10章）")
    return rc


if __name__ == "__main__":
    sys.exit(main())
