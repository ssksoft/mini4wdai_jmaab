# -*- coding: utf-8 -*-
"""Entry point of the headless (MuJoCo) simulator.  headless.bat calls this with the .venv Python.

    headless check                         environment check (packages, course data, a short test run)
    headless lc_brake [options]            run a controller          (= sil/run_headless.py)
    headless sweep lc_brake --param ...    parameter sweep           (= tools/sweep_headless.py)
    headless lc_brake --view               run, then replay it in the 3D viewer window
    headless lc_brake --video [top]        run, then write sim_out/lc_brake/run.mp4
    headless view  [lc_brake] [--speed 0.5] [--loop]       replay the last run in the 3D viewer (no re-run)
    headless video [lc_brake] [--cam top] [--speed 0.5]    make run.mp4 from the last run (no re-run)
    headless --help                        this text
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PYSIM, "sil"))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        print("実行オプション（headless <制御> --help でも表示）:")
        import run_headless
        try:
            run_headless.main(["--help"])
        except SystemExit:
            pass
        return 0
    cmd = argv[0]
    if cmd == "check":
        import check_headless
        return check_headless.main(argv[1:])
    if cmd == "sweep":
        import sweep_headless
        return sweep_headless.main(argv[1:])
    if cmd in ("view", "video"):
        import replay_headless
        return (replay_headless.main_view if cmd == "view" else replay_headless.main_video)(argv[1:])
    import run_headless
    try:
        res = run_headless.main(argv)
    except run_headless.SetupError as e:
        print("ERROR: %s" % e)
        return 1
    return 0 if res.get("outcome") is not None else 1


if __name__ == "__main__":
    sys.exit(main())
