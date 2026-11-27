# -*- coding: utf-8 -*-
"""Parameter sweep with the headless simulator:

    headless sweep lc_brake --param BRAKE_S=0.08,0.12,0.16 --param LEAD_DEG=20,30,40 --motor machdash_pro --laps 5
    headless sweep lc_brake --param RUN_DUTY=0.6,0.7,0.8 --seeds 0,1,2

Every combination of the --param values (x every seed) is run once.  Options other than --param / --seeds / --out
(--motor, --battery, --mass, --laps, --timeout, ...) are passed to every run unchanged.
Writes  sim_out/sweep_<controller>_<date-time>/results.csv  (one line per run) and prints a table:
for each combination, how many seeds finished and the mean total time.
"""
import argparse
import csv
import itertools
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PYSIM = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PYSIM, "sil"))
import run_headless as RH  # noqa: E402


def parse_values(text):
    out = []
    for v in text.split(","):
        v = v.strip()
        try:
            out.append(json.loads(v))
        except ValueError:
            out.append(v)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("controller")
    ap.add_argument("--param", action="append", default=[], metavar="KEY=V1,V2,...", help="values to try (repeatable)")
    ap.add_argument("--seeds", default="0", help="IMU noise seeds, e.g. 0,1,2 (default 0)")
    ap.add_argument("--out", default=None)
    a, rest = ap.parse_known_args(argv)
    if not a.param:
        ap.error("--param KEY=V1,V2,... を1つ以上指定してください")
    keys, grids = [], []
    for p in a.param:
        k, v = p.split("=", 1)
        keys.append(k)
        grids.append(parse_values(v))
    seeds = [int(s) for s in a.seeds.split(",")]
    name = os.path.splitext(os.path.basename(a.controller))[0]
    out = a.out or os.path.join(PYSIM, "sim_out", "sweep_%s_%s" % (name, time.strftime("%Y%m%d_%H%M%S")))
    os.makedirs(out, exist_ok=True)
    combos = list(itertools.product(*grids))
    n = len(combos) * len(seeds)
    print("== パラメーター探索: %s  %d 通り x seed %d 個 = %d 回" % (name, len(combos), len(seeds), n))
    rows, i, t0 = [], 0, time.time()
    for combo in combos:
        for seed in seeds:
            i += 1
            args = [a.controller, "--quiet", "--seed", str(seed), "--out", os.path.join(out, "run_%03d" % i)] + rest
            for k, v in zip(keys, combo):
                args += ["--param", "%s=%s" % (k, json.dumps(v))]
            try:
                r = RH.main(args)
            except RH.SetupError as e:
                print("ERROR: %s" % e)
                return 1
            row = dict(zip(keys, combo), seed=seed, outcome=r["outcome"], total=r["total"],
                       laps=" ".join("%.3f" % x for x in r["laps"]), vpeak=r["vpeak_mps"], run="run_%03d" % i)
            rows.append(row)
            print("  [%3d/%d] %s seed=%d -> %s %s" % (i, n, " ".join("%s=%s" % kv for kv in zip(keys, combo)), seed,
                                                    RH.OUTCOME_JA.get(r["outcome"], r["outcome"]),
                                                    "%.3f s" % r["total"] if r["total"] else ""))
    with open(os.path.join(out, "results.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=keys + ["seed", "outcome", "total", "laps", "vpeak", "run"])
        w.writeheader()
        w.writerows(rows)
    print("")
    print("== まとめ（完走数 / seed 数、完走したときの平均合計タイム）")
    print("  " + "  ".join("%-10s" % k for k in keys) + "  完走     平均[s]")
    best = None
    for combo in combos:
        rs = [r for r in rows if tuple(r[k] for k in keys) == combo]
        ok = [r["total"] for r in rs if r["outcome"] == "ok"]
        mean = sum(ok) / len(ok) if ok else None
        print("  " + "  ".join("%-10s" % v for v in combo) + "  %d/%d      %s" % (len(ok), len(rs), "%.3f" % mean if mean else "-"))
        if len(ok) == len(rs) and mean is not None and (best is None or mean < best[1]):
            best = (combo, mean)
    if best:
        print("全 seed で完走した中で最速: " + ", ".join("%s=%s" % kv for kv in zip(keys, best[0])) + "  平均 %.3f s" % best[1])
    else:
        print("全 seed で完走した組み合わせはありませんでした")
    print("結果: %s  （%d 回, %.0f 秒）" % (os.path.join(out, "results.csv"), n, time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
