#!/usr/bin/env python3
"""Run the deterministic OKVIS2 reference (tools/build_okvis_reference.py) on one EuRoC ASL sequence.

    python3 tools/run_okvis_reference.py MH_01_easy --tag run1 [--dump] [--dump-every eval=50]
           [--solve-dump --solve-every 20 --solve-full-every 4]   (M4 solver dump, patch 0008)
           [--graph-dump [--graph-lm-every 2 --graph-lm-sub 8 --graph-tpeval-every 50]]   (M5 graph dump, patch 0009)

Dataset: external/vio/data/<seq>/mav0 (tools/vio_harness/fetch_seq_stream.py). Outputs go to
runs/okvis_port/reference_runs/<seq>/<tag>/ : final.csv (final trajectory), causal.csv (causal
trajectory written by the publishing thread), log.txt, sha256 of each in sums.txt, and with --dump the
okvis_port instrumentation dumps (imu_*.bin) in <tag>/dumps/. Never commit these (EuRoC, non-commercial).
"""
import argparse, hashlib, os, shutil, subprocess, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
V = REPO / "external/vio"
BUILD = REPO / "runs/okvis_port/reference_build/build"
CONFIG = REPO / "okvis_port/reference/configs/okvis_mono_euroc_deterministic.yaml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("--tag", default="run1")
    ap.add_argument("--dump", action="store_true")
    ap.add_argument("--dump-every", default=None)
    ap.add_argument("--kin-every", default=None, help='M2 dump sampling (patch 0005), e.g. "all=0,projh=50,mul_t=500" '
                    '(all=N sets the default for every kind first; 0 = count only; default every call)')
    ap.add_argument("--err-every", default=None, help='M3 dump sampling (patch 0007), e.g. "all=0,reproj=1000,pose=1" '
                    '(all=N sets the default for every kind first; 0 = count only; default every call)')
    ap.add_argument("--solve-dump", action="store_true", help="OKVIS_PORT_SOLVE_DUMP_DIR (patch 0008): per-Solve() "
                    "snapshot/iteration dump solve.bin into <tag>/dumps (independent of --dump)")
    ap.add_argument("--solve-every", default=None, help="OKVIS_PORT_SOLVE_EVERY: every Nth realtime solve gets a "
                    "snapshot (level 1; default 1); full-graph solves always do")
    ap.add_argument("--solve-full-every", default=None, help="OKVIS_PORT_SOLVE_FULL_EVERY: every Mth snapshotted "
                    "realtime solve also dumps the full vectors (level 2; default 0 = never)")
    ap.add_argument("--solve-sparse-full-every", default=None, help="OKVIS_PORT_SOLVE_SPARSE_FULL_EVERY: level 2 for "
                    "every Kth full-graph solve (default 10)")
    ap.add_argument("--graph-dump", action="store_true", help="OKVIS_PORT_GRAPH_DUMP_DIR (patch 0009, module M5): "
                    "graph.bin (TwoPose* compute/convert/Evaluate, updateLandmarks) and problem.bin (ceres::Problem "
                    "bookkeeping log) into <tag>/dumps (independent of --dump / --solve-dump)")
    ap.add_argument("--graph-tpeval-every", default=None, help="OKVIS_PORT_GRAPH_TPEVAL_EVERY (default 50)")
    ap.add_argument("--graph-lm-every", default=None, help="OKVIS_PORT_GRAPH_LM_EVERY: every Nth updateLandmarks call (default 2)")
    ap.add_argument("--graph-lm-sub", default=None, help="OKVIS_PORT_GRAPH_LM_SUB: every Kth landmark of a recorded call (default 8)")
    ap.add_argument("--graph-problem-full-every", default=None, help="OKVIS_PORT_GRAPH_PROBLEM_FULL_EVERY: full program order every Nth Solve (default 50)")
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--data-dir", default=None, help="dataset dir name under external/vio/data (default: seq); lets "
                    "several runs go in parallel on symlinked copies, outputs are written next to the dataset")
    ap.add_argument("--trace", action="store_true", help="OKVIS_PORT_TRACE_DIR: per-solve trace (patch 0003)")
    ap.add_argument("--no-aslr", action="store_true", help="run under `setarch -R` (determinism stress test)")
    ap.add_argument("--env", action="append", default=[], help="extra KEY=VAL (e.g. MALLOC_PERTURB_=165)")
    args = ap.parse_args()

    root = V / "deps/root/usr"
    ocv = V / "deps/opencv"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join(map(str, [root / "lib/x86_64-linux-gnu",
                                                root / "lib/x86_64-linux-gnu/openblas-pthread", ocv / "lib"]))
    env["OMP_NUM_THREADS"] = "1"
    for kv in args.env:
        k, v = kv.split("=", 1)
        env[k] = v
    data = V / "data" / (args.data_dir or args.seq) / "mav0"
    out = REPO / "runs/okvis_port/reference_runs" / args.seq / args.tag
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    if args.dump:
        (out / "dumps").mkdir()
        env["OKVIS_PORT_DUMP_DIR"] = str(out / "dumps")
        env["OKVIS_PORT_KIN_DUMP_DIR"] = str(out / "dumps")
        env["OKVIS_PORT_ERR_DUMP_DIR"] = str(out / "dumps")
        if args.err_every:
            env["OKVIS_PORT_ERR_EVERY"] = args.err_every
        if args.kin_every:
            env["OKVIS_PORT_KIN_EVERY"] = args.kin_every
        if args.dump_every:
            env["OKVIS_PORT_DUMP_EVERY"] = args.dump_every
    if args.solve_dump:
        (out / "dumps").mkdir(exist_ok=True)
        env["OKVIS_PORT_SOLVE_DUMP_DIR"] = str(out / "dumps")
        for k, v in [("OKVIS_PORT_SOLVE_EVERY", args.solve_every), ("OKVIS_PORT_SOLVE_FULL_EVERY", args.solve_full_every),
                     ("OKVIS_PORT_SOLVE_SPARSE_FULL_EVERY", args.solve_sparse_full_every)]:
            if v is not None:
                env[k] = v
    if args.graph_dump:
        (out / "dumps").mkdir(exist_ok=True)
        env["OKVIS_PORT_GRAPH_DUMP_DIR"] = str(out / "dumps")
        for k, v in [("OKVIS_PORT_GRAPH_TPEVAL_EVERY", args.graph_tpeval_every), ("OKVIS_PORT_GRAPH_LM_EVERY", args.graph_lm_every),
                     ("OKVIS_PORT_GRAPH_LM_SUB", args.graph_lm_sub), ("OKVIS_PORT_GRAPH_PROBLEM_FULL_EVERY", args.graph_problem_full_every)]:
            if v is not None:
                env[k] = v
    if args.trace:
        (out / "trace").mkdir()
        env["OKVIS_PORT_TRACE_DIR"] = str(out / "trace")
    for f in data.glob("okvis2-*"):
        f.unlink()
    t0 = time.time()
    with open(out / "log.txt", "w") as log:
        cmd = ([ "setarch", "-R"] if args.no_aslr else []) + [str(BUILD / "okvis_app_synchronous"), args.config, str(data)]
        rc = subprocess.run(cmd,
                            stdout=log, stderr=subprocess.STDOUT, env=env).returncode
    wall = time.time() - t0
    sums = []
    for name, src in [("final.csv", "okvis2-slam-final_trajectory.csv"), ("causal.csv", "okvis2-slam_trajectory.csv")]:
        p = data / src
        if p.exists():
            shutil.move(str(p), out / name)
            sums.append(f"{hashlib.sha256((out / name).read_bytes()).hexdigest()}  {name}")
    for f in data.glob("okvis2-*"):
        f.unlink()
    (out / "sums.txt").write_text("\n".join(sums) + f"\nexit_code {rc} wall_s {wall:.1f}\n")
    print("\n".join(sums), f"\nexit {rc} wall {wall:.1f}s -> {out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
