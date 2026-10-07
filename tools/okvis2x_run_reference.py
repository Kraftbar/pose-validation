#!/usr/bin/env python3
"""Run the deterministic OKVIS2-X reference (tools/build_okvis2x_reference.py) on a EuRoC ASL folder.

    python3 tools/okvis2x_run_reference.py MH_01_easy --tag nogps_mono1 [--config <yaml>] [--data-root runs/okvis2x_port/data]
Outputs in runs/okvis2x_port/reference_runs/<seq>/<tag>/ : log.txt, final.csv (okvis2-*-final_trajectory.csv),
causal.csv (okvis2-*_trajectory.csv), global_final.csv (GNSS runs: okvis2-*-global-final_trajectory.csv),
sums.txt (sha256 of each). Never commit (EuRoC-derived).
"""
import argparse, hashlib, os, shutil, subprocess, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
V = REPO / "external/vio"
BUILD = REPO / "runs/okvis2x_port/reference_build/build"
CFG = REPO / "okvis2x_port/reference/configs/okvis2x_mono_euroc_deterministic.yaml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--config", default=str(CFG))
    ap.add_argument("--data-root", default=str(REPO / "runs/okvis2x_port/data"))
    ap.add_argument("--app", default="okvis_app_synchronous")
    ap.add_argument("--consolidated", action="store_true", help="enable every okvis_port observe-only dump (patches 0005-0015) with the "
                    "sampling of tags m8 (tools/run_okvis_reference.py CONSOLIDATED) into <tag>/dumps, plus ransac.bin / place.bin")
    a = ap.parse_args()
    root, ocv = V / "deps/root/usr", V / "deps/opencv"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join(map(str, [root / "lib/x86_64-linux-gnu", root / "lib/x86_64-linux-gnu/openblas-pthread", ocv / "lib"]))
    env["OMP_NUM_THREADS"] = "1"
    data = Path(a.data_root) / a.seq / "mav0"
    out = REPO / "runs/okvis2x_port/reference_runs" / a.seq / a.tag
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    if a.consolidated:
        d = str(out / "dumps")
        (out / "dumps").mkdir()
        for k in ("DUMP", "KIN_DUMP", "ERR_DUMP", "SOLVE_DUMP", "GRAPH_DUMP"):
            env[f"OKVIS_PORT_{k}_DIR"] = d
        env["OKVIS_PORT_RANSAC_DIR"] = d
        env["OKVIS_PORT_PLACE_DIR"] = d
        env.update({
            "OKVIS_PORT_DUMP_EVERY": "prop=4,preint=4,append=20,eval=1000", "OKVIS_PORT_KIN_EVERY": "all=3000",
            "OKVIS_PORT_ERR_EVERY": "all=1000,reproj=3000,llt=20,pplus=20,pplusj=40,pminusj=2000,hplus=300,hplusj=400,pose=10,sab=10,relpose=1,ctor=1",
            "OKVIS_PORT_SOLVE_EVERY": "100", "OKVIS_PORT_SOLVE_FULL_EVERY": "3", "OKVIS_PORT_SOLVE_SPARSE_FULL_EVERY": "4",
            "OKVIS_PORT_GRAPH_TPEVAL_EVERY": "200", "OKVIS_PORT_GRAPH_LM_EVERY": "8", "OKVIS_PORT_GRAPH_LM_SUB": "16",
            "OKVIS_PORT_GRAPH_PROBLEM_FULL_EVERY": "200"})
    t0 = time.time()
    with open(out / "log.txt", "w") as log:
        rc = subprocess.run([str(BUILD / a.app), a.config, str(data), str(out)], stdout=log, stderr=subprocess.STDOUT, env=env).returncode
    wall = time.time() - t0
    sums = []
    for f in sorted(out.glob("okvis2-*")):
        n = f.name
        new = ("global_final.csv" if "global-final_trajectory" in n else "final.csv" if n.endswith("-final_trajectory.csv")
               else "causal.csv" if n.endswith("_trajectory.csv") and "global" not in n and "final" not in n
               else n)
        f.rename(out / new)
    for f in sorted(out.iterdir()):
        if f.suffix == ".csv":
            sums.append(f"{hashlib.sha256(f.read_bytes()).hexdigest()}  {f.name}")
        elif f.name.endswith(".g2o") or "map" in f.name:
            f.unlink()
    (out / "sums.txt").write_text("\n".join(sums) + f"\nexit_code {rc} wall_s {wall:.1f}\n")
    print("\n".join(sums), f"\nexit {rc} wall {wall:.1f}s -> {out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
