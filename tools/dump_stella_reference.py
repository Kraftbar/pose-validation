#!/usr/bin/env python3
"""Run the stella_port single-threaded reference driver on a TUM sequence.

Writes:
  runs/stella_port/reference_dumps/<seq>/{frames_before.tsv,frames_after.tsv}
    -- see stella_port/reference/README.md "Dumps"/"Dump timing"
  runs/tum_compare/stella_vslam_st/<seq>/{trajectory.tum,run.json}
    -- tools/tum_eval.py-compatible, so the single-threaded reference's
       ATE/coverage can be scored next to runs/tum_compare/stella_vslam/<seq>
       (the multi-threaded numbers from external/candidates/run_stella.sh)

Requires tools/build_stella_reference.py to have been run first (build under
runs/stella_port/reference_build/).

Usage:
    python3 tools/dump_stella_reference.py fr1_xyz [--max-frames N] [--light]
    python3 tools/dump_stella_reference.py fr1_xyz fr1_desk
    # opt-in fault-injection variant (patch 0010; canonical dumps never use it):
    python3 tools/dump_stella_reference.py fr1_xyz fr1_desk --force-path bow:5
      -> runs/stella_port/reference_dumps_force_bow/<seq>/ (canonical dumps and
         runs/tum_compare/ are NOT touched)
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEPS = ROOT / "external/candidates/deps/root/usr"
BUILD_ROOT = ROOT / "runs/stella_port/reference_build"
INSTALL_DIR = BUILD_ROOT / "install"
BIN = BUILD_ROOT / "driver_build/run_stella_reference"
VOCAB = ROOT / "external/candidates/orb_vocab.fbow"
CFG = ROOT / "stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml"
DATA_ROOT = ROOT / "runs/orb_port/paper_bench/data"
DUMP_ROOT = ROOT / "runs/stella_port/reference_dumps"
COMPARE_ROOT = ROOT / "runs/tum_compare/stella_vslam_st"

SEQ_DATA_DIRS = {
    "fr1_xyz": "rgbd_dataset_freiburg1_xyz",
    "fr1_desk": "rgbd_dataset_freiburg1_desk",
    "fr1_floor": "rgbd_dataset_freiburg1_floor",
    "fr2_xyz": "rgbd_dataset_freiburg2_xyz",
    "fr3_long_office": "rgbd_dataset_freiburg3_long_office_household",
}


def run_one(seq: str, max_frames: int, light: bool, force_path: str = ""):
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq]
    if not data_dir.exists():
        print(f"skip {seq}: {data_dir} not found", file=sys.stderr)
        return

    dump_root = DUMP_ROOT
    if force_path:
        dump_root = DUMP_ROOT.parent / ("reference_dumps_force_" + force_path.split(":")[0])
    dump_dir = dump_root / seq
    if dump_dir.exists():
        shutil.rmtree(dump_dir)
    dump_dir.mkdir(parents=True)

    compare_dir = COMPARE_ROOT / seq
    if not force_path:
        compare_dir.mkdir(parents=True, exist_ok=True)

    env_path = (f"{INSTALL_DIR}/lib:{DEPS}/lib:{DEPS}/lib/x86_64-linux-gnu:"
                f"/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
    import os
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = env_path
    env["OMP_NUM_THREADS"] = "1"
    if force_path:
        env["STELLA_PORT_FORCE_PATH"] = force_path

    cmd = [str(BIN), str(VOCAB), str(CFG), str(data_dir), str(dump_dir), str(max_frames)]
    if light:
        cmd.append("--light")
    print("+", " ".join(cmd))
    t0 = time.time()
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    wall = time.time() - t0
    (dump_dir / "stderr.log").write_text(proc.stdout + proc.stderr)

    n_lines = 0
    rgb_txt = data_dir / "rgb.txt"
    if rgb_txt.exists():
        n_lines = sum(1 for line in rgb_txt.read_text().splitlines() if line and not line.startswith("#"))

    if force_path:
        print(f"{seq}: exit={proc.returncode} wall={wall:.1f}s (force-path {force_path}) -> {dump_dir}")
        return
    traj_src = dump_dir / "trajectory.tum"
    if traj_src.exists():
        shutil.copy(traj_src, compare_dir / "trajectory.tum")
    run_json = {
        "wall_s": wall,
        "frames_in": n_lines,
        "exit_code": proc.returncode,
        "notes": "stella_port single-threaded deterministic reference (tools/dump_stella_reference.py); "
                 "run_stella_reference / synchronize_background_modules(), OMP_NUM_THREADS=1, "
                 "DETERMINISTIC=ON, use_fixed_seed config",
    }
    (compare_dir / "run.json").write_text(json.dumps(run_json, indent=2))
    print(f"{seq}: exit={proc.returncode} wall={wall:.1f}s -> {compare_dir}")
    if proc.returncode != 0:
        print(proc.stdout[-4000:], file=sys.stderr)
        print(proc.stderr[-4000:], file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+", choices=list(SEQ_DATA_DIRS.keys()))
    ap.add_argument("--max-frames", type=int, default=-1, help="-1 = full sequence")
    ap.add_argument("--light", action="store_true")
    ap.add_argument("--force-path", default="", help="opt-in fault injection, e.g. bow:5 or robust:7 (patch 0010)")
    args = ap.parse_args()

    if not BIN.exists():
        print(f"error: {BIN} not found -- run tools/build_stella_reference.py first", file=sys.stderr)
        return 1

    for seq in args.seqs:
        run_one(seq, args.max_frames, args.light, args.force_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
