#!/usr/bin/env python3
"""Run the stella_port reference driver in loop-dump mode (patch 0013, module 7).

Writes runs/stella_port/reference_loop[_eigen]/<seq>/ :
  loop_events.tsv  library trace of the loop detector / Sim3 validation / correct_loop
  loop_snap.tsv    map snapshot at every global-optimization step (+ post pose graph / post loop BA)
  loop_kfobs.tsv   keypoints + descriptors of every keyframe
  kf_destroyed.tsv keyframe destruction schedule (patch 0011)
  frames_*.tsv, trajectory.tum

--eigen-solver sets STELLA_PORT_EIGEN_SOLVER=1 (patch 0013 part 3): the pose graph optimization and the
global BA use g2o's LinearSolverEigen instead of the LGPL CSparse solver, which is what the C port
reimplements (bit-exact target). Output goes to reference_loop_eigen/. Without the flag the canonical
CSparse behaviour is used (reference_loop/), which the port matches to ~1e-14 only.

Usage:
    python3 tools/dump_stella_loop.py fr3_long_office fr1_desk fr1_xyz [--eigen-solver] [--out-root DIR]
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import dump_stella_reference as ref  # noqa: E402


def yaml_get(path, section, key, default):
    sec = None
    for line in Path(path).read_text().splitlines():
        if line and not line.startswith(" ") and line.rstrip().endswith(":"):
            sec = line.rstrip()[:-1]
        elif sec == section and line.strip().startswith(key + ":"):
            return line.split(":", 1)[1].strip()
    return default


def yaml_with_overrides(base_path, overrides):
    """Returns the text of base_path with `Section.key=value` overrides applied (new sections appended)."""
    text = Path(base_path).read_text()
    for ov in overrides:
        sec_key, val = ov.split("=", 1)
        sec, key = sec_key.split(".", 1)
        lines = text.splitlines()
        header = sec + ":"
        idx = next((i for i, l in enumerate(lines) if l.strip() == header and not l.startswith(" ")), None)
        if idx is None:
            lines += ["", header, f"  {key}: {val}"]
        else:
            lines.insert(idx + 1, f"  {key}: {val}")
        text = "\n".join(lines) + "\n"
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+", choices=list(ref.SEQ_DATA_DIRS.keys()))
    ap.add_argument("--eigen-solver", action="store_true")
    ap.add_argument("--out-root", default="")
    ap.add_argument("--max-frames", type=int, default=-1)
    ap.add_argument("--config", default="", help="reference YAML (default: the deterministic TUM config)")
    ap.add_argument("--snap-from", type=int, default=0, help="write K/L snapshot rows only for steps >= N (trace stays complete)")
    ap.add_argument("--env", action="append", default=[], help="extra KEY=VALUE for the driver (opt-in stress knobs)")
    ap.add_argument("--yaml-set", action="append", default=[],
                    help="Section.key=value override of the reference YAML (repeatable), e.g. LoopDetector.min_continuity=0")
    ap.add_argument("--suffix", default="", help="output dir suffix, e.g. _stress")
    args = ap.parse_args()
    if not ref.BIN.exists():
        print(f"error: {ref.BIN} not found -- run tools/build_stella_reference.py first", file=sys.stderr)
        return 1
    root = Path(args.out_root) if args.out_root else ROOT / ("runs/stella_port/reference_loop_eigen" if args.eigen_solver else "runs/stella_port/reference_loop")
    env_path = (f"{ref.INSTALL_DIR}/lib:{ref.DEPS}/lib:{ref.DEPS}/lib/x86_64-linux-gnu:"
                f"/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
    for seq in args.seqs:
        data_dir = ref.DATA_ROOT / ref.SEQ_DATA_DIRS[seq]
        if not data_dir.exists():
            print(f"skip {seq}: {data_dir} not found", file=sys.stderr)
            continue
        out = root / (seq + args.suffix)
        if out.exists():
            shutil.rmtree(out)
        out.mkdir(parents=True)
        config = args.config or str(ref.CFG)
        if args.yaml_set:
            config = str(out / "config.yaml")
            Path(config).write_text(yaml_with_overrides(args.config or ref.CFG, args.yaml_set))
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = env_path
        env["OMP_NUM_THREADS"] = "1"
        for kv in args.env:
            k, v = kv.split("=", 1)
            env[k] = v
        if args.eigen_solver:
            env["STELLA_PORT_EIGEN_SOLVER"] = "1"
        cmd = [str(ref.BIN), str(ref.VOCAB), config, str(data_dir), str(out), str(args.max_frames), "--loop-dump"] + (["--loop-snap-from", str(args.snap_from)] if args.snap_from else [])
        print("+", " ".join(cmd), flush=True)
        t0 = time.time()
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
        (out / "stderr.log").write_text(proc.stdout + proc.stderr)
        # marker read by stella_port/c/check_sv_loop.c (bit-exact vs tolerance mode) + the knobs of this run
        (out / "loop_params.txt").write_text(
            f"eigen_solver={1 if args.eigen_solver else 0}\nconfig={config}\n"
            + "".join(f"env {kv}\n" for kv in args.env)
            + "".join(f"yaml {kv}\n" for kv in args.yaml_set)
            + f"thr_neighbor_keyframes={yaml_get(config, 'GlobalOptimizer', 'thr_neighbor_keyframes', 15)}\n"
            + f"min_num_shared_lms_graph={yaml_get(config, 'GraphOptimizer', 'min_num_shared_lms', 100)}\n"
            + f"loop_ba_num_iter={yaml_get(config, 'GlobalOptimizer', 'num_iter', 10)}\n")
        print(f"{seq}: exit={proc.returncode} wall={time.time() - t0:.1f}s -> {out}")
        if proc.returncode != 0:
            print(proc.stdout[-3000:], file=sys.stderr)
            print(proc.stderr[-3000:], file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
