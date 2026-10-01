#!/usr/bin/env python3
"""Build + run the stella_port module-4b reference tool
(stella_port/reference_tools/dump_stella_g2o_pose.cc) on fr1_xyz/fr1_desk,
writing every real pose_optimizer_g2o::optimize() call's inputs/outputs to
runs/stella_port/reference_g2o/<seq>/{calls,obs,outliers}.tsv

Requires runs/stella_port/reference_build/install
(tools/build_stella_reference.py, with
stella_port/reference/patches/0006-pose-optimizer-g2o-trace.patch applied
-- it is picked up automatically since build_stella_reference.py applies
every patches/*.patch file) to already exist. Reuses the same CMake
project + build dir as dump_stella_init.py (frame_bow_tool_build).

With --ba the module-4b part-2 tool (dump_stella_g2o_ba, needs
0007-ba-trace.patch) is built/run instead and writes
runs/stella_port/reference_g2o_ba/<seq>/ba_calls.bin (every local BA call,
the initial-map global BA, and two real global_bundle_adjuster::optimize()
calls on the final map; format documented in
stella_port/reference_tools/dump_stella_g2o_ba.cc).

Usage:
    python3 tools/dump_stella_g2o.py fr1_xyz fr1_desk [--max-frames N] [--ba]
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEPS = ROOT / "external/candidates/deps/root/usr"
BUILD_ROOT = ROOT / "runs/stella_port/reference_build"
INSTALL_DIR = BUILD_ROOT / "install"
TOOL_SRC = ROOT / "stella_port/reference_tools"
TOOL_BUILD = BUILD_ROOT / "frame_bow_tool_build"
BIN = TOOL_BUILD / "dump_stella_g2o_pose"
BIN_BA = TOOL_BUILD / "dump_stella_g2o_ba"
BIN_REPLAY = TOOL_BUILD / "replay_ba_eigen"
VOCAB = ROOT / "external/candidates/orb_vocab.fbow"
CFG = ROOT / "stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml"
DATA_ROOT = ROOT / "runs/orb_port/paper_bench/data"
OUT_ROOT = ROOT / "runs/stella_port/reference_g2o"
OUT_ROOT_BA = ROOT / "runs/stella_port/reference_g2o_ba"

OPENCV_PKGCONFIG = "/tmp/pose-opencv/pkgconfig"

SEQ_DATA_DIRS = {
    "fr1_xyz": "rgbd_dataset_freiburg1_xyz",
    "fr1_desk": "rgbd_dataset_freiburg1_desk",
}


def build_tool(target="dump_stella_g2o_pose"):
    env = os.environ.copy()
    env["PKG_CONFIG_PATH"] = OPENCV_PKGCONFIG + ":" + env.get("PKG_CONFIG_PATH", "")
    TOOL_BUILD.mkdir(parents=True, exist_ok=True)
    cxx_flags = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
    libarch = DEPS / "lib/x86_64-linux-gnu"
    localopencv_lib = "/tmp/localopencv/root/usr/lib/x86_64-linux-gnu"
    linker_flags = (f"-L{libarch} -L{localopencv_lib} -Wl,--allow-shlib-undefined "
                     f"-Wl,-rpath,{libarch} -Wl,-rpath,{INSTALL_DIR}/lib -Wl,-rpath,{localopencv_lib}")
    subprocess.run([
        "cmake", "-S", str(TOOL_SRC), "-B", str(TOOL_BUILD),
        f"-DSTELLA_REFERENCE_INSTALL_DIR={INSTALL_DIR}",
        f"-DCMAKE_PREFIX_PATH={INSTALL_DIR};{DEPS};{DEPS}/lib/cmake",
        f"-DCMAKE_CXX_FLAGS={cxx_flags}",
        f"-DCMAKE_EXE_LINKER_FLAGS={linker_flags}",
    ], check=True, env=env)
    subprocess.run(["cmake", "--build", str(TOOL_BUILD), "-j", str(os.cpu_count()), "--target", target], check=True, env=env)


def run_one(seq: str, max_frames: int, out_dir: Path, binary: Path = None):
    binary = binary or BIN
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq]
    if not data_dir.exists():
        print(f"skip {seq}: {data_dir} not found", file=sys.stderr)
        return
    out_dir.mkdir(parents=True, exist_ok=True)

    libarch = DEPS / "lib/x86_64-linux-gnu"
    env_path = (f"{INSTALL_DIR}/lib:{DEPS}/lib:{libarch}:"
                f"/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = env_path
    env["OMP_NUM_THREADS"] = "1"

    cmd = [str(binary), str(VOCAB), str(CFG), str(data_dir), str(out_dir), str(max_frames)]
    print("+", " ".join(cmd))
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    (out_dir / "stderr.log").write_text(proc.stdout + proc.stderr)
    if proc.returncode != 0:
        print(proc.stdout[-4000:], file=sys.stderr)
        print(proc.stderr[-4000:], file=sys.stderr)
        raise SystemExit(f"{seq}: dump_stella_g2o_pose failed (exit {proc.returncode})")
    print(f"{seq}: -> {out_dir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+", choices=list(SEQ_DATA_DIRS.keys()))
    ap.add_argument("--max-frames", type=int, default=-1)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--ba", action="store_true", help="run the BA call dumper (dump_stella_g2o_ba) instead")
    ap.add_argument("--ba-replay", action="store_true",
                    help="re-solve the captured global BA graphs of runs/stella_port/reference_g2o_ba/<seq> with the "
                         "real g2o LinearSolverEigen (replay_ba_eigen) -> ba_eigen_replay.bin")
    ap.add_argument("--check-determinism", action="store_true",
                     help="run each sequence twice into <out>/run1,run2 and diff")
    args = ap.parse_args()

    if not INSTALL_DIR.exists():
        print(f"error: {INSTALL_DIR} not found -- run tools/build_stella_reference.py first", file=sys.stderr)
        return 1

    if args.ba_replay:
        if not args.skip_build:
            build_tool("replay_ba_eigen")
        env = os.environ.copy()
        libarch = DEPS / "lib/x86_64-linux-gnu"
        env["LD_LIBRARY_PATH"] = (f"{INSTALL_DIR}/lib:{DEPS}/lib:{libarch}:/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                                  f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
        for seq in args.seqs:
            subprocess.run([str(BIN_REPLAY), str(OUT_ROOT_BA / seq)], check=True, env=env)
        return 0

    if not args.skip_build:
        build_tool("dump_stella_g2o_ba" if args.ba else "dump_stella_g2o_pose")

    out_root = OUT_ROOT_BA if args.ba else OUT_ROOT
    binary = BIN_BA if args.ba else BIN
    data_files = ["ba_calls.bin"] if args.ba else ["calls.tsv", "obs.tsv", "outliers.tsv"]
    out_root.mkdir(parents=True, exist_ok=True)
    for seq in args.seqs:
        if args.check_determinism:
            d1 = out_root / seq / "run1"
            d2 = out_root / seq / "run2"
            run_one(seq, args.max_frames, d1, binary)
            run_one(seq, args.max_frames, d2, binary)
            # stderr.log carries wall-clock timestamps in spdlog lines, not
            # data; compare only the actual dump files.
            mismatched = [f for f in data_files if subprocess.run(
                ["diff", "-q", str(d1 / f), str(d2 / f)]).returncode != 0]
            if mismatched:
                raise SystemExit(f"{seq}: non-deterministic in {mismatched}")
            print(f"{seq}: deterministic (run1 == run2 on {data_files})")
        else:
            run_one(seq, args.max_frames, out_root / seq, binary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
