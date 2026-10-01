#!/usr/bin/env python3
"""Build + run the stella_port module-3 reference tool
(stella_port/reference_tools/dump_stella_init.cc) on fr1_xyz/fr1_desk,
writing per-attempt monocular-initializer data (matches, RNG draws, best
H/F + cost + inliers, hypotheses, final verdict) to
runs/stella_port/reference_init/<seq>/{attempts,matches,rng,hyps,final,inliers}.tsv

Requires runs/stella_port/reference_build/install (tools/build_stella_reference.py)
to already exist -- this tool links against that same install, like
stella_port/reference/driver and dump_frame_bow do. Reuses the SAME CMake
project as the module-2 tool (stella_port/reference_tools/CMakeLists.txt
now builds both dump_frame_bow and dump_stella_init), so the build dir
(frame_bow_tool_build) is shared.

Usage:
    python3 tools/dump_stella_init.py fr1_xyz fr1_desk [--max-frames N]
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
BIN = TOOL_BUILD / "dump_stella_init"
VOCAB = ROOT / "external/candidates/orb_vocab.fbow"
CFG = ROOT / "stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml"
DATA_ROOT = ROOT / "runs/orb_port/paper_bench/data"
OUT_ROOT = ROOT / "runs/stella_port/reference_init"

OPENCV_PKGCONFIG = "/tmp/pose-opencv/pkgconfig"

SEQ_DATA_DIRS = {
    "fr1_xyz": "rgbd_dataset_freiburg1_xyz",
    "fr1_desk": "rgbd_dataset_freiburg1_desk",
}


def build_tool():
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
    subprocess.run(["cmake", "--build", str(TOOL_BUILD), "-j", str(os.cpu_count()), "--target", "dump_stella_init"], check=True, env=env)


def run_one(seq: str, max_frames: int):
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq]
    if not data_dir.exists():
        print(f"skip {seq}: {data_dir} not found", file=sys.stderr)
        return
    out_dir = OUT_ROOT / seq
    out_dir.mkdir(parents=True, exist_ok=True)

    libarch = DEPS / "lib/x86_64-linux-gnu"
    env_path = (f"{INSTALL_DIR}/lib:{DEPS}/lib:{libarch}:"
                f"/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = env_path
    env["OMP_NUM_THREADS"] = "1"

    cmd = [str(BIN), str(VOCAB), str(CFG), str(data_dir), str(out_dir), str(max_frames)]
    print("+", " ".join(cmd))
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    (out_dir / "stderr.log").write_text(proc.stdout + proc.stderr)
    if proc.returncode != 0:
        print(proc.stdout[-4000:], file=sys.stderr)
        print(proc.stderr[-4000:], file=sys.stderr)
        raise SystemExit(f"{seq}: dump_stella_init failed (exit {proc.returncode})")
    print(f"{seq}: -> {out_dir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+", choices=list(SEQ_DATA_DIRS.keys()))
    ap.add_argument("--max-frames", type=int, default=-1)
    ap.add_argument("--skip-build", action="store_true")
    args = ap.parse_args()

    if not INSTALL_DIR.exists():
        print(f"error: {INSTALL_DIR} not found -- run tools/build_stella_reference.py first", file=sys.stderr)
        return 1

    if not args.skip_build:
        build_tool()

    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    for seq in args.seqs:
        run_one(seq, args.max_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
