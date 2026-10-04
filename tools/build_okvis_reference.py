#!/usr/bin/env python3
"""Build the okvis_port single-threaded, deterministic OKVIS2 reference.

Mirrors tools/build_stella_reference.py. Never edits external/vio/okvis2 in place:
  1. Copies external/vio/okvis2 (no .git, no build/) -> runs/okvis_port/reference_build/src
  2. Applies okvis_port/reference/patches/*.patch (in order, -p1) to that copy
  3. Configures+builds it (Release layout, CXX flags -O2 -DNDEBUG -ffp-contract=off
     -fno-fast-math, no -march=native) -> runs/okvis_port/reference_build/build
  4. Writes runs/okvis_port/reference_build/provenance.json (upstream + submodule
     commit shas, compiler, flags, patch sha256, deps versions)

The dependency prefix (Eigen 3.4.0, glog, gflags, boost, openblas, OpenCV 4.6.0 with a
stub highgui) is the existing external/vio/deps tree described by external/vio/env.sh.

Usage: python3 tools/build_okvis_reference.py [--skip-copy] [--skip-patch] [--jobs N]
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
V = REPO / "external/vio"
SRC_ORIG = V / "okvis2"
BUILD_ROOT = REPO / "runs/okvis_port/reference_build"
SRC_COPY = BUILD_ROOT / "src"
BUILD_DIR = BUILD_ROOT / "build"
PATCHES_DIR = REPO / "okvis_port/reference/patches"
CONFIG = REPO / "okvis_port/reference/configs/okvis_mono_euroc_deterministic.yaml"

CXX_FLAGS = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
SUBMODULES = ["brisk", "ceres-solver", "DBoW2", "opengv", "googletest"]


def run(cmd, cwd=None, env=None, check=True):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, cwd=cwd, env=env, check=check)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_commit(path):
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except Exception:
        return None


def build_env():
    """Re-create external/vio/env.sh in Python."""
    root = V / "deps/root/usr"
    ocv = V / "deps/opencv"
    libs = [root / "lib/x86_64-linux-gnu", root / "lib/x86_64-linux-gnu/openblas-pthread", ocv / "lib"]
    env = os.environ.copy()
    env["VROOT"] = str(root)
    env["CMAKE_PREFIX_PATH"] = f"{root}:{ocv}"
    env["LD_LIBRARY_PATH"] = ":".join(map(str, libs)) + ":" + env.get("LD_LIBRARY_PATH", "")
    env["LIBRARY_PATH"] = ":".join(map(str, libs))
    env["CPATH"] = ":".join(map(str, [root / "include", root / "include/x86_64-linux-gnu",
                                      root / "include/eigen3", ocv / "include/opencv4"]))
    return env


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-copy", action="store_true", help="reuse existing src copy (patches already applied)")
    ap.add_argument("--skip-patch", action="store_true")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    patches = sorted(PATCHES_DIR.glob("*.patch"))
    if not args.skip_copy:
        if SRC_COPY.exists():
            shutil.rmtree(SRC_COPY)
        BUILD_ROOT.mkdir(parents=True, exist_ok=True)
        print(f"copying {SRC_ORIG} -> {SRC_COPY}")
        shutil.copytree(SRC_ORIG, SRC_COPY,
                        ignore=shutil.ignore_patterns(".git", "build", "*.pt", "meshes", "googletest"),
                        symlinks=True)
        # googletest is only needed with BUILD_TESTS=ON (we use OFF); keep an empty dir so the
        # copy_if_exists() calls in external/CMakeLists.txt are unaffected.
        (SRC_COPY / "external/googletest").mkdir(exist_ok=True)
        # okvis_frontend's CMake unconditionally copies resources/fast-scnn.pt next to the lib; the
        # CNN is off (USE_NN=OFF) and the weights have no license statement, so use an empty placeholder.
        (SRC_COPY / "resources").mkdir(exist_ok=True)
        (SRC_COPY / "resources/fast-scnn.pt").write_bytes(b"")
        if not args.skip_patch:
            for patch in patches:
                print(f"applying {patch.name}")
                run(["patch", "-p1", "-i", str(patch)], cwd=SRC_COPY)
    applied = [{"name": p.name, "sha256": sha256(p)} for p in patches]

    env = build_env()
    V_ = str(V)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    run([
        "cmake", "-S", str(SRC_COPY), "-B", str(BUILD_DIR),
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_CXX_FLAGS_RELEASE={CXX_FLAGS}",
        f"-DCMAKE_C_FLAGS_RELEASE={CXX_FLAGS}",
        "-DBUILD_APPS=ON", "-DBUILD_TESTS=OFF", "-DBUILD_ROS2=OFF", "-DUSE_NN=OFF", "-DUSE_GPU=OFF",
        "-DHAVE_LIBREALSENSE=OFF", "-DDO_TIMING=OFF",
        "-DSUITESPARSE=OFF", "-DCXSPARSE=OFF", "-DEIGENSPARSE=ON", "-DMINIGLOG=OFF", "-DLAPACK=ON",
        f"-DBLAS_LIBRARIES={V_}/deps/root/usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so",
        f"-DLAPACK_LIBRARIES={V_}/deps/root/usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so",
        f"-DEigen3_DIR={V_}/deps/root/usr/share/eigen3/cmake",
        f"-DOpenCV_DIR={V_}/deps/opencv/lib/cmake/opencv4",
        f"-DBoost_DIR={V_}/deps/root/usr/lib/x86_64-linux-gnu/cmake/Boost-1.83.0",
        f"-Dgflags_DIR={V_}/deps/root/usr/lib/x86_64-linux-gnu/cmake/gflags",
        f"-DGLOG_INCLUDE_DIR={V_}/deps/root/usr/include",
        f"-DGLOG_LIBRARY={V_}/deps/root/usr/lib/x86_64-linux-gnu/libglog.so",
    ], env=env)
    run(["cmake", "--build", str(BUILD_DIR), "-j", str(args.jobs), "--target", "okvis_app_synchronous"], env=env)

    gcc_version = subprocess.run(["gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    eigen_h = (V / "deps/root/usr/include/eigen3/Eigen/src/Core/util/Macros.h").read_text()
    eigen_ver = ".".join(
        next(l.split()[-1] for l in eigen_h.splitlines() if l.startswith(f"#define EIGEN_{k}_VERSION"))
        for k in ("WORLD", "MAJOR", "MINOR"))
    provenance = {
        "okvis2_commit": git_commit(SRC_ORIG),
        "submodule_commits": {m: git_commit(SRC_ORIG / "external" / m) for m in SUBMODULES},
        "compiler": gcc_version,
        "cxx_flags": CXX_FLAGS,
        "eigen_version": eigen_ver,
        "ceres": "external/ceres-solver (submodule above), EIGENSPARSE=ON, SUITESPARSE=OFF, CXSPARSE=OFF, "
                 "threading model OPENMP (num_threads forced to 1 by the deterministic config)",
        "patches_applied": applied,
        "config": {"path": str(CONFIG.relative_to(REPO)), "sha256": sha256(CONFIG) if CONFIG.exists() else None},
        "deps_prefix": "external/vio/deps (see external/vio/env.sh)",
        "note": "no -march=native (SSE2 baseline), no fast-math, FMA contraction off; same flags as stella_port reference",
    }
    (BUILD_ROOT / "provenance.json").write_text(json.dumps(provenance, indent=2))
    print(f"wrote {BUILD_ROOT / 'provenance.json'}")


if __name__ == "__main__":
    sys.exit(main())
