#!/usr/bin/env python3
"""Build the stella_port single-threaded, deterministic reference.

Never edits external/candidates/stella_vslam in place. Instead:
  1. Copies external/candidates/stella_vslam -> runs/stella_port/reference_build/src
  2. Applies stella_port/reference/patches/*.patch (in order) to that copy
  3. Configures+builds it with -DDETERMINISTIC=ON -DUSE_OPENMP=OFF and
     -O2 -ffp-contract=off -fno-fast-math (no -march=native), installs to
     runs/stella_port/reference_build/install
  4. Configures+builds stella_port/reference/driver (run_stella_reference)
     against that install
  5. Writes runs/stella_port/reference_build/provenance.json

Requires the environment described in stella_port/reference/README.md
"Build" (OpenCV 4.6 under /tmp/pose-opencv + /tmp/localopencv, deps under
external/candidates/deps/root, g2o likewise) -- see runs/oneshot/environment.sh.

Usage: python3 tools/build_stella_reference.py [--skip-copy] [--skip-patch]
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
SRC_ORIG = REPO / "external/candidates/stella_vslam"
DEPS = REPO / "external/candidates/deps/root/usr"
BUILD_ROOT = REPO / "runs/stella_port/reference_build"
SRC_COPY = BUILD_ROOT / "src"
BUILD_DIR = BUILD_ROOT / "build"
INSTALL_DIR = BUILD_ROOT / "install"
DRIVER_SRC = REPO / "stella_port/reference/driver"
DRIVER_BUILD = BUILD_ROOT / "driver_build"
PATCHES_DIR = REPO / "stella_port/reference/patches"

OPENCV_PKGCONFIG = "/tmp/pose-opencv/pkgconfig"
LOCALOPENCV_LIB = "/tmp/localopencv/root/usr/lib/x86_64-linux-gnu"
LOCALOPENCV_LIB2 = "/tmp/localopencv/root/usr/lib"


def run(cmd, cwd=None, env=None, check=True):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, cwd=cwd, env=env, check=check)


def sha256(path):
    h = hashlib.sha256()
    h.update(Path(path).read_bytes())
    return h.hexdigest()


def git_commit(path):
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                              capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-copy", action="store_true", help="reuse existing src copy (patches already applied)")
    ap.add_argument("--skip-patch", action="store_true")
    args = ap.parse_args()

    if not args.skip_copy:
        if SRC_COPY.exists():
            shutil.rmtree(SRC_COPY)
        BUILD_ROOT.mkdir(parents=True, exist_ok=True)
        print(f"copying {SRC_ORIG} -> {SRC_COPY}")
        shutil.copytree(SRC_ORIG, SRC_COPY)

        applied = []
        if not args.skip_patch:
            for patch in sorted(PATCHES_DIR.glob("*.patch")):
                print(f"applying {patch.name}")
                run(["patch", "-p1", "-i", str(patch)], cwd=SRC_COPY)
                applied.append({"name": patch.name, "sha256": sha256(patch)})
    else:
        applied = [{"name": p.name, "sha256": sha256(p)} for p in sorted(PATCHES_DIR.glob("*.patch"))]

    env = os.environ.copy()
    env["PKG_CONFIG_PATH"] = OPENCV_PKGCONFIG + ":" + env.get("PKG_CONFIG_PATH", "")
    libarch = DEPS / "lib/x86_64-linux-gnu"

    cxx_flags = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
    linker_flags = (f"-L{libarch} -L{LOCALOPENCV_LIB} -Wl,--allow-shlib-undefined "
                     f"-Wl,-rpath,{libarch} -Wl,-rpath,{INSTALL_DIR}/lib -Wl,-rpath,{LOCALOPENCV_LIB}")

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    run([
        "cmake", "-S", str(SRC_COPY), "-B", str(BUILD_DIR),
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        "-DCMAKE_BUILD_TYPE=",
        f"-DCMAKE_CXX_FLAGS={cxx_flags}",
        f"-DCMAKE_C_FLAGS={cxx_flags}",
        f"-DCMAKE_INSTALL_PREFIX={INSTALL_DIR}",
        f"-DCMAKE_PREFIX_PATH={DEPS};{DEPS}/lib/cmake",
        "-DBUILD_WITH_MARCH_NATIVE=OFF",
        "-DBOW_FRAMEWORK=FBoW",
        "-DUSE_CCACHE=OFF",
        "-DUSE_OPENMP=OFF",
        "-DDETERMINISTIC=ON",
        f"-DCMAKE_EXE_LINKER_FLAGS={linker_flags}",
        f"-DCMAKE_SHARED_LINKER_FLAGS={linker_flags}",
    ], env=env)
    run(["cmake", "--build", str(BUILD_DIR), "-j", str(os.cpu_count())], env=env)
    run(["cmake", "--install", str(BUILD_DIR)], env=env)

    DRIVER_BUILD.mkdir(parents=True, exist_ok=True)
    run([
        "cmake", "-S", str(DRIVER_SRC), "-B", str(DRIVER_BUILD),
        f"-DSTELLA_REFERENCE_INSTALL_DIR={INSTALL_DIR}",
        f"-DCMAKE_PREFIX_PATH={INSTALL_DIR};{DEPS};{DEPS}/lib/cmake",
        f"-DCMAKE_CXX_FLAGS={cxx_flags}",
        f"-DCMAKE_EXE_LINKER_FLAGS={linker_flags}",
    ], env=env)
    run(["cmake", "--build", str(DRIVER_BUILD), "-j", str(os.cpu_count())], env=env)

    gcc_version = subprocess.run(["gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    provenance = {
        "stella_vslam_commit": git_commit(SRC_ORIG),
        "g2o_commit": git_commit(REPO / "external/candidates/g2o"),
        "compiler": gcc_version,
        "cxx_flags": cxx_flags,
        "cmake_options": {
            "DETERMINISTIC": "ON (upstream stella_vslam CMake option -- id_less<shared_ptr<T>> ordering "
                              "instead of pointer-hash unordered_set/unordered_map at most call sites, "
                              "see src/stella_vslam/type.h nondeterministic:: aliases)",
            "USE_OPENMP": "OFF (default upstream too -- #pragma omp is ignored without -fopenmp, so "
                          "feature/orb_extractor.cc, match/stereo.cc, mapping_module.cc parallel loops "
                          "run as plain sequential loops)",
            "BUILD_WITH_MARCH_NATIVE": "OFF",
        },
        "patches_applied": applied,
        "g2o_solver_modules_linked": {
            "LinearSolverEigen": "local_bundle_adjuster_g2o.cc (local BA), pose_optimizer_g2o.cc, "
                                  "transform_optimizer.cc -- BSD (g2o core + Eigen)",
            "LinearSolverCSparse": "global_bundle_adjuster.cc (initial-map global BA), graph_optimizer.cc "
                                    "(loop-closing pose-graph optimization) -- links g2o's csparse solver "
                                    "module against CXSparse (LGPL-2.1+, found at "
                                    "external/candidates/deps/root/usr/include/suitesparse). NOT avoided "
                                    "in this build: loop closing and initial-map global BA exercise this "
                                    "path. See docs/stella_vslam_license_audit.md rule 4 and "
                                    "stella_port/reference/README.md \"License notes\".",
        },
        "config_determinism_settings": {
            "Initializer.use_fixed_seed": True,
            "Relocalizer.use_fixed_seed": True,
            "LoopDetector.use_fixed_seed": True,
            "Mapping.enable_interruption_of_landmark_generation": False,
            "Mapping.enable_interruption_before_local_BA": False,
        },
        "runtime_env": {"OMP_NUM_THREADS": "1 (belt-and-suspenders; USE_OPENMP=OFF already removes -fopenmp)"},
    }
    (BUILD_ROOT / "provenance.json").write_text(json.dumps(provenance, indent=2))
    print(f"wrote {BUILD_ROOT / 'provenance.json'}")


if __name__ == "__main__":
    sys.exit(main())
