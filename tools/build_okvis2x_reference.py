#!/usr/bin/env python3
"""Build the okvis2x_port deterministic OKVIS2-X reference (GNSS path), modelled on tools/build_okvis_reference.py.

Never edits external/gnss/OKVIS2-X. Steps:
  1. Copies external/gnss/OKVIS2-X (no .git, no build/, no weights) -> runs/okvis2x_port/reference_build/src.
     okvis_multisensor_processing/CMakeLists.txt is taken from git HEAD (the working tree carries an earlier
     hand edit); the supereight2 submodule is not checked out in external/, so it is cloned (MPL-2.0, pinned to the
     commit the X tree records) into runs/okvis2x_port/deps/se2 and copied to src/supereight2.
  2. Applies okvis2x_port/reference/patches/*.patch (-p1) to the copy.
  3. Configures + builds (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math, no -march=native) against the
     external/vio/deps prefix (Eigen 3.4.0, glog, gflags, boost, openblas, OpenCV 4.6.0 stub highgui, TBB, GeographicLib).
  4. Writes runs/okvis2x_port/reference_build/provenance.json.

Usage: python3 tools/build_okvis2x_reference.py [--skip-copy] [--skip-patch] [--jobs N<=4] [--targets a,b]
"""
import argparse, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
V = REPO / "external/vio"
SRC_ORIG = REPO / "external/gnss/OKVIS2-X"
BUILD_ROOT = REPO / "runs/okvis2x_port/reference_build"
SRC_COPY = BUILD_ROOT / "src"
BUILD_DIR = BUILD_ROOT / "build"
PATCHES_DIR = REPO / "okvis2x_port/reference/patches"
PCL_STUB = REPO / "okvis2x_port/reference/pcl_stub"
SE2_DIR = REPO / "runs/okvis2x_port/deps/se2"
CONFIG = REPO / "okvis2x_port/reference/configs/okvis2x_mono_euroc_deterministic.yaml"
CXX_FLAGS = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
SUBMODULES = ["brisk", "ceres-solver", "DBoW2", "opengv", "googletest"]
RESTORE_FROM_HEAD = ["okvis_multisensor_processing/CMakeLists.txt"]


def run(cmd, cwd=None, env=None, check=True):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, cwd=cwd, env=env, check=check)


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def git(path, *a):
    try:
        return subprocess.run(["git", "-C", str(path), *a], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def build_env():
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


def ensure_se2():
    want = git(SRC_ORIG, "ls-tree", "HEAD", "supereight2").split()[2]
    if not (SE2_DIR / ".git").exists():
        SE2_DIR.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", "https://github.com/ethz-mrl/supereight2.git", str(SE2_DIR)])
    run(["git", "-C", str(SE2_DIR), "checkout", "-q", want])
    run(["git", "-C", str(SE2_DIR), "submodule", "update", "--init", "--recursive"])
    return want


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-copy", action="store_true")
    ap.add_argument("--skip-patch", action="store_true")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--copy-only", action="store_true", help="copy + patch, do not build")
    ap.add_argument("--targets", default="okvis_app_synchronous,okvis2x_app_synchronous")
    args = ap.parse_args()
    args.jobs = min(args.jobs, 4)

    patches = sorted(PATCHES_DIR.glob("*.patch"))
    se2_commit = None
    if not args.skip_copy:
        se2_commit = ensure_se2()
        if SRC_COPY.exists():
            shutil.rmtree(SRC_COPY)
        BUILD_ROOT.mkdir(parents=True, exist_ok=True)
        print(f"copying {SRC_ORIG} -> {SRC_COPY}")
        shutil.copytree(SRC_ORIG, SRC_COPY, symlinks=True,
                        ignore=shutil.ignore_patterns(".git", "build", "*.pt", "meshes", "googletest", "supereight2"))
        for rel in RESTORE_FROM_HEAD:
            (SRC_COPY / rel).write_text(git(SRC_ORIG, "show", f"HEAD:{rel}") + "\n")
        shutil.copytree(SE2_DIR, SRC_COPY / "supereight2", ignore=shutil.ignore_patterns(".git", "doc", "test", "googletest"))
        (SRC_COPY / "external/googletest").mkdir(exist_ok=True)
        (SRC_COPY / "resources").mkdir(exist_ok=True)
        for f in ("fast-scnn.pt", "depth-model.pt"):   # CMake copies them next to the libs; placeholders (USE_NN=OFF)
            (SRC_COPY / "resources" / f).write_bytes(b"")
        shutil.copy(SRC_ORIG / "resources/small_voc.yml.gz", SRC_COPY / "resources/small_voc.yml.gz")
        if not args.skip_patch:
            for p in patches:
                print(f"applying {p.name}")
                run(["patch", "-p1", "-i", str(p)], cwd=SRC_COPY)
    if args.copy_only:
        return 0
    applied = [{"name": p.name, "sha256": sha256(p)} for p in patches]

    env = build_env()
    V_ = str(V)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    run([
        "cmake", "-S", str(SRC_COPY), "-B", str(BUILD_DIR),
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_CXX_FLAGS_RELEASE={CXX_FLAGS}", f"-DCMAKE_C_FLAGS_RELEASE={CXX_FLAGS}",
        "-DBUILD_APPS=ON", "-DBUILD_TESTS=OFF", "-DBUILD_ROS2=OFF", "-DUSE_NN=OFF", "-DUSE_GPU=OFF",
        "-DHAVE_LIBREALSENSE=OFF", "-DDO_TIMING=OFF", "-DSE_TEST=OFF", "-DSE_APP=OFF",
        f"-DOKVIS2X_PCL_STUB={PCL_STUB}",
        "-DSUITESPARSE=OFF", "-DCXSPARSE=OFF", "-DEIGENSPARSE=ON", "-DMINIGLOG=OFF", "-DLAPACK=ON",
        f"-DBLAS_LIBRARIES={V_}/deps/root/usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so",
        f"-DLAPACK_LIBRARIES={V_}/deps/root/usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so",
        f"-DEigen3_DIR={V_}/deps/root/usr/share/eigen3/cmake",
        f"-DOpenCV_DIR={V_}/deps/opencv/lib/cmake/opencv4",
        f"-DBoost_DIR={V_}/deps/root/usr/lib/x86_64-linux-gnu/cmake/Boost-1.83.0",
        f"-Dgflags_DIR={V_}/deps/root/usr/lib/x86_64-linux-gnu/cmake/gflags",
        f"-DTBB_DIR={V_}/deps/root/usr/lib/x86_64-linux-gnu/cmake/TBB",
        f"-DGLOG_INCLUDE_DIR={V_}/deps/root/usr/include",
        f"-DGLOG_LIBRARY={V_}/deps/root/usr/lib/x86_64-linux-gnu/libglog.so",
        f"-DCMAKE_MODULE_PATH={V_}/deps/root/usr/share/cmake/geographiclib",
        f"-DGeographicLib_LIBRARIES={V_}/deps/root/usr/lib/x86_64-linux-gnu/libGeographicLib.so",
        f"-DGeographicLib_INCLUDE_DIRS={V_}/deps/root/usr/include",
    ], env=env)
    for t in args.targets.split(","):
        run(["cmake", "--build", str(BUILD_DIR), "-j", str(args.jobs), "--target", t], env=env)

    gcc = subprocess.run(["gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    prov = {
        "okvis2x_commit": git(SRC_ORIG, "rev-parse", "HEAD"),
        "supereight2_commit": se2_commit or git(SE2_DIR, "rev-parse", "HEAD"),
        "submodule_commits": {m: git(SRC_ORIG / "external" / m, "rev-parse", "HEAD") for m in SUBMODULES},
        "compiler": gcc, "cxx_flags": CXX_FLAGS, "targets": args.targets,
        "patches_applied": applied,
        "config": {"path": str(CONFIG.relative_to(REPO)), "sha256": sha256(CONFIG) if CONFIG.exists() else None},
        "deps_prefix": "external/vio/deps (see external/vio/env.sh); GeographicLib and TBB come from it",
        "note": "no -march=native, no fast-math, FMA contraction off; same flags as okvis_port reference. "
                "okvis_multisensor_processing/CMakeLists.txt restored from git HEAD (PCL stub via patch 0004)",
    }
    (BUILD_ROOT / "provenance.json").write_text(json.dumps(prov, indent=2))
    print("wrote", BUILD_ROOT / "provenance.json")


if __name__ == "__main__":
    sys.exit(main())
