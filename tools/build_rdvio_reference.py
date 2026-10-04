#!/usr/bin/env python3
"""Build the rdvio_port single-threaded, deterministic RD-VIO reference.

Mirrors tools/build_okvis_reference.py. Never edits external/vio3/rd_vio:
  1. `git archive HEAD` of external/vio3/rd_vio (pristine upstream commit, no local build fixes)
     -> runs/rdvio_port/reference_build/src
  2. applies rdvio_port/reference/patches/*.patch (in order, -p1)
  3. Ceres 2.2.0 (the pristine okvis2 submodule, the same one okvis_port validated against) is built once by
     runs/rdvio_port/reference_build/build_ceres.sh into ceres-install (Eigen 3.4.0 from external/vio/deps)
  4. configures + builds librdvio (-DTHREADING=OFF, -O2 -DNDEBUG -ffp-contract=off -fno-fast-math) and
     rdvio_port/reference/driver/rdvio_ref_driver.cpp -> runs/rdvio_port/reference_build/build/rdvio_ref_driver
  5. writes provenance.json (upstream sha, patch sha256, compiler, flags, dependency versions)

Usage: python3 tools/build_rdvio_reference.py [--skip-copy] [--jobs N]
"""
import argparse, hashlib, json, os, shutil, subprocess, sys, tarfile, io
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
V = REPO / "external/vio"
UP = REPO / "external/vio3/rd_vio"
ROOT = REPO / "runs/rdvio_port/reference_build"
SRC = ROOT / "src"
BUILD = ROOT / "build"
CERES = ROOT / "ceres-install"
PATCHES = REPO / "rdvio_port/reference/patches"
DRIVER = REPO / "rdvio_port/reference/driver/rdvio_ref_driver.cpp"
CXX_FLAGS = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"


def run(cmd, cwd=None, env=None):
    print("+", " ".join(map(str, cmd)), flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def env():
    root = V / "deps/root/usr"; ocv = V / "deps/opencv"
    libs = [root / "lib/x86_64-linux-gnu", root / "lib/x86_64-linux-gnu/openblas-pthread", ocv / "lib"]
    e = os.environ.copy()
    e["CMAKE_PREFIX_PATH"] = f"{CERES}:{root}:{ocv}"
    e["LD_LIBRARY_PATH"] = ":".join(map(str, libs)) + ":" + e.get("LD_LIBRARY_PATH", "")
    e["LIBRARY_PATH"] = ":".join(map(str, libs))
    e["CPATH"] = ":".join(map(str, [root / "include", root / "include/x86_64-linux-gnu", root / "include/eigen3", ocv / "include/opencv4"]))
    return e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-copy", action="store_true")
    ap.add_argument("--jobs", type=int, default=8)
    a = ap.parse_args()
    patches = sorted(PATCHES.glob("[0-9]*.patch"))
    if not a.skip_copy:
        if SRC.exists():
            shutil.rmtree(SRC)
        SRC.mkdir(parents=True)
        tar = subprocess.run(["git", "-C", str(UP), "archive", "HEAD"], capture_output=True, check=True).stdout
        tarfile.open(fileobj=io.BytesIO(tar)).extractall(SRC)
        for p in patches:
            print("applying", p.name)
            run(["patch", "-p1", "-i", str(p)], cwd=SRC)
    if not (CERES / "lib").exists():
        sys.exit(f"missing {CERES}: run {ROOT}/build_ceres.sh first")
    e = env()
    run(["cmake", "-S", str(SRC), "-B", str(BUILD), "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", "-DTHREADING=OFF",
         f"-DCMAKE_CXX_FLAGS_RELEASE={CXX_FLAGS}", f"-DEigen3_DIR={V}/deps/root/usr/share/eigen3/cmake",
         f"-DOpenCV_DIR={V}/deps/opencv/lib/cmake/opencv4", f"-DCeres_DIR={CERES}/lib/cmake/Ceres",
         f"-Dglog_DIR={V}/deps/root/usr/lib/x86_64-linux-gnu/cmake/glog", f"-Dgflags_DIR={V}/deps/root/usr/lib/x86_64-linux-gnu/cmake/gflags",
         f"-Dyaml-cpp_DIR={REPO}/external/vio3/shim/yaml-cpp"], env=e)
    run(["cmake", "--build", str(BUILD), "-j", str(a.jobs)], env=e)
    # driver: compile + link against the static library and the transitive deps CMake resolved
    inc = [f"-I{SRC}/src/{m}/include" for m in ("rdvio", "rdvio_estimation", "rdvio_extra", "rdvio_geometry", "rdvio_map", "rdvio_util")]
    ocvl = [str(V / "deps/opencv/lib" / f"libopencv_{m}.so") for m in ("video", "calib3d", "features2d", "flann", "imgcodecs", "imgproc", "core")]
    glog = V / "deps/root/usr/lib/x86_64-linux-gnu"
    cmd = ["c++", *CXX_FLAGS.split(), "-std=gnu++17", "-DYAML_CPP_STATIC_DEFINE", *inc, f"-I{SRC}/3rd/spdlog/include",
           f"-I{CERES}/include", f"-I{REPO}/external/vio3/MSCEqF/build/_deps/yaml-cpp-src/include", "-isystem", str(V / "deps/opencv/include/opencv4"),
           str(DRIVER), "-o", str(BUILD / "rdvio_ref_driver"), f"-L{BUILD}/src", "-lrdvio",
           str(REPO / "external/vio3/MSCEqF/build/_deps/yaml-cpp-build/libyaml-cpp.a"), *ocvl,
           str(CERES / "lib/libceres.a"), str(glog / "libglog.so"), str(glog / "libgflags.so"),
           str(glog / "openblas-pthread/libopenblas.so"), "-lpthread",
           f"-Wl,-rpath,{V}/deps/opencv/lib:{glog}:{glog}/openblas-pthread", "-Wl,--allow-shlib-undefined"]
    run(cmd, env=e)
    cc = subprocess.run(["c++", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    prov = {"upstream": "Jianxff/rd_vio", "upstream_sha": subprocess.run(["git", "-C", str(UP), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(),
            "patches": [{"name": p.name, "sha256": sha256(p)} for p in patches], "driver_sha256": sha256(DRIVER),
            "cxx_flags": CXX_FLAGS, "compiler": cc, "ceres": "2.2.0 (okvis2 submodule 85331393, pristine)", "eigen": "3.4.0 (external/vio/deps)",
            "opencv": "4.6.0 (external/vio/deps)", "threading": "OFF"}
    (ROOT / "provenance.json").write_text(json.dumps(prov, indent=1))
    print("built", BUILD / "rdvio_ref_driver")


if __name__ == "__main__":
    main()
