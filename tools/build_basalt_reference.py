#!/usr/bin/env python3
"""Build the basalt_port single-threaded, deterministic, headless Basalt reference (basalt_ref_driver).

Mirrors tools/build_rdvio_reference.py / build_okvis_reference.py. Never edits external/vio/basalt_src:
  1. copies external/vio/basalt_src (GitLab master + the pinned third-party checkouts in basalt_src/_deps, see
     basalt_port/PLAN.md section 1) without .git / vcpkg / docs / tests -> runs/basalt_port/reference_build/src
  2. applies basalt_port/reference/patches/*.patch (in order, -p1)
  3. replaces the upstream CMakeLists.txt (vcpkg + Pangolin + RealSense + rosbag, -O3 -march=native) by
     basalt_port/reference/CMakeLists.reference.txt (flags -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, no -march),
     and copies basalt_port/reference/driver/basalt_ref_driver.cpp (headless vio.cpp, TBB parallelism 1, OpenCV threads 0)
  4. fetches fmt / nlohmann-json / CLI11 headers as Ubuntu .deb files (apt-get download, no root) into
     runs/basalt_port/debs and unpacks them to reference_build/deps_root (fmt is used header-only)
  5. configures + builds opengv (static, from source) + libbasalt (static) + basalt_ref_driver, writes provenance.json
Eigen 3.4.0, oneTBB 2021.11 and OpenCV 4.6.0 come from external/vio/deps.

Usage: python3 tools/build_basalt_reference.py [--jobs 4] [--skip-copy]
Run:   LD_LIBRARY_PATH=external/vio/deps/root/usr/lib/x86_64-linux-gnu:external/vio/deps/opencv/lib \
       runs/basalt_port/reference_build/build/basalt_ref_driver --dataset-path <EuRoC seq> \
         --cam-calib <src>/data/euroc_ds_calib.json --config-path <src>/data/euroc_config.json --out traj.tum
"""
import argparse, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
UP = REPO / "external/vio/basalt_src"
V = REPO / "external/vio/deps"
PORT = REPO / "basalt_port/reference"
ROOT = Path(os.environ.get("BASALT_REF_ROOT", REPO / "runs/basalt_port/reference_build"))   # override: build a patched variant elsewhere
SRC, BUILD, DEPS, DEBS = ROOT / "src", ROOT / "build", ROOT / "deps_root", REPO / "runs/basalt_port/debs"
CXX_FLAGS = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
DEB_PKGS = ["libfmt-dev", "libfmt9", "nlohmann-json3-dev", "libcli11-dev"]
SKIP = {".git", "vcpkg", "doc", "python", "scripts", "test"}


def run(cmd, cwd=None):
    print("+", " ".join(map(str, cmd)), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def copy_tree():
    if SRC.exists():
        shutil.rmtree(SRC)
    def ignore(d, names):
        return [n for n in names if n in SKIP or (Path(d).name == "thirdparty" and n == "vcpkg") or n == ".git"]
    shutil.copytree(UP, SRC, ignore=ignore)


def fetch_debs():
    DEBS.mkdir(parents=True, exist_ok=True)
    if not any(DEBS.glob("libfmt-dev*.deb")):
        for p in DEB_PKGS:
            run(["apt-get", "download", p], cwd=DEBS)
    DEPS.mkdir(parents=True, exist_ok=True)
    for d in sorted(DEBS.glob("*.deb")):
        run(["dpkg-deb", "-x", str(d), str(DEPS)])


def check_sqrt_cov_inv():
    """ODR guard (basalt_port/PLAN.md): preintegration.h calls an unqualified sqrt(float) inside
    IntegratedImuMeasurement<float>::compute_sqrt_cov_inv, which resolves to float sqrtss in the linearization translation
    units but to ::sqrt(double) in a unit with different includes. The linker keeps one inline copy; the reference trajectory
    (a7d3e7a3...) needs the float one. Fail the build if the linked copy uses sqrtsd."""
    dis = subprocess.run(["objdump", "-d", "-C", "--no-show-raw-insn", str(BUILD / "basalt_ref_driver")],
                         capture_output=True, text=True).stdout
    i = dis.find("<basalt::IntegratedImuMeasurement<float>::compute_sqrt_cov_inv() const>:")
    body = dis[i:dis.find("\n\n", i)]
    if "sqrtss" not in body or "sqrtsd" in body:
        sys.exit("ODR guard: compute_sqrt_cov_inv<float> in the linked driver is not the float (sqrtss) copy")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--skip-copy", action="store_true")
    a = ap.parse_args()
    patches = sorted((PORT / "patches").glob("[0-9]*.patch"))
    if not a.skip_copy:
        copy_tree()
        for p in patches:
            print("applying", p.name)
            run(["patch", "-p1", "-i", str(p)], cwd=SRC)
        shutil.move(SRC / "CMakeLists.txt", SRC / "CMakeLists.upstream.txt")
        shutil.copy(PORT / "CMakeLists.reference.txt", SRC / "CMakeLists.txt")
        (SRC / "driver").mkdir()
        shutil.copy(PORT / "driver/basalt_ref_driver.cpp", SRC / "driver/basalt_ref_driver.cpp")
    fetch_debs()
    e = os.environ.copy()
    e.pop("CPATH", None)
    dr = DEPS / "usr"
    run(["cmake", "-S", str(SRC), "-B", str(BUILD), "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
         f"-DEIGEN_INC={V}/root/usr/include/eigen3", f"-DDEPS_INC={dr}/include", f"-DTBB_INC={V}/root/usr/include",
         f"-DTBB_LIB={V}/root/usr/lib/x86_64-linux-gnu/libtbb.so", f"-DOCV_ROOT={V}/opencv"])
    run(["cmake", "--build", str(BUILD), "-j", str(a.jobs)])
    check_sqrt_cov_inv()
    cc = subprocess.run(["c++", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    deps = {}
    for d in sorted((UP / "_deps").iterdir()):
        deps[d.name] = subprocess.run(["git", "-C", str(d), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    prov = {"upstream": "https://gitlab.com/VladyslavUsenko/basalt.git (master, shallow)",
            "upstream_sha": subprocess.run(["git", "-C", str(UP), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(),
            "third_party_checkouts": deps,
            "patches": [{"name": p.name, "sha256": sha256(p)} for p in patches],
            "cmakelists_sha256": sha256(PORT / "CMakeLists.reference.txt"), "driver_sha256": sha256(PORT / "driver/basalt_ref_driver.cpp"),
            "cxx_flags": CXX_FLAGS, "compiler": cc, "eigen": "3.4.0 (external/vio/deps)", "tbb": "oneTBB 2021.11 (external/vio/deps), parallelism 1",
            "opencv": "4.6.0 (external/vio/deps)", "binary_sha256": sha256(BUILD / "basalt_ref_driver")}
    (ROOT / "provenance.json").write_text(json.dumps(prov, indent=1))
    print("built", BUILD / "basalt_ref_driver")


if __name__ == "__main__":
    main()
