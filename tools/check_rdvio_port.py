#!/usr/bin/env python3
"""Run the rdvio_port bit-exactness harnesses (tolerance 0, memcmp).

  python3 tools/check_rdvio_port.py [--tag m1] [--oracle] [--sanitize]
Per module: compile rdvio_port/c/check_rd_*.c (C99, -ffp-contract=off) together with the port and the okvis_port kernels it reuses,
replay the dump files of runs/rdvio_port/<tag>/<seq>/dump (reference run with patch 0003) and, with --oracle, the random-input records
written by the real RD-VIO classes (rdvio_port/reference_tools/rd_imu_oracle.cc, built against the reference build).
Exit status 0 iff there is no mismatch.
"""
import argparse, subprocess, sys, os
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PORT = REPO / "rdvio_port"
OK = REPO / "okvis_port/c"
BUILD = REPO / "runs/rdvio_port/check_build"
REF = REPO / "runs/rdvio_port/reference_build"
V = REPO / "external/vio"

STELLA = REPO / "stella_port/c"
MODULES = {  # name -> (harness, port sources, oracle source, extra flags, extra (non-port) sources)
    "m1": ("check_rd_imu.c", ["rd_imu.c", "rd_lie.c", "rd_eigen.c"], "rd_imu_oracle.cc", [], []),
    "m3": ("check_rd_m3.c", ["rd_rand.c", "rd_ransac.c", "rd_poisson.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_lie.c"], "rd_m3_oracle.cc", [], [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m2": ("check_rd_m2.c", ["rd_factor.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_lie.c"], "rd_m2_oracle.cc", [], [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
}
OK_SRC = ["ok_eigen.c", "ok_dense.c"]


def run(cmd, **kw):
    print("+", " ".join(map(str, cmd)), flush=True)
    return subprocess.run(cmd, **kw)


def build_oracle(name):
    src = PORT / "reference_tools" / name
    exe = REF / "build" / name.replace(".cc", "")
    if exe.exists() and exe.stat().st_mtime > src.stat().st_mtime:
        return exe
    s = REF / "src"
    inc = [f"-I{s}/src/{m}/include" for m in ("rdvio", "rdvio_estimation", "rdvio_extra", "rdvio_geometry", "rdvio_map", "rdvio_util")]
    dl = V / "deps/root/usr/lib/x86_64-linux-gnu"
    ocv = [str(V / "deps/opencv/lib" / f"libopencv_{m}.so") for m in ("calib3d", "features2d", "flann", "imgcodecs", "imgproc", "core", "video")]
    env = os.environ.copy()
    env["CPATH"] = ":".join(map(str, [V / "deps/root/usr/include", V / "deps/root/usr/include/x86_64-linux-gnu", V / "deps/root/usr/include/eigen3", V / "deps/opencv/include/opencv4"]))
    cmd = ["g++", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", "-std=gnu++17", *inc, f"-I{s}/3rd/spdlog/include", f"-I{REF}/ceres-install/include",
           str(src), "-o", str(exe), f"-L{REF}/build/src", "-lrdvio", *ocv, f"{REF}/ceres-install/lib/libceres.a", str(dl / "libglog.so"), str(dl / "libgflags.so"),
           str(dl / "openblas-pthread/libopenblas.so"), str(REPO / "external/vio3/MSCEqF/build/_deps/yaml-cpp-build/libyaml-cpp.a"), "-lpthread"]
    run(cmd, env=env, check=True)
    return exe


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="m1")
    ap.add_argument("--tag2", default="m2", help="dump run tag for the m2 module")
    ap.add_argument("--tag3", default="m3", help="dump run tag for the m3 module")
    ap.add_argument("--seq", default="MH_01_easy")
    ap.add_argument("--oracle", action="store_true", help="also replay random inputs recorded from the real classes")
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--count", default="20000")
    ap.add_argument("--sanitize", action="store_true")
    ap.add_argument("--modules", default="m1,m2,m3")
    a = ap.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    rc = 0
    env = os.environ.copy()
    root = V / "deps/root/usr"
    env["LD_LIBRARY_PATH"] = f"{root}/lib/x86_64-linux-gnu:{root}/lib/x86_64-linux-gnu/openblas-pthread:{V}/deps/opencv/lib:" + env.get("LD_LIBRARY_PATH", "")
    for m in a.modules.split(","):
        harness, srcs, oracle_src, xflags, xsrcs = MODULES[m]
        exe = BUILD / harness.replace(".c", "")
        flags = ["-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", *xflags] + (["-fsanitize=address,undefined", "-g"] if a.sanitize else [])
        run(["cc", *flags, "-o", str(exe), str(PORT / "c" / harness), *[str(PORT / "c" / x) for x in srcs], *[str(OK / x) for x in OK_SRC], *xsrcs, "-lm"], check=True)
        dirs = []
        dd = REPO / "runs/rdvio_port" / (a.tag if m == "m1" else a.tag2 if m == "m2" else a.tag3) / a.seq / "dump"
        if dd.exists():
            dirs.append(("dump " + str(dd), dd))
        if a.oracle:
            orc = build_oracle(oracle_src)
            for sd in a.seeds.split(","):
                od = REPO / "runs/rdvio_port" / f"oracle{m}_{sd}"
                od.mkdir(parents=True, exist_ok=True)
                run([str(orc), str(od), sd, a.count], env=env, check=True)
                dirs.append((f"oracle seed {sd} x{a.count}", od))
        for name, d in dirs:
            print(f"== {m}: {name}")
            r = run([str(exe), str(d)])
            rc |= r.returncode
    sys.exit(rc)


if __name__ == "__main__":
    main()
