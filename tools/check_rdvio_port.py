#!/usr/bin/env python3
"""Run the rdvio_port bit-exactness harnesses (tolerance 0, memcmp).

  python3 tools/check_rdvio_port.py [--modules m1,m2,m3,m4,m5,m6,m7,m9,m10] [--tag m1] [--oracle] [--sanitize]
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
OK_DIR = OK
BUILD = REPO / "runs/rdvio_port/check_build"
REF = REPO / "runs/rdvio_port/reference_build"
V = REPO / "external/vio"

STELLA = REPO / "stella_port/c"
M4_SRC = ["rd_solve.c", "rd_solve_linear.c", "rd_static.c", "rd_factor.c", "rd_imu.c", "rd_lie.c", "rd_eigen.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_marg.c", "rd_seig.c"]
MODULES = {  # name -> (harness, port sources, oracle source, extra flags, extra (non-port) sources)
    "m1": ("check_rd_imu.c", ["rd_imu.c", "rd_lie.c", "rd_eigen.c"], "rd_imu_oracle.cc", [], []),
    "m3": ("check_rd_m3.c", ["rd_rand.c", "rd_ransac.c", "rd_poisson.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_lie.c"], "rd_m3_oracle.cc", [], [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m4": ("check_rd_solve.c", M4_SRC, "rd_m4_oracle.cc", ["-I" + str(OK_DIR)], [str(OK_DIR / x) for x in ("ok_blas.c", "ok_sparse.c", "ok_amd.c")] + [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m5": ("check_rd_m5.c", ["rd_marg.c", "rd_seig.c", "rd_imu.c", "rd_factor.c", "rd_lie.c", "rd_eigen.c"], "rd_m5_oracle.cc", [], []),
    "m9": ("check_rd_init.c", ["rd_sys_init.c", "rd_map.c", "rd_solver_glue.c", "rd_sys_config.c", "rd_yaml.c", "rd_sys_eigen.c", "rd_solve.c", "rd_solve_linear.c", "rd_static.c", "rd_factor.c", "rd_imu.c", "rd_lie.c", "rd_eigen.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_marg.c", "rd_seig.c", "rd_rand.c", "rd_ransac.c", "rd_poisson.c"], None, ["-I" + str(OK_DIR)], [str(OK_DIR / x) for x in ("ok_kin.c", "ok_blas.c", "ok_sparse.c", "ok_amd.c")] + [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m10": ("check_rd_swt.c", ["rd_sys_swt.c", "rd_sys_init.c", "rd_map.c", "rd_solver_glue.c", "rd_sys_config.c", "rd_yaml.c", "rd_sys_eigen.c", "rd_solve.c", "rd_solve_linear.c", "rd_static.c", "rd_factor.c", "rd_imu.c", "rd_lie.c", "rd_eigen.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_marg.c", "rd_seig.c", "rd_rand.c", "rd_ransac.c", "rd_poisson.c"], None, ["-I" + str(OK_DIR)], [str(OK_DIR / x) for x in ("ok_kin.c", "ok_blas.c", "ok_sparse.c", "ok_amd.c")] + [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m6": ("check_rd_map.c", ["rd_map.c", "rd_imu.c", "rd_eigen.c", "rd_rand.c", "rd_ransac.c", "rd_poisson.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_lie.c"], None, [], [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
    "m2": ("check_rd_m2.c", ["rd_factor.c", "rd_geom.c", "rd_svd.c", "rd_qr.c", "rd_lie.c"], "rd_m2_oracle.cc", [], [str(STELLA / x) for x in ("sv_eigen_svd.c", "sv_eigen_qr.c", "sv_eigen_eigensolver.c")]),
}
OK_SRC = ["ok_eigen.c", "ok_dense.c"]
# dump channels of patches 0003-0005 switched off for the m5 oracle run
M5_QUIET = "integ pred pie plus rpe rot wahba ess5 hom4 decess dechom tri2 trin trk tang glp slp fess frot fhom pess phom pois".split()


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
    cmd = ["g++", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", "-std=gnu++17", *inc, f"-I{s}/3rd/spdlog/include", f"-I{REF}/ceres-install-m4/include",
           str(src), "-o", str(exe), f"-L{REF}/build/src", "-lrdvio", *ocv, f"{REF}/ceres-install-m4/lib/libceres.a", str(dl / "libglog.so"), str(dl / "libgflags.so"),
           str(dl / "openblas-pthread/libopenblas.so"), str(REPO / "external/vio3/MSCEqF/build/_deps/yaml-cpp-build/libyaml-cpp.a"), "-lpthread"]
    run(cmd, env=env, check=True)
    return exe


def static_test():
    """rd_m4_static_test.cc: the all-static kernels of SchurEliminator<2,3,3> (rd_static.c) against the real Ceres small_blas.h / invert_psd_matrix.h."""
    ce = REF / "ceres-install-m4"
    vr = V / "deps/root/usr"
    cf = ["-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math", f"-I{OK}"]
    objs = []
    for src in (PORT / "c/rd_static.c", OK / "ok_blas.c"):
        o = BUILD / (src.stem + "_st.o")
        run(["gcc", *cf, "-c", str(src), "-o", str(o)], check=True)
        objs.append(str(o))
    exe = BUILD / "rd_m4_static_test"
    run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", f"-I{vr}/include/eigen3", f"-I{vr}/include", f"-I{vr}/include/x86_64-linux-gnu",
         f"-I{ce}/include", f"-I{REF}/ceres-src/internal", str(PORT / "reference_tools/rd_m4_static_test.cc"), *objs, "-o", str(exe),
         f"-L{vr}/lib/x86_64-linux-gnu", "-lglog", "-lgflags", "-lm", f"-Wl,-rpath,{vr}/lib/x86_64-linux-gnu"], check=True)
    return run([str(exe)]).returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--static-test", action="store_true", help="also run rd_m4_static_test (M4 static kernels vs the real Ceres headers)")
    ap.add_argument("--tag", default="m1")
    ap.add_argument("--tag2", default="m23", help="dump run tag for the m2 module")
    ap.add_argument("--tag3", default="m23", help="dump run tag for the m3 module")
    ap.add_argument("--seq", default="MH_01_easy")
    ap.add_argument("--oracle", action="store_true", help="also replay random inputs recorded from the real classes")
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--count", default="20000")
    ap.add_argument("--sanitize", action="store_true")
    ap.add_argument("--count4", default="300", help="number of random Ceres problems per seed for the m4 oracle (rd_m4_oracle)")
    ap.add_argument("--tag4", default="m5", help="dump run tag for the m4 module (solve.bin of run_rdvio_reference.py --solve-dump)")
    ap.add_argument("--tag10", default="m10", help="tracker-log run tag for the m10 module (swt.bin of patch 0012: RDVIO_PORT_SWT_DIR)")
    ap.add_argument("--tag9", default="m9", help="initializer-log run tag for the m9 module (init.bin of patch 0011: RDVIO_PORT_INIT_DIR)")
    ap.add_argument("--tag6", default="m6", help="map-log run tag for the m6 module (map.bin of patch 0009: RDVIO_PORT_MAP_DIR, see rdvio_port/HANDOVER.md M6)")
    ap.add_argument("--tag5", default="m5", help="dump run tag for the m5 module (marg.bin, run_rdvio_reference.py --dump --dump-every marg=3 ...)")
    ap.add_argument("--chains5", default="40", help="number of random marginalisation chains per seed for the m5 oracle (rd_m5_oracle)")
    ap.add_argument("--steps5", default="4", help="marginalisations per chain for the m5 oracle")
    ap.add_argument("--modules", default="m1,m2,m3,m4,m5")
    a = ap.parse_args()
    if a.static_test or any(m != "m7" for m in a.modules.split(",")):
        BUILD.mkdir(parents=True, exist_ok=True)
    rc = static_test() if a.static_test else 0
    env = os.environ.copy()
    root = V / "deps/root/usr"
    env["LD_LIBRARY_PATH"] = f"{root}/lib/x86_64-linux-gnu:{root}/lib/x86_64-linux-gnu/openblas-pthread:{V}/deps/opencv/lib:" + env.get("LD_LIBRARY_PATH", "")
    for m in a.modules.split(","):
        if m == "m7":
            # M7 is a dependency-free leaf with its own image fixtures/build tree.
            cmd = [sys.executable, "-B", str(PORT / "reference_cv/run.py")]
            if not a.oracle:
                cmd.append("--reuse")
            if a.sanitize:
                cmd.append("--sanitize")
            rc |= run(cmd).returncode
            continue
        harness, srcs, oracle_src, xflags, xsrcs = MODULES[m]
        exe = BUILD / harness.replace(".c", "")
        flags = ["-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", *xflags] + (["-fsanitize=address,undefined", "-g"] if a.sanitize else [])
        run(["cc", *flags, "-o", str(exe), str(PORT / "c" / harness), *[str(PORT / "c" / x) for x in srcs], *[str(OK / x) for x in OK_SRC], *xsrcs, "-lm"], check=True)
        dirs = []
        dd = REPO / "runs/rdvio_port" / (a.tag if m == "m1" else a.tag2 if m == "m2" else a.tag3 if m == "m3" else a.tag4 if m == "m4" else a.tag6 if m == "m6" else a.tag9 if m == "m9" else a.tag10 if m == "m10" else a.tag5) / a.seq / "dump"
        if dd.exists():
            dirs.append(("dump " + str(dd), dd))
        if a.oracle and oracle_src:
            orc = build_oracle(oracle_src)
            for sd in a.seeds.split(","):
                od = REPO / "runs/rdvio_port" / f"oracle{m}_{sd}"
                od.mkdir(parents=True, exist_ok=True)
                if m == "m5":   # the marg.bin records are written by the dump instrumentation (patch 0007) of the real class; eval.bin by the oracle
                    e5 = dict(env, RDVIO_PORT_DUMP_DIR=str(od), RDVIO_PORT_MARG_FULL_EVERY="1",
                              RDVIO_PORT_DUMP_EVERY=",".join(f"{k}=0" for k in M5_QUIET) + ",marg=1")
                    run([str(orc), str(od), sd, a.chains5, a.steps5], env=e5, check=True)
                    dirs.append((f"oracle seed {sd} x{a.chains5} chains x{a.steps5} steps", od))
                    continue
                run([str(orc), str(od), sd, a.count4 if m == "m4" else a.count], env=env, check=True)
                dirs.append((f"oracle seed {sd} x{a.count4 if m == 'm4' else a.count}", od))
        for name, d in dirs:
            print(f"== {m}: {name}")
            r = run([str(exe), str(d)], cwd=str(REPO))
            rc |= r.returncode
        if m == "m9" and a.oracle:   # the Eigen kernels of the initializer against the real Eigen 3.4.0 (rd_m9_eigen_test.cc)
            objs = []
            for c in ("rd_qr.c", "rd_sys_eigen.c", "rd_lie.c"):
                o = BUILD / (c[:-2] + ".m9.o"); run(["cc", "-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math", "-c", str(PORT / "c" / c), "-o", str(o)], check=True); objs.append(str(o))
            for c in (STELLA / "sv_eigen_svd.c", STELLA / "sv_eigen_qr.c", OK_DIR / "ok_eigen.c"):
                o = BUILD / (c.stem + ".m9.o"); run(["cc", "-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math", "-c", str(c), "-o", str(o)], check=True); objs.append(str(o))
            et = BUILD / "rd_m9_eigen_test"
            run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", "-I" + str(V / "deps/root/usr/include/eigen3"),
                 str(PORT / "reference_tools/rd_m9_eigen_test.cc"), *objs, "-o", str(et)], check=True)
            for sd in a.seeds.split(","):
                print(f"== m9: Eigen oracle seed {sd}")
                rc |= run([str(et), sd, "20000"]).returncode
    sys.exit(rc)


if __name__ == "__main__":
    main()
