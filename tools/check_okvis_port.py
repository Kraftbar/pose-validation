#!/usr/bin/env python3
"""Build and run every okvis_port/c/check_*.c harness over the reference dumps, print a table, and exit
non-zero on any mismatch (or build/run failure). Mirrors tools/check_stella_port.py.

Each harness declares its sources on its first line:
    /* OK_PORT_SOURCES: check_x.c a.c b.c */  (paths relative to okvis_port/c/; the harness lists itself)
and must accept
    <seq_label> <fixtures_dir|-> <dump_dir> [max_records]
printing "<label>: <mismatches>/<total>" as its last stdout line (optionally preceded by indented per-kind
lines "  <kind>: <m>/<t> (...)") and exiting 0 iff mismatches == 0.

Dumps: runs/okvis_port/reference_runs/<seq>/<tag>/dumps/ produced by
    python3 tools/run_okvis_reference.py <seq> --tag run1 --dump --dump-every "prop=1,preint=1,append=5,eval=200"

Usage:
    python3 tools/check_okvis_port.py [--seqs MH_01_easy] [--tag run1[,cov,...]] [--max N] [--eigen-tests]

--tag takes a comma-separated list of run tags (each with its own dumps dir); every harness is run over every
tag. The M2 harnesses (check_ok_kin*, check_ok_cam*) read the kin_*.bin files that patch 0005 writes into the same
dumps dir as the M1 imu_*.bin files.

The M4 harness (check_ok_solve) reads solve.bin (patch 0008, run_okvis_reference.py --solve-dump); tags with only a
solve.bin (m4, s4) run just that harness, tags without one skip it. The M5 harnesses read graph.bin (check_ok_graph:
TwoPose* terms, updateLandmarks) and problem.bin (check_ok_problem: ceres::Problem program order), both from
patch 0009 / --graph-dump (m5, s5). check_ok_vigraph replays the ViGraph mutation log of patch 0010 that
the same problem.bin carries from tag m6/s6 on (see okvis_port/c/ok_vigraph.h). check_ok_vslam (module 6) replays the
ViSlamBackend entry records of patch 0011 (tags m7/s7 on) through the C backend and compares every graph call it makes.
check_ok_frontend (modules 7b-7d) additionally runs the C frontend on the logged descriptors of patch 0012 (tags m8/s8 on, the
consolidated run: tools/run_okvis_reference.py --consolidated) and compares every backend call the frontend makes; with the add-on
files ransac.bin (patch 0013) and place.bin (patch 0014) in the dumps dir the OpenGV RANSAC runs and the whole place recognition block
(DBoW2 query, verifyRecognisedPlace, attemptLoopClosure) run natively and are compared with the log. With --data ROOT
(ROOT/<seq>/gray/cam<i>.gray + ROOT/<seq>/mav0/imu0/data.csv from tools/okvis_port_images.py) BRISK (Codex's ok_brisk*) runs
natively on the images too: its keypoints and descriptors are compared with the log and replace the logged ones. With --data,
check_ok_system (module 8) also runs the whole C system (ok_system.c) from the images, the IMU csv and the run's config, compares
every backend call with the log and the two trajectory files with the reference run byte for byte; without --data it is skipped.

--eigen-tests additionally builds okvis_port/reference_tools/eigen_*_test.cc against the real Eigen 3.4.0
(external/vio/deps, flags -O2 -DNDEBUG -ffp-contract=off -fno-fast-math) and runs them (random-case,
tolerance-0 comparisons of the C evaluation-order models), and okvis_port/reference_tools/okvis_*_test.cc, which
compare the C modules against the real OKVIS2 classes (okvis_time, okvis_kinematics, okvis_cv; headers/sources from
external/vio/okvis2, OpenCV from external/vio/deps/opencv) on random inputs including edge values.
okvis_solve_dense_test / okvis_solve_sparse_test compare the M4 kernels against real Eigen and the header-only Ceres
kernels (small_blas.h, invert_psd_matrix.h from the Ceres source tree inside external/vio/okvis2). okvis_opengv_test (module 7c)
links the real OpenGV library of the reference build (test directive `OK_PORT_TEST_LIBS: opengv`: patched OpenGV headers, libopengv.a,
and the shadow adapters of okvis_port/reference_tools/shadow so that the UNMODIFIED OKVIS2 Frame*SacProblem headers run on random data).
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
C_DIR = ROOT / "okvis_port/c"
TOOLS_DIR = ROOT / "okvis_port/reference_tools"
RUNS = ROOT / "runs/okvis_port/reference_runs"
BUILD_DIR = ROOT / "runs/okvis_port/c_build"
EIGEN_INC = ROOT / "external/vio/deps/root/usr/include/eigen3"
OKVIS_SRC = ROOT / "external/vio/okvis2"
OCV = ROOT / "external/vio/deps/opencv"
TEST_SRC_RE = re.compile(r"//\s*OK_PORT_TEST_SRC:\s*(.+)")
TEST_C_RE = re.compile(r"//\s*OK_PORT_TEST_C:\s*(.+)")
TEST_LIBS_RE = re.compile(r"//\s*OK_PORT_TEST_LIBS:\s*(.+)")
CERES = ROOT / "external/vio/deps/ceres"
REF_SRC = ROOT / "runs/okvis_port/reference_build/src"
REF_BUILD = ROOT / "runs/okvis_port/reference_build/build"
VROOT = ROOT / "external/vio/deps/root/usr"
SOURCES_RE = re.compile(r"/\*\s*OK_PORT_SOURCES:\s*(.+?)\s*(?:\*/)?\s*$")
CFLAGS = ["-std=c99", "-Wall", "-Wextra", "-O2", "-ffp-contract=off", "-fno-fast-math"]


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def discover_harnesses(globs):
    out = []
    paths = sorted({p for g in globs for p in C_DIR.glob(g + ".c")})
    for path in paths:
        with open(path) as f:
            first = f.readline()
        m = SOURCES_RE.search(first)
        if not m:
            print(f"warning: {path} has no /* OK_PORT_SOURCES: ... */ first line, skipping", file=sys.stderr)
            continue
        out.append((path, m.group(1).split()))
    return out


def build_harness(path, sources):
    out = BUILD_DIR / path.stem
    run(["gcc", *CFLAGS, "-o", str(out)] + [str(C_DIR / s) for s in sources] + ["-lm"])
    return out


def has_vigraph_log(dump_dir):
    """True if problem.bin starts with the mutation records of patch 0010 (tags >= 32 appear within the first records)."""
    path = dump_dir / "problem.bin"
    if not path.exists():
        return False
    import struct
    with open(path, "rb") as f:
        for _ in range(64):
            h = f.read(12)
            if len(h) < 12:
                return False
            tag, ln = struct.unpack("<IQ", h)
            if tag >= 32:
                return True
            f.seek(ln, 1)
    return False


def has_backend_log(dump_dir):
    """True if problem.bin carries the ViSlamBackend entry records of patch 0011 (tags >= 128 within the first records)."""
    path = dump_dir / "problem.bin"
    if not path.exists():
        return False
    import struct
    with open(path, "rb") as f:
        for _ in range(200):
            h = f.read(12)
            if len(h) < 12:
                return False
            tag, ln = struct.unpack("<IQ", h)
            if tag >= 128:
                return True
            f.seek(ln, 1)
    return False


def has_frontend_log(dump_dir):
    """True if problem.bin carries the frontend inputs of patch 0012 (record 161: the BRISK descriptors of a multiframe)."""
    path = dump_dir / "problem.bin"
    if not path.exists():
        return False
    import struct
    with open(path, "rb") as f:
        for _ in range(400):
            h = f.read(12)
            if len(h) < 12:
                return False
            tag, ln = struct.unpack("<IQ", h)
            if tag == 161:
                return True
            f.seek(ln, 1)
    return False


def eigen_tests():
    """Build+run the C++ cross-checks against real Eigen / the real OKVIS2 classes. Returns list of (name, ok)."""
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    obj = BUILD_DIR / "ok_eigen.o"
    run(["gcc", *CFLAGS, "-c", str(C_DIR / "ok_eigen.c"), "-o", str(obj)])
    results = []
    for src in sorted(TOOLS_DIR.glob("eigen_*_test.cc")):
        exe = BUILD_DIR / src.stem
        run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", f"-I{EIGEN_INC}",
             str(src), str(obj), "-o", str(exe)])
        proc = subprocess.run([str(exe)], capture_output=True, text=True)
        print(proc.stdout, end="")
        results.append((src.stem, proc.returncode == 0))
    for src in sorted(TOOLS_DIR.glob("okvis_*_test.cc")):
        head = src.read_text().splitlines()[:8]
        srcs = [m.group(1).split() for l in head for m in [TEST_SRC_RE.search(l)] if m]
        cs = [m.group(1).split() for l in head for m in [TEST_C_RE.search(l)] if m]
        srcs = srcs[0] if srcs else []
        cs = cs[0] if cs else []
        libs = [m.group(1).split() for l in head for m in [TEST_LIBS_RE.search(l)] if m]
        libs = libs[0] if libs else []
        objs = []
        for c in cs:
            o = BUILD_DIR / (Path(c).stem + ".o")
            run(["gcc", *CFLAGS, "-c", str(C_DIR / c), "-o", str(o)])
            objs.append(str(o))
        exe = BUILD_DIR / src.stem
        incs = [f"-I{EIGEN_INC}", f"-I{OCV}/include/opencv4"] + [f"-I{p}" for p in sorted(OKVIS_SRC.glob("okvis_*/include"))]
        extra_link = []
        if "opengv" in libs:  # real OpenGV (BSD-3, reference build: patched headers + libopengv.a); the shadow adapters come first
            incs = [f"-I{TOOLS_DIR}/shadow", f"-I{REF_SRC}/external/opengv/include"] + incs
            extra_link += [str(REF_BUILD / "external/opengv/libopengv.a")]
        if "ceres" in libs:  # okvis_ceres classes derive from ceres::SizedCostFunction / Manifold (BSD-3, reference only)
            incs += [f"-I{CERES}/include", f"-I{OKVIS_SRC}/external/ceres-solver/internal",  # header-only internals
                     f"-I{VROOT}/include", f"-I{VROOT}/include/x86_64-linux-gnu"]
            extra_link += [f"{CERES}/lib/libceres.a", f"-L{VROOT}/lib/x86_64-linux-gnu", "-lglog", "-lgflags", "-fopenmp", "-lpthread",
                          f"-Wl,-rpath,{VROOT}/lib/x86_64-linux-gnu",
                          # ceres::internal::DenseQR / DenseCholesky reference LAPACK symbols (okvis_align4_test builds a DenseQR): OpenBLAS of the deps
                          f"{VROOT}/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so", f"-Wl,-rpath,{VROOT}/lib/x86_64-linux-gnu/openblas-pthread"]
        if "imgcodecs" in libs:  # cv::imencode / imdecode (OpenCV 4.6 of the reference)
            extra_link += ["-lopencv_imgcodecs"]
        if "zlib" in libs:  # system zlib, to write test PNGs
            extra_link += ["-lz"]
        run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", *incs, str(src),
             *[str(OKVIS_SRC / s) for s in srcs], *objs, f"-L{OCV}/lib", "-lopencv_core", "-lopencv_imgproc",
             f"-Wl,-rpath,{OCV}/lib", *extra_link, "-lm", "-o", str(exe)])
        env = dict(os.environ, LD_LIBRARY_PATH=f"{OCV}/lib:{VROOT}/lib/x86_64-linux-gnu")
        proc = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
        lines = [l for l in proc.stdout.splitlines() if l.strip()]
        print("\n".join(lines[-1:]))  # the summary line; per-section lines only on failure
        if proc.returncode != 0:
            print("\n".join(l for l in lines if not l.split()[-1].startswith("0/")))
        results.append((src.stem, proc.returncode == 0))
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seqs", default=None, help="comma-separated sequence names (default: all with dumps)")
    ap.add_argument("--tag", default="run1", help="comma-separated run tags")
    ap.add_argument("--max", type=int, default=-1)
    ap.add_argument("--runs-dir", default=None, metavar="DIR",
                    help="reference_runs root (default runs/okvis_port/reference_runs; OKVIS2-X: runs/okvis2x_port/reference_runs)")
    ap.add_argument("--eigen-tests", action="store_true")
    ap.add_argument("--gnss-e2e", action="store_true",
                    help="OKVIS2-X end-to-end: the C app (OKVIS_PORT_OKVIS2X=1) against the reference runs clean_gps_a (GNSS on) and clean_off "
                         "(tools/okvis2x_check_gnss.py --stage-b: stage a, GNSS off and stage b (robust_gps_init: true) cases; needs runs/okvis2x_port/{data,data_r1,data_r2,reference_runs} and the gray packs, ~25 min)")
    ap.add_argument("--native-solve", type=int, default=0, metavar="N",
                    help="check_ok_vslam: solve every Nth graph optimise() natively on the C graph (ok_sv_solve) instead of "
                         "applying the logged solver output, and compare the result with the log (1 = every solve)")
    ap.add_argument("--data", default=None, metavar="ROOT",
                    help="ROOT/<seq>/{gray/cam<i>.gray, mav0/imu0/data.csv} (tools/okvis_port_images.py): check_ok_frontend runs "
                         "BRISK natively on the images, check_ok_system runs the whole C system end to end")
    ap.add_argument("--harness", default="check_ok_imu*,check_ok_kin*,check_ok_cam*,check_ok_param*,check_ok_err*,check_ok_solve*,check_ok_graph*,check_ok_problem*,check_ok_vigraph*,check_ok_vslam*,check_ok_frontend*,check_ok_system*",
                    help="comma-separated globs of okvis_port/c/ harnesses that consume the reference dumps "
                         "(other modules, e.g. check_ok_brisk*, have their own dump trees/runners)")
    args = ap.parse_args()

    global RUNS
    if args.runs_dir:
        RUNS = Path(args.runs_dir).resolve()
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    harnesses = discover_harnesses(args.harness.split(","))
    if not harnesses:
        print("error: no okvis_port/c/check_*.c harnesses found", file=sys.stderr)
        return 1
    tags = args.tag.split(",")
    runs = []  # (seq, tag)
    for tag in tags:
        tag_seqs = args.seqs.split(",") if args.seqs else sorted(
            p.name for p in RUNS.iterdir() if (p / tag / "dumps").is_dir()) if RUNS.exists() else []
        runs += [(seq, tag) for seq in tag_seqs]
    if not runs:
        print(f"error: no dumps under {RUNS}/<seq>/{args.tag}/dumps -- run tools/run_okvis_reference.py --dump",
              file=sys.stderr)
        return 1

    built = [(p.stem, build_harness(p, srcs)) for p, srcs in harnesses]
    rows = []  # (harness, seq, kind lines, m, t, ok)
    any_fail = False
    for name, exe in built:
        for seq0, tag in runs:
            seq = seq0 if len(tags) == 1 else f"{seq0}/{tag}"
            dump_dir = RUNS / seq0 / tag / "dumps"
            # every harness needs its own dump files; tags recorded without them are skipped
            if name.startswith(("check_ok_err", "check_ok_param")) and not list(dump_dir.glob("err_*.bin")):
                continue  # tag recorded before patch 0007: no M3 dumps
            if name.startswith("check_ok_solve") and not (dump_dir / "solve.bin").exists():
                continue  # tag without the M4 solver dump (patch 0008, --solve-dump)
            if name.startswith("check_ok_graph") and not (dump_dir / "graph.bin").exists():
                continue  # tag without the M5 graph dump (patch 0009, --graph-dump)
            if name.startswith("check_ok_problem") and not (dump_dir / "problem.bin").exists():
                continue  # tag without the M5 Problem log (patch 0009, --graph-dump)
            if name.startswith("check_ok_vigraph") and not has_vigraph_log(dump_dir):
                continue  # tag recorded before patch 0010 (no ViGraph mutation records in problem.bin)
            if name.startswith("check_ok_vslam") and not has_backend_log(dump_dir):
                continue  # tag recorded before patch 0011 (no backend entry records in problem.bin)
            if name.startswith(("check_ok_frontend", "check_ok_system")) and not has_frontend_log(dump_dir):
                continue  # tag recorded before patch 0012 (no descriptor records in problem.bin)
            if name.startswith("check_ok_system") and not (args.data and (dump_dir / "place.bin").exists()):
                continue  # the end-to-end run needs the images / IMU csv (--data) and the place-recognition log
            if not name.startswith(("check_ok_solve", "check_ok_graph", "check_ok_problem", "check_ok_vigraph", "check_ok_vslam", "check_ok_frontend", "check_ok_system")) and not list(dump_dir.glob("imu_*.bin")):
                continue  # solver/graph-only tag (m4/s4/m5/s5): no M1-M3 dumps
            cmd = [str(exe), seq, "-", str(dump_dir)] + ([str(args.max)] if args.max > 0 else [])
            env = dict(os.environ)
            if name.startswith(("check_ok_vslam", "check_ok_frontend", "check_ok_system")) and args.native_solve > 0:
                env["OK_NATIVE_SOLVE"] = str(args.native_solve)
            if name.startswith(("check_ok_frontend", "check_ok_system")) and args.data:
                seq_data = Path(args.data) / seq0
                if not (seq_data / "gray/cam0.gray").exists() or not (seq_data / "mav0/imu0/data.csv").exists():
                    print(f"error: {seq_data}/gray or mav0/imu0 missing -- run tools/okvis_port_images.py {seq0}", file=sys.stderr)
                    return 1
                env["OK_BRISK_IMAGES"] = str(seq_data / "gray")
                env["OK_SYSTEM_DATA"] = str(seq_data)
            proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
            lines = proc.stdout.rstrip().splitlines()
            last = lines[-1] if lines else ""
            m = re.match(r".*:\s*(\d+)/(\d+)\s*$", last)
            if not m:
                print(proc.stdout)
                print(proc.stderr, file=sys.stderr)
                rows.append((name, seq, [], None, None, False))
                any_fail = True
                continue
            mism, total = int(m.group(1)), int(m.group(2))
            ok = mism == 0 and total > 0 and proc.returncode == 0  # nothing compared == failure
            any_fail |= not ok
            rows.append((name, seq, [l for l in lines[:-1] if l.startswith("  ")], mism, total, ok))
            if not ok and proc.stderr:
                print(proc.stderr[-2000:], file=sys.stderr)

    print()
    print(f"{'harness':<16} {'seq':<18} {'mismatches':>12} {'compared':>14}  status")
    print("-" * 74)
    for name, seq, kinds, mism, total, ok in rows:
        for k in kinds:
            print(f"  {k.strip()}")
        print(f"{name:<16} {seq:<18} {str(mism) if mism is not None else '?':>12} "
              f"{str(total) if total is not None else '?':>14}  {'PASS' if ok else 'FAIL'}")

    if args.eigen_tests:
        print("\nEigen cross-checks (real Eigen 3.4.0 vs C evaluation-order models):")
        for name, ok in eigen_tests():
            print(f"  {name:<28} {'PASS' if ok else 'FAIL'}")
            any_fail |= not ok
    if args.gnss_e2e:
        print("\nOKVIS2-X GNSS end to end (C app vs deterministic OKVIS2-X reference):")
        any_fail |= subprocess.run([sys.executable, str(ROOT / "tools/okvis2x_check_gnss.py"), "--stage-b"]).returncode != 0
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
