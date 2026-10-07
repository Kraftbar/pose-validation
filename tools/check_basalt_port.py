#!/usr/bin/env python3
"""basalt_port check runner. Module m0 (dump harness): build basalt_port/c/check_bs_dump.c, make sure the reference dumps exist
(running the instrumented reference when they do not), run the reader on every dump dir, and enforce

  * the instrumented trajectory (dump on, and with --also-off dump off) is byte-identical to the reference hash,
  * the reader parses every record exactly (<label>: 0/<total>),
  * the executed-path counters answer M0 open question (a): float estimator only, ABS_QR linearisation only,
    marginalisation only through MargHelper::marginalizeHelperSqrtToSqrt, no SC/REL_SC, no nullspace/eigen debug path.

Harness convention (as in tools/check_okvis_port.py): each harness source declares its sources on its first line
    /* BS_PORT_SOURCES: check_bs_dump.c [more.c ...] */   (paths relative to basalt_port/c/)
and takes   <label> <dump_dir>   and prints "<label>: <mismatches>/<total>" as its last stdout line.

Usage:
    python3 tools/check_basalt_port.py [--modules m0] [--tags all,full] [--regen] [--also-off] [--seq MH_01_easy]

Dumps: runs/basalt_port/dumps/<tag>/{flow,imu,iter,marg,summary}.bin, written by
    BASALT_PORT_DUMP_DIR=<dir> [BASALT_PORT_DUMP_FULL=1] [BASALT_PORT_DUMP_EVERY=flow=N,imu=N,iter=N,marg=N] basalt_ref_driver ...
(layouts: basalt_port/c/check_bs_dump.c header comment, basalt_port/PLAN.md).
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
C_DIR = ROOT / "basalt_port/c"
BUILD = ROOT / "runs/basalt_port/c_build"
DUMPS = ROOT / "runs/basalt_port/dumps"
OUT = ROOT / "runs/basalt_port/out"
DRIVER = ROOT / "runs/basalt_port/reference_build/build/basalt_ref_driver"
LIBS = [ROOT / "external/vio/deps/root/usr/lib/x86_64-linux-gnu", ROOT / "external/vio/deps/opencv/lib"]
CALIB = ROOT / "external/vio/basalt_src/data/euroc_ds_calib.json"
CONFIG = ROOT / "external/vio/basalt_src/data/euroc_config.json"
SEQ_ROOT = ROOT / "runs/okvis2x_port/data"
EXPECTED_SHA = {"MH_01_easy": "a7d3e7a334591ce25157aed3777b17967269e1d67a2fa03c489cda11be32f1bf",
                # reference_build trajectories of the other EuRoC sequences (M9; the C driver matches each byte for byte)
                "V1_01_easy": "279b7f3d310ac45506cd4910198690eeb92823ff5cbf11af92eb0d173e109039",
                "V1_03_difficult": "153a7eca190110ba515ab6d72eafbb31cfee6757ae7a220cd74df29f83580d2e",
                "MH_03_medium": "f74ca186c19ea169e542359b7aeb4a162f6c1963c01120c845cf71463fe4c828",
                "MH_02_easy": "ff00c12ec798875d3b4d488b09b1d8465dfa4bb924cda87e9efe9030e98e907f",
                "MH_04_difficult": "59a37bfcf17e17c2c9739a09fa513ae76caeb9bc5146f98a6f63ec7da8d2adb7",
                "MH_05_difficult": "900b60f3c88990a5fb2ff8be9af5e8ed5861b72b1fa7e54267a643bb2eb3e51b",
                "V1_02_medium": "480b1028ca561522c215f1acaec86d1198042c66f05df4be84b73dcd65cf89a9",
                "V2_01_easy": "0b954057d84b68dfdcd9fb58f2ab8639fa23f7d5e9b4e412375e98471aa0987b",
                "V2_02_medium": "0b723d19c4a3af9b32dc89397a7b2b633883d355d6079ddfadc7a99de8c8403d",
                "V2_03_difficult": "a09635d44b4f68617f10c52172faa46e961fdd093599e97e839db552da0a70eb"}
CFLAGS = ["-std=c99", "-Wall", "-Wextra", "-O2", "-ffp-contract=off", "-fno-fast-math"]
SOURCES_RE = re.compile(r"/\*\s*BS_PORT_SOURCES:\s*(.+?)\s*(?:\*/)?\s*$")
# dump tags: name -> env of the instrumented run
TAGS = {
    "all": {},                                   # every record of every stream, digests only (~55 MB)
    "full": {"BASALT_PORT_DUMP_FULL": "1",       # sampled, with dense H,b,inc / Q2Jp,Q2r,prior,H_new,b_new arrays (~33 MB)
             "BASALT_PORT_DUMP_EVERY": "flow=50,imu=50,iter=40,marg=15"},
}
# executed-path facts the M0 answers rely on: counter -> predicate on its value
PATH_FACTS = {
    "est_ctor_float": lambda v: v == 1, "est_ctor_double": lambda v: v == 0,
    "lin_create_abs_qr": lambda v: v > 0, "lin_create_abs_sc": lambda v: v == 0, "lin_create_rel_sc": lambda v: v == 0,
    "marg_helper_sqrt_to_sqrt": lambda v: v > 0, "marg_helper_sq_to_sqrt": lambda v: v == 0, "marg_helper_sq_to_sq": lambda v: v == 0,
    "log_marg_nullspace": lambda v: v == 0, "ldlt_retry": lambda v: v == 0, "lm_invalid": lambda v: v == 0,
}


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def ref_env(extra):
    e = os.environ.copy()
    e["LD_LIBRARY_PATH"] = ":".join(str(p) for p in LIBS) + (":" + e["LD_LIBRARY_PATH"] if e.get("LD_LIBRARY_PATH") else "")
    for k in ("BASALT_PORT_DUMP_DIR", "BASALT_PORT_DUMP_FULL", "BASALT_PORT_DUMP_EVERY", "BASALT_EXP_ORDER"):
        e.pop(k, None)
    e.update(extra)
    return e


def run_reference(seq, out, env_extra):
    cmd = [str(DRIVER), "--dataset-path", str(SEQ_ROOT / seq), "--cam-calib", str(CALIB), "--config-path", str(CONFIG), "--out", str(out)]
    t0 = time.time()
    subprocess.run(cmd, env=ref_env(env_extra), check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("  reference run %.0f s -> %s" % (time.time() - t0, out), flush=True)
    return sha256(out)


def build_reader(src):
    first = src.read_text().splitlines()[0]
    m = SOURCES_RE.search(first)
    files = m.group(1).split() if m else [src.name]
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / src.stem
    run(["gcc"] + CFLAGS + [str(C_DIR / f) for f in files] + ["-o", str(exe), "-lm"])
    return exe


def m0(args):
    ok = True
    seq = args.seq
    if not DRIVER.exists():
        sys.exit("reference driver missing: python3 tools/build_basalt_reference.py")
    expect = EXPECTED_SHA.get(seq)
    OUT.mkdir(parents=True, exist_ok=True)
    if args.also_off:
        h = run_reference(seq, OUT / f"m0_off_{seq}.tum", {})
        print("  dump OFF trajectory sha256 %s %s" % (h, "OK" if h == expect else "MISMATCH (expected %s)" % expect))
        ok &= h == expect
    readers = [build_reader(p) for p in sorted(C_DIR.glob("check_bs_dump*.c"))]
    for tag in args.tags.split(","):
        d = DUMPS / tag
        if args.regen or not (d / "summary.bin").exists():
            d.mkdir(parents=True, exist_ok=True)
            for f in d.glob("*.bin"):
                f.unlink()
            env = dict(TAGS[tag])
            env["BASALT_PORT_DUMP_DIR"] = str(d)
            h = run_reference(seq, OUT / f"m0_{tag}_{seq}.tum", env)
            print("  dump ON (%s) trajectory sha256 %s %s" % (tag, h, "OK" if h == expect else "MISMATCH (expected %s)" % expect))
            ok &= h == expect
        for exe in readers:
            r = subprocess.run([str(exe), f"{seq}/{tag}", str(d)], capture_output=True, text=True)
            sys.stdout.write(r.stdout)
            sys.stderr.write(r.stderr)
            last = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ""
            good = r.returncode == 0 and re.search(r": 0/\d+$", last) is not None
            print("  reader %s on %s: %s" % (exe.name, tag, "PASS" if good else "FAIL"))
            ok &= good
            counters = {m.group(1): int(m.group(2)) for m in re.finditer(r"^\s{4}(\w+)\s+(\d+)$", r.stdout, re.M)}
            for k, pred in PATH_FACTS.items():
                if k not in counters or not pred(counters[k]):
                    print("  PATH FACT FAILED: %s = %s" % (k, counters.get(k)))
                    ok = False
    return ok


def m1(args):
    """M1: Sophus/basalt-headers Lie kernels (bs_lie.c, bs_eigenf.c) vs the real classes, tolerance 0 (bs_lie_test.cc)."""
    d = ROOT / "external/vio/basalt_src/_deps"
    out = ROOT / "runs/basalt_port/m1"
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    for san, suffix in ([("", "")] + ([("-fsanitize=address,undefined -fno-sanitize-recover=undefined", "_asan")] if args.m1_asan else [])):
        sf = san.split()
        objs = []
        for c in ("bs_eigenf", "bs_lie"):
            o = out / (c + suffix + ".o")
            run(["gcc", "-std=c99", "-Wall", "-Wextra", "-O2", "-ffp-contract=off", "-fno-fast-math", *sf, "-c", str(C_DIR / (c + ".c")), "-o", str(o)])
            objs.append(str(o))
        exe = out / ("bs_lie_test" + suffix)
        run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", "-DEIGEN_DONT_PARALLELIZE", *sf,
             f"-I{C_DIR}", f"-I{ROOT}/runs/basalt_port/reference_build/src/include", f"-I{d}/basalt-headers/include", f"-I{d}/Sophus",
             f"-I{d}/cereal/include", f"-I{d}/magic_enum/include", "-isystem", str(ROOT / "external/vio/deps/root/usr/include/eigen3"),
             "-isystem", str(ROOT / "external/vio/deps/root/usr/include"), "-isystem", str(ROOT / "external/vio/deps/root/usr/include/nlohmann"),
             str(ROOT / "basalt_port/reference_tools/bs_lie_test.cc"), *objs, "-o", str(exe)])
        for seed in args.m1_seeds.split(","):
            n = args.m1_cases // (2 if suffix else 1)
            r = subprocess.run([str(exe), seed, str(n)], capture_output=True, text=True)
            last = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ""
            good = r.returncode == 0 and re.search(r": 0/\d+$", last) is not None
            print("  bs_lie_test%s seed %s x %d: %s  [%s]" % (suffix, seed, n, last, "PASS" if good else "FAIL"))
            if not good:
                sys.stdout.write(r.stdout[-3000:] + r.stderr[-2000:])
            ok &= good
    return ok


def _m23_build(name, c_src, test_src, san, extra=()):
    """gcc the C port, g++ the oracle with the reference flags; returns the oracle exe (runs/basalt_port/m23)."""
    d = ROOT / "runs/basalt_port/reference_build/src"
    out = ROOT / "runs/basalt_port/m23"
    out.mkdir(parents=True, exist_ok=True)
    sf = san.split()
    suffix = "_asan" if san else ""
    obj = out / (name + suffix + ".o")
    exe = out / (name + "_test" + suffix)
    run(["gcc", "-std=c99", "-Wall", "-Wextra", "-O2", "-ffp-contract=off", "-fno-fast-math", *sf, *extra, f"-I{C_DIR}", "-c", str(C_DIR / c_src), "-o", str(obj)])
    run(["g++", "-std=c++17", "-O2", "-DNDEBUG", "-ffp-contract=off", "-fno-fast-math", "-Wno-deprecated-declarations", "-DFMT_HEADER_ONLY", "-DEIGEN_DONT_PARALLELIZE",
         *sf, *extra, f"-I{C_DIR}", f"-I{d}/include", f"-I{d}/_deps/basalt-headers/include", f"-I{d}/_deps/Sophus", f"-I{d}/_deps/cereal/include",
         f"-I{d}/_deps/magic_enum/include", f"-I{d}/thirdparty/ros/include", "-isystem", str(ROOT / "external/vio/deps/root/usr/include/eigen3"),
         "-isystem", str(ROOT / "runs/basalt_port/reference_build/deps_root/usr/include"), "-isystem", str(ROOT / "external/vio/deps/root/usr/include"),
         "-isystem", str(ROOT / "external/vio/deps/opencv/include/opencv4"), str(ROOT / "basalt_port/reference_tools" / test_src), str(obj), "-o", str(exe)])
    return exe


def _m23_run(exe, argv, label, pat=r": 0/\d+$"):
    r = subprocess.run([str(exe), *argv], capture_output=True, text=True)
    last = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ""
    good = r.returncode == 0 and re.search(pat, last) is not None
    print("  %s: %s  [%s]" % (label, last, "PASS" if good else "FAIL"))
    if not good:
        sys.stdout.write(r.stdout[-3000:] + r.stderr[-2000:])
    return good


def m2(args):
    """M2: double-sphere camera (bs_cam.c) vs basalt-headers DoubleSphereCamera<float/double>, tolerance 0 (bs_cam_test.cc), + FLOW-dump replay."""
    ok = True
    for san in ("", "-fsanitize=address,undefined -fno-sanitize-recover=undefined"):
        exe = _m23_build("bs_cam", "bs_cam.c", "bs_cam_test.cc", san)
        seeds = args.m23_seeds.split(",") if not san else args.m23_seeds.split(",")[:1]
        for seed in seeds:
            ok &= _m23_run(exe, [seed, str(args.m23_cases)], "bs_cam_test%s seed %s x %d" % ("_asan" if san else "", seed, args.m23_cases))
        if not san:
            ok &= _m23_run(exe, ["1", str(args.m23_cases), "sens"], "bs_cam_test sensitivity (1-ulp perturbed fx must be caught)", r"bundles differ")
            flow = DUMPS / "all/flow.bin"
            if flow.exists():
                ok &= _m23_run(exe, ["replay", str(flow)], "bs_cam_test replay all/flow.bin")
    return ok


def m3(args):
    """M3: IMU preintegration (bs_imu.c) vs IntegratedImuMeasurement<float> + ImuBlock::linearizeImu, tolerance 0 (bs_imu_test.cc), + IMU dump replay."""
    ok = True
    for san in ("", "-fsanitize=address,undefined -fno-sanitize-recover=undefined"):
        exe = _m23_build("bs_imu", "bs_imu.c", "bs_imu_test.cc", san, extra=("-DBS_IMU_COV",))
        n = args.m23_cases if not san else args.m23_cases // 4
        seeds = args.m23_seeds.split(",") if not san else args.m23_seeds.split(",")[:1]
        for seed in seeds:
            ok &= _m23_run(exe, [seed, str(n)], "bs_imu_test%s seed %s x %d" % ("_asan" if san else "", seed, n))
        if not san:
            ok &= _m23_run(exe, ["1", str(n), "sens"], "bs_imu_test sensitivity (1-ulp perturbed sample must be caught)", r"bs_imu sens: [1-9]\d*/\d+$")
            ok &= _m23_run(exe, ["prim", "1", str(n)], "bs_imu_test prim (GEBP / LDLT / triangular solve vs Eigen)")
            imu = DUMPS / "all/imu.bin"
            if imu.exists():
                ok &= _m23_run(exe, ["replay", str(imu)], "bs_imu_test replay all/imu.bin")
            # ODR hazard (PLAN.md 4d): the oracle must use the float-sqrt copy of compute_sqrt_cov_inv<float> (sqrtss), like the reference driver
            dis = subprocess.run(["objdump", "-d", "--no-show-raw-insn", "-C", str(exe)], capture_output=True, text=True).stdout
            m = re.search(r"IntegratedImuMeasurement<float>::compute_sqrt_cov_inv\(\) const>:\n(.*?)\n\n", dis, re.S)
            good = bool(m) and "sqrtss" in m.group(1) and "cvtss2sd" not in m.group(1)
            print("  oracle compute_sqrt_cov_inv<float> uses sqrtss (no cvtss2sd): %s" % ("PASS" if good else "FAIL"))
            ok &= good
    return ok


def m4(args):
    """M4: image path (bs_image.c / bs_fast.c) vs real basalt ManagedImagePyr / detectKeypoints / cv::FAST / cv::imread, tolerance 0 (bs_image_test.cc), + FLOW replay."""
    m4d = ROOT / "runs/basalt_port/m4"
    seq = ROOT / "runs/okvis2x_port/data" / args.seq
    ok = True
    for suffix, san in (("", []), ("_asan", ["-fsanitize=address,undefined", "-fno-sanitize-recover=undefined"])):
        r = subprocess.run(["sh", str(m4d / "build.sh"), *san], env={**os.environ, "SUFFIX": suffix}, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            return False
        exe = m4d / ("bs_image_test" + suffix)
        cases = [("sort", ["sort", "1"]), ("fast", ["fast", "1"]), ("image", ["image", str(seq), "10" if not san else "100"]), ("detect", ["detect", str(seq), "10" if not san else "120"])]
        if not san:
            cases += [("sort seed 2", ["sort", "2"]), ("fast seed 2", ["fast", "2"])]
        for label, argv in cases:
            ok &= _m23_run(exe, argv, "bs_image_test%s %s" % (suffix, label))
        flow = DUMPS / "all/flow.bin"
        if flow.exists() and seq.exists():
            ok &= _m23_run(exe, ["replay", str(flow), str(seq)], "bs_image_test%s replay all/flow.bin (image hashes + new keypoints)" % suffix)
        if not san:
            ok &= _m23_run(exe, ["sens"], "bs_image_test sensitivity (wrong threshold / stable sort must be caught)", r"bs_image sens: 0/\d+$")
    return ok


def m5(args):
    """M5: optical-flow frontend (bs_patch.c / bs_flow.c) vs the real FrameToFrameOpticalFlow<float, Pattern51> / OpticalFlowPatch / Image::interp / SE2 / Eigen,
    tolerance 0 (bs_flow_test.cc), + full FLOW-dump replay (all frames, ids + 6 floats per keypoint). Build: runs/basalt_port/m5/build.sh."""
    m5d = ROOT / "runs/basalt_port/m5"
    seq = ROOT / "runs/okvis2x_port/data" / args.seq
    ok = True
    for suffix, san in (("", []), ("_asan", ["-fsanitize=address,undefined", "-fno-sanitize-recover=undefined"])):
        r = subprocess.run(["sh", str(m5d / "build.sh"), *san], env={**os.environ, "SUFFIX": suffix}, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            return False
        exe = m5d / ("bs_flow_test" + suffix)
        sd = ["seed", "2"] if san else ["seed", "1"]
        cases = [("eigen", ["eigen", *sd]), ("interp", ["interp", *sd]), ("calib", ["calib", str(CALIB), str(CONFIG)]),
                 ("patch", ["patch", str(seq), *sd]), ("synth", ["synth", *sd, "4" if san else "6"]),
                 ("flow 150 real frames", ["flow", str(seq), "1500", "150" if san else "300"])]
        if not san:
            cases += [("eigen seed 2", ["eigen", "seed", "2"]), ("patch seed 2", ["patch", str(seq), "seed", "2"]), ("synth seed 2", ["synth", "seed", "2", "6"]),
                      ("flow frames 0..299", ["flow", str(seq), "0", "300"]), ("flow frames 3300..3681", ["flow", str(seq), "3300", "400"])]
        for label, argv in cases:
            ok &= _m23_run(exe, argv, "bs_flow_test%s %s" % (suffix, label))
        flow = DUMPS / "all/flow.bin"
        if flow.exists() and seq.exists():
            argv = ["replay", str(flow), str(seq), str(CALIB), str(CONFIG)] + (["600"] if san else [])
            ok &= _m23_run(exe, argv, "bs_flow_test%s replay all/flow.bin (%s frames, ids + keypoints bit-exact)" % (suffix, "first 600" if san else "all 3682"))
    return ok


def _m6_run(argv, label, pat, cwd=None, env=None):
    r = subprocess.run([str(a) for a in argv], capture_output=True, text=True, cwd=cwd, env=env)
    last = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ""
    good = r.returncode == 0 and re.search(pat, last) is not None
    print("  %s: %s  [%s]" % (label, last, "PASS" if good else "FAIL"))
    if not good:
        sys.stdout.write(r.stdout[-3000:] + r.stderr[-2000:])
    return good


def m6(args):
    """M6: libstdc++ unordered-container order model (bs_hashorder.c), LandmarkDatabase (bs_lmdb.c), LinearizationAbsQR + estimator-side error /
    marginalisation-prior evaluation (bs_linabsqr.c) vs the real classes, tolerance 0 (bs_hashorder_test.cc, bs_linabsqr_test.cc), + replay of the
    patch-0004 dump (m6.bin: lmdb op log, order digests, PROBLEM records) against iter.bin / marg.bin. Artefacts: runs/basalt_port/m6/."""
    m6d = ROOT / "runs/basalt_port/m6"
    ok = True
    # ---- builds (plain + ASan/UBSan C port; the C++ oracle is never sanitised, see build_la.sh)
    for script, env in (("build_hash.sh", {}), ("build_hash.sh", {"SUF": "_asan", "OUT": "bs_hashorder_test_asan"}),
                        ("build_la.sh", {}), ("build_la.sh", {"SUF": "_asan", "OUT": "bs_linabsqr_test_asan", "LDX": "-fsanitize=address,undefined"})):
        san = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"] if env.get("SUF") else []
        r = subprocess.run(["sh", str(m6d / script), *san], env={**os.environ, **env}, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            return False
    seeds = args.m6_seeds.split(",")
    for san, sfx in (("", ""), ("asan", "_asan")):
        div = 4 if san else 1
        for sd in (seeds if not san else seeds[:1]):
            ok &= _m6_run([m6d / ("bs_hashorder_test" + sfx), sd, 1500 // div], "bs_hashorder_test%s seed %s x %d" % (sfx, sd, 1500 // div), r"mismatches 0$")
        exe = m6d / ("bs_linabsqr_test" + sfx)
        for sd in (seeds if not san else seeds[:1]):
            ok &= _m6_run([exe, "lmdb", sd, 3000 // div], "bs_linabsqr_test%s lmdb seed %s x %d" % (sfx, sd, 3000 // div), r", 0 mismatches$")
            ok &= _m6_run([exe, "prim", sd, 3000 // div], "bs_linabsqr_test%s prim seed %s x %d (GEMM/GEMV + landmark-block statements)" % (sfx, sd, 3000 // div), r", 0 mismatches$")
            ok &= _m6_run([exe, "problem", sd, 100 // div], "bs_linabsqr_test%s problem seed %s x %d (LinearizationAbsQR + error / prior functions)" % (sfx, sd, 100 // div), r", 0 mismatches$")
    # ---- dump with patch 0004 (separate build tree: the M0 reference_build is not touched)
    drv = m6d / "ref0004/build/basalt_ref_driver"
    dd = m6d / "dumps/m6full"
    if args.m6_regen or not (dd / "m6.bin").exists() or not (dd / "summary.bin").exists():
        if args.m6_regen or not drv.exists():
            run(["python3", str(ROOT / "tools/build_basalt_reference.py"), "--jobs", "3"], env={**os.environ, "BASALT_REF_ROOT": str(m6d / "ref0004")})
        (m6d / "out").mkdir(parents=True, exist_ok=True)
        dd.mkdir(parents=True, exist_ok=True)
        for f in dd.glob("*.bin"):
            f.unlink()
        cmd = [str(drv), "--dataset-path", str(ROOT / "runs/okvis2x_port/data" / args.seq), "--cam-calib", str(CALIB), "--config-path", str(CONFIG)]
        procs = []
        for name, extra in (("off", {}), ("full", {"BASALT_PORT_DUMP_DIR": str(dd), "BASALT_PORT_DUMP_FULL": "1", "BASALT_PORT_M6": "1",
                                                   "BASALT_PORT_DUMP_EVERY": "flow=50,imu=50,iter=10,marg=5"})):
            procs.append((name, subprocess.Popen(cmd + ["--out", str(m6d / "out" / f"m6_{name}.tum")], env=ref_env(extra),
                                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)))
        for name, pr in procs:
            pr.wait()
            h = sha256(m6d / "out" / f"m6_{name}.tum")
            good = h == EXPECTED_SHA.get(args.seq)
            print("  reference with patch 0004, dump %s: trajectory sha256 %s %s" % (name, h, "OK" if good else "MISMATCH"))
            ok &= good
    exe = BUILD / "check_bs_dump"
    run(["gcc"] + CFLAGS + [str(C_DIR / "check_bs_dump.c"), "-o", str(exe), "-lm"])
    ok &= _m6_run([exe, "m6full", dd], "reader check_bs_dump on m6full (flow/imu/iter/marg/m6)", r": 0/\d+$")
    ok &= _m6_run([m6d / "bs_linabsqr_test", "replay", dd], "bs_linabsqr_test replay m6full (ORDER / UNCONN / PROBLEM / ITER_STEP / MARG)", r", 0 mismatches$")
    ok &= _m6_run([m6d / "bs_linabsqr_test_asan", "replay", dd, "60"], "bs_linabsqr_test_asan replay m6full (first 60 problems)", r", 0 mismatches$")
    if args.m6_mutations:
        r = subprocess.run(["sh", str(m6d / "mutate_hash.sh")], capture_output=True, text=True)
        sys.stdout.write(r.stdout)
        r = subprocess.run(["sh", str(m6d / "mutate_la.sh"), "1", "40"], capture_output=True, text=True)
        sys.stdout.write(r.stdout)
    return ok


def _m78_build(m78d, san):
    """build.sh builds the C port objects + both oracles (bs_vio_opt_test, bs_marg_test); the sanitised build links the C port only."""
    ok = True
    for test in ("bs_vio_opt_test", "bs_marg_test"):
        env = {**os.environ, "TEST": test}
        flags = []
        if san:
            env.update({"SUF": "_asan", "OUT": test + "_asan", "LDX": "-fsanitize=address,undefined"})
            flags = ["-g", "-fsanitize=address,undefined", "-fno-sanitize-recover=all"]
        r = subprocess.run(["sh", str(m78d / "build.sh"), *flags], env=env, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            ok = False
    return ok


def _m78_dump(args, m78d, name, every):
    """reference with patches 0001-0005 (runs/basalt_port/m78/ref0005): trajectory hash with the dump off and on, dump in m78d/dumps/<name>."""
    drv = m78d / "ref0005/build/basalt_ref_driver"
    dd = m78d / "dumps" / name
    ok = True
    if args.m78_regen or not drv.exists():
        run(["python3", str(ROOT / "tools/build_basalt_reference.py"), "--jobs", "3"], env={**os.environ, "BASALT_REF_ROOT": str(m78d / "ref0005")})
    if args.m78_regen or not (dd / "m78.bin").exists() or not (dd / "summary.bin").exists():
        (m78d / "out").mkdir(parents=True, exist_ok=True)
        dd.mkdir(parents=True, exist_ok=True)
        for f in dd.glob("*.bin"):
            f.unlink()
        cmd = [str(drv), "--dataset-path", str(ROOT / "runs/okvis2x_port/data" / args.seq), "--cam-calib", str(CALIB), "--config-path", str(CONFIG)]
        procs = []
        for tag, extra in (("off", {}), (name, {"BASALT_PORT_DUMP_DIR": str(dd), "BASALT_PORT_DUMP_FULL": "1", "BASALT_PORT_M6": "1", "BASALT_PORT_M78": "1",
                                               "BASALT_PORT_DUMP_EVERY": every})):
            procs.append((tag, subprocess.Popen(cmd + ["--out", str(m78d / "out" / f"m78_{tag}.tum")], env=ref_env({**extra, "BASALT_PORT_M6": extra.get("BASALT_PORT_M6", "0"),
                                                "BASALT_PORT_M78": extra.get("BASALT_PORT_M78", "0")}), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)))
        for tag, pr in procs:
            pr.wait()
            h = sha256(m78d / "out" / f"m78_{tag}.tum")
            good = h == EXPECTED_SHA.get(args.seq)
            print("  reference with patch 0005, dump %s: trajectory sha256 %s %s" % (tag, h, "OK" if good else "MISMATCH"))
            ok &= good
    exe = BUILD / "check_bs_dump"
    run(["gcc"] + CFLAGS + [str(C_DIR / "check_bs_dump.c"), "-o", str(exe), "-lm"])
    ok &= _m6_run([exe, "m78" + name, dd], "reader check_bs_dump on %s (flow/imu/iter/marg/m6/m78)" % name, r": 0/\d+$")
    return ok, dd


def m7(args):
    """M7: dense LDLT (bs_vio_opt.c) vs Eigen::LDLT<Ref<MatX>>, the lambda update, random estimator states vs the real SqrtKeypointVioEstimator<float>::optimize(),
    and the replay of the patch-0005 dump (every OPT_BEGIN / ITER_STEP / OPT_END of the sampled calls; --m78-full: all 19,803 LM steps).  Artefacts: runs/basalt_port/m78/."""
    m78d = ROOT / "runs/basalt_port/m78"
    ok = _m78_build(m78d, False) and _m78_build(m78d, True)
    if not ok:
        return False
    seeds = args.m78_seeds.split(",")
    for san, sfx in (("", ""), ("asan", "_asan")):
        div = 4 if san else 1
        exe = m78d / ("bs_vio_opt_test" + sfx)
        for sd in (seeds if not san else seeds[:1]):
            ok &= _m6_run([exe, "ldlt", sd, 3000 // div], "bs_vio_opt_test%s ldlt seed %s x %d (LDLT factor + solve)" % (sfx, sd, 3000 // div), r", 0 mismatches$", cwd=m78d)
            ok &= _m6_run([exe, "optimize", sd, 600 // div], "bs_vio_opt_test%s optimize seed %s x %d (vs the real optimize())" % (sfx, sd, 600 // div), r", 0 mismatches$", cwd=m78d)
        if not san:
            ok &= _m6_run([exe, "lambda", "1", 100000], "bs_vio_opt_test lambda x 100000", r", 0 mismatches$", cwd=m78d)
            ok &= _m6_run([exe, "prior", "1", 200], "bs_vio_opt_test prior x 200 (constructor prior vs bs_vio_init_marg_prior)", r", 0 mismatches$", cwd=m78d)
    name, every = ("full", "flow=50,imu=50,iter=1,marg=1") if args.m78_full else ("s10", "flow=50,imu=50,iter=10,marg=5")
    good, dd = _m78_dump(args, m78d, name, every)
    ok &= good
    ok &= _m6_run([m78d / "bs_vio_opt_test", "replay", dd], "bs_vio_opt_test replay %s (OPT_BEGIN / ITER_STEP / OPT_END, lambda_vee)" % name, r", 0 mismatches$", cwd=m78d)
    ok &= _m6_run([m78d / "bs_vio_opt_test_asan", "replay", dd, "60"], "bs_vio_opt_test_asan replay %s (first 60 optimize() calls)" % name, r", 0 mismatches$", cwd=m78d)
    if args.m78_mutations:
        r = subprocess.run(["python3", str(m78d / "mutate.py"), "ldlt"], capture_output=True, text=True, cwd=m78d)
        sys.stdout.write(r.stdout)
    if args.m78_full and not args.m78_keep:
        import shutil
        shutil.rmtree(dd, ignore_errors=True)
    return ok


def m8(args):
    """M8: MargHelper<float>::marginalizeHelperSqrtToSqrt (bs_marg.c) and marginalize() vs the real classes, + replay of every MARG record of the patch-0005 dump
    (kf selection, aom, idx sets, H_new / b_new, final prior, state / lmdb after).  Artefacts: runs/basalt_port/m78/."""
    m78d = ROOT / "runs/basalt_port/m78"
    ok = _m78_build(m78d, False) and _m78_build(m78d, True)
    if not ok:
        return False
    seeds = args.m78_seeds.split(",")
    for san, sfx in (("", ""), ("asan", "_asan")):
        div = 4 if san else 1
        exe = m78d / ("bs_marg_test" + sfx)
        for sd in (seeds if not san else seeds[:1]):
            ok &= _m6_run([exe, "helper", sd, 1500 // div], "bs_marg_test%s helper seed %s x %d (marginalizeHelperSqrtToSqrt)" % (sfx, sd, 1500 // div), r", 0 mismatches$", cwd=m78d)
            ok &= _m6_run([exe, "marginalize", sd, 600 // div], "bs_marg_test%s marginalize seed %s x %d (vs the real marginalize())" % (sfx, sd, 600 // div), r", 0 mismatches$", cwd=m78d)
    name, every = ("full", "flow=50,imu=50,iter=1,marg=1") if args.m78_full else ("s10", "flow=50,imu=50,iter=10,marg=5")
    good, dd = _m78_dump(args, m78d, name, every)
    ok &= good
    ok &= _m6_run([m78d / "bs_marg_test", "replay", dd], "bs_marg_test replay %s (every MARG record)" % name, r", 0 mismatches$", cwd=m78d)
    ok &= _m6_run([m78d / "bs_marg_test_asan", "replay", dd, "200"], "bs_marg_test_asan replay %s (first 200 marginalizations)" % name, r", 0 mismatches$", cwd=m78d)
    if args.m78_mutations:
        r = subprocess.run(["python3", str(m78d / "mutate.py"), "householder"], capture_output=True, text=True, cwd=m78d)
        sys.stdout.write(r.stdout)
    if args.m78_full and not args.m78_keep:
        import shutil
        shutil.rmtree(dd, ignore_errors=True)
    return ok

def _m9_seq_dir(seq):
    for root in (SEQ_ROOT, ROOT / "runs/basalt_port/data"):
        if (root / seq / "mav0/cam0/data.csv").exists():
            return root / seq
    return None


def m9(args):
    """M9: the application.  (1) float JacobiSVD<Matrix4f> / BundleAdjustmentBase<float>::triangulate / StereographicParam::project vs the real classes
    (bs_svd_test.cc, tolerance 0); (2) config / calibration json readers vs the real cereal loaders (bs_app_cfg_test.cc); (3) THE END CRITERION: the pure-C99
    driver basalt_c_euroc (basalt_port/c/basalt_c_euroc.c) writes the TUM trajectory of --seq (default MH_01_easy; --m9-seqs a,b for more) and its sha256
    must equal the reference driver's (MH_01_easy: a7d3e7a3...; other sequences: a reference run is made, or reused from runs/basalt_port/m9/ref_<seq>.tum).
    --m9-frames N limits both runs to the first N frames (diagnosis only: a prefix run is not a claim).  Artefacts: runs/basalt_port/m9/."""
    m9d = ROOT / "runs/basalt_port/m9"
    ok = True
    seeds = args.m9_seeds.split(",")
    for script in ("build_svd.sh", "build_cfg.sh", "build_app.sh"):
        r = subprocess.run(["sh", str(m9d / script)], capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            return False
    for sd in seeds:
        ok &= _m6_run([m9d / "bs_svd_test", sd, args.m9_cases], "bs_svd_test seed %s x %d (JacobiSVD 4x4, triangulate, StereographicParam::project)" % (sd, args.m9_cases), r"^bs_svd: 0/\d+$", cwd=m9d)
    ok &= _m6_run([m9d / "bs_app_cfg_test", CONFIG, CALIB], "bs_app_cfg_test (VioConfig / Calibration<double> cereal loaders vs C)", r"^bs_app_cfg: 0/\d+$", cwd=m9d)
    if args.m9_asan:
        r = subprocess.run(["sh", str(m9d / "build_app.sh"), "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-g"],
                           env={**os.environ, "OUT": "basalt_c_euroc_asan"}, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stdout.write(r.stdout[-3000:] + r.stderr[-3000:])
            return False
        sd0 = _m9_seq_dir(args.m9_seqs.split(",")[0])
        if sd0 is not None:
            outs = []
            for exe in ("basalt_c_euroc", "basalt_c_euroc_asan"):
                o = m9d / ("asan_cmp_%s.tum" % exe)
                r = subprocess.run([str(m9d / exe), "--dataset-path", str(sd0), "--cam-calib", str(CALIB), "--config-path", str(CONFIG), "--out", str(o), "--quiet", "1", "--max-frames", "200"],
                                   capture_output=True, text=True)
                if r.returncode != 0:
                    sys.stdout.write(r.stderr[-3000:])
                    ok = False
                outs.append(o)
            same = all(o.exists() for o in outs) and sha256(outs[0]) == sha256(outs[1])
            print("  ASan+UBSan driver, first 200 frames: no report, trajectory identical to the plain build  [%s]" % ("PASS" if same and ok else "FAIL"))
            ok &= same
    frames = args.m9_frames
    OUT.mkdir(parents=True, exist_ok=True)
    for seq in args.m9_seqs.split(","):
        sd = _m9_seq_dir(seq)
        if sd is None:
            print("  %s: dataset missing (SEQ_ROOT or runs/basalt_port/data; tools/vio_harness/fetch_seq_stream.py %s cam0,cam1,imu0)" % (seq, seq))
            ok = False
            continue
        tag = "%s%s" % (seq, "_%df" % frames if frames else "")
        tum = m9d / ("c_%s.tum" % tag)
        argv = [m9d / "basalt_c_euroc", "--dataset-path", sd, "--cam-calib", CALIB, "--config-path", CONFIG, "--out", tum, "--quiet", "1"] + (["--max-frames", frames] if frames else [])
        t0 = time.time()
        r = subprocess.run([str(a) for a in argv], capture_output=True, text=True)
        if r.returncode != 0:
            print("  C driver failed on %s: %s" % (seq, r.stderr[-1500:]))
            ok = False
            continue
        ch = sha256(tum)
        print("  C driver %s: %.0f s, %s" % (tag, time.time() - t0, r.stderr.strip().splitlines()[-1]))
        if seq in EXPECTED_SHA and not frames:
            rh = EXPECTED_SHA[seq]
        else:
            ref = m9d / ("ref_%s.tum" % tag)
            if not ref.exists():
                run_reference_path(sd, ref, frames)
            rh = sha256(ref)
        good = ch == rh
        print("  %s trajectory sha256 C %s / reference %s  [%s]" % (tag, ch[:16], rh[:16], "PASS" if good else "FAIL"))
        ok &= good
        if not good and (m9d / ("ref_%s.tum" % tag)).exists():
            a_ = (m9d / ("ref_%s.tum" % tag)).read_text().splitlines()
            b_ = tum.read_text().splitlines()
            for i, (x, y) in enumerate(zip(a_, b_)):
                if x != y:
                    print("    first differing line %d:\n      ref %s\n      C   %s" % (i, x, y))
                    break
    return ok


def run_reference_path(sd, out, frames):
    cmd = [str(DRIVER), "--dataset-path", str(sd), "--cam-calib", str(CALIB), "--config-path", str(CONFIG), "--out", str(out)] + (["--max-frames", str(frames)] if frames else [])
    t0 = time.time()
    subprocess.run(cmd, env=ref_env({}), check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("  reference run %.0f s -> %s" % (time.time() - t0, out), flush=True)



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modules", default="m0")
    ap.add_argument("--tags", default="all,full")
    ap.add_argument("--seq", default="MH_01_easy")
    ap.add_argument("--regen", action="store_true", help="re-run the instrumented reference even if the dump exists")
    ap.add_argument("--m1-seeds", default="1,2,3,4,5,6", help="m1: oracle seeds")
    ap.add_argument("--m1-cases", type=int, default=40000, help="m1: cases per function and seed (floor 20000 required by the spec)")
    ap.add_argument("--m1-asan", action="store_true", help="m1: also build/run with ASan+UBSan (cases / 2)")
    ap.add_argument("--m23-seeds", default="1,2,3,4,5,6", help="m2/m3: oracle seeds")
    ap.add_argument("--m23-cases", type=int, default=20000, help="m2/m3: cases per seed (floor 20000 required by the spec)")
    ap.add_argument("--m6-seeds", default="1,2,3", help="m6: oracle seeds")
    ap.add_argument("--m6-regen", action="store_true", help="m6: rebuild the patch-0004 reference (BASALT_REF_ROOT) and regenerate runs/basalt_port/m6/dumps/m6full")
    ap.add_argument("--m6-mutations", action="store_true", help="m6: also run the deliberate-bug sensitivity scripts (runs/basalt_port/m6/mutate_*.sh, ~10 min)")
    ap.add_argument("--m78-seeds", default="1,2,3", help="m7/m8: oracle seeds")
    ap.add_argument("--m78-regen", action="store_true", help="m7/m8: rebuild the patch-0005 reference (runs/basalt_port/m78/ref0005) and regenerate the dump")
    ap.add_argument("--m78-full", action="store_true", help="m7/m8: replay EVERY optimize() call (19,803 LM steps) and marginalization (iter=1, marg=1 dump, ~1.4 GB, deleted afterwards unless --m78-keep)")
    ap.add_argument("--m78-keep", action="store_true", help="m7/m8: keep the full dump")
    ap.add_argument("--m78-mutations", action="store_true", help="m7/m8: also run a subset of the deliberate-bug sensitivity script (runs/basalt_port/m78/mutate.py, all of it ~15 min)")
    ap.add_argument("--m9-seeds", default="1,2,3", help="m9: bs_svd_test seeds")
    ap.add_argument("--m9-cases", type=int, default=20000, help="m9: bs_svd_test cases per class")
    ap.add_argument("--m9-seqs", default="MH_01_easy", help="m9: sequences for the end-to-end hash comparison (datasets under runs/okvis2x_port/data or runs/basalt_port/data)")
    ap.add_argument("--m9-frames", type=int, default=0, help="m9: limit both drivers to the first N frames (diagnosis)")
    ap.add_argument("--m9-asan", action="store_true", help="m9: also build the ASan+UBSan driver (basalt_c_euroc_asan)")
    ap.add_argument("--also-off", action="store_true", help="also run the reference with the dump off and compare the trajectory hash")
    a = ap.parse_args()
    ok = True
    for m in a.modules.split(","):
        if m == "m0":
            ok &= bool(m0(a))
        elif m == "m1":
            ok &= bool(m1(a))
        elif m == "m2":
            ok &= bool(m2(a))
        elif m == "m3":
            ok &= bool(m3(a))
        elif m == "m4":
            ok &= bool(m4(a))
        elif m == "m5":
            ok &= bool(m5(a))
        elif m == "m6":
            ok &= bool(m6(a))
        elif m == "m7":
            ok &= bool(m7(a))
        elif m == "m8":
            ok &= bool(m8(a))
        elif m == "m9":
            ok &= bool(m9(a))
        else:
            sys.exit("unknown module " + m)
    print("basalt_port checks:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
