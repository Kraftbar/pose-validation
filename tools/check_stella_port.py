#!/usr/bin/env python3
"""Build and run every stella_port/c/check_*.c harness over every sequence
under runs/stella_port/reference_dumps/, print a table, and exit non-zero on
any mismatch (or build/run failure).

Each harness's first line declares its own sources:
    /* SV_PORT_SOURCES: a.c b.c */
(paths relative to stella_port/c/). The harness itself must accept
    <seq_label> <fixtures_dir> <dump_dir> [max_frames]
and print exactly one line "<seq_label>: <mismatches>/<total>", exiting 0
iff mismatches == 0 (see stella_port/c/check_sv_extract.c).

Fixture PGMs (the exact grayscale image stella_vslam's own OpenCV 4.6.0
build feeds to orb_extractor::extract(), per-frame, in frame_idx order) are
generated on demand into runs/stella_port/fixtures/<seq>/ via
tools/dump_stella_fixtures.cc, built once against the same OpenCV
(/tmp/pose-opencv, PKG_CONFIG_PATH set by runs/oneshot/environment.sh) the
reference driver itself links.

Usage:
    python3 tools/check_stella_port.py [--seqs fr1_xyz,fr1_desk] [--max-frames N]
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
C_DIR = ROOT / "stella_port/c"
DUMP_ROOT = ROOT / "runs/stella_port/reference_dumps"
FIXTURES_ROOT = ROOT / "runs/stella_port/fixtures"
DATA_ROOT = ROOT / "runs/orb_port/paper_bench/data"
BUILD_DIR = ROOT / "runs/stella_port/c_build"
# module 7 (loop closing): check_sv_loop* harnesses run over the loop dumps of tools/dump_stella_loop.py
# (each sub-directory holding loop_events.tsv; the --eigen-solver dumps are the bit-exact target) instead of
# the per-frame dumps above. Additive: every other harness is unaffected.
LOOP_DUMP_ROOT = ROOT / "runs/stella_port/reference_loop_eigen"

SEQ_DATA_DIRS = {
    "fr1_xyz": "rgbd_dataset_freiburg1_xyz",
    "fr1_desk": "rgbd_dataset_freiburg1_desk",
    "fr1_floor": "rgbd_dataset_freiburg1_floor",
    "fr2_xyz": "rgbd_dataset_freiburg2_xyz",
    "fr3_long_office": "rgbd_dataset_freiburg3_long_office_household",
}

OPENCV_PKGCONFIG = "/tmp/pose-opencv/pkgconfig"
LOCALOPENCV_LIB = "/tmp/localopencv/root/usr/lib/x86_64-linux-gnu"
LOCALOPENCV_LIB2 = "/tmp/localopencv/root/usr/lib"
POSE_OPENCV_LIB = "/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu"

SOURCES_RE = re.compile(r"/\*\s*SV_PORT_SOURCES:\s*(.+?)\s*(?:\*/)?\s*$")


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def pkgconfig(flag, pkg="opencv4"):
    env = os.environ.copy()
    env["PKG_CONFIG_PATH"] = OPENCV_PKGCONFIG
    out = subprocess.run(["pkg-config", flag, pkg], capture_output=True, text=True, env=env, check=True)
    return out.stdout.split()


def build_fixture_tool():
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    out = BUILD_DIR / "dump_stella_fixtures"
    if out.exists():
        return out
    cflags = pkgconfig("--cflags")
    libs = pkgconfig("--libs")
    cmd = (
        ["g++", "-O2", "-std=c++14", str(ROOT / "tools/dump_stella_fixtures.cc"), "-o", str(out)]
        + cflags + libs
        + ["-Wl,--allow-shlib-undefined",
           f"-Wl,-rpath,{POSE_OPENCV_LIB}", f"-Wl,-rpath,{LOCALOPENCV_LIB}"]
    )
    run(cmd)
    return out


def ensure_fixtures(seq, fixture_tool):
    seq_fixtures = FIXTURES_ROOT / seq
    data_dir_name = SEQ_DATA_DIRS.get(seq)
    data_dir = DATA_ROOT / data_dir_name if data_dir_name else None
    if seq_fixtures.exists() and any(seq_fixtures.glob("*.pgm")):
        return seq_fixtures
    if not data_dir or not data_dir.exists():
        print(f"warning: no TUM source data for {seq} ({data_dir}); "
              f"cannot generate fixtures", file=sys.stderr)
        return None
    seq_fixtures.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{LOCALOPENCV_LIB}:{LOCALOPENCV_LIB2}:" + env.get("LD_LIBRARY_PATH", "")
    run([str(fixture_tool), str(data_dir), str(seq_fixtures)], env=env)
    return seq_fixtures


def discover_harnesses():
    harnesses = []
    for path in sorted(C_DIR.glob("check_*.c")):
        with open(path) as f:
            first_line = f.readline()
        m = SOURCES_RE.search(first_line)
        if not m:
            print(f"warning: {path} has no /* SV_PORT_SOURCES: ... */ first line, skipping",
                  file=sys.stderr)
            continue
        sources = m.group(1).split()
        harnesses.append((path.stem, sources))
    return harnesses


def build_harness(name, sources):
    out = BUILD_DIR / name
    cmd = ["gcc", "-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math",
           "-o", str(out)] + [str(C_DIR / s) for s in sources] + ["-lm"]
    run(cmd)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seqs", default=None, help="comma-separated seq names (default: all under reference_dumps/)")
    ap.add_argument("--max-frames", type=int, default=-1)
    args = ap.parse_args()

    if not DUMP_ROOT.exists():
        print(f"error: {DUMP_ROOT} does not exist -- run tools/dump_stella_reference.py first", file=sys.stderr)
        return 1

    if args.seqs:
        seqs = args.seqs.split(",")
    else:
        seqs = sorted(p.name for p in DUMP_ROOT.iterdir() if p.is_dir())

    # loop-closing dumps live in their own tree (LOOP_DUMP_ROOT); --seqs may name those too, e.g. fr3_long_office
    loop_filter = seqs if args.seqs else None
    seqs = [q for q in seqs if (DUMP_ROOT / q).is_dir()]
    if not seqs and not (loop_filter and LOOP_DUMP_ROOT.exists()):
        print("error: no sequences found", file=sys.stderr)
        return 1

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    fixture_tool = build_fixture_tool()

    seq_fixtures = {}
    for seq in seqs:
        seq_fixtures[seq] = ensure_fixtures(seq, fixture_tool)

    harnesses = discover_harnesses()
    if not harnesses:
        print("error: no stella_port/c/check_*.c harnesses found", file=sys.stderr)
        return 1

    built = {}
    for name, sources in harnesses:
        built[name] = build_harness(name, sources)

    loop_seqs = []
    if LOOP_DUMP_ROOT.exists():
        for p in sorted(LOOP_DUMP_ROOT.iterdir()):
            if not (p / "loop_events.tsv").exists():
                continue
            if loop_filter and not any(p.name == q or p.name.startswith(q + "_") for q in loop_filter):
                continue
            loop_seqs.append(p.name)

    rows = []  # (harness, seq, mismatches, total, ok)
    any_fail = False
    for name in built:
        exe = built[name]
        for seq in (loop_seqs if name.startswith("check_sv_loop") else seqs):
            is_loop = name.startswith("check_sv_loop")
            fdir = "-" if is_loop else seq_fixtures.get(seq)
            dump_dir = (LOOP_DUMP_ROOT if is_loop else DUMP_ROOT) / seq
            if not fdir:
                rows.append((name, seq, None, None, False))
                any_fail = True
                continue
            cmd = [str(exe), seq, str(fdir), str(dump_dir)]
            if args.max_frames > 0:
                cmd.append(str(args.max_frames))
            proc = subprocess.run(cmd, capture_output=True, text=True)
            line = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else ""
            m = re.match(r".*:\s*(\d+)/(\d+)\s*$", line)
            if not m:
                print(proc.stdout, file=sys.stdout)
                print(proc.stderr, file=sys.stderr)
                rows.append((name, seq, None, None, False))
                any_fail = True
                continue
            mism, total = int(m.group(1)), int(m.group(2))
            ok = (mism == 0) and (proc.returncode == 0)
            if not ok:
                any_fail = True
            rows.append((name, seq, mism, total, ok))

    print()
    print(f"{'harness':<20} {'seq':<24} {'mismatches':>12} {'total':>10}  status")
    print("-" * 78)
    for name, seq, mism, total, ok in rows:
        mism_s = str(mism) if mism is not None else "?"
        total_s = str(total) if total is not None else "?"
        print(f"{name:<20} {seq:<24} {mism_s:>12} {total_s:>10}  {'PASS' if ok else 'FAIL'}")

    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
