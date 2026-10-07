#!/usr/bin/env python3
"""End-to-end check of the C app (okvis_port/c/okvis_c_euroc.c, OKVIS_PORT_OKVIS2X=1) against the OKVIS2-X deterministic reference
runs (tools/okvis2x_run_reference.py), GNSS on and off.

    python3 tools/okvis2x_check_gnss.py [--cases clean_gps_a:gps,clean_off:off] [--seq MH_01_easy] [--data-root runs/okvis2x_port/data]
                                        [--images external/vio/data/okvis_brisk_tmp] [--max-frames N]

A case is <reference tag>:<gps|gpsb|off>[:<data root dir under runs/okvis2x_port, default data>]. gps = okvis2x_mono_euroc_gps_robustfalse_deterministic.yaml,
gpsb = okvis2x_mono_euroc_gps_robusttrue_deterministic.yaml (stage b, robust_gps_init: true), off = okvis2x_mono_euroc_deterministic.yaml.
Stage b cases: gps_b_mono1:gpsb (MH_01 5 Hz data: the robust init never completes), gps_b_r2_1:gpsb:data_r2 (20 Hz, 1/2 cm, seed 5, 30 s blackout at
60 s: robust init + dropout + re-init, with Align4DoF_Ceres), gps_b_r1_1:gpsb:data_r1 (20 Hz, 2/4 cm: RANSAC passes, the 1 degree yaw gate never does).
Compared: causal.csv byte for byte; final.csv columns 1-17 (X writes uninitialised garbage into columns 18-21 when GNSS is off, and the
C app writes NrGps / SID / gpsMode / keyframe id there); global_final.csv byte for byte (gps cases). Exit 0 iff every case is identical.
Work dir: runs/okvis2x_port/gnss_int (gitignored with runs/): the sequence dir of symlinks, the object files and the outputs.
"""
import argparse, hashlib, os, subprocess, sys, time, zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
C_DIR = ROOT / "okvis_port/c"
WORK = ROOT / "runs/okvis2x_port/gnss_int"
CFGS = ROOT / "okvis2x_port/reference/configs"
VOC = ROOT / "runs/okvis_port/vocabulary/small_voc.bin"
CFLAGS = ["-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math"]


def build():
    first = (C_DIR / "check_ok_system.c").read_text().splitlines()[0]
    sources = first.split(":", 1)[1].split("*/")[0].split()[1:] + ["okvis_c_euroc.c", "ok_png.c"]
    objdir = WORK / ("obj_" + str(zlib.crc32(" ".join(CFLAGS).encode())))
    objdir.mkdir(parents=True, exist_ok=True)
    hm = max(x.stat().st_mtime for x in list(C_DIR.glob("*.h")) + list(C_DIR.glob("*.inc")))
    objs, procs = [], []
    for f in sources:
        s = C_DIR / f
        o = objdir / (s.stem + ".o")
        objs.append(str(o))
        if not o.exists() or o.stat().st_mtime < max(s.stat().st_mtime, hm):
            procs.append(subprocess.Popen(["gcc", *CFLAGS, "-c", str(s), "-o", str(o)]))
            if len(procs) >= 4:
                for p in procs:
                    if p.wait():
                        sys.exit("compile failed")
                procs = []
    for p in procs:
        if p.wait():
            sys.exit("compile failed")
    exe = WORK / "okvis_c_euroc_x"
    tmp = WORK / f"okvis_c_euroc_x.{os.getpid()}"       # link beside, then rename: concurrent invocations never see a half-written / busy exe
    subprocess.run(["gcc", *CFLAGS, *objs, "-lm", "-o", str(tmp)], check=True)
    os.replace(tmp, exe)
    return exe


def seq_dir(name, data_root, images, seq):
    d = WORK / f"seq_{name}"
    (d / "mav0").mkdir(parents=True, exist_ok=True)
    for link, target in ((d / "gray", Path(images) / seq / "gray"), (d / "mav0/imu0", Path(data_root) / seq / "mav0/imu0"),
                         (d / "mav0/gps0", Path(data_root) / seq / "mav0/gps0")):
        if link.is_symlink() or link.exists():
            link.unlink()
        if target.exists():
            link.symlink_to(target.resolve())
    return d


def rows(path, ncols):
    return [[t.strip() for t in l.split(",")][:ncols] for l in Path(path).read_text().splitlines()]


def compare_cols(a, b, ncols=17):
    ra, rb = rows(a, ncols), rows(b, ncols)
    if len(ra) != len(rb):
        return f"row count {len(ra)} vs {len(rb)}"
    for i, (x, y) in enumerate(zip(ra, rb)):
        if x != y:
            j = next(k for k in range(min(len(x), len(y))) if x[k] != y[k])
            return f"first difference row {i} col {j + 1}: {x[j]} vs {y[j]}"
    return None


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default="clean_gps_a:gps,clean_off:off")
    ap.add_argument("--stage-b", action="store_true", help="append the stage-b cases (gps_b_mono1, gps_b_r2_1, gps_b_r1_1) to --cases")
    ap.add_argument("--seq", default="MH_01_easy")
    ap.add_argument("--data-root", default=str(ROOT / "runs/okvis2x_port/data"))
    ap.add_argument("--images", default=str(ROOT / "external/vio/data/okvis_brisk_tmp"))
    ap.add_argument("--max-frames", type=int, default=-1)
    a = ap.parse_args()
    exe = build()
    ok_all = True
    cases = a.cases + (",gps_b_mono1:gpsb,gps_b_r2_1:gpsb:data_r2,gps_b_r1_1:gpsb:data_r1" if a.stage_b else "")
    for case in cases.split(","):
        tag, mode, *rest = case.split(":")
        data_root = str(ROOT / "runs/okvis2x_port" / rest[0]) if rest else a.data_root
        ref = ROOT / "runs/okvis2x_port/reference_runs" / a.seq / tag
        out = WORK / "out" / tag
        out.mkdir(parents=True, exist_ok=True)
        sd = seq_dir(tag, data_root, a.images, a.seq)
        cfg = CFGS / {"gps": "okvis2x_mono_euroc_gps_robustfalse_deterministic.yaml", "gpsb": "okvis2x_mono_euroc_gps_robusttrue_deterministic.yaml",
                      "off": "okvis2x_mono_euroc_deterministic.yaml"}[mode]
        env = dict(os.environ, OKVIS_PORT_OKVIS2X="1")
        cmd = [str(exe), str(cfg), str(sd), str(VOC), str(out)] + ([str(a.max_frames)] if a.max_frames > 0 else [])
        t0 = time.time()
        r = subprocess.run(cmd, env=env, capture_output=True, text=True)
        wall = time.time() - t0
        msgs = []
        if r.returncode:
            msgs.append(f"app exit {r.returncode}: {r.stderr[-300:]}")
        else:
            if a.max_frames > 0:
                n = len((out / "causal.csv").read_text().splitlines())
                ra = rows(out / "causal.csv", 17)
                rr = rows(ref / "causal.csv", 17)[:n]
                m = None if ra == rr else "causal prefix differs"
            else:
                m = None if sha(out / "causal.csv") == sha(ref / "causal.csv") else compare_cols(out / "causal.csv", ref / "causal.csv") or "causal.csv bytes differ"
            if m: msgs.append("causal: " + m)
            if a.max_frames <= 0:
                m = compare_cols(out / "final.csv", ref / "final.csv")
                if m: msgs.append("final cols 1-17: " + m)
                if mode != "off" and sha(out / "global_final.csv") != sha(ref / "global_final.csv"):
                    msgs.append("global_final.csv differs")
        ok = not msgs
        ok_all &= ok
        extra = f" causal {sha(out / 'causal.csv')}" + (f" global {sha(out / 'global_final.csv')}" if mode != "off" and (out / 'global_final.csv').exists() else "") if not r.returncode else ""
        print(f"{case:<20} {'PASS' if ok else 'FAIL'}  wall {wall:5.0f}s{extra}  {'; '.join(msgs)}", flush=True)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
