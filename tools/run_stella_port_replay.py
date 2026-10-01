#!/usr/bin/env python3
"""Continuous replay of the pure-C stella_vslam port (stella_port/c/sv_run.c -> sv_system) against the reference.

sv_system runs the whole sequence with NO teacher forcing (own tracker / mapper / loop closer / BoW database /
keyframe lifetimes) on the gray fixtures of tools/dump_stella_fixtures.cc. The run writes the reference driver's own
text formats and this tool diffs them against the canonical reference dumps:

  fr1_xyz, fr1_desk  vs runs/stella_port/reference_dumps/<seq>/ (full dumps): frame_trace.tsv (track path, initial
                     and final pose bits, ref keyframe, tracked / reliable counts), kf_decision.tsv (keyframe
                     decisions incl. every sub-flag), frames_after.tsv (keyframe / landmark counts), frames_before.tsv,
                     matches.tsv (landmark of every keypoint of every frame), keyframes.tsv / landmarks.tsv (the whole
                     map after every frame: poses, covisibilities, spanning tree, landmark position / descriptor /
                     normal / valid range / counters / reference keyframe / observations) and kf_destroyed.tsv
                     (keyframe object lifetimes, derived by the port from the loop detector's retention rule).
  fr3_long_office    vs runs/stella_port/reference_loop_eigen/<seq>/ (the Eigen-solver reference, --loop-dump): the
                     per-frame poses (frames_before.tsv), keyframe / landmark counts (frames_after.tsv), the
                     trajectory, the keyframe destruction schedule and -- through the accepted loop -- the whole map
                     right after the loop BA (loop_snap.tsv phase 2: keyframe poses / spanning tree, landmark
                     position / descriptor / normal / range / counters / observations).

then scores the port's trajectory with tools/tum_eval.py's own association + benchmark.ate_rmse (vs the reference
trajectory and vs ground truth).

usage: python3 tools/run_stella_port_replay.py [--seqs fr1_xyz,fr1_desk,fr3_long_office] [--max-frames N]
                                               [--keep] [--no-run] [--out-root DIR]
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT))
import check_stella_port as csp  # noqa: E402

C_DIR = ROOT / "stella_port/c"
BUILD = ROOT / "runs/stella_port/c_build/sv_run"
REPLAY_ROOT = ROOT / "runs/stella_port/replay"
DUMPS = ROOT / "runs/stella_port/reference_dumps"
LOOP_DUMPS = ROOT / "runs/stella_port/reference_loop_eigen"
VOCAB = ROOT / "external/candidates/orb_vocab.fbow"


def build():
    first = (C_DIR / "sv_run.c").open().readline()
    m = csp.SOURCES_RE.search(first)
    srcs = m.group(1).split()
    BUILD.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["gcc", "-std=c99", "-O2", "-ffp-contract=off", "-fno-fast-math", "-o", str(BUILD)] + [str(C_DIR / s) for s in srcs] + ["-lm"]
    subprocess.run(cmd, check=True)


def fixtures(seq):
    # the OpenCV 4.6 build the fixture tool links needs a few extra shared libraries on this machine
    # (libtbb from the pose-opencv build, gdcm / gdal / armadillo from the vocab work's extracted deps, read-only)
    extra = [csp.POSE_OPENCV_LIB, str(ROOT / "runs/stella_port/vocab/deps/usr/lib/x86_64-linux-gnu"),
             str(ROOT / "runs/stella_port/vocab/deps/usr/lib")]
    os.environ["LD_LIBRARY_PATH"] = ":".join(extra + [os.environ.get("LD_LIBRARY_PATH", "")])
    tool = csp.build_fixture_tool()
    d = csp.ensure_fixtures(seq, tool)
    if d is None:
        raise SystemExit(f"no fixtures for {seq}")
    return d


# ---------------------------------------------------------------------------------------------------------------
# comparison helpers
# ---------------------------------------------------------------------------------------------------------------
class Report:
    def __init__(self, seq):
        self.seq = seq
        self.rows = []  # (name, n_compared, n_diff, first_divergence)
        self.notes = []

    def add(self, name, total, bad, first):
        self.rows.append((name, total, bad, first))

    def ok(self):
        return all(r[2] == 0 for r in self.rows)


def compare_lines(name, ref_path, my_path, key_cols=1, limit_frames=None):
    """Line-by-line compare of two frame-keyed TSVs; returns (total, diff, first divergence description)."""
    total = bad = 0
    first = None
    with open(ref_path) as fr, open(my_path) as fm:
        rh, mh = fr.readline(), fm.readline()
        if rh != mh:
            return 1, 1, f"header differs: {rh.strip()!r} vs {mh.strip()!r}"
        while True:
            a, b = fr.readline(), fm.readline()
            if not a and not b:
                break
            if limit_frames is not None and a:
                if int(a.split("\t", 1)[0]) >= limit_frames:
                    break
            total += 1
            if a != b:
                bad += 1
                if first is None:
                    ca, cb = a.rstrip("\n").split("\t"), b.rstrip("\n").split("\t")
                    cols = [i for i in range(max(len(ca), len(cb))) if i >= len(ca) or i >= len(cb) or ca[i] != cb[i]]
                    hdr = rh.rstrip("\n").split("\t")
                    cname = ",".join(hdr[i] if i < len(hdr) else str(i) for i in cols[:4])
                    first = f"frame {ca[0] if a else '?'}: columns [{cname}]"
                    if not a or not b:
                        first = "length differs"
    return total, bad, first


def read_blocks(path):
    """Yields (frame, [rows]) for a frame-keyed snapshot TSV (rows split by tab, header skipped)."""
    with open(path) as f:
        f.readline()
        cur, rows = None, []
        for line in f:
            c = line.rstrip("\n").split("\t")
            fr = int(c[0])
            if fr != cur:
                if cur is not None:
                    yield cur, rows
                cur, rows = fr, []
            rows.append(c)
        if cur is not None:
            yield cur, rows


def compare_snapshots(ref_path, my_path, kind, ncols_id=1):
    """Compares whole-map snapshots frame by frame (rows canonicalised by id). kind: 'kf' or 'lm'."""
    ref = {}
    total = bad = 0
    first = None
    ref_iter = read_blocks(ref_path)
    r_cur = next(ref_iter, None)
    hdr = {"kf": ["frame", "kf", "pose_9g", "pose_hex", "bad", "covisibilities", "parent", "children"],
           "lm": ["frame", "lm", "pos_9g", "pos_hex", "descriptor", "normal_9g", "normal_hex", "min", "max", "observed",
                  "observable", "ref_kf", "observations"]}[kind]
    for frame, rows in read_blocks(my_path):
        while r_cur is not None and r_cur[0] < frame:
            r_cur = next(ref_iter, None)
        if r_cur is None or r_cur[0] != frame:
            total += 1
            bad += 1
            first = first or f"frame {frame}: no reference snapshot"
            continue
        a = {int(r[1]): r for r in r_cur[1]}
        b = {int(r[1]): r for r in rows}
        total += len(a) + len(set(b) - set(a))
        if a == b:
            continue
        for k in sorted(set(a) | set(b)):
            if a.get(k) == b.get(k):
                continue
            bad += 1
            if first is None:
                if k not in a:
                    first = f"frame {frame}: {kind} {k} only in port"
                elif k not in b:
                    first = f"frame {frame}: {kind} {k} only in reference"
                else:
                    cols = [hdr[i] if i < len(hdr) else str(i) for i in range(len(a[k])) if i >= len(b[k]) or a[k][i] != b[k][i]]
                    first = f"frame {frame}: {kind} {k} columns {cols}"
    return total, bad, first


def load_destroyed_ref(path):
    d = {}
    with open(path) as f:
        f.readline()
        for line in f:
            fr, ph, kf = line.split()
            d[int(kf)] = (int(fr), int(ph))
    return d


def load_loop_log(path):
    erased, loops, inits, resets = {}, [], [], []
    with open(path) as f:
        f.readline()
        for line in f:
            k, fr, a, b = line.rstrip("\n").split("\t")
            if k == "erased":
                erased[int(a)] = (int(fr), int(b))
            elif k == "loop":
                loops.append((int(fr), int(a), int(b)))
            elif k == "init":
                inits.append(int(fr))
            elif k == "reset":
                resets.append(int(fr))
    return erased, loops, inits, resets


def compare_lifetimes(rep, ref_destroyed, my_erased):
    # ref: kf -> (destroy frame, phase); port: kf -> (erase frame, destroy frame or -1)
    mism = []
    for kf in sorted(set(ref_destroyed) | set(my_erased)):
        r = ref_destroyed.get(kf)
        m = my_erased.get(kf)
        rd = r[0] if r else None
        md = m[1] if m and m[1] >= 0 else None
        if rd != md:
            mism.append((kf, rd, md))
    rep.add("keyframe destruction frame (kf_destroyed.tsv)", len(set(ref_destroyed) | set(my_erased)), len(mism),
            f"kf {mism[0][0]}: reference {mism[0][1]}, port {mism[0][2]}" if mism else None)


# fr3_long_office: loop_snap.tsv phase 2 rows vs the port's snapshot after the accepted loop --------------------
def hexf(s):
    return float.fromhex(s)


def compare_loop_snap(rep, ref_dir, my_dir, step):
    import struct

    def f32(x):
        return struct.unpack("f", struct.pack("f", x))[0]

    ref_k, ref_l = {}, {}
    with open(ref_dir / "loop_snap.tsv") as f:
        for line in f:
            c = line.rstrip("\n").split("\t")
            if int(c[0]) != step or c[1] != "2":
                continue
            (ref_k if c[2] == "K" else ref_l)[int(c[3])] = c
    if not ref_k:
        rep.notes.append("no loop_snap phase-2 rows (run without accepted loop?)")
        return
    # port snapshot at the accepted-loop frame = first block of snapshots, plus the final block
    my_k = {}
    my_l = {}
    frames = []
    with open(my_dir / "keyframes.tsv") as f:
        f.readline()
        for line in f:
            c = line.rstrip("\n").split("\t")
            fr = int(c[0])
            if fr not in frames:
                frames.append(fr)
            my_k.setdefault(fr, {})[int(c[1])] = c
    with open(my_dir / "landmarks.tsv") as f:
        f.readline()
        for line in f:
            c = line.rstrip("\n").split("\t")
            my_l.setdefault(int(c[0]), {})[int(c[1])] = c
    if not frames:
        rep.notes.append("port produced no snapshot")
        return
    fr = frames[0]  # the frame of the accepted loop
    total = bad = 0
    first = None
    mk, ml = my_k[fr], my_l.get(fr, {})
    for k in sorted(set(ref_k) | set(mk)):
        total += 1
        if k not in ref_k or k not in mk:
            bad += 1
            first = first or f"kf {k} present only in {'reference' if k in ref_k else 'port'}"
            continue
        r, m = ref_k[k], mk[k]
        rp = [hexf(x) for x in r[4].split(",")]
        mp = [hexf(x) for x in m[3].split(",")]
        ok = rp == mp and int(r[7]) == int(m[6]) and [x for x in r[8].split(",") if x] == [x for x in m[7].split(",") if x]
        if not ok:
            bad += 1
            first = first or f"kf {k} (pose/parent/children)"
    rep.add(f"kf state after accepted loop (frame {fr}, step {step})", total, bad, first)
    total = bad = 0
    first = None
    for k in sorted(set(ref_l) | set(ml)):
        total += 1
        if k not in ref_l or k not in ml:
            bad += 1
            first = first or f"lm {k} present only in {'reference' if k in ref_l else 'port'}"
            continue
        r, m = ref_l[k], ml[k]
        rpos = [hexf(x) for x in r[4].split(",")]
        mpos = [hexf(x) for x in m[3].split(",")]
        rn = [hexf(x) for x in r[6].split(",")]
        mn = [hexf(x) for x in m[6].split(",")]
        ok = (rpos == mpos and r[5] == m[4] and rn == mn and f32(hexf(r[7])) == f32(float(m[7])) and f32(hexf(r[8])) == f32(float(m[8]))
              and r[9] == m[9] and r[10] == m[10] and r[11] == m[11] and r[12] == m[12])
        if not ok:
            bad += 1
            first = first or f"lm {k}"
    rep.add(f"landmark state after accepted loop (frame {fr}, step {step})", total, bad, first)


# ---------------------------------------------------------------------------------------------------------------
def ate_table(seq, my_traj, ref_traj):
    import numpy as np
    import tum_eval as te
    out = {}
    gt_ts, gt_pos = te.read_groundtruth(seq)
    p_ts, p_pos = te.read_trajectory_tum(my_traj)
    out["n_port"] = int(len(p_ts))
    r = te.ate_over(p_ts, p_pos, gt_ts, gt_pos)
    out["port_vs_gt"] = r.get("ate_rmse") if r.get("ok") else None
    if ref_traj is not None and Path(ref_traj).exists():
        r_ts, r_pos = te.read_trajectory_tum(ref_traj)
        out["n_ref"] = int(len(r_ts))
        r2 = te.ate_over(r_ts, r_pos, gt_ts, gt_pos)
        out["ref_vs_gt"] = r2.get("ate_rmse") if r2.get("ok") else None
        r3 = te.ate_over(p_ts, p_pos, r_ts, r_pos)
        out["port_vs_ref"] = r3.get("ate_rmse") if r3.get("ok") else None
        # element-wise (same timestamps) position deviation
        if len(r_ts) == len(p_ts):
            dev = np.linalg.norm(r_pos - p_pos, axis=1) if np.allclose(r_ts, p_ts) else None
            out["max_abs_dev_m"] = float(dev.max()) if dev is not None else None
    return out


def run_seq(seq, args):
    rep = Report(seq)
    seq_dir = csp.DATA_ROOT / csp.SEQ_DATA_DIRS[seq]
    out = Path(args.out_root) / seq
    # sequences without a full per-frame dump use the light Eigen-solver reference (reference_loop_eigen)
    is_loop = not (DUMPS / seq / "frame_trace.tsv").exists()
    ref_dir = (LOOP_DUMPS if is_loop else DUMPS) / seq
    if not args.no_run:
        fx = fixtures(seq)
        if out.exists():
            for p in out.iterdir():
                p.unlink()
        out.mkdir(parents=True, exist_ok=True)
        cmd = [str(BUILD), str(VOCAB), str(seq_dir), str(fx), str(out)]
        if args.max_frames >= 0:
            cmd.append(str(args.max_frames))
        cmd += ["--snap-loop"] if is_loop else ["--snap-every", "1"]
        print("+", " ".join(cmd), flush=True)
        t0 = time.time()
        proc = subprocess.run(cmd, capture_output=True, text=True)
        (out / "run.log").write_text(proc.stdout + proc.stderr)
        print(f"{seq}: sv_run exit={proc.returncode} wall={time.time() - t0:.1f}s", flush=True)
        if proc.returncode != 0:
            print(proc.stderr[-2000:])
            rep.add("sv_run exit code", 1, 1, f"exit {proc.returncode}")
            return rep, {}
    lim = args.max_frames if args.max_frames >= 0 else None
    if not is_loop:
        for name in ("frame_trace", "kf_decision", "frames_after", "frames_before", "matches"):
            t, b, f = compare_lines(name, ref_dir / f"{name}.tsv", out / f"{name}.tsv", limit_frames=lim)
            rep.add(f"{name}.tsv", t, b, f)
        if lim is None:
            t, b, f = compare_snapshots(ref_dir / "keyframes.tsv", out / "keyframes.tsv", "kf")
            rep.add("keyframes.tsv (map snapshot after every frame)", t, b, f)
            t, b, f = compare_snapshots(ref_dir / "landmarks.tsv", out / "landmarks.tsv", "lm")
            rep.add("landmarks.tsv (map snapshot after every frame)", t, b, f)
    else:
        for name in ("frames_before", "frames_after"):
            t, b, f = compare_lines(name, ref_dir / f"{name}.tsv", out / f"{name}.tsv", limit_frames=lim)
            rep.add(f"{name}.tsv", t, b, f)
    erased, loops, inits, resets = load_loop_log(out / "loop_log.tsv")
    if lim is None:
        compare_lifetimes(rep, load_destroyed_ref(ref_dir / "kf_destroyed.tsv"), erased)
    rep.notes.append(f"port: inits at frames {inits}, resets {resets}, accepted loops (frame, cur kf, candidate kf) {loops}")
    if is_loop and lim is None:
        # reference loop from loop_events.tsv: "<step>\tL0\t<candidate>\t<cur>\t<same_root>"
        ref_loops = []
        with open(ref_dir / "loop_events.tsv") as f:
            for line in f:
                c = line.rstrip("\n").split("\t")
                if len(c) > 2 and c[1] == "L0":
                    ref_loops.append((int(c[3]), int(c[2])))
        mine = [(cur, cand) for (_fr, cur, cand) in loops]
        rep.add("accepted loops (cur kf, candidate kf)", max(len(ref_loops), len(mine)), 0 if ref_loops == mine else 1,
                None if ref_loops == mine else f"reference {ref_loops}, port {mine}")
        if ref_loops and mine:
            compare_loop_snap(rep, ref_dir, out, ref_loops[0][0])
    ate = {}
    if (out / "trajectory.tum").exists():
        try:
            ate = ate_table(seq, out / "trajectory.tum", ref_dir / "trajectory.tum")
        except Exception as e:  # noqa: BLE001
            rep.notes.append(f"ATE failed: {e}")
    if not args.keep and not args.no_run:
        for name in ("keyframes.tsv", "landmarks.tsv", "matches.tsv"):
            p = out / name
            if p.exists():
                p.unlink()
    return rep, ate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seqs", default="fr1_xyz,fr1_desk,fr3_long_office,fr1_floor,fr2_xyz")
    ap.add_argument("--max-frames", type=int, default=-1)
    ap.add_argument("--keep", action="store_true", help="keep the big snapshot / matches files")
    ap.add_argument("--no-run", action="store_true", help="only compare an existing run directory")
    ap.add_argument("--out-root", default=str(REPLAY_ROOT))
    args = ap.parse_args()
    if not args.no_run:
        build()
    results = {}
    rc = 0
    for seq in args.seqs.split(","):
        rep, ate = run_seq(seq, args)
        results[seq] = {"rows": rep.rows, "notes": rep.notes, "ate": ate}
        print(f"\n== {seq} ==")
        for name, total, bad, first in rep.rows:
            print(f"  {name:62s} {bad}/{total}" + (f"   first divergence: {first}" if bad else ""))
        for n in rep.notes:
            print("  note:", n)
        if ate:
            print("  ATE (m):", json.dumps({k: (round(v, 6) if isinstance(v, float) else v) for k, v in ate.items()}))
        if not rep.ok():
            rc = 1
    Path(args.out_root).mkdir(parents=True, exist_ok=True)
    (Path(args.out_root) / "summary.json").write_text(json.dumps(results, indent=1, default=str))
    return rc


if __name__ == "__main__":
    sys.exit(main())
