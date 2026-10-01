#!/usr/bin/env python3
"""Run the repo's 7 monocular SLAM implementations on the lossless per-sequence
videos built by tools/tum_compare_build_videos.py, and write each system's
results into the shared runs/tum_compare/<system>/<seq>/ layout consumed by
tools/tum_eval.py.

Does not modify any SLAM implementation source or benchmark*.py: it imports
benchmark.build_slam_command (unmodified) to construct each impl's CLI, the
same command-building path benchmark.py itself uses, so invocation stays
consistent with the rest of the repo.

Frame index -> timestamp mapping uses the per-sequence manifest.json (rgb.txt
order, 1:1 with the video's decode order) written by the video builder.

Orientation: none of the 7 implementations' JSON timelines export a rotation,
only a camera center ('xyz' per frame) -- consistent with how benchmark.py's
own ate_rmse operates (translation-only Sim3 alignment of camera centers).
trajectory.tum therefore carries the identity quaternion (0,0,0,1) in every
row to satisfy the TUM file format; no scorer here or in benchmark.py reads
the quaternion columns.

Usage:
    python3 tools/tum_compare_run.py --impls all --seqs all --workers 4
"""
import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import benchmark  # noqa: E402  (reused, unmodified: build_slam_command)

VIDEOS_DIR = ROOT / 'runs' / 'tum_compare' / '_videos'
OUT_ROOT = ROOT / 'runs' / 'tum_compare'

IMPLS = ('python', 'cpp', 'c', 'pure_c', 'pure_c_brief', 'pure_c_orb', 'pure_c_plus')
SEQ_IDS = ('fr1_xyz', 'fr1_desk', 'fr1_floor', 'fr2_xyz', 'fr3_long_office')

# Locally-extracted (no-root) opencv runtime deps missing from the
# runs/oneshot/environment.sh prebuilt lib dump on this machine (libopencv_dnn,
# libgdcm*, libgdal -- see runs/tum_compare/NOTES.md). Only affects the
# OpenCV-linked impls (cpp, c); harmless to prepend for all.
EXTRA_LD_PATH = '/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib'
BASE_LD_PATH = '/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:/tmp/pose-opencv/root/usr/lib'

SECONDS_BIG = 1_000_000.0  # upper bound on frame count; real runs stop at video EOF
TIMEOUT_INTERNAL = 1800.0  # impl's own internal deadline (seconds)
TIMEOUT_SUBPROCESS = 2400.0  # backstop wall-clock kill


def load_manifest(seq_id):
    p = VIDEOS_DIR / f'{seq_id}_manifest.json'
    if not p.exists():
        return None
    return json.loads(p.read_text())


def run_one(impl, seq_id, manifest, omp_threads, force):
    out_dir = OUT_ROOT / impl / seq_id
    traj_path = out_dir / 'trajectory.tum'
    run_json_path = out_dir / 'run.json'
    if traj_path.exists() and run_json_path.exists() and not force:
        return f'[{impl}/{seq_id}] cached'

    out_dir.mkdir(parents=True, exist_ok=True)
    video = Path(manifest['video'])
    raw_json = out_dir / 'raw_metrics.json'

    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = f"{BASE_LD_PATH}:{EXTRA_LD_PATH}:{env.get('LD_LIBRARY_PATH', '')}"
    env['OMP_NUM_THREADS'] = str(omp_threads)
    env['OPENBLAS_NUM_THREADS'] = '1'

    script_name = 'simple_slam.py'
    cmd = benchmark.build_slam_command(
        ROOT, impl, video, SECONDS_BIG, TIMEOUT_INTERNAL, [], raw_json, script_name,
    )
    if impl == 'python':
        cmd = [str(PYTHON_BIN)] + cmd[1:]

    t0 = time.time()
    notes = ''
    exit_code = -1
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=TIMEOUT_SUBPROCESS)
        exit_code = r.returncode
        if r.returncode != 0:
            notes = f'nonzero exit; stderr tail: {r.stderr[-800:]}'
    except subprocess.TimeoutExpired:
        notes = f'subprocess wall timeout after {TIMEOUT_SUBPROCESS}s'
    wall_s = time.time() - t0

    frames_in = manifest['n_frames']
    timestamps = manifest['timestamps']
    n_pose_frames = 0
    if raw_json.exists():
        try:
            data = json.loads(raw_json.read_text())
            timeline = data.get('timeline', [])
            lines = []
            for f in timeline:
                if 'xyz' not in f:
                    continue
                fid = f['frame_id']
                if fid < 0 or fid >= len(timestamps):
                    continue
                x, y, z = f['xyz']
                ts = timestamps[fid]
                lines.append(f'{ts:.6f} {x:.6f} {y:.6f} {z:.6f} 0.0 0.0 0.0 1.0')
            traj_path.write_text('\n'.join(lines) + ('\n' if lines else ''))
            n_pose_frames = len(lines)
        except Exception as e:
            notes = (notes + f'; failed to parse raw_metrics.json: {e}').strip('; ')
    else:
        notes = (notes + '; raw_metrics.json not produced').strip('; ')
        traj_path.write_text('')

    run_json_path.write_text(json.dumps({
        'wall_s': wall_s,
        'frames_in': frames_in,
        'frames_with_pose': n_pose_frames,
        'exit_code': exit_code,
        'notes': notes,
    }, indent=2))
    status = 'OK' if exit_code == 0 and n_pose_frames > 0 else 'FAIL'
    return f'[{impl}/{seq_id}] {status} frames_with_pose={n_pose_frames}/{frames_in} wall={wall_s:.1f}s'


PYTHON_BIN = Path('/home/nybo/venvs/pose/bin/python3')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--impls', default=','.join(IMPLS))
    ap.add_argument('--seqs', default=','.join(SEQ_IDS))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()

    impls = IMPLS if args.impls == 'all' else args.impls.split(',')
    seqs = SEQ_IDS if args.seqs == 'all' else args.seqs.split(',')

    manifests = {}
    for seq_id in seqs:
        m = load_manifest(seq_id)
        if m is None:
            print(f'SKIP {seq_id}: no manifest/video (run tum_compare_build_videos.py first)')
            continue
        manifests[seq_id] = m

    # Prebuild native/pure-C binaries serially to avoid concurrent `gcc -o` races.
    print('Prebuilding binaries...')
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = f"{BASE_LD_PATH}:{EXTRA_LD_PATH}:{env.get('LD_LIBRARY_PATH', '')}"
    for impl in impls:
        try:
            if impl in {'cpp', 'c'}:
                benchmark.ensure_native_binary(ROOT, impl)
            elif impl in benchmark.PURE_C_BINARIES:
                src, bin_name = benchmark.PURE_C_BINARIES[impl]
                benchmark.ensure_pure_c_binary(ROOT, source_name=src, binary_name=bin_name)
        except Exception as e:
            print(f'  prebuild failed for {impl}: {e}')

    omp_threads = max(1, 16 // max(1, args.workers) // 2)
    jobs = [(impl, seq_id) for seq_id in seqs for impl in impls if seq_id in manifests]
    print(f'Running {len(jobs)} jobs with {args.workers} workers, OMP_NUM_THREADS={omp_threads} each')

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = {
            ex.submit(run_one, impl, seq_id, manifests[seq_id], omp_threads, args.force): (impl, seq_id)
            for impl, seq_id in jobs
        }
        for fut in as_completed(futs):
            impl, seq_id = futs[fut]
            try:
                print(fut.result())
            except Exception as e:
                print(f'[{impl}/{seq_id}] EXCEPTION: {e}')


if __name__ == '__main__':
    main()
