"""
Run ORB-SLAM2 (monocular, TUM mode) on test_freiburgxyz525.mp4
and compare ATE against our simple_slam.py results.

Prerequisites: run setup_orbslam2.sh first.

Usage:
    python3 run_orbslam_benchmark.py
    python3 run_orbslam_benchmark.py --slam_dir ~/orb_slam2 --reextract
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import cv2


# ---------------------------------------------------------------------------
# TUM freiburg1 calibration (fr1/xyz — 640x480)
# Source: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
# ---------------------------------------------------------------------------
TUM_FR1_YAML = """%YAML:1.0

Camera.type: PinHole

Camera.fx: 517.306408
Camera.fy: 516.469215
Camera.cx: 318.643040
Camera.cy: 255.313989

Camera.k1: 0.262383
Camera.k2: -0.953104
Camera.p1: -0.005358
Camera.p2: 0.002628
Camera.k3: 1.163314

Camera.width: 640
Camera.height: 480
Camera.fps: 25

Camera.RGB: 1

ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
Viewer.UseViewer: 0
"""


# ---------------------------------------------------------------------------
# Umeyama alignment (same as benchmark.py)
# ---------------------------------------------------------------------------
def umeyama(src: np.ndarray, dst: np.ndarray):
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_s, dst - mu_d
    var_s = (src_c ** 2).sum() / n
    if var_s < 1e-12:
        return np.eye(3), np.zeros(3), 1.0
    cov = dst_c.T @ src_c / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = float((D * S.diagonal()).sum() / var_s)
    t = mu_d - scale * R @ mu_s
    return R, t, scale


def ate(est: np.ndarray, gt: np.ndarray):
    R, t, sc = umeyama(est, gt)
    aligned = (sc * R @ est.T).T + t
    errs = np.linalg.norm(aligned - gt, axis=1)
    return {
        'rmse':   float(np.sqrt((errs ** 2).mean())),
        'median': float(np.median(errs)),
        'max':    float(errs.max()),
        'scale':  sc,
        'n':      len(errs),
    }


# ---------------------------------------------------------------------------
# Extract frames from video to TUM-style directory
# ---------------------------------------------------------------------------
def extract_frames(video_path: Path, out_dir: Path, fps: float = 25.0):
    out_dir.mkdir(parents=True, exist_ok=True)
    rgb_dir = out_dir / 'rgb'
    rgb_dir.mkdir(exist_ok=True)
    rgb_txt = out_dir / 'rgb.txt'

    if rgb_txt.exists() and len(list(rgb_dir.glob('*.png'))) > 0:
        print(f"  Frames already extracted to {rgb_dir}, skipping.")
        return

    print(f"  Extracting frames from {video_path} ...")
    cap = cv2.VideoCapture(str(video_path))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or fps
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    lines = ['# timestamp filename']
    idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        ts = idx / actual_fps
        name = f"{ts:.6f}.png"
        cv2.imwrite(str(rgb_dir / name), frame)
        lines.append(f"{ts:.6f} rgb/{name}")
        idx += 1
        if idx % 100 == 0:
            print(f"    {idx}/{n_total} frames", end='\r')
    cap.release()
    rgb_txt.write_text('\n'.join(lines) + '\n')
    print(f"\n  Extracted {idx} frames → {out_dir}")


# ---------------------------------------------------------------------------
# Write YAML calibration
# ---------------------------------------------------------------------------
def write_yaml(out_path: Path):
    out_path.write_text(TUM_FR1_YAML)
    print(f"  Wrote calibration: {out_path}")


# ---------------------------------------------------------------------------
# Run ORB-SLAM2 mono_tum
# ---------------------------------------------------------------------------
def run_orbslam2(slam_dir: Path, seq_dir: Path, yaml_path: Path, traj_out: Path) -> bool:
    binary = slam_dir / 'Examples' / 'Monocular' / 'mono_tum'
    vocab  = slam_dir / 'Vocabulary' / 'ORBvoc.txt'
    rgb_txt = seq_dir / 'rgb.txt'

    if not binary.exists():
        print(f"ERROR: binary not found: {binary}")
        print("Run setup_orbslam2.sh first.")
        return False
    if not vocab.exists():
        print(f"ERROR: vocabulary not found: {vocab}")
        return False

    # mono_tum expects: Vocabulary Settings Sequence
    cmd = [str(binary), str(vocab), str(yaml_path), str(seq_dir)]
    print(f"  $ {' '.join(cmd)}")

    env = os.environ.copy()
    # Suppress display in WSL
    env.pop('DISPLAY', None)

    t0 = time.time()
    result = subprocess.run(
        cmd,
        env=env,
        capture_output=False,   # let output stream to terminal
        text=True,
        check=False,
        cwd=str(slam_dir),
    )
    elapsed = time.time() - t0
    print(f"  ORB-SLAM2 exited with code {result.returncode} in {elapsed:.1f}s")

    # ORB-SLAM2 mono_tum writes KeyFrameTrajectory.txt in cwd
    kf_traj = slam_dir / 'KeyFrameTrajectory.txt'
    frame_traj = slam_dir / 'CameraTrajectory.txt'  # if saved
    for candidate in [frame_traj, kf_traj]:
        if candidate.exists():
            shutil.copy(candidate, traj_out)
            print(f"  Trajectory saved: {traj_out} (from {candidate.name})")
            return True

    print("  WARNING: no trajectory file found after running ORB-SLAM2.")
    return False


# ---------------------------------------------------------------------------
# Parse ORB-SLAM2 TUM-format trajectory: timestamp tx ty tz qx qy qz qw
# ---------------------------------------------------------------------------
def parse_tum_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Returns (timestamps Nx1, positions Nx3) camera centers in world frame."""
    timestamps, positions = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        ts = float(parts[0])
        tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
        timestamps.append(ts)
        positions.append([tx, ty, tz])
    return np.array(timestamps), np.array(positions)


# ---------------------------------------------------------------------------
# Load GT camera centers from npz
# ---------------------------------------------------------------------------
def load_gt(npz_path: Path, fps: float = 25.0):
    data = np.load(npz_path)
    poses = data['pose']  # (N, 4, 4) world->cam
    R = poses[:, :3, :3]
    t = poses[:, :3, 3]
    centers = (-R.transpose(0, 2, 1) @ t[:, :, None]).squeeze(-1)
    timestamps = np.arange(len(centers)) / fps
    return timestamps, centers


# ---------------------------------------------------------------------------
# Match estimated trajectory timestamps to GT timestamps (nearest neighbor)
# ---------------------------------------------------------------------------
def match_trajectories(est_ts, est_pos, gt_ts, gt_pos, max_diff=0.1):
    matched_est, matched_gt = [], []
    for i, ts in enumerate(est_ts):
        diffs = np.abs(gt_ts - ts)
        j = int(np.argmin(diffs))
        if diffs[j] < max_diff:
            matched_est.append(est_pos[i])
            matched_gt.append(gt_pos[j])
    return np.array(matched_est), np.array(matched_gt)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--slam_dir', default=str(Path.home() / 'orb_slam2'))
    parser.add_argument('--video', default='test_freiburgxyz525.mp4')
    parser.add_argument('--gt_npz', default='test_freiburgxyz525.npz')
    parser.add_argument('--seq_dir', default='/tmp/freiburg_tum')
    parser.add_argument('--reextract', action='store_true')
    parser.add_argument('--skip_run', action='store_true', help='Skip ORB-SLAM2 run, use existing trajectory')
    args = parser.parse_args()

    root = Path(__file__).parent
    slam_dir = Path(args.slam_dir)
    video_path = root / args.video
    gt_path = root / args.gt_npz
    seq_dir = Path(args.seq_dir)
    yaml_path = seq_dir / 'TUM_fr1.yaml'
    traj_out = root / 'runs' / 'benchmark' / 'orbslam2_trajectory.txt'
    traj_out.parent.mkdir(parents=True, exist_ok=True)

    if not video_path.exists():
        print(f"Video not found: {video_path}")
        sys.exit(1)
    if not gt_path.exists():
        print(f"GT not found: {gt_path}")
        sys.exit(1)

    # Step 1: extract frames
    if args.reextract and seq_dir.exists():
        shutil.rmtree(seq_dir)
    print("[1] Extracting frames")
    extract_frames(video_path, seq_dir)
    write_yaml(yaml_path)

    # Step 2: run ORB-SLAM2
    if not args.skip_run:
        print("[2] Running ORB-SLAM2")
        ok = run_orbslam2(slam_dir, seq_dir, yaml_path, traj_out)
        if not ok:
            print("ORB-SLAM2 failed or produced no trajectory.")
            sys.exit(1)
    else:
        if not traj_out.exists():
            print(f"--skip_run set but no trajectory at {traj_out}")
            sys.exit(1)
        print(f"[2] Skipping ORB-SLAM2 run, using {traj_out}")

    # Step 3: compute ATE
    print("[3] Computing ATE")
    est_ts, est_pos = parse_tum_trajectory(traj_out)
    gt_ts, gt_pos = load_gt(gt_path, fps=25.0)
    print(f"  ORB-SLAM2 trajectory: {len(est_ts)} poses")
    print(f"  GT:                   {len(gt_ts)} poses")

    est_m, gt_m = match_trajectories(est_ts, est_pos, gt_ts, gt_pos)
    if len(est_m) < 3:
        print("Not enough matched poses for ATE computation.")
        sys.exit(1)
    print(f"  Matched pairs: {len(est_m)}")

    result = ate(est_m, gt_m)
    print(f"\n  ATE RMSE:   {result['rmse']:.4f} m")
    print(f"  ATE median: {result['median']:.4f} m")
    print(f"  ATE max:    {result['max']:.4f} m")
    print(f"  Scale:      {result['scale']:.5f}")

    # Step 4: load our simple_slam result for comparison
    our_label = 'simple_slam'
    our_path = root / 'runs' / 'benchmark' / 'test_freiburgxyz525.json'
    if our_path.exists():
        our_label = 'simple_slam v1.24'
    else:
        our_path = root / 'runs' / 'benchmark' / 'test_freiburgxyz525_v122.json'
        if our_path.exists():
            our_label = 'simple_slam v1.22'
    our_ate = None
    if our_path.exists():
        m = json.loads(our_path.read_text())
        tl = m.get('timeline', [])
        fids = [f['frame_id'] for f in tl if 'xyz' in f]
        est_xyz = np.array([f['xyz'] for f in tl if 'xyz' in f])
        _, gt_centers = load_gt(gt_path, fps=25.0)
        valid = [i for i, fid in enumerate(fids) if fid < len(gt_centers)]
        if len(valid) >= 3:
            our_ate = ate(est_xyz[valid], gt_centers[[fids[i] for i in valid]])

    # Step 5: print comparison table
    print(f"\n{'='*60}")
    print("COMPARISON — test_freiburgxyz525 (monocular, no IMU)")
    print(f"{'='*60}")
    print(f"{'System':<20} {'ATE RMSE':>10} {'ATE med':>10} {'ATE max':>10} {'n':>6}")
    print('-' * 60)
    print(f"{'ORB-SLAM2':<20} {result['rmse']:>10.4f} {result['median']:>10.4f} {result['max']:>10.4f} {result['n']:>6}")
    if our_ate:
        print(f"{our_label:<20} {our_ate['rmse']:>10.4f} {our_ate['median']:>10.4f} {our_ate['max']:>10.4f} {our_ate['n']:>6}")
    print(f"\n  All errors in metres after Umeyama similarity alignment.")

    # Save comparison JSON
    comparison = {
        'sequence': 'test_freiburgxyz525',
        'orbslam2': result,
        'simple_slam': our_ate,
        'simple_slam_label': our_label,
        'simple_slam_source': str(our_path.relative_to(root)) if our_path.exists() else None,
    }
    out_json = root / 'runs' / 'benchmark' / 'comparison_orbslam2.json'
    out_json.write_text(json.dumps(comparison, indent=2))
    print(f"\n  Saved → {out_json}")

    # Update BENCHMARKS.md
    _update_benchmarks_md(root, result, our_ate)


def _update_benchmarks_md(root: Path, orb: dict, ours: dict | None):
    bm = root / 'BENCHMARKS.md'
    if not bm.exists():
        return
    content = bm.read_text()
    orb_line = (f"| ORB-SLAM2 | {orb['rmse']:.4f} m | {orb['median']:.4f} m "
                f"| {orb['max']:.4f} m | — | — | — | reference |")
    if 'ORB-SLAM2' not in content:
        # Insert after the v1.22 row
        insert_after = '| v1.22'
        idx = content.find(insert_after)
        if idx != -1:
            end = content.find('\n', idx) + 1
            content = content[:end] + orb_line + '\n' + content[end:]
            bm.write_text(content)
            print(f"  Updated BENCHMARKS.md with ORB-SLAM2 result.")


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    main()
