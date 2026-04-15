"""
Benchmark local SLAM implementations on local test videos.
For videos with a matching .npz ground truth, computes ATE (Absolute Trajectory Error)
using Umeyama alignment (handles monocular scale ambiguity).
For all videos reports heuristic score (points/frames/inliers).

Usage:
    python benchmark.py --impl python
    python benchmark.py --impl cpp,c --video test_freiburgxyz525 --seconds 5
    python benchmark.py --impl all --video test_freiburgxyz525
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


DEFAULT_BENCHMARK_SECONDS = 30.0
IMPLEMENTATIONS = ('python', 'cpp', 'c', 'pure_c', 'pure_c_brief', 'pure_c_orb')


# ---------------------------------------------------------------------------
# Umeyama similarity alignment  (estimate: Nx3, reference: Nx3)
# Returns aligned estimate, scale, R, t such that  est_aligned = s*R @ est.T + t
# ---------------------------------------------------------------------------
def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """Align src -> dst via similarity transform.  Returns (R, t, scale)."""
    assert src.shape == dst.shape and src.ndim == 2 and src.shape[1] == 3
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    var_s = (src_c ** 2).sum() / n
    cov = (dst_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = float((D * S.diagonal()).sum() / var_s) if with_scale and var_s > 1e-12 else 1.0
    t = mu_d - scale * R @ mu_s
    return R, t, scale


def apply_alignment(src: np.ndarray, R, t, scale) -> np.ndarray:
    return (scale * R @ src.T).T + t


def ate_rmse(est: np.ndarray, gt: np.ndarray) -> dict:
    """Compute ATE after Umeyama alignment.  est and gt are Nx3 camera centers."""
    R, t, scale = umeyama_alignment(est, gt, with_scale=True)
    est_aligned = apply_alignment(est, R, t, scale)
    errors = np.linalg.norm(est_aligned - gt, axis=1)
    return {
        'ate_rmse': float(np.sqrt((errors ** 2).mean())),
        'ate_median': float(np.median(errors)),
        'ate_max': float(errors.max()),
        'scale': float(scale),
        'n_frames': int(len(errors)),
    }


# ---------------------------------------------------------------------------
# GT helpers
# ---------------------------------------------------------------------------
def load_gt_centers(npz_path: str) -> np.ndarray:
    """Load Nx3 camera centers from a .npz with 'pose' key (Nx4x4 world->cam)."""
    d = np.load(npz_path)
    poses = d['pose']  # (N, 4, 4)
    R = poses[:, :3, :3]
    t = poses[:, :3, 3]
    # camera center = -R^T @ t
    centers = (-R.transpose(0, 2, 1) @ t[:, :, None]).squeeze(-1)
    return centers.astype(np.float64)


# ---------------------------------------------------------------------------
# Run SLAM
# ---------------------------------------------------------------------------
def needs_rebuild(target: Path, sources: list[Path]) -> bool:
    if not target.exists():
        return True
    target_mtime = target.stat().st_mtime
    return any(source.exists() and source.stat().st_mtime > target_mtime for source in sources)


def ensure_native_binary(root: Path, impl: str) -> Path:
    binary_name = {
        'cpp': 'simple_slam_opt',
        'c': 'simple_slam_c',
    }.get(impl)
    if not binary_name:
        raise ValueError(f'Unsupported native implementation: {impl}')

    binary_path = root / 'build-native' / binary_name
    sources = [root / 'CMakeLists.txt']
    if impl == 'cpp':
        sources.append(root / 'simple_slam_opt.cpp')
    else:
        sources.extend([
            root / 'simple_slam_c.c',
            root / 'simple_slam_c_shim.cpp',
            root / 'simple_slam_c_shim.h',
            root / 'pure_c_math.h',
        ])

    if not needs_rebuild(binary_path, sources):
        return binary_path

    print(f'  building native target `{binary_name}` ...')
    configure = subprocess.run(
        ['cmake', '-S', str(root), '-B', str(root / 'build-native')],
        capture_output=True, text=True, check=False,
    )
    if configure.returncode != 0:
        if configure.stdout.strip():
            print(configure.stdout.strip())
        if configure.stderr.strip():
            print(configure.stderr.strip(), file=sys.stderr)
        raise RuntimeError('Failed to configure native build')

    build = subprocess.run(
        ['cmake', '--build', str(root / 'build-native'), '-j2', '--target', binary_name],
        capture_output=True, text=True, check=False,
    )
    if build.returncode != 0:
        if build.stdout.strip():
            print(build.stdout.strip())
        if build.stderr.strip():
            print(build.stderr.strip(), file=sys.stderr)
        raise RuntimeError(f'Failed to build native target `{binary_name}`')

    if not binary_path.exists():
        raise RuntimeError(f'Native binary missing after build: {binary_path}')
    return binary_path


def ensure_pure_c_binary(root: Path, source_name: str = 'simple_slam_c.c', binary_name: str = 'simple_slam_pure_c') -> Path:
    root = root.resolve()
    compiler = shutil.which('gcc')
    if not compiler:
        raise RuntimeError('gcc not found; cannot build `simple_slam_pure_c`')

    binary_path = root / binary_name
    sources = [root / source_name, root / 'pure_c_math.h']
    if not needs_rebuild(binary_path, sources):
        return binary_path

    build = subprocess.run(
        [compiler, '-O3', '-march=native', '-fopenmp', source_name, '-o', str(binary_path), '-lm'],
        capture_output=True, text=True, check=False, cwd=root,
    )
    if build.returncode != 0:
        if build.stdout.strip():
            print(build.stdout.strip())
        if build.stderr.strip():
            print(build.stderr.strip(), file=sys.stderr)
        raise RuntimeError('Failed to build native target `simple_slam_pure_c`')

    if not binary_path.exists():
        raise RuntimeError(f'Native binary missing after build: {binary_path}')
    return binary_path


def build_slam_command(root: Path, impl: str, video: Path, seconds: float, timeout: float,
                       extra_args: list, out_json: Path) -> list[str]:
    if impl == 'python':
        return [
            sys.executable, 'simple_slam.py',
            '--video_path', str(video),
            '--seconds', str(seconds),
            '--timeout', str(timeout),
            '--no_imshow', '--no_plot',
            '--metrics_out', str(out_json),
        ] + extra_args
    if impl in {'cpp', 'c'}:
        binary = ensure_native_binary(root, impl)
        return [
            str(binary),
            '--video_path', str(video),
            '--seconds', str(seconds),
            '--timeout', str(timeout),
            '--metrics_out', str(out_json),
        ] + extra_args
    if impl == 'pure_c':
        binary = ensure_pure_c_binary(root, source_name='simple_slam_c.c', binary_name='simple_slam_pure_c')
        return [
            str(binary),
            '--video_path', str(video),
            '--seconds', str(seconds),
            '--timeout', str(timeout),
            '--metrics_out', str(out_json),
        ] + extra_args
    if impl == 'pure_c_brief':
        binary = ensure_pure_c_binary(root, source_name='simple_slam_c_brief.c', binary_name='simple_slam_pure_c_brief')
        return [
            str(binary),
            '--video_path', str(video),
            '--seconds', str(seconds),
            '--timeout', str(timeout),
            '--metrics_out', str(out_json),
        ] + extra_args
    if impl == 'pure_c_orb':
        binary = ensure_pure_c_binary(root, source_name='simple_slam_c_orb.c', binary_name='simple_slam_pure_c_orb')
        return [
            str(binary),
            '--video_path', str(video),
            '--seconds', str(seconds),
            '--timeout', str(timeout),
            '--metrics_out', str(out_json),
        ] + extra_args
    raise ValueError(f'Unsupported implementation: {impl}')


def run_slam(root: Path, impl: str, video: Path, seconds: float, timeout: float,
             extra_args: list, out_json: Path) -> bool:
    out_json.parent.mkdir(parents=True, exist_ok=True)
    cmd = build_slam_command(root, impl, video, seconds, timeout, extra_args, out_json)
    print(f"  $ {' '.join(cmd)}")
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    elapsed = time.time() - t0
    if r.stdout.strip():
        print(r.stdout.strip())
    if r.returncode != 0 and r.stderr.strip():
        print(r.stderr.strip(), file=sys.stderr)
    return r.returncode == 0 and out_json.exists()


# ---------------------------------------------------------------------------
# Heuristic score (same as sweep_slam / analyze_runs)
# ---------------------------------------------------------------------------
def heuristic_score(m: dict) -> float:
    return (m.get('points', 0)
            + 0.2 * m.get('frames', 0)
            + 0.05 * m.get('avg_inliers_after_first', 0.0)
            - 0.02 * max(0, m.get('keyframes', 0) - 10))


def format_scale(scale: float) -> str:
    scale = float(scale)
    if scale == 0.0:
        return '0'
    abs_scale = abs(scale)
    if abs_scale < 1e-3 or abs_scale >= 1e4:
        return f'{scale:.3e}'
    return f'{scale:.4f}'


def seconds_tag(seconds: float) -> str:
    seconds = float(seconds)
    if seconds.is_integer():
        return f'{int(seconds)}s'
    return f"{str(seconds).replace('.', 'p')}s"


def benchmark_json_path(out_dir: Path, stem: str, seconds: float, impl: str) -> Path:
    impl_suffix = '' if impl == 'python' else f'_{impl}'
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / f'{stem}{impl_suffix}.json'
    return out_dir / f'{stem}{impl_suffix}_{seconds_tag(seconds)}.json'


def summary_json_path(out_dir: Path, seconds: float, impl: str) -> Path:
    impl_suffix = '' if impl == 'python' else f'_{impl}'
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / f'summary{impl_suffix}.json'
    return out_dir / f'summary{impl_suffix}_{seconds_tag(seconds)}.json'


def parse_impls(value: str) -> list[str]:
    if value == 'all':
        return list(IMPLEMENTATIONS)
    impls = [item.strip() for item in value.split(',') if item.strip()]
    invalid = [item for item in impls if item not in IMPLEMENTATIONS]
    if invalid:
        raise ValueError(f'Unknown implementations: {", ".join(invalid)}')
    return impls


def select_videos(root: Path, requested: list[str] | None) -> list[Path]:
    videos = sorted(root.glob('test_*.mp4'))
    if not requested:
        return videos

    selected = []
    requested_set = set(requested)
    for video in videos:
        if video.name in requested_set or video.stem in requested_set:
            selected.append(video)

    missing = [name for name in requested if name not in {v.name for v in selected} and name not in {v.stem for v in selected}]
    if missing:
        raise ValueError(f'Unknown video(s): {", ".join(missing)}')
    return selected


def select_gt_videos(root: Path, requested: list[str] | None) -> list[Path]:
    candidates: dict[str, Path] = {}
    search_dirs = [root, root / 'external' / 'twitchslam' / 'videos']
    for directory in search_dirs:
        if not directory.exists():
            continue
        for video in sorted(directory.glob('test_*.mp4')):
            if not video.with_suffix('.npz').exists():
                continue
            candidates.setdefault(video.stem, video)

    if not requested:
        return [candidates[stem] for stem in sorted(candidates)]

    selected = []
    requested_set = set(requested)
    for stem in sorted(candidates):
        video = candidates[stem]
        if video.name in requested_set or video.stem in requested_set:
            selected.append(video)

    missing = [name for name in requested if name not in {v.name for v in selected} and name not in {v.stem for v in selected}]
    if missing:
        raise ValueError(f'Unknown GT-backed video(s): {", ".join(missing)}')
    return selected


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=DEFAULT_BENCHMARK_SECONDS,
                        help='Seconds of video to process per run')
    parser.add_argument('--timeout', type=float, default=120.0, help='Hard timeout per run (seconds)')
    parser.add_argument('--out_dir', type=str, default='runs/benchmark', help='Directory for output JSONs')
    parser.add_argument('--impl', type=str, default='python',
                        help='Implementation: python, cpp, c, pure_c, pure_c_brief, comma-separated list, or all')
    parser.add_argument('--video', action='append', default=None,
                        help='Video stem or filename to benchmark; may be repeated')
    parser.add_argument('--all_gt', action='store_true',
                        help='Benchmark all videos in the repo that have adjacent ground-truth .npz files')
    parser.add_argument('--force', action='store_true', help='Re-run even if output already exists')
    args = parser.parse_args()

    root = Path('.')
    out_dir = Path(args.out_dir)
    impls = parse_impls(args.impl)

    # Discover test videos
    videos = select_gt_videos(root, args.video) if args.all_gt else select_videos(root, args.video)
    if not videos:
        message = 'No GT-backed test_*.mp4 found.' if args.all_gt else 'No test_*.mp4 found in current directory.'
        print(message)
        return

    results = []
    print(f"\n{'='*60}")
    print(f"Benchmarking {len(videos)} videos x {len(impls)} impls  ({args.seconds}s each)")
    print(f"{'='*60}\n")

    for video in videos:
        stem = video.stem
        gt_npz = video.with_suffix('.npz')

        print(f"[{stem}]")
        for impl in impls:
            out_json = benchmark_json_path(out_dir, stem, args.seconds, impl)
            print(f"  ({impl})")
            if out_json.exists() and not args.force:
                print(f"    cached → {out_json}")
                m = json.loads(out_json.read_text())
            else:
                ok = run_slam(root, impl, video, args.seconds, args.timeout, [], out_json)
                if not ok:
                    print(f"    FAILED")
                    results.append({'video': stem, 'impl': impl, 'error': 'slam failed'})
                    continue
                m = json.loads(out_json.read_text())

            row = {
                'video': stem,
                'impl': impl,
                'frames': m.get('frames', 0),
                'points': m.get('points', 0),
                'keyframes': m.get('keyframes', 0),
                'avg_inliers': round(m.get('avg_inliers_after_first', 0.0), 2),
                'pnp_frames': m.get('pnp_frames', 0),
                'ba_runs': m.get('ba_runs', 0),
                'duration_sec': m.get('duration_sec', 0.0),
                'heuristic_score': round(heuristic_score(m), 1),
                'out_json': str(out_json),
            }

            if gt_npz.exists():
                timeline = m.get('timeline', [])
                est_xyz = [f['xyz'] for f in timeline if 'xyz' in f]
                if len(est_xyz) >= 3:
                    est = np.array(est_xyz, dtype=np.float64)
                    gt_all = load_gt_centers(str(gt_npz))
                    frame_ids = [f['frame_id'] for f in timeline if 'xyz' in f]
                    valid = [i for i, fid in enumerate(frame_ids) if fid < len(gt_all)]
                    if len(valid) >= 3:
                        est_matched = est[valid]
                        gt_matched = gt_all[[frame_ids[i] for i in valid]]
                        ate = ate_rmse(est_matched, gt_matched)
                        row.update({
                            'ate_rmse': round(ate['ate_rmse'], 4),
                            'ate_median': round(ate['ate_median'], 4),
                            'ate_max': round(ate['ate_max'], 4),
                            'ate_scale': float(ate['scale']),
                            'ate_scale_display': format_scale(ate['scale']),
                            'ate_n': ate['n_frames'],
                        })
                    else:
                        row['ate_note'] = 'not enough matched frames'
                else:
                    row['ate_note'] = 'no xyz in timeline (re-run to update)'
            else:
                row['ate_note'] = 'no GT'

            results.append(row)
            print(f"    frames={row['frames']}  points={row['points']}  kf={row['keyframes']}  "
                  f"avg_inl={row['avg_inliers']}  score={row['heuristic_score']}")
            if 'ate_rmse' in row:
                print(f"    ATE  rmse={row['ate_rmse']}  median={row['ate_median']}  "
                      f"max={row['ate_max']}  scale={row['ate_scale_display']}  n={row['ate_n']}")
            elif 'ate_note' in row:
                print(f"    ATE  {row['ate_note']}")
        print()

    # Summary table
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    hdr = f"{'impl':<8} {'video':<24} {'frames':>7} {'points':>7} {'score':>8} {'ATE_rmse':>10} {'ATE_n':>6}"
    print(hdr)
    print('-' * len(hdr))
    for r in results:
        ate_str = f"{r['ate_rmse']:>10.4f}" if 'ate_rmse' in r else f"{'n/a':>10}"
        ate_n_str = f"{r['ate_n']:>6}" if 'ate_n' in r else f"{'n/a':>6}"
        print(f"{r.get('impl','?'):<8} {r['video']:<24} {r.get('frames',0):>7} {r.get('points',0):>7} "
              f"{r.get('heuristic_score',0):>8.1f} {ate_str} {ate_n_str}")

    # Save summary
    summary_impl = 'all' if len(impls) > 1 else impls[0]
    summary_path = summary_json_path(out_dir, args.seconds, summary_impl)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(results, indent=2))
    print(f"\nSaved summary → {summary_path}")


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    main()
