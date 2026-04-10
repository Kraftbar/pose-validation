import itertools
import json
import math
import os
import random
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


def find_videos(pattern: str = "test_*.mp4"):
    return sorted(Path('.').glob(pattern))


def run_cmd(cmd, cwd=None):
    print("$", " ".join(cmd))
    start = time.time()
    try:
        out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, check=False)
        dur = time.time() - start
        print(out.stdout.strip())
        if out.stderr.strip():
            print(out.stderr.strip(), file=sys.stderr)
        return out.returncode, dur
    except FileNotFoundError as e:
        print(f"Failed to run command: {e}")
        return 127, 0.0


def score_metric(m: dict) -> float:
    points = m.get('points', 0)
    frames = m.get('frames', 0)
    avg_inl = m.get('avg_inliers_after_first', 0.0)
    kfs = m.get('keyframes', 0)
    # Weight points highest, then frames, then inliers, mildly penalize too many keyframes
    return float(points + 0.2 * frames + 0.05 * avg_inl - 0.02 * max(0, kfs - 10))


def ensure_dir(p: Path):
    p.parent.mkdir(parents=True, exist_ok=True)


def build_cmd(video: Path, seconds: int, timeout: int, params: dict, metrics_path: Path):
    ensure_dir(metrics_path)
    cmd = [
        sys.executable,
        'simple_slam.py',
        '--video_path', str(video),
        '--seconds', str(seconds),
        '--timeout', str(timeout),
        '--no_imshow', '--no_plot',
        '--metrics_out', str(metrics_path),
        '--use_pnp', str(params['use_pnp']),
        '--pnp_min_corr', str(params['pnp_min_corr']),
        '--kf_min_inliers', str(params['kf_min_inliers']),
        '--kf_max_rot_deg', str(params['kf_max_rot_deg']),
    ]
    # Optional knobs if provided
    if 'inlier_min_for_tri' in params:
        cmd += ['--inlier_min_for_tri', str(params['inlier_min_for_tri'])]
    if 'max_points' in params:
        cmd += ['--max_points', str(params['max_points'])]
    if 'cull_min_obs' in params:
        cmd += ['--cull_min_obs', str(params['cull_min_obs'])]
    return cmd


def param_grid_seed():
    return {
        'use_pnp': [1],
        'pnp_min_corr': [8, 12, 16],
        'kf_min_inliers': [30, 40, 60],
        'kf_max_rot_deg': [3.0, 5.0, 8.0],
        'inlier_min_for_tri': [16, 24, 32],
        'max_points': [15000, 25000, 35000],
        'cull_min_obs': [2, 3],
    }


def sample_random(n: int, seed_best: dict | None = None) -> list[dict]:
    rng = random.Random(42)
    cfgs = []
    for _ in range(n):
        cfg = {
            'use_pnp': 1,
            'pnp_min_corr': rng.randint(6, 20),
            'kf_min_inliers': rng.randint(20, 80),
            'kf_max_rot_deg': round(rng.uniform(2.0, 12.0), 2),
            'inlier_min_for_tri': rng.randint(12, 40),
            'max_points': rng.choice([12000, 15000, 25000, 35000, 50000]),
            'cull_min_obs': rng.choice([2, 3]),
        }
        if seed_best:
            # small perturbation around best
            for k in ['pnp_min_corr', 'kf_min_inliers', 'inlier_min_for_tri']:
                v = int(round(seed_best.get(k, cfg[k])))
                cfg[k] = max(1, v + rng.randint(-4, 4))
            cfg['kf_max_rot_deg'] = max(1.0, float(seed_best.get('kf_max_rot_deg', cfg['kf_max_rot_deg'])) + rng.uniform(-2.0, 2.0))
        cfgs.append(cfg)
    return cfgs


def tag_from_params(params: dict, seconds: int) -> str:
    core = "_".join(f"{k}-{params[k]}" for k in sorted(params.keys()))
    return f"sec-{seconds}_" + core


def run_configs_on_videos(configs: list[dict], videos: list[Path], seconds: int, timeout: int, jobs: int) -> list[dict]:
    out_root = Path('runs')
    results: list[dict] = []
    tasks = []
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
        for cfg in configs:
            tag = tag_from_params(cfg, seconds)
            futs = []
            for vid in videos:
                metrics_path = out_root / vid.stem / f"{tag}.json"
                if metrics_path.exists():
                    # skip running, but still record existing
                    try:
                        m = json.loads(metrics_path.read_text())
                        results.append({'video': vid.name, 'params': cfg, 'seconds': seconds, 'metrics_path': str(metrics_path), 'metrics': m})
                    except Exception:
                        pass
                    continue
                cmd = build_cmd(vid, seconds, timeout, cfg, metrics_path)
                fut = ex.submit(run_cmd, cmd)
                tasks.append((fut, vid, cfg, metrics_path))
            # Gather as they finish
        for fut, vid, cfg, metrics_path in tasks:
            code, dur = fut.result()
            if code == 0 and metrics_path.exists():
                try:
                    m = json.loads(metrics_path.read_text())
                    results.append({'video': vid.name, 'params': cfg, 'seconds': seconds, 'metrics_path': str(metrics_path), 'metrics': m})
                except Exception:
                    pass
            else:
                fail = {
                    'video': str(vid),
                    'params': cfg,
                    'returncode': code,
                    'duration_sec': dur,
                }
                metrics_path.with_suffix('.fail.json').write_text(json.dumps(fail, indent=2))
    return results


def aggregate_scores(results: list[dict]) -> dict:
    # Aggregate per-config average score across videos
    by_cfg: dict[str, list[float]] = {}
    detail: dict[str, dict] = {}
    for r in results:
        m = r.get('metrics') or {}
        s = score_metric(m)
        key = json.dumps({k: r['params'][k] for k in sorted(r['params'].keys())}, sort_keys=True)
        by_cfg.setdefault(key, []).append(s)
        detail.setdefault(key, {'params': r['params'], 'per_video': []})['per_video'].append({'video': r['video'], 'score': s, 'frames': m.get('frames', 0), 'points': m.get('points', 0)})
    leaderboard = []
    for key, scores in by_cfg.items():
        avg = sum(scores) / len(scores)
        leaderboard.append({'params_key': key, 'avg_score': avg, **detail[key]})
    leaderboard.sort(key=lambda x: x['avg_score'], reverse=True)
    Path('runs').mkdir(exist_ok=True)
    Path('runs/leaderboard.json').write_text(json.dumps(leaderboard, indent=2))
    return {'leaderboard': leaderboard, 'detail': detail}


def successive_halving(videos: list[Path], stage_seconds: list[int], timeout: int, jobs: int):
    # Stage 0: seed grid
    grid = param_grid_seed()
    combos = list(itertools.product(*grid.values()))
    keys = list(grid.keys())
    seed_configs = [dict(zip(keys, vals)) for vals in combos]
    print(f"Stage 1: running {len(seed_configs)} grid configs @ {stage_seconds[0]}s")
    res1 = run_configs_on_videos(seed_configs, videos, seconds=stage_seconds[0], timeout=timeout, jobs=jobs)
    agg1 = aggregate_scores(res1)
    top_k = max(3, len(agg1['leaderboard']) // 4)
    top_configs = [e['params'] for e in agg1['leaderboard'][:top_k]]

    # Stage 2: random explore around top
    random_configs = []
    for best in top_configs:
        random_configs += sample_random(6, seed_best=best)
    print(f"Stage 2: running {len(random_configs)} random configs @ {stage_seconds[1]}s")
    res2 = run_configs_on_videos(random_configs, videos, seconds=stage_seconds[1], timeout=timeout, jobs=jobs)
    agg2 = aggregate_scores(res2)
    merged = agg1['leaderboard'] + agg2['leaderboard']
    merged.sort(key=lambda x: x['avg_score'], reverse=True)
    top_k2 = max(3, len(merged) // 5)
    finalists = [e['params'] for e in merged[:top_k2]]

    # Stage 3: finalists, longer seconds
    if len(stage_seconds) > 2:
        print(f"Stage 3: running {len(finalists)} finalists @ {stage_seconds[2]}s")
        res3 = run_configs_on_videos(finalists, videos, seconds=stage_seconds[2], timeout=timeout, jobs=jobs)
        aggregate_scores(res3)


def main():
    videos_glob = os.environ.get('SLAM_VIDEOS', 'test_*.mp4')
    videos = find_videos(videos_glob)
    if not videos:
        print(f"No videos found matching {videos_glob}")
        return

    # Duration per stage and timeout (env overrides)
    seconds_s1 = int(os.environ.get('SLAM_SECONDS_S1', '4'))
    seconds_s2 = int(os.environ.get('SLAM_SECONDS_S2', '8'))
    seconds_s3 = int(os.environ.get('SLAM_SECONDS_S3', '15'))
    timeout = int(os.environ.get('SLAM_TIMEOUT', '60'))
    jobs = int(os.environ.get('SLAM_JOBS', '1'))

    successive_halving(videos, [seconds_s1, seconds_s2, seconds_s3], timeout=timeout, jobs=jobs)
    print("Done adaptive sweep. See runs/leaderboard.json and runs_summary.csv after analyze_runs.py.")


if __name__ == '__main__':
    main()
