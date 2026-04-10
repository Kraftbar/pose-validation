import json
from pathlib import Path
import csv
from typing import Any, Dict, List, Tuple


def read_metrics(root: Path) -> List[Tuple[Path, Dict[str, Any]]]:
    items: List[Tuple[Path, Dict[str, Any]]] = []
    for p in root.glob('runs/*/*.json'):
        if p.parent.name == 'best':
            continue
        if p.suffix != '.json' or p.name.endswith('.fail.json'):
            continue
        if p.name == 'summary.json' or p.name.startswith('comparison_'):
            continue
        try:
            data = json.loads(p.read_text())
        except Exception:
            continue
        if not isinstance(data, dict):
            continue
        items.append((p, data))
    return items


def parse_params_from_name(name: str) -> Dict[str, Any]:
    # Expect pattern like use_pnp-1_pnp_min_corr-12_kf_min_inliers-40_...
    params: Dict[str, Any] = {}
    for part in name.replace('.json', '').split('_'):
        if '-' in part:
            k, v = part.split('-', 1)
            try:
                if v.isdigit():
                    params[k] = int(v)
                else:
                    params[k] = float(v)
            except Exception:
                params[k] = v
    return params


def score_metric(m: Dict[str, Any]) -> float:
    # Simple heuristic: prioritize more points and frames processed within time
    points = m.get('points', 0)
    frames = m.get('frames', 0)
    avg_inl = m.get('avg_inliers_after_first', 0.0)
    kfs = m.get('keyframes', 0)
    # Weight points highest, then frames, then inliers, mildly penalize too many keyframes
    return points + 0.2 * frames + 0.05 * avg_inl - 0.02 * max(0, kfs - 10)


def summarize(items: List[Tuple[Path, Dict[str, Any]]]) -> None:
    if not items:
        print('No metrics found under runs/')
        return
    rows: List[Dict[str, Any]] = []
    for p, m in items:
        params = parse_params_from_name(p.name)
        video_path = m.get('video_path')
        if isinstance(video_path, str) and video_path:
            video_name = Path(video_path).stem
        else:
            video_name = p.stem
        row = {
            'group': p.parent.name,
            'video': video_name,
            'path': str(p),
            'score': round(score_metric(m), 3),
            'frames': m.get('frames', 0),
            'points': m.get('points', 0),
            'avg_inliers': m.get('avg_inliers_after_first', 0.0),
            'keyframes': m.get('keyframes', 0),
            'pnp_frames': m.get('pnp_frames', 0),
            'ba_runs': m.get('ba_runs', 0),
            **params,
        }
        rows.append(row)

    # Write CSV summary
    out_csv = Path('runs_summary.csv')
    # Union of all keys for columns
    keys: List[str] = []
    for r in rows:
        for k in r.keys():
            if k not in keys:
                keys.append(k)
    with out_csv.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f'Wrote {out_csv} with {len(rows)} rows')

    # Best per video
    videos = {}
    for r in rows:
        videos.setdefault(r['video'], []).append(r)
    best_dir = Path('runs/best')
    best_dir.mkdir(parents=True, exist_ok=True)
    for p in best_dir.glob('*_best.json'):
        p.unlink()
    for vid, rs in videos.items():
        best = sorted(rs, key=lambda x: x['score'], reverse=True)[0]
        out = best_dir / f'{vid}_best.json'
        out.write_text(json.dumps(best, indent=2))
    print(f'Wrote best-per-video JSONs to {best_dir}')


if __name__ == '__main__':
    summarize(read_metrics(Path('.')))
