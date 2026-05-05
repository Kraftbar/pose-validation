"""
Run and compare native SLAM implementations against the Python baseline.

This wrapper calls `benchmark.py --impl all`, then prints a comparison table with
runtime speedup relative to Python for each video.

Usage:
    python benchmark_native.py --video test_freiburgxyz525 --seconds 5
    python benchmark_native.py --seconds 30 --force
    python benchmark_native.py --skip_run --summary_json runs/benchmark/summary_all_5s.json

Outputs:
    runs/benchmark/native_comparison_5s.json
    runs/benchmark/native_comparison_5s.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable


DEFAULT_BENCHMARK_SECONDS = 30.0
IMPLEMENTATION_ORDER = ('python', 'cpp', 'c', 'pure_c', 'pure_c_brief', 'pure_c_orb', 'pure_c_plus')


def seconds_tag(seconds: float) -> str:
    seconds = float(seconds)
    if seconds.is_integer():
        return f'{int(seconds)}s'
    return f"{str(seconds).replace('.', 'p')}s"


def summary_json_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'summary_all.json'
    return out_dir / f'summary_all_{seconds_tag(seconds)}.json'


def infer_seconds_from_summary_path(path: Path) -> float | None:
    match = re.fullmatch(r'summary_all_(\d+(?:p\d+)?)s\.json', path.name)
    if not match:
        return DEFAULT_BENCHMARK_SECONDS if path.name == 'summary_all.json' else None
    return float(match.group(1).replace('p', '.'))


def run_benchmark(args: argparse.Namespace) -> Path:
    cmd = [
        sys.executable,
        'benchmark.py',
        '--impl',
        'all',
        '--seconds',
        str(args.seconds),
        '--timeout',
        str(args.timeout),
        '--out_dir',
        args.out_dir,
    ]
    if args.all_gt:
        cmd.append('--all_gt')
    for video in args.video or []:
        cmd.extend(['--video', video])
    if args.force:
        cmd.append('--force')
    if args.workers > 1:
        cmd.extend(['--workers', str(args.workers)])

    print(f"$ {' '.join(cmd)}", flush=True)
    result = subprocess.run(cmd, text=True, check=False)
    if result.returncode != 0:
        raise SystemExit(result.returncode)
    return summary_json_path(Path(args.out_dir), args.seconds)


def load_summary(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(f'Summary JSON not found: {path}')
    data = json.loads(path.read_text())
    if not isinstance(data, list):
        raise ValueError(f'Unexpected summary JSON format: {path}')
    return data


def group_by_video(rows: Iterable[dict]) -> dict[str, dict[str, dict]]:
    grouped: dict[str, dict[str, dict]] = {}
    for row in rows:
        video = row.get('video')
        impl = row.get('impl')
        if not video or not impl:
            continue
        grouped.setdefault(video, {})[impl] = row
    return grouped


def format_float(value: float | None, digits: int = 4) -> str:
    if value is None:
        return 'n/a'
    return f'{value:.{digits}f}'


def format_speedup(value: float | None) -> str:
    if value is None:
        return 'n/a'
    return f'{value:.2f}x'


def compute_comparison_rows(summary_rows: list[dict]) -> list[dict]:
    grouped = group_by_video(summary_rows)
    comparison_rows: list[dict] = []

    for video in sorted(grouped):
        per_impl = grouped[video]
        python_row = per_impl.get('python')
        python_duration = float(python_row.get('duration_sec', 0.0)) if python_row else None
        python_ate = float(python_row['ate_rmse']) if python_row and 'ate_rmse' in python_row else None

        for impl in IMPLEMENTATION_ORDER:
            row = per_impl.get(impl)
            if not row:
                continue

            duration = float(row.get('duration_sec', 0.0))
            ate_rmse = float(row['ate_rmse']) if 'ate_rmse' in row else None
            speedup_vs_python = None
            ate_delta_vs_python = None

            if python_duration and duration > 0:
                speedup_vs_python = python_duration / duration
            if python_ate is not None and ate_rmse is not None:
                ate_delta_vs_python = ate_rmse - python_ate

            comparison_rows.append({
                'video': video,
                'impl': impl,
                'frames': int(row.get('frames', 0)),
                'points': int(row.get('points', 0)),
                'duration_sec': duration,
                'speedup_vs_python': speedup_vs_python,
                'ate_rmse': ate_rmse,
                'ate_delta_vs_python': ate_delta_vs_python,
                'heuristic_score': float(row.get('heuristic_score', 0.0)),
                'summary_row': row,
            })
    return comparison_rows


def print_comparison_table(rows: list[dict]) -> None:
    print(f"\n{'='*86}")
    print('NATIVE COMPARISON')
    print(f"{'='*86}")
    header = (
        f"{'impl':<8} {'video':<24} {'frames':>7} {'points':>7} "
        f"{'runtime':>9} {'speedup':>9} {'ATE':>9} {'ΔATE':>9}"
    )
    print(header)
    print('-' * len(header))
    for row in rows:
        print(
            f"{row['impl']:<8} {row['video']:<24} {row['frames']:>7} {row['points']:>7} "
            f"{row['duration_sec']:>9.3f} {format_speedup(row['speedup_vs_python']):>9} "
            f"{format_float(row['ate_rmse']):>9} {format_float(row['ate_delta_vs_python']):>9}"
        )


def write_comparison_json(path: Path, rows: list[dict], source_summary: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'source_summary': str(source_summary),
        'rows': [
            {
                'video': row['video'],
                'impl': row['impl'],
                'frames': row['frames'],
                'points': row['points'],
                'duration_sec': row['duration_sec'],
                'speedup_vs_python': row['speedup_vs_python'],
                'ate_rmse': row['ate_rmse'],
                'ate_delta_vs_python': row['ate_delta_vs_python'],
                'heuristic_score': row['heuristic_score'],
                'summary_row': row['summary_row'],
            }
            for row in rows
        ],
    }
    path.write_text(json.dumps(payload, indent=2))


def write_comparison_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    def rounded(value: float | None, digits: int) -> str:
        if value is None:
            return ''
        return f'{value:.{digits}f}'

    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                'video',
                'impl',
                'frames',
                'points',
                'duration_sec',
                'speedup_vs_python',
                'ate_rmse',
                'ate_delta_vs_python',
                'heuristic_score',
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({
                'video': row['video'],
                'impl': row['impl'],
                'frames': row['frames'],
                'points': row['points'],
                'duration_sec': rounded(row['duration_sec'], 3),
                'speedup_vs_python': rounded(row['speedup_vs_python'], 2),
                'ate_rmse': rounded(row['ate_rmse'], 4),
                'ate_delta_vs_python': rounded(row['ate_delta_vs_python'], 4),
                'heuristic_score': rounded(row['heuristic_score'], 1),
            })


def comparison_json_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'native_comparison.json'
    return out_dir / f'native_comparison_{seconds_tag(seconds)}.json'


def comparison_csv_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'native_comparison.csv'
    return out_dir / f'native_comparison_{seconds_tag(seconds)}.csv'


def gt_tracking_json_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'gt_tracking.json'
    return out_dir / f'gt_tracking_{seconds_tag(seconds)}.json'


def gt_tracking_csv_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'gt_tracking.csv'
    return out_dir / f'gt_tracking_{seconds_tag(seconds)}.csv'


def gt_tracking_md_path(out_dir: Path, seconds: float) -> Path:
    if float(seconds) == DEFAULT_BENCHMARK_SECONDS:
        return out_dir / 'gt_tracking.md'
    return out_dir / f'gt_tracking_{seconds_tag(seconds)}.md'


def compute_gt_tracking_rows(summary_rows: list[dict]) -> list[dict]:
    grouped = group_by_video(row for row in summary_rows if 'ate_rmse' in row)
    tracking_rows: list[dict] = []

    for video in sorted(grouped):
        per_impl = grouped[video]
        python_row = per_impl.get('python')
        python_duration = float(python_row.get('duration_sec', 0.0)) if python_row else None
        best_ate = min(float(row['ate_rmse']) for row in per_impl.values())

        for impl in IMPLEMENTATION_ORDER:
            row = per_impl.get(impl)
            if not row:
                continue
            duration = float(row.get('duration_sec', 0.0))
            speedup_vs_python = None
            if python_duration and duration > 0:
                speedup_vs_python = python_duration / duration

            tracking_rows.append({
                'video': video,
                'impl': impl,
                'frames': int(row.get('frames', 0)),
                'points': int(row.get('points', 0)),
                'duration_sec': duration,
                'speedup_vs_python': speedup_vs_python,
                'ate_rmse': float(row['ate_rmse']),
                'ate_median': float(row['ate_median']),
                'ate_max': float(row['ate_max']),
                'ate_n': int(row['ate_n']),
                'delta_vs_best_ate': float(row['ate_rmse']) - best_ate,
            })

    tracking_rows.sort(key=lambda row: (row['video'], row['ate_rmse'], IMPLEMENTATION_ORDER.index(row['impl'])))
    return tracking_rows


def write_gt_tracking_json(path: Path, rows: list[dict], source_summary: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'source_summary': str(source_summary),
        'rows': rows,
    }
    path.write_text(json.dumps(payload, indent=2))


def write_gt_tracking_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                'video',
                'impl',
                'frames',
                'points',
                'duration_sec',
                'speedup_vs_python',
                'ate_rmse',
                'delta_vs_best_ate',
                'ate_median',
                'ate_max',
                'ate_n',
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({
                'video': row['video'],
                'impl': row['impl'],
                'frames': row['frames'],
                'points': row['points'],
                'duration_sec': f"{row['duration_sec']:.3f}",
                'speedup_vs_python': '' if row['speedup_vs_python'] is None else f"{row['speedup_vs_python']:.2f}",
                'ate_rmse': f"{row['ate_rmse']:.4f}",
                'delta_vs_best_ate': f"{row['delta_vs_best_ate']:.4f}",
                'ate_median': f"{row['ate_median']:.4f}",
                'ate_max': f"{row['ate_max']:.4f}",
                'ate_n': row['ate_n'],
            })


def write_gt_tracking_markdown(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    grouped: dict[str, list[dict]] = {}
    for row in rows:
        grouped.setdefault(row['video'], []).append(row)

    lines = ['# GT Tracking', '']
    for video in sorted(grouped):
        lines.append(f'## {video}')
        lines.append('')
        lines.append('| Impl | Frames | Runtime (s) | Speedup vs Python | ATE RMSE | Δ vs Best | ATE Median | ATE Max | ATE N |')
        lines.append('|------|--------|-------------|-------------------|----------|-----------|------------|---------|-------|')
        for row in grouped[video]:
            speedup = 'n/a' if row['speedup_vs_python'] is None else f"{row['speedup_vs_python']:.2f}x"
            lines.append(
                f"| {row['impl']} | {row['frames']} | {row['duration_sec']:.3f} | {speedup} | "
                f"{row['ate_rmse']:.4f} | {row['delta_vs_best_ate']:.4f} | {row['ate_median']:.4f} | {row['ate_max']:.4f} | {row['ate_n']} |"
            )
        lines.append('')

    path.write_text('\n'.join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=DEFAULT_BENCHMARK_SECONDS)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--out_dir', default='runs/benchmark')
    parser.add_argument('--all_gt', action='store_true',
                        help='Benchmark all videos in the repo that have adjacent ground-truth .npz files')
    parser.add_argument('--video', action='append', default=None,
                        help='Video stem or filename to benchmark; may be repeated')
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--workers', type=int, default=1,
                        help='Parallel SLAM jobs passed through to benchmark.py (default 1 = serial)')
    parser.add_argument('--skip_run', action='store_true',
                        help='Only read an existing summary JSON and print the comparison table')
    parser.add_argument('--summary_json', default='',
                        help='Optional explicit summary JSON path to read')
    parser.add_argument('--csv_out', default='',
                        help='Optional explicit CSV output path; defaults beside the JSON output')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    source_summary = Path(args.summary_json) if args.summary_json else summary_json_path(out_dir, args.seconds)
    if args.skip_run and args.summary_json:
        inferred_seconds = infer_seconds_from_summary_path(source_summary)
        if inferred_seconds is not None:
            args.seconds = inferred_seconds
    if not args.skip_run:
        source_summary = run_benchmark(args)

    summary_rows = load_summary(source_summary)
    comparison_rows = compute_comparison_rows(summary_rows)
    print_comparison_table(comparison_rows)
    gt_tracking_rows = compute_gt_tracking_rows(summary_rows)

    comparison_path = comparison_json_path(out_dir, args.seconds)
    comparison_csv = Path(args.csv_out) if args.csv_out else comparison_csv_path(out_dir, args.seconds)
    write_comparison_json(comparison_path, comparison_rows, source_summary)
    write_comparison_csv(comparison_csv, comparison_rows)
    gt_tracking_json = gt_tracking_json_path(out_dir, args.seconds)
    gt_tracking_csv = gt_tracking_csv_path(out_dir, args.seconds)
    gt_tracking_md = gt_tracking_md_path(out_dir, args.seconds)
    write_gt_tracking_json(gt_tracking_json, gt_tracking_rows, source_summary)
    write_gt_tracking_csv(gt_tracking_csv, gt_tracking_rows)
    write_gt_tracking_markdown(gt_tracking_md, gt_tracking_rows)
    print(f"\nSaved comparison → {comparison_path}")
    print(f"Saved CSV → {comparison_csv}")
    print(f"Saved GT tracking → {gt_tracking_json}")
    print(f"Saved GT CSV → {gt_tracking_csv}")
    print(f"Saved GT markdown → {gt_tracking_md}")


if __name__ == '__main__':
    main()
