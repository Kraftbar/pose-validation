#!/usr/bin/env python3
"""Shared scorer for the TUM RGB-D original-sequence comparison.

Reads runs/tum_compare/<system>/<seq>/{trajectory.tum,run.json} written by
tools/tum_compare_run.py (the repo's 7 impls) and
tools/tum_compare_import_orbslam2.py (ORB-SLAM2 upstream/port rows), scores
each (system, seq) pair against the original TUM groundtruth.txt, and writes
runs/tum_compare/table.md + table.json.

Metrics per (system, seq):
  - coverage        = frames with a pose / frames in rgb.txt
  - ate_tracked_m   = Sim3-aligned RMSE (via benchmark.ate_rmse -- Umeyama is
                       never reimplemented here) over only the frames the
                       system produced a pose for, associated to GT by
                       nearest timestamp within 0.02s (the TUM tools'
                       convention).
  - ate_all_m       = same alignment/association, but first every rgb.txt
                       frame is given a pose via hold-last-pose (frames
                       before the system's first pose are excluded and
                       counted in ate_all_excluded).
  - runtime_s / rtf = wall_s from run.json, and wall_s / sequence duration.
  - ate_kf_m        = same as ate_tracked_m but over run.json's optional
                       "keyframe_trajectory" file, if present.

Only scores what exists: a missing trajectory.tum/run.json for a
(system, seq) pair is silently skipped (reported as "missing" in the
per-system summary), not an error.

CLI:
    python3 tools/tum_eval.py [--systems s1,s2,...|all] [--seqs a,b,...|all]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from benchmark import ate_rmse  # noqa: E402  (reused, unmodified -- Umeyama lives here only)

DATA_ROOT = ROOT / 'runs' / 'orb_port' / 'paper_bench' / 'data'
COMPARE_ROOT = ROOT / 'runs' / 'tum_compare'

SEQ_DATA_DIRS = {
    'fr1_xyz': 'rgbd_dataset_freiburg1_xyz',
    'fr1_desk': 'rgbd_dataset_freiburg1_desk',
    'fr1_floor': 'rgbd_dataset_freiburg1_floor',
    'fr2_xyz': 'rgbd_dataset_freiburg2_xyz',
    'fr3_long_office': 'rgbd_dataset_freiburg3_long_office_household',
}
ALL_SEQS = tuple(SEQ_DATA_DIRS.keys())
ASSOC_MAX_DIFF = 0.02  # seconds, TUM tools' convention


# ---------------------------------------------------------------------------
# TUM file readers
# ---------------------------------------------------------------------------
def read_tum_list(path: Path):
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        out.append(line.split())
    return out


def read_rgb_frame_timestamps(seq_id: str) -> np.ndarray:
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq_id]
    rows = read_tum_list(data_dir / 'rgb.txt')
    return np.array([float(r[0]) for r in rows], dtype=np.float64)


def read_groundtruth(seq_id: str):
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq_id]
    rows = read_tum_list(data_dir / 'groundtruth.txt')
    ts = np.array([float(r[0]) for r in rows], dtype=np.float64)
    pos = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows], dtype=np.float64)
    return ts, pos


def read_trajectory_tum(path: Path):
    ts, pos = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        ts.append(float(parts[0]))
        pos.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(ts, dtype=np.float64), np.array(pos, dtype=np.float64)


# ---------------------------------------------------------------------------
# Association + ATE (own code; alignment itself stays on benchmark.ate_rmse)
# ---------------------------------------------------------------------------
def associate_nearest(est_ts, est_pos, gt_ts, gt_pos, max_diff=ASSOC_MAX_DIFF):
    matched_est, matched_gt = [], []
    for i, ts in enumerate(est_ts):
        diffs = np.abs(gt_ts - ts)
        j = int(np.argmin(diffs))
        if diffs[j] <= max_diff:
            matched_est.append(est_pos[i])
            matched_gt.append(gt_pos[j])
    if not matched_est:
        return np.zeros((0, 3)), np.zeros((0, 3))
    return np.array(matched_est), np.array(matched_gt)


def ate_over(est_ts, est_pos, gt_ts, gt_pos):
    m_est, m_gt = associate_nearest(est_ts, est_pos, gt_ts, gt_pos)
    if len(m_est) < 3:
        return {'ok': False, 'reason': f'too few associated frames ({len(m_est)})'}
    res = ate_rmse(m_est, m_gt)
    res['ok'] = True
    res['n_associated'] = int(len(m_est))
    return res


def hold_last_pose(traj_ts, traj_pos, frame_ts):
    """For every timestamp in frame_ts, return the most recent traj pose at
    or before it. Frames before the first traj pose are excluded."""
    order = np.argsort(traj_ts)
    traj_ts = traj_ts[order]
    traj_pos = traj_pos[order]
    out_ts, out_pos = [], []
    n_excluded = 0
    idx = 0
    n = len(traj_ts)
    for ft in np.sort(frame_ts):
        while idx + 1 < n and traj_ts[idx + 1] <= ft:
            idx += 1
        if n == 0 or traj_ts[idx] > ft:
            n_excluded += 1
            continue
        out_ts.append(ft)
        out_pos.append(traj_pos[idx])
    if not out_ts:
        return np.zeros(0), np.zeros((0, 3)), n_excluded
    return np.array(out_ts), np.array(out_pos), n_excluded


# ---------------------------------------------------------------------------
# Per (system, seq) scoring
# ---------------------------------------------------------------------------
def score_pair(system: str, seq_id: str):
    sys_dir = COMPARE_ROOT / system / seq_id
    traj_path = sys_dir / 'trajectory.tum'
    run_json_path = sys_dir / 'run.json'
    if not traj_path.exists() or not run_json_path.exists():
        return None

    run_info = json.loads(run_json_path.read_text())
    frame_ts = read_rgb_frame_timestamps(seq_id)
    frames_in = run_info.get('frames_in', len(frame_ts))
    gt_ts, gt_pos = read_groundtruth(seq_id)
    duration = float(frame_ts[-1] - frame_ts[0]) if len(frame_ts) > 1 else 0.0

    traj_ts, traj_pos = read_trajectory_tum(traj_path)
    n_pose = len(traj_ts)
    coverage = n_pose / frames_in if frames_in else 0.0

    row = {
        'system': system,
        'seq': seq_id,
        'frames_in': int(frames_in),
        'n_pose': int(n_pose),
        'coverage': coverage,
        'wall_s': run_info.get('wall_s'),
        'rtf': (run_info.get('wall_s') / duration) if run_info.get('wall_s') is not None and duration > 0 else None,
        'exit_code': run_info.get('exit_code'),
        'notes': run_info.get('notes', ''),
    }

    ate_tracked = ate_over(traj_ts, traj_pos, gt_ts, gt_pos) if n_pose >= 3 else {'ok': False, 'reason': 'no poses'}
    row['ate_tracked_m'] = ate_tracked.get('ate_rmse') if ate_tracked.get('ok') else None
    row['ate_tracked_n'] = ate_tracked.get('n_associated') if ate_tracked.get('ok') else 0
    row['ate_tracked_note'] = '' if ate_tracked.get('ok') else ate_tracked.get('reason', '')

    if n_pose >= 1:
        held_ts, held_pos, n_excl = hold_last_pose(traj_ts, traj_pos, frame_ts)
        row['ate_all_excluded'] = int(n_excl)
        ate_all = ate_over(held_ts, held_pos, gt_ts, gt_pos) if len(held_ts) >= 3 else {'ok': False, 'reason': 'too few held frames'}
        row['ate_all_m'] = ate_all.get('ate_rmse') if ate_all.get('ok') else None
        row['ate_all_n'] = ate_all.get('n_associated') if ate_all.get('ok') else 0
        row['ate_all_note'] = '' if ate_all.get('ok') else ate_all.get('reason', '')
    else:
        row['ate_all_excluded'] = int(frames_in)
        row['ate_all_m'] = None
        row['ate_all_n'] = 0
        row['ate_all_note'] = 'no poses'

    kf_rel = run_info.get('keyframe_trajectory')
    if kf_rel:
        kf_path = sys_dir / kf_rel
        if kf_path.exists():
            kf_ts, kf_pos = read_trajectory_tum(kf_path)
            ate_kf = ate_over(kf_ts, kf_pos, gt_ts, gt_pos) if len(kf_ts) >= 3 else {'ok': False, 'reason': 'too few kf poses'}
            row['ate_kf_m'] = ate_kf.get('ate_rmse') if ate_kf.get('ok') else None
            row['ate_kf_n'] = ate_kf.get('n_associated') if ate_kf.get('ok') else 0
        else:
            row['ate_kf_m'] = None
            row['ate_kf_n'] = 0
    else:
        row['ate_kf_m'] = None
        row['ate_kf_n'] = 0

    return row


def discover_systems():
    if not COMPARE_ROOT.exists():
        return []
    out = []
    for p in sorted(COMPARE_ROOT.iterdir()):
        if p.is_dir() and not p.name.startswith('_'):
            out.append(p.name)
    return out


# ---------------------------------------------------------------------------
# Table rendering
# ---------------------------------------------------------------------------
def fmt(v, spec='{:.4f}'):
    return spec.format(v) if isinstance(v, (int, float)) and v is not None else 'n/a'


def build_tables(rows):
    by_system = {}
    for r in rows:
        by_system.setdefault(r['system'], []).append(r)

    lines = []
    lines.append('# TUM RGB-D original-sequence comparison\n')
    lines.append(
        '| System | Seq | Coverage | ATE-tracked (m) | n | ATE-all (m) | excl | ATE-KF (m) | wall (s) | RTF |'
    )
    lines.append('|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        lines.append(
            f"| {r['system']} | {r['seq']} | {fmt(r['coverage'], '{:.1%}')} | "
            f"{fmt(r['ate_tracked_m'])} | {r['ate_tracked_n']} | "
            f"{fmt(r['ate_all_m'])} | {r['ate_all_excluded']} | "
            f"{fmt(r['ate_kf_m'])} | {fmt(r['wall_s'], '{:.1f}')} | {fmt(r['rtf'], '{:.2f}')} |"
        )

    lines.append('\n## Per-system summary\n')
    lines.append('| System | Mean ATE-tracked (m) | Mean ATE-all (m) | Seqs scored | Seqs cov<50% |')
    lines.append('|---|---|---|---|---|')
    summary = {}
    for system, srows in by_system.items():
        tracked_vals = [r['ate_tracked_m'] for r in srows if r['ate_tracked_m'] is not None]
        all_vals = [r['ate_all_m'] for r in srows if r['ate_all_m'] is not None]
        low_cov = sum(1 for r in srows if r['coverage'] < 0.5)
        mean_tracked = float(np.mean(tracked_vals)) if tracked_vals else None
        mean_all = float(np.mean(all_vals)) if all_vals else None
        summary[system] = {
            'mean_ate_tracked_m': mean_tracked,
            'mean_ate_all_m': mean_all,
            'n_seqs_scored': len(srows),
            'n_seqs_low_coverage': low_cov,
        }
        lines.append(
            f"| {system} | {fmt(mean_tracked)} | {fmt(mean_all)} | {len(srows)} | {low_cov} |"
        )

    return '\n'.join(lines) + '\n', summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--systems', default='all')
    ap.add_argument('--seqs', default='all')
    args = ap.parse_args()

    systems = discover_systems() if args.systems == 'all' else args.systems.split(',')
    seqs = list(ALL_SEQS) if args.seqs == 'all' else args.seqs.split(',')

    rows = []
    for system in systems:
        for seq_id in seqs:
            row = score_pair(system, seq_id)
            if row is not None:
                rows.append(row)

    md, summary = build_tables(rows)
    (COMPARE_ROOT / 'table.md').write_text(md)
    (COMPARE_ROOT / 'table.json').write_text(json.dumps({'rows': rows, 'summary': summary}, indent=2))
    print(md)


if __name__ == '__main__':
    main()
