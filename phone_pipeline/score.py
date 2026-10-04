#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Scoring of the phone pipeline outputs (runs/phone_pipeline/<seq>/...) with the repo's scorer (gnss_fusion/tools/gf_cases -> gnss_eval.score ->
benchmark.umeyama_alignment; nothing re-implemented). Every output is resampled to the CAMERA FRAME times (linear interpolation between neighbouring
output poses, no extrapolation, bracket <= 2.5 s), so a GNSS-only / bridged stretch counts as covered and coverage is "frames with a pose / frames".

usage: score.py <seq> [--out runs/phone_pipeline/<seq>/scores.json]      (python with numpy: external/gnss/venv/bin/python)
"""
import sys, json, re
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'gnss_fusion/tools')); sys.path.insert(0, str(HERE))
from gf_cases import phone_case, score_traj, gnss_alone  # noqa: E402
from benchmark import umeyama_alignment, apply_alignment  # noqa: E402
import run as R  # noqa: E402

OUT = R.OUT
INIT_WAIT = 30.0   # causal results are scored from 30 s after the start (as in gnss_fusion sections 11/12): before that the smoother has no alignment yet
MAXGAP = 2.5


def frame_times(seq):
    c = R.cfg_of(seq)
    ts = [float(l.split()[0]) for l in open(R.rp(c['rgb_dir']) + '/rgb.txt') if l.strip() and not l.startswith('#')]
    return np.array(ts)


def resample(tr, ft, maxgap=MAXGAP):
    """tr: Nx8+ (t x y z q), ft: frame times -> (positions at the frame times that are bracketed by two poses <= maxgap apart, mask)"""
    if tr is None or len(tr) < 2: return np.zeros((0, 3)), np.zeros(len(ft), bool)
    t = tr[:, 0]
    j = np.searchsorted(t, ft)
    ok = (j > 0) & (j < len(t))
    j = np.clip(j, 1, len(t) - 1)
    t0, t1 = t[j - 1], t[j]
    ok &= (t1 - t0) <= maxgap
    w = np.where(t1 > t0, (ft - t0) / np.maximum(t1 - t0, 1e-9), 0.0)
    # exact hits and equal stamps
    p = (1 - w)[:, None] * tr[j - 1, 1:4] + w[:, None] * tr[j, 1:4]
    return p[ok], ok


def metrics(case, tr, ft, tmin=None, geo=True):
    """SE3 / Sim3 ATE, scale ratio (estimated / true), un-aligned (geo) error and coverage of a trajectory resampled to the frame times"""
    f = ft if tmin is None else ft[ft >= tmin]
    p, ok = resample(tr, f)
    r = dict(coverage=float(ok.sum() / max(len(f), 1)), n_frames=int(len(f)))
    if ok.sum() < 20: return r
    pose = np.c_[f[ok], p, np.tile([0, 0, 0, 1.0], (ok.sum(), 1))]
    s = score_traj(case, pose)
    if 'ate_se3' not in s: return r
    r.update(se3=s['ate_se3'], sim3=s['ate_sim3'], scale=(1.0 / s['scale']) if s.get('scale') else None, n_scored=s['n_scored'])
    if geo and 'ate_noalign' in s: r['geo'] = s['ate_noalign']
    return r


def read_traj(path, cols=8):
    if not Path(path).exists() or Path(path).stat().st_size == 0: return None
    a = np.loadtxt(path, ndmin=2)
    return a[np.argsort(a[:, 0], kind='stable')]


def raw_sv(case, sv_dir, ft):
    """stella_vio output without fusion: camera-only map(s) in arbitrary scale. Reported: Sim3 ATE with one similarity per map (pooled), and the SE3 number of
    the raw coordinates (meaningless scale, shown to make the point), coverage = poses / frames."""
    a = read_traj(sv_dir / 'trajectory_maps.tum')
    if a is None or len(a) < 5: return dict(coverage=0.0)
    r = dict(coverage=float(len(a) / len(ft)), n_maps=int(len(np.unique(a[:, 8]))))
    gp, ok = case.gt.at(a[:, 0])
    e2, nsc = [], 0
    for mid in np.unique(a[:, 8]):
        k = (a[:, 8] == mid) & ok
        if k.sum() < 10: continue
        Rm, tm, sm = umeyama_alignment(a[k, 1:4], gp[k], with_scale=True)
        e2.append(np.linalg.norm(apply_alignment(a[k, 1:4], Rm, tm, sm) - gp[k], axis=1) ** 2); nsc += int(k.sum())
    if e2: r['sim3_per_map'] = float(np.sqrt(np.concatenate(e2).mean())); r['n_scored'] = nsc
    s = score_traj(case, np.c_[a[:, :8]])
    if 'ate_se3' in s: r['se3_raw_units'] = s['ate_se3']; r['scale'] = (1.0 / s['scale']) if s.get('scale') else None
    return r


def fix_rms(d_traj, fixes_path, tmin=None):
    """geo-referencing sanity check: rms distance [m] between the fused output (resampled at the fix epochs) and the fixes themselves (the phone GT frames are
    LiDAR / dataset frames, not ENU, so a geo error against the GT does not exist; this shows that the output lives in the ENU frame of the fixes)"""
    if d_traj is None or not fixes_path: return None
    fx = np.array([[float(x) for x in l.split(',')[:4]] for l in Path(fixes_path).read_text().splitlines()[1:]]); fx[:, 0] *= 1e-9
    if tmin is not None: fx = fx[fx[:, 0] >= tmin]
    p, ok = resample(d_traj, fx[:, 0])
    return float(np.sqrt(((p - fx[ok, 1:4]) ** 2).sum(1).mean())) if ok.sum() > 5 else None


def fused(case, d, ft, fixes_path=None):
    res = {}
    iw = json.loads((d / 'run.json').read_text()).get('init_wait_s', INIT_WAIT)
    bt = read_traj(d / 'batch.out')
    res['batch'] = metrics(case, bt, ft)
    ct = read_traj(d / 'causal.live')
    if ct is not None:
        ct = ct[(ct[:, 8].astype(int) & 1) == 1]   # GF_ST_INIT: aligned (geo-referenced / metric) poses only
    res['causal'] = metrics(case, ct, ft, tmin=ft[0] + iw)
    if fixes_path and (d / 'fix.txt').stat().st_size > 0:
        res['batch']['fix_rms'] = fix_rms(bt, fixes_path); res['causal']['fix_rms'] = fix_rms(ct, fixes_path, tmin=ft[0] + iw)
    if ct is not None and len(ct):
        res['causal']['first_aligned_s'] = float(ct[0, 0] - ft[0])
        res['causal']['coverage_all'] = float(resample(ct, ft)[1].mean())    # share of ALL frames (incl. the first 30 s)
    # tracked-only: the odometry sample times (what earlier gnss_fusion tables score), no bridged / GNSS-only nodes
    odom_t = np.loadtxt(d / 'odom.txt', ndmin=2)[:, 0]
    if bt is not None:
        m = np.isin(np.round(bt[:, 0], 6), np.round(odom_t, 6))
        if m.sum() > 20:
            s = score_traj(case, bt[m]); res['batch_tracked'] = dict(se3=s.get('ate_se3'), sim3=s.get('ate_sim3'), n=s.get('n_scored'))
    ct2 = read_traj(d / 'causal.out')
    if ct2 is not None:
        m = np.isin(np.round(ct2[:, 0], 6), np.round(odom_t, 6)) & (ct2[:, 0] >= ft[0] + iw)
        if m.sum() > 20:
            s = score_traj(case, ct2[m]); res['causal30_tracked'] = dict(se3=s.get('ate_se3'), sim3=s.get('ate_sim3'), n=s.get('n_scored'))
    for k in ('batch', 'causal'):
        txt = (d / f'{k}.stdout').read_text() if (d / f'{k}.stdout').exists() else ''
        m = re.search(r'timing add_odom.*?mean=([\d.]+) us', txt)
        res[f'{k}_stdout'] = txt.strip().splitlines()[:1]
        for ln in txt.splitlines():
            if ln.startswith('timing'): res.setdefault(f'{k}_timing', []).append(ln)
    return res


def evaluate(seq):
    c = R.cfg_of(seq); base = OUT / seq
    ft = frame_times(seq)
    some = next(iter(sorted(base.glob('sv_*/trajectory_maps.tum'))))
    case = phone_case(seq, str(some.resolve()), cam_only=False, label=seq)
    res = dict(seq=seq, n_frames=int(len(ft)), duration_s=float(ft[-1] - ft[0]))
    if c['fixes']:
        g = gnss_alone(case); res['gnss_alone'] = dict(se3=g.get('ate_se3'), sim3=g.get('ate_sim3'), noalign=g.get('ate_noalign'), n=g.get('n_scored'))
    else:
        g = gnss_alone(case); res['gnss_alone_unused'] = dict(se3=g.get('ate_se3'), sim3=g.get('ate_sim3'), n=g.get('n_scored'))
    for sv in sorted(base.glob('sv_*')):
        v = sv.name[3:]; res[f'raw_{v}'] = raw_sv(case, sv, ft)
        if (sv / 'run.json').exists(): res[f'raw_{v}']['timing'] = json.loads((sv / 'run.json').read_text())
        for fd in sorted(base.glob(f'fuse_{v}_*')):
            res[f'{v}|{fd.name[len("fuse_" + v + "_"):]}'] = fused(case, fd, ft, R.rp(c['fixes']) if c['fixes'] else None)
    for k in ('gait.json',):
        if (base / k).exists(): res['gait_timing'] = json.loads((base / k).read_text())
    return res


if __name__ == '__main__':
    seq = sys.argv[1]
    out = Path(sys.argv[sys.argv.index('--out') + 1]) if '--out' in sys.argv else OUT / seq / 'scores.json'
    r = evaluate(seq)
    out.write_text(json.dumps(r, indent=1, default=float))
    for k, v in r.items():
        if isinstance(v, dict) and ('batch' in v or 'sim3_per_map' in v or 'coverage' in v):
            b = v.get('batch', v); c_ = v.get('causal', {})
            f = lambda x: '-' if x is None else f'{x:.2f}'
            print(f"{k:22s} batch se3 {f(b.get('se3'))} sim3 {f(b.get('sim3', b.get('sim3_per_map')))} scale {f(b.get('scale'))} cov {b.get('coverage', 0)*100:.0f}% geo {f(b.get('geo'))} | causal se3 {f(c_.get('se3'))} sim3 {f(c_.get('sim3'))} cov {c_.get('coverage', 0)*100:.0f}% geo {f(c_.get('geo'))}")
        else:
            print(k, v if not isinstance(v, dict) else {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items()})
