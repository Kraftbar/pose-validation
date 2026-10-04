#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Shared helpers for the gnss_fusion validation: the test cases (saved trajectories + fixes from the GNSS-VIO study), a C driver wrapper
and the scoring wrapper around tools/gnss_harness/gnss_eval.py (which is built on benchmark.umeyama_alignment; nothing is reimplemented here).
Own code (MIT)."""
import sys, json, subprocess, time
from pathlib import Path
import numpy as np

REPO = Path('/home/nybo/github/pose-validation')
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = ROOT / 'work'
GF_RUN = ROOT / 'c' / 'gf_run'
sys.path.insert(0, str(REPO / 'tools')); sys.path.insert(0, str(REPO / 'tools' / 'gnss_harness')); sys.path.insert(0, str(REPO))
import gnss_loose_fusion as lf  # noqa: E402  (the python reference, own code)
from gnss_eval import GT, read_traj, score, antenna  # noqa: E402

G = REPO / 'external' / 'gnss'
ROB = G / 'rob'
RSA_OKVIS = [-0.01, -0.03, -0.06]
T_BC_MOBILE = np.array([[0.01916709, -0.99980408, -0.00494212], [-0.99955275, -0.01904832, -0.02305337], [0.02295471, 0.00538177, -0.99972202]])
T_BC_ADVIO = np.linalg.inv(np.array([[0.9999763379093255, -0.004079205042965442, -0.005539287650170447], [-0.004066386342107199, -0.9999890330121858, 0.0023234365646622014], [-0.00554870467502187, -0.0023008567036498766, -0.9999819588046867]]))
OFFSETS = json.loads((REPO / 'tools/gnss_harness/phone_offsets.json').read_text())


def load_sorted(p):
    a = read_traj(p)
    return a[np.argsort(a[:, 0])]


def fill_gaps(tr, maxgap=0.5):
    """what tools/gnss_harness/phone_fusion.py does for the python smoother: bridge pose gaps with constant-velocity pseudo-poses"""
    rows = [tr[0]]
    for i in range(1, len(tr)):
        if tr[i, 0] - tr[i - 1, 0] > maxgap:
            for tt in np.arange(tr[i - 1, 0] + 0.5, tr[i, 0] - 0.25, 0.5):
                w = (tt - tr[i - 1, 0]) / (tr[i, 0] - tr[i - 1, 0]); r = tr[i - 1].copy(); r[0] = tt; r[1:4] = (1 - w) * tr[i - 1, 1:4] + w * tr[i, 1:4]; rows.append(r)
        rows.append(tr[i])
    return np.array(rows)


def gravity_up(tr, seq, imu_path):
    """mean specific force direction in the camera-only SLAM frame (as phone_fusion.py --cam-only); returns the unit 'up' vector"""
    T_BC = T_BC_ADVIO if seq.startswith('advio') else T_BC_MOBILE
    imu = np.loadtxt(imu_path, delimiter=',', comments='#'); ti = imu[:, 0] * 1e-9
    k = np.clip(np.searchsorted(tr[:, 0], ti), 0, len(tr) - 1); ok = np.abs(tr[k, 0] - ti) < 0.1
    R = lf_quats(tr[k[ok], 4:8])
    up = np.einsum('nij,jk,nk->ni', R, T_BC.T, imu[ok, 4:7]).mean(0)
    return up / np.linalg.norm(up)


def lf_quats(q):
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    return np.array([lf.quat_R(x) for x in q])


def align_up(up):
    """same rotation as gf_align_up()/phone_fusion.py: up -> +z"""
    v = np.cross(up, [0, 0, 1.0]); c = float(up @ [0, 0, 1.0]); s = np.linalg.norm(v)
    if s < 1e-9: return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1 - c) / s ** 2


def rotate_traj(tr, Ra):
    out = tr.copy(); out[:, 1:4] = tr[:, 1:4] @ Ra.T
    out[:, 4:8] = np.array([lf.R_quat(Ra @ lf.quat_R(q)) for q in tr[:, 4:8]])
    return out


class Case:
    """name, odometry traj (TUM-like array, odometry frame), fixes (t E N U sh sv), rsa, GT, geo flag, metric flag, up (None if gravity aligned)"""
    def __init__(self, name, traj, gps, rsa, gt, geo, metric=True, up=None, desc='', t0=None, blackout=None, score_rsa=None):
        self.name, self.traj, self.gps, self.rsa, self.gt, self.geo, self.metric, self.up, self.desc = name, traj, gps, rsa, gt, geo, metric, up, desc
        self.score_rsa = rsa if score_rsa is None else score_rsa   # lever arm from the odometry body to the GT reference point (drone: RTK midpoint)
        self.t0 = t0 if t0 is not None else traj[0, 0]
        self.blackout = blackout


def complex_case(variant):
    S = G / 'seq' / f'complex_{variant}'
    tr = load_sorted(G / 'out/okvis2x_mono_nogps_complex/okvis2-vio-final_trajectory.csv')
    gps = lf.read_gps(S / 'gps0/data.csv')
    gt = GT.from_pvt(G / 'seq/complex')
    o = json.load(open(G / 'seq/complex/origin.json'))
    return Case(f'complex_{variant}', tr, gps, RSA_OKVIS, gt, True, desc=f'GVINS complex_environment, OKVIS2 mono VIO + {variant} fixes', t0=o['t0'])


def phone_case(seq, run, cam_only=False, label=None):
    """phone sequence with the real iPhone / ADVIO fixes. run = directory name under external/gnss/rob/out (or a path to a traj file)."""
    d = ROB / seq
    p = Path(run) if str(run).startswith('/') else ROB / 'out' / run / 'traj.txt'
    tr = load_sorted(p)
    gps = lf.read_gps(d / 'gps0/data.csv')
    if seq == 'outdoor1': gt = GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887)
    else: gt = GT.from_tum(d / 'gt.tum', dt=OFFSETS[seq])
    up = None
    if cam_only:
        up = gravity_up(tr, seq, d / 'imu0/data.csv')
    return Case(label or f'{seq}_{Path(run).parent.name if str(run).startswith("/") else run}', tr, gps, [0, 0, 0], gt, False, metric=not cam_only, up=up,
                desc=f'{seq} {run}' + (' (camera-only: gravity from accel, scale from fixes)' if cam_only else ''))


def drone_case(seq, odom, label):
    """INSANE drone sequence (runs/drone_compare/<seq>/<odom>/trajectory.tum = OKVIS2 mono body poses, PX4 GNSS in ENU, dual-RTK reference)"""
    D = REPO / 'external' / 'drone' / f'ins_{seq}'
    tr = load_sorted(REPO / 'runs' / 'drone_compare' / seq / odom / 'trajectory.tum')
    gps = lf.read_gps(D / 'mav0' / 'gps0' / 'data.csv')
    gt = GT.from_tum(D / 'gt_enu.tum')
    lev = json.load(open(REPO / 'external' / 'drone' / f'ins_{seq}_cfg' / 'lever.json'))['r_RTK']
    return Case(label, tr, gps, [0, 0, 0], gt, True, desc=f'INSANE {seq} {odom} + PX4 GNSS', score_rsa=lev)


def xrslam_case(seq, label):
    """XRSLAM mono-inertial on a phone sequence (runs/gnss_compare/more_systems/<seq>_xrslam/traj.txt, IMU body poses, gravity-aligned world)"""
    d = ROB / seq
    tr = load_sorted(REPO / 'runs' / 'gnss_compare' / 'more_systems' / f'{seq}_xrslam' / 'traj.txt')
    tr = tr[np.isfinite(tr).all(1) & (np.linalg.norm(tr[:, 4:8], axis=1) > 0.5)]
    gps = lf.read_gps(d / 'gps0' / 'data.csv')
    if seq == 'outdoor1': gt = GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887)
    else: gt = GT.from_tum(d / 'gt.tum', dt=OFFSETS[seq])
    return Case(label, tr, gps, [0, 0, 0], gt, False, desc=f'{seq} XRSLAM mono-inertial')


def prepared(case):
    """trajectory as seen by the smoother: gravity-aligned (camera-only) + gaps bridged (python smoother's requirement)"""
    tr = case.traj
    if case.up is not None: tr = rotate_traj(tr, align_up(case.up))
    return tr


def write_odom(path, tr, ups=None):
    with open(path, 'w') as f:
        if ups is not None: f.write('up %.17g %.17g %.17g\n' % tuple(ups))
        for r in tr: f.write(' '.join('%.17g' % v for v in r[:8]) + '\n')


def write_fixes(path, gps):
    np.savetxt(path, gps[:, :6], fmt=['%.9f'] + ['%.6f'] * 3 + ['%.4f'] * 2)


def run_c(case, mode, cfg=(), tag='', odom_tr=None, lookahead=None, timing=False, up_raw=False, speed_file=None):
    """run gf_run on a case. Returns dict(out=array Nx8, live=array or None, wall=s, stdout=str). odom_tr overrides the prepared trajectory.
    up_raw: pass the raw (un-rotated) trajectory with the gravity direction to the library instead of pre-rotating (exercises gf_set_gravity)."""
    WORK.mkdir(exist_ok=True)
    base = WORK / f'{case.name}{tag}_{mode}'
    if up_raw and case.up is not None:
        tr = case.traj if odom_tr is None else odom_tr
        write_odom(f'{base}.odom', tr, ups=case.up)
    else:
        tr = prepared(case) if odom_tr is None else odom_tr
        write_odom(f'{base}.odom', tr)
    write_fixes(f'{base}.fix', case.gps)
    cmd = [str(GF_RUN), '--odom', f'{base}.odom', '--fix', f'{base}.fix', '--out', f'{base}.out', '--mode', mode, '--nodes', f'{base}.nodes']
    if mode == 'causal': cmd += ['--out-live', f'{base}.live']
    if lookahead: cmd += ['--lookahead', str(lookahead)]
    if timing: cmd += ['--timing']
    if speed_file: cmd += ['--speed', str(speed_file)]
    c = list(cfg)
    c.append('rsa=%.6f,%.6f,%.6f' % tuple(case.rsa))
    if not case.metric: c.append('metric=0')
    if up_raw and case.up is not None: c.append('gravity_aligned=0')
    t = time.time()
    r = subprocess.run(cmd + c, capture_output=True, text=True)
    wall = time.time() - t
    if r.returncode: raise RuntimeError(r.stderr + r.stdout)
    out = np.loadtxt(f'{base}.out')
    live = np.loadtxt(f'{base}.live') if mode == 'causal' and Path(f'{base}.live').stat().st_size else None
    nodes = np.loadtxt(f'{base}.nodes')
    return dict(out=out, live=live, wall=wall, stdout=r.stdout, nodes=nodes)


def score_traj(case, tr, windows=None):
    """gnss_eval.score on a poses array; SE3 ATE (aligned) + no-align error (geo-referenced output)"""
    if tr is None or len(tr) < 20: return {}
    s = score(tr, case.gt, case.score_rsa, len(tr), tr[-1, 0] - tr[0, 0], geo=True, windows=windows)
    return s


def gnss_alone(case):
    """the fixes themselves scored as a trajectory (positions only)"""
    fx = case.gps[:, :4].copy()
    return score(fx, case.gt, [0, 0, 0], len(fx), fx[-1, 0] - fx[0, 0], geo=True)
