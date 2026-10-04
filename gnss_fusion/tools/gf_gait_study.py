#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Section-12 study: gait (step cadence) speed prior in the C smoother.
  gait   : accuracy of the gait speed per sequence / calibration mode against the GT speed (3 s epochs, 6 s window)
  fusion : per case (6 sequences x {stella, orb3mono, okvis, xrslam}) GNSS alone / robust preset / + gait (generic, per-user held-out or cross-user, online GNSS), batch + causal
  pdr    : no odometry: gait + gyro-heading PDR alone and + GNSS
usage: gf_gait_study.py [gait|fusion|pdr|all] [--cases a,b] [--workers N]     outputs work/gait_*.json, work/gait_study.md"""
import sys, json, time, argparse, subprocess
from pathlib import Path
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, '/home/nybo/github/pose-validation/tools/phone_diag')
from gf_cases import *  # noqa
from gf_table import restrict, INIT_WAIT
import gait, check_gait, common  # noqa
from gnss_eval import antenna

MOBILE = ['outdoor1', 'outdoor2', 'indoor1', 'indoor2']
SEQS = MOBILE + ['advio15', 'advio20']
EPOCH_DT, WINDOW = 3.0, 6.0
import os
GAIT_CFG = ['speed=1', 'speed_align=1', 'speed_scale_rw_rel=1'] + [x for x in os.environ.get('GF_GAIT_EXTRA', '').split(',') if x]
OUTSUF = os.environ.get('GF_GAIT_SUF', '')
PRESET = ['preset=robust']


# ------------------------------------------------------------------------------------------------------------------------------ gait accuracy
_gt_cache = {}


def seq_data(seq):
    if seq not in _gt_cache: _gt_cache[seq] = common.load(seq)
    return _gt_cache[seq]


def epochs_with_gt(seq, model=None, online=False, tag='gen'):
    fx = check_gait.fixes_for(seq) if online else None
    ep, _ = check_gait.run_c(seq, tag, model=model, online=online, fixes=fx, window=WINDOW, edt=EPOCH_DT)
    d = seq_data(seq)
    gt = np.array([gait.gt_speed_window(d, e[0] - d['t0'], WINDOW) / WINDOW for e in ep])
    return ep, gt


def fit_c(seqs):
    """per-user model constant from held-out sequences with a reference: c = sum v_gt / sum cad^2 over walking epochs (GT > 0.5 m/s)"""
    num = den = 0.0
    for s in seqs:
        ep, gt = epochs_with_gt(s)
        m = np.isfinite(gt) & (ep[:, 3] == 0) & (gt > 0.5)
        num += gt[m].sum(); den += (ep[m, 2] ** 2).sum()
    return num / den


def calibrations():
    cal = {}
    for s in MOBILE: cal[s] = fit_c([x for x in MOBILE if x != s])
    for s in ('advio15', 'advio20'): cal[s] = fit_c(MOBILE)       # cross-user (different person and phone), NOT per-user
    cal['self'] = {s: fit_c([s]) for s in ('outdoor1', 'outdoor2', 'indoor1', 'indoor2', 'advio20')}   # leaky upper bound, accuracy table only
    return cal


def gait_accuracy(cal):
    rows = []
    modes = lambda s: ([('generic', dict())] + ([('per-user (held-out)', dict(model=cal[s]))] if s in MOBILE else [('cross-user (Mobile cal)', dict(model=cal[s]))])
                       + ([('online GNSS', dict(online=True))] if s in ('outdoor1', 'outdoor2', 'advio20') else [])
                       + ([('self (leak)', dict(model=cal['self'][s]))] if s in cal['self'] else []))
    for s in SEQS:
        for name, kw in modes(s):
            ep, gt = epochs_with_gt(s, tag='acc', **kw)
            ok = np.isfinite(gt)
            w = ok & (gt > 0.5)
            walk = w & (ep[:, 3] == 0)
            r = ep[walk, 4] / gt[walk]
            still = ok & (gt < 0.2)
            rows.append(dict(seq=s, mode=name, n_epochs=int(ok.sum()), n_gt_walk=int(w.sum()), walk_detected=int(walk.sum()),
                             median=float(np.median(r)) if walk.sum() else None, q25=float(np.percentile(r, 25)) if walk.sum() else None, q75=float(np.percentile(r, 75)) if walk.sum() else None,
                             dist_ratio=float(ep[walk, 4].sum() / gt[walk].sum()) if walk.sum() else None,
                             p10_90=[float(np.percentile(r, 10)), float(np.percentile(r, 90))] if walk.sum() else None,
                             rel_rms=float(np.sqrt(np.mean((r - 1) ** 2))) if walk.sum() else None,
                             sigma_cover=float(np.mean(np.abs(ep[walk, 4] - gt[walk]) <= ep[walk, 5])) if walk.sum() else None,
                             n_gt_still=int(still.sum()), still_detected=int((still & (ep[:, 3] == 1)).sum()), false_still=int((w & (ep[:, 3] == 1)).sum()),
                             n_gt_walk_other=int((w & (ep[:, 3] == 2)).sum()), k_end=float(ep[-1, 7])))
    return rows


# ------------------------------------------------------------------------------------------------------------------------------ speed files
def speed_file(seq, mode, cal, path):
    kw = {}
    if mode == 'user' or mode == 'cross': kw = dict(model=cal[seq])
    elif mode == 'onl': kw = dict(online=True)
    fx = check_gait.fixes_for(seq) if kw.get('online') else None
    ep, _ = check_gait.run_c(seq, 'sp_' + mode, model=kw.get('model'), online=kw.get('online', False), fixes=fx, window=WINDOW, edt=EPOCH_DT)
    n = 0
    with open(path, 'w') as f:
        for r in ep:
            if int(r[3]) == 0: f.write('%.9f %.6f %.6f %.3f 0\n' % (r[0], r[4], r[5], r[6])); n += 1
            elif int(r[3]) == 1: f.write('%.9f 0 %.6f %.3f 1\n' % (r[0], r[5], r[6])); n += 1
    return n


# ------------------------------------------------------------------------------------------------------------------------------ cases
def seq_gt(seq):
    if seq == 'outdoor1': return GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887)
    return GT.from_tum(ROB / seq / 'gt.tum', dt=OFFSETS[seq])


INDOOR = ('indoor1', 'indoor2', 'advio15')


def build_case(label):
    """label = <o1|o2|i1|i2|a15|a20>_<stella|orb3mono|okvis|xrslam>"""
    short = {'o1': 'outdoor1', 'o2': 'outdoor2', 'i1': 'indoor1', 'i2': 'indoor2', 'a15': 'advio15', 'a20': 'advio20'}
    sk, sysn = label.split('_'); seq = short[sk]
    if sysn == 'xrslam': c = xrslam_case(seq, label)
    else:
        if sysn == 'stella': run = {'outdoor1': 'outdoor1_stella_up', 'outdoor2': 'outdoor2_stella_up', 'indoor1': 'indoor1_stella_up_lowfast', 'indoor2': 'indoor2_stella_up_lowfast',
                                    'advio15': 'advio15_stella_up', 'advio20': 'advio20_stella_up'}[seq]
        elif sysn == 'orb3mono': run = f'{seq}_orb3_mono'
        else: run = f'{seq}_okvis_default'
        if label == 'o1_okvis': run = G / 'out/okvis2x_mobile_nogps_outdoor1/okvis2-vio-final_trajectory.csv'
        c = phone_case(seq, run, cam_only=sysn in ('stella', 'orb3mono'), label=label)
    c.seq = seq
    if seq in INDOOR: c.gps = np.zeros((0, 6))
    return c


ALL_CASES = [f'{s}_{o}' for s in ('o1', 'o2', 'i1', 'i2', 'a15', 'a20') for o in ('stella', 'orb3mono', 'okvis', 'xrslam')]


def path_ratio(case, tr, step=1.0):
    if tr is None or len(tr) < 20: return None
    t = tr[:, 0]; A = antenna(tr, case.score_rsa)
    gp, ok = case.gt.at(t)
    idx = []; last = -1e9
    for i in range(len(t)):
        if ok[i] and t[i] - last >= step: idx.append(i); last = t[i]
    idx = np.array(idx)
    if len(idx) < 10: return None
    dt = np.diff(t[idx]); good = dt <= 3.0 * step
    a = np.linalg.norm(np.diff(A[idx], axis=0), axis=1)[good].sum(); p = np.linalg.norm(np.diff(gp[idx], axis=0), axis=1)[good].sum()
    return float(a / p) if p > 0 else None


def metr(case, out, raw_t=None, tmin=None):
    if out is None or np.ndim(out) < 2 or len(out) < 20: return None
    o = restrict(out, raw_t, tmin) if raw_t is not None else out
    s = score_traj(case, o)
    if 'ate_se3' not in s: return None
    return dict(se3=s['ate_se3'], sim3=s['ate_sim3'], scale=(1.0 / s['scale']) if s.get('scale') else None, n=s['n_scored'], path=path_ratio(case, o))


def fusion_case(args):
    label, speed_files = args
    case = build_case(label)
    raw = prepared(case); raw_t = raw[:, 0]
    R = dict(label=label, seq=case.seq, indoor=case.seq in INDOOR)
    R['odom'] = metr(case, raw[:, :8])
    if len(case.gps):
        fx = case.gps[:, :4]; g = score(fx, case.gt, [0, 0, 0], len(fx), 1, geo=True)
        R['gnss'] = dict(se3=g.get('ate_se3'), sim3=g.get('ate_sim3'), scale=(1.0 / g['scale']) if g.get('scale') else None, n=g.get('n_scored'))
    tmin_c = raw_t[0] + INIT_WAIT
    runs = {}
    if len(case.gps): runs['base'] = (PRESET, None)
    for mode, sf in speed_files.items(): runs['gait_' + mode] = (PRESET + GAIT_CFG, sf)
    if len(case.gps):   # outdoor sequence, but also without the fixes: what the gait alone gives to the odometry's scale on a long walk
        for mode, sf in speed_files.items():
            if mode != 'onl': runs['ng_gait_' + mode] = (PRESET + GAIT_CFG, sf)
    import copy
    nofix = copy.copy(case); nofix.gps = np.zeros((0, 6)); nofix.name = label + '_ng'
    for key, (cfg, sf) in runs.items():
        cs = nofix if key.startswith('ng_') else case
        for mode in ('batch', 'causal'):
            try:
                r = run_c(cs, mode, tag='_g_' + key, odom_tr=raw, cfg=cfg, speed_file=sf, timing=False)
            except RuntimeError as e:
                R[f'{key}_{mode}'] = dict(error=str(e)[:200]); continue
            R[f'{key}_{mode}'] = metr(case, r['out'], raw_t, tmin_c if mode == 'causal' else None)
            if mode == 'causal' and r['live'] is not None:
                live = r['live']; live = live[(live[:, 8].astype(int) & 1) == 1]
                R[f'{key}_live'] = metr(case, live[:, :8], raw_t, tmin_c)
            R[f'{key}_wall_{mode}'] = r['wall']
            R[f'{key}_stdout_{mode}'] = r['stdout'].strip()
    return R


def run_fusion(cases, cal, workers):
    # speed files per (seq, mode)
    sfs = {}
    for s in SEQS:
        modes = ['gen'] + (['user'] if s in MOBILE else ['cross']) + (['onl'] if s in ('outdoor1', 'outdoor2', 'advio20') else [])
        for m in modes:
            p = WORK / f'speed_{s}_{m}.txt'
            speed_file(s, 'gen' if m == 'gen' else m, cal, p); sfs[(s, m)] = str(p)
    short = {'o1': 'outdoor1', 'o2': 'outdoor2', 'i1': 'indoor1', 'i2': 'indoor2', 'a15': 'advio15', 'a20': 'advio20'}
    tasks = []
    for c in cases:
        seq = short[c.split('_')[0]]
        tasks.append((c, {m: v for (s, m), v in sfs.items() if s == seq}))
    with Pool(workers) as p: res = p.map(fusion_case, tasks, chunksize=1)
    return {r['label']: r for r in res}


# ------------------------------------------------------------------------------------------------------------------------------ PDR (no odometry)
def pdr_traj(seq, model=None, online=False, step=0.1):
    """position (t, x, y, 0, quaternion about z) from the gait odometer and the gyro heading about gravity (10 Hz)"""
    t, a, w = gait.load_imu(seq)
    cfg = gait.Config(); cfg.online = 1 if online else 0
    g = gait.Gait(cfg)
    if model: g.set_model(model)
    fx = check_gait.fixes_for(seq) if online else []
    fi = 0
    rows = []; x = y = 0.0; last_odo = 0.0; last_t = -1e9; last_hd = 0.0
    for i in range(len(t)):
        while fi < len(fx) and fx[fi][0] <= t[i]: g.gnss_fix(*fx[fi]); fi += 1
        g.push(t[i], a[i], w[i])
        if t[i] - last_t >= step:
            d = g.odo - last_odo; hd = 0.5 * (g.heading + last_hd) if last_t > -1e8 else g.heading
            x += d * np.cos(hd); y += d * np.sin(hd)
            rows.append([t[i], x, y, 0.0, 0.0, 0.0, np.sin(0.5 * g.heading), np.cos(0.5 * g.heading)])
            last_odo = g.odo; last_hd = g.heading; last_t = t[i]
    return np.array(rows)


PDR_CFG = ['preset=robust', 'odom_sp=0.3', 'odom_kp=0.1', 'yaw_rw_deg=2.0', 'scale_rw=0.02', 'scale_prior=0.3']


def pdr_case(seq, cal, mode):
    kw = {}
    if mode == 'user' or mode == 'cross': kw = dict(model=cal[seq])
    elif mode == 'onl': kw = dict(online=True)
    tr = pdr_traj(seq, **kw)
    gps = lf.read_gps(ROB / seq / 'gps0' / 'data.csv') if seq not in INDOOR else np.zeros((0, 6))
    c = Case(f'pdr_{seq}_{mode}', tr, gps, [0, 0, 0], seq_gt(seq), False, metric=True)
    c.seq = seq
    return c


def pdr_one(args):
    seq, mode, cal = args
    c = pdr_case(seq, cal, mode)
    R = dict(seq=seq, mode=mode, alone=metr(c, c.traj))
    if len(c.gps):
        raw_t = c.traj[:, 0]
        for m in ('batch', 'causal'):
            r = run_c(c, m, tag='_pdr', odom_tr=c.traj, cfg=PDR_CFG)
            R['fused_' + m] = metr(c, r['out'], raw_t, raw_t[0] + INIT_WAIT if m == 'causal' else None)
        fx = c.gps[:, :4]; g = score(fx, c.gt, [0, 0, 0], len(fx), 1, geo=True)
        R['gnss'] = dict(se3=g.get('ate_se3'), sim3=g.get('ate_sim3'))
    return R


def run_pdr(cal, workers):
    tasks = []
    for s in SEQS:
        modes = ['gen'] + (['user'] if s in MOBILE else ['cross']) + (['onl'] if s in ('outdoor1', 'outdoor2', 'advio20') else [])
        for m in modes: tasks.append((s, m, cal))
    with Pool(workers) as p: res = p.map(pdr_one, tasks, chunksize=1)
    return res


# ------------------------------------------------------------------------------------------------------------------------------ zero-velocity factor + timing
def run_zupt():
    """drone INSANE outdoor_1 (55 s on the ground, then flight): stationary epochs of the gait module (IMU at rest) -> zero-velocity factor.
    (a) full OKVIS2 odometry (already stationary-accurate), (b) odometry cut 12..50 s so the fixes make GNSS-only nodes (the factor is the only motion model there)."""
    from gf_table import CASES
    D = REPO / 'external/drone/ins_o1'
    cmd = [str(ROOT / 'c' / 'gf_gait_run'), '--imu', str(D / 'mav0/imu0/data.csv'), '--epochs', str(WORK / 'drone_o1.ep'), '--epoch-dt', str(EPOCH_DT)]
    subprocess.run(cmd, check=True)
    ep = np.loadtxt(WORK / 'drone_o1.ep'); gt = np.loadtxt(D / 'gt_enu.tum')
    gts = []
    for r in ep:
        m = (gt[:, 0] > r[0] - EPOCH_DT) & (gt[:, 0] <= r[0])
        gts.append(np.linalg.norm(np.diff(gt[m, 1:3], axis=0), axis=1).sum() / EPOCH_DT if m.sum() > 2 else np.nan)
    gts = np.array(gts); still = np.isfinite(gts) & (gts < 0.1); move = np.isfinite(gts) & (gts > 0.5)
    R = dict(n_epochs=len(ep), n_gt_still=int(still.sum()), still_flagged=int((still & (ep[:, 3] == 1)).sum()), n_gt_moving=int(move.sum()), moving_flagged_still=int((move & (ep[:, 3] == 1)).sum()),
             moving_flagged_walk=int((move & (ep[:, 3] == 0)).sum()))
    sp = WORK / 'sp_drone.txt'
    with open(sp, 'w') as f:
        for r in ep:
            if int(r[3]) == 1: f.write('%.9f 0 0.05 %.1f 1\n' % (r[0], EPOCH_DT * 2))
    case = CASES['o1d_okvis'](); raw = prepared(case); raw_t = raw[:, 0]; t0 = raw_t[0]
    cut = raw[(raw[:, 0] < t0 + 12) | (raw[:, 0] > t0 + 50)]
    for mode in ('batch', 'causal'):
        for lab, cfg, spf in (('base', PRESET, None), ('zupt', PRESET + ['speed=1'], sp)):
            r = run_c(case, mode, tag='_zu' + lab, odom_tr=raw, cfg=cfg, speed_file=spf)
            sc = score_traj(case, restrict(r['out'], raw_t, t0 + 30 if mode == 'causal' else None))
            R[f'full_{lab}_{mode}'] = dict(se3=sc['ate_se3'], noalign=sc['ate_noalign'])
            r = run_c(case, mode, tag='_zuc' + lab, odom_tr=cut, cfg=cfg, speed_file=spf)
            out = r['out']; m = (out[:, 0] > t0 + 12) & (out[:, 0] < t0 + 50)
            gp, ok = case.gt.at(out[m, 0]); o = out[m, 1:4][ok]
            R[f'cut_{lab}_{mode}'] = dict(n=int(ok.sum()), rms_err=float(np.sqrt(((o - gp[ok]) ** 2).sum(1).mean())), scatter=float(np.sqrt(((o - o.mean(0)) ** 2).sum(1).mean())))
    return R


def run_timing():
    """per-call cost: gf_gait_push per IMU sample, gf_add_odom (new node) with / without speed measurements (o1_stella causal, 5827 odom samples, 394 fixes)"""
    import re
    R = {}
    t, a, w = gait.load_imu('outdoor1')
    cmd = [str(ROOT / 'c' / 'gf_gait_run'), '--imu', str(common.SEQ['outdoor1'][0]), '--epochs', str(WORK / 'timing.ep')]
    t_ = time.time(); subprocess.run(cmd, check=True); dt = time.time() - t_
    R['gait_run_s'] = dt; R['gait_samples'] = len(t); R['gait_us_per_sample_incl_io'] = 1e6 * dt / len(t)
    case = build_case('o1_stella'); raw = prepared(case)
    sp = WORK / 'speed_outdoor1_gen.txt'
    for lab, cfg, spf in (('base', PRESET, None), ('gait', PRESET + GAIT_CFG, sp)):
        r = run_c(case, 'causal', tag='_tm' + lab, odom_tr=raw, cfg=cfg, speed_file=spf, timing=True)
        R[lab] = {m.group(1): dict(mean=float(m.group(2)), p50=float(m.group(3)), p99=float(m.group(4)), max=float(m.group(5))) for m in
                  re.finditer(r'timing (add_odom \(new node\)|add_odom \(no node\)|add_fix)\s+n=\d+ mean=([\d.]+) us p50=([\d.]+) p99=([\d.]+) max=([\d.]+)', r['stdout'])}
    return R


# ------------------------------------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('what', nargs='?', default='all'); ap.add_argument('--cases', default=''); ap.add_argument('--workers', type=int, default=8)
    a = ap.parse_args()
    WORK.mkdir(exist_ok=True)
    t0 = time.time()
    calf = WORK / 'gait_cal.json'
    if calf.exists() and a.what != 'gait': cal = json.loads(calf.read_text())
    else:
        cal = calibrations(); calf.write_text(json.dumps(cal, indent=1)); print('calibration', cal, flush=True)
    if a.what in ('gait', 'all'):
        rows = gait_accuracy(cal); (WORK / 'gait_accuracy.json').write_text(json.dumps(rows, indent=1)); print('gait accuracy done %.0fs' % (time.time() - t0), flush=True)
    if a.what in ('fusion', 'all'):
        cases = a.cases.split(',') if a.cases else ALL_CASES
        res = run_fusion(cases, cal, a.workers); (WORK / (f'gait_fusion{OUTSUF}.json' if not a.cases else 'gait_fusion_part.json')).write_text(json.dumps(res, indent=1, default=float))
        print('fusion done %.0fs' % (time.time() - t0), flush=True)
    if a.what in ('zupt', 'all'):
        (WORK / 'gait_zupt.json').write_text(json.dumps(run_zupt(), indent=1, default=float)); print('zupt done', flush=True)
    if a.what in ('timing', 'all'):
        (WORK / 'gait_timing.json').write_text(json.dumps(run_timing(), indent=1, default=float)); print('timing done', flush=True)
    if a.what in ('pdr', 'all'):
        res = run_pdr(cal, a.workers); (WORK / 'gait_pdr.json').write_text(json.dumps(res, indent=1, default=float)); print('pdr done %.0fs' % (time.time() - t0), flush=True)


if __name__ == '__main__':
    main()
