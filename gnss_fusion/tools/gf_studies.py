#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Improvement studies for the C library (own code):
  outliers : (a) robust loss + chi^2 gating on fixes with synthetic outliers / multipath bursts
  gaps     : GNSS-only nodes while the odometry is lost (output continues through tracking loss)
  drone    : INSANE o1 (55 s stationary start, yaw unobservable until the drone moves) and m14, geo-referenced error over time
  loss     : (b) odometry tracking loss: gap with the same frame, restart in a NEW frame (new map), scored vs python smoother with bridged gaps
  velocity : GNSS velocity factor (extension)
  blackout : (c) GNSS blackout bridging: error growth vs time, and the library's sigma_h estimate
usage: gf_studies.py outliers|loss|gaps|drone|blackout|velocity [--out work/study_<name>.json]"""
import sys, json
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa
from gf_table import restrict, CASES
from compare_py import py_run


def geo_err(case, out, tmin=None, tmax=None):
    """un-aligned antenna position error vs GT (ENU runs) for each pose"""
    A = antenna(out, case.rsa); gp, ok = case.gt.at(out[:, 0])
    e = np.linalg.norm(A - gp, axis=1)
    m = ok.copy()
    if tmin is not None: m &= out[:, 0] >= tmin
    if tmax is not None: m &= out[:, 0] <= tmax
    return out[m, 0], e[m]


def rms(e): return float(np.sqrt((e ** 2).mean())) if len(e) else float('nan')


def with_gps(case, gps):
    c = Case(case.name, case.traj, gps, case.rsa, case.gt, case.geo, case.metric, case.up, case.desc, case.t0)
    return c


def fmt(v): return '   -  ' if v is None or v != v else f'{v:6.2f}'


# ------------------------------------------------------------------------------------------------ (a)
def study_outliers():
    rng = np.random.default_rng(7)
    rows = {}
    for nm in ('complex_sim', 'o1_orb3mono', 'o1_okvis'):
        base = CASES[nm]()
        for kind in ('clean', 'spikes', 'burst', 'both'):
            g = base.gps.copy(); t0 = g[0, 0]
            if kind in ('spikes', 'both'):
                idx = rng.choice(len(g), size=max(3, len(g) // 20), replace=False)
                ang = rng.uniform(0, 2 * np.pi, len(idx)); mag = rng.uniform(5, 12, len(idx)) * g[idx, 4] + 15
                g[idx, 1] += mag * np.cos(ang); g[idx, 2] += mag * np.sin(ang)
            if kind in ('burst', 'both'):
                span = g[-1, 0] - t0
                for f in (0.35, 0.7):    # two 20 s multipath bursts with a common offset, reported sigma unchanged
                    m = (g[:, 0] >= t0 + f * span) & (g[:, 0] < t0 + f * span + 20)
                    g[m, 1] += 25 * 0.8; g[m, 2] += 25 * 0.6
            case = with_gps(base, g)
            raw = prepared(case)
            res = {}
            for mode in ('batch', 'causal'):
                for lbl, cfg in (('huber(py)', []), ('none', ['loss=0']), ('cauchy', ['loss=2', 'loss_k=2.385']), ('huber+gate', ['gate_chi2=16.27', 'robust_init=1']),
                                 ('cauchy+gate', ['loss=2', 'loss_k=2.385', 'gate_chi2=16.27', 'robust_init=1']), ('robust v1', ['preset=robust1']), ('robust', ['preset=robust'])):
                    r = run_c(case, mode, tag='_out', odom_tr=raw, cfg=cfg)
                    s = score_traj(case, restrict(r['out'], raw[:, 0], raw[0, 0] + 30 if mode == 'causal' else None))
                    res[f'{mode}:{lbl}'] = (s.get('ate_se3'), s.get('ate_noalign'))
            gs = score(g[:, :4], case.gt, [0, 0, 0], len(g), 1, geo=True)
            res['gnss'] = (gs.get('ate_se3'), gs.get('ate_noalign'))
            rows[f'{nm}/{kind}'] = res
            print(f"{nm:12s} {kind:7s} GNSS {fmt(res['gnss'][0])} | batch: " + ' '.join(f"{k.split(':')[1]} {fmt(v[0])}" for k, v in res.items() if k.startswith('batch'))
                  + ' | causal: ' + ' '.join(f"{k.split(':')[1]} {fmt(v[0])}" for k, v in res.items() if k.startswith('causal')), flush=True)
    return rows


# ------------------------------------------------------------------------------------------------ (b)
def make_loss_case(base, gaps, new_frame, rng_seed=11):
    """cut the odometry in the given (start, end) windows [s since start]; after each gap the odometry restarts either in the same frame
    (new_frame=False: position jump, e.g. relocalisation drift) or in an unrelated gravity-aligned frame (rotated about z, translated)."""
    rng = np.random.default_rng(rng_seed)
    tr = prepared(base).copy(); t0 = tr[0, 0]
    flags = np.zeros(len(tr), int); keep = np.ones(len(tr), bool)
    for (a, b) in gaps:
        m = (tr[:, 0] >= t0 + a) & (tr[:, 0] < t0 + b); keep &= ~m
        after = np.where(tr[:, 0] >= t0 + b)[0]
        if len(after) == 0: continue
        i0 = after[0]
        yaw = rng.uniform(-np.pi, np.pi) if new_frame else 0.0
        off = rng.uniform(-60, 60, 3) * np.array([1, 1, 0.0])
        if not new_frame: off = np.array([rng.uniform(15, 25), rng.uniform(-25, -15), 0.0])   # same frame, jump by a relocalisation
        R = lf.Rz(yaw)
        tr[i0:, 1:4] = tr[i0:, 1:4] @ R.T + off
        tr[i0:, 4:8] = np.array([lf.R_quat(R @ lf.quat_R(q)) for q in tr[i0:, 4:8]])
        flags[i0] = 1 if new_frame else 2
    return tr[keep], flags[keep]


def run_c_flags(case, mode, tr, flags, cfg):
    WORK.mkdir(exist_ok=True)
    base = WORK / f'{case.name}_flags_{mode}'
    with open(f'{base}.odom', 'w') as f:
        for r, fl in zip(tr, flags): f.write(' '.join('%.17g' % v for v in r[:8]) + (f' {int(fl)}' if fl else '') + '\n')
    write_fixes(f'{base}.fix', case.gps)
    cmd = [str(GF_RUN), '--odom', f'{base}.odom', '--fix', f'{base}.fix', '--out', f'{base}.out', '--mode', mode, 'rsa=%.6f,%.6f,%.6f' % tuple(case.rsa)] + list(cfg)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode: raise RuntimeError(r.stderr + r.stdout)
    return np.loadtxt(f'{base}.out'), r.stdout


def study_loss():
    rows = {}
    for nm in ('complex_sim', 'complex_rtk'):
        base = CASES[nm]()
        for gname, gaps in (('2 gaps 25s/20s', [(150, 175), (300, 320)]),):
            for new_frame in (False, True):
                tr, flags = make_loss_case(base, gaps, new_frame)
                res = {}
                # python smoother (bridges the gaps with constant-velocity pseudo-poses, one continuous frame)
                trf = fill_gaps(tr)
                out_py, *_ = lf.fuse(trf, base.gps, base.rsa, 1.0, 0.0), None
                out_py = out_py[0] if isinstance(out_py, tuple) else out_py
                s = score_traj(base, restrict(out_py, tr[:, 0])); res['python batch (bridged)'] = (s.get('ate_se3'), s.get('ate_noalign'))
                for mode in ('batch', 'causal'):
                    for lbl, cfg in (('C base, no flags (auto gap)', []), ('C robust v1', ['preset=robust1']), ('C robust', ['preset=robust'])):
                        fl = np.zeros_like(flags) if 'no flags' in lbl else flags
                        out, so = run_c_flags(base, mode, tr, fl, cfg)
                        s = score_traj(base, restrict(out, tr[:, 0], tr[0, 0] + 30 if mode == 'causal' else None))
                        res[f'C {mode} {lbl}'] = (s.get('ate_se3'), s.get('ate_noalign'))
                    out, so = run_c_flags(base, mode, tr, flags, ['preset=robust'])
                    s = score_traj(base, restrict(out, tr[:, 0], tr[0, 0] + 30 if mode == 'causal' else None)); res[f'C {mode} robust + flags'] = (s.get('ate_se3'), s.get('ate_noalign'))
                key = f"{nm} {gname} {'NEW frame' if new_frame else 'same frame, jump'}"
                rows[key] = res
                print(key); [print(f'    {k:42s} SE3 {fmt(v[0])}  geo {fmt(v[1])}') for k, v in res.items()]
                sys.stdout.flush()
    return rows


# ------------------------------------------------------------------------------------------------ GNSS-only nodes during odometry gaps
def study_gaps():
    """odometry cut for 25 / 60 / 120 s (same frame with a position jump, or an unrelated new frame); error of the fused output (un-aligned, vs GT) inside the gap,
    in the 30 s after it and over the whole run, with and without GNSS-only nodes. Without them there is no output inside the gap (only the first sample after it)."""
    rows = {}
    for nm in ('complex_sim', 'complex_rtk'):
        base = CASES[nm](); t0 = prepared(base)[0, 0]
        for D in (25, 60, 120):
            for new_frame in (False, True):
                tr, fl = make_loss_case(base, [(150, 150 + D)], new_frame)
                gp = base.gps; m = (gp[:, 0] >= t0 + 150) & (gp[:, 0] < t0 + 150 + D); gg, ok = base.gt.at(gp[m, 0])
                ge = rms(np.linalg.norm(gp[m, 1:4] - gg, axis=1))
                key = f"{nm} gap {D}s {'new frame' if new_frame else 'same frame'}"; res = {'gnss_in_gap': ge}
                for mode in ('batch', 'causal'):
                    for lbl, cfg in (('off', ['preset=robust', 'gnss_only_nodes=0']), ('on', ['preset=robust'])):
                        out, so = run_c_flags(base, mode, tr, fl, cfg)
                        _, eg = geo_err(base, out, t0 + 150, t0 + 150 + D); _, ea = geo_err(base, out, t0 + 150 + D, t0 + 150 + D + 30); _, ew = geo_err(base, out, t0 + 30)
                        res[f'{mode} {lbl}'] = dict(gap=rms(eg), n_gap=len(eg), after30=rms(ea), whole=rms(ew))
                rows[key] = res
                print(key + f"  GNSS in gap {ge:5.2f}")
                for k, v in res.items():
                    if k != 'gnss_in_gap': print(f"    {k:14s} in gap {fmt(v['gap'])} ({v['n_gap']:3d} poses)  30 s after {fmt(v['after30'])}  whole {fmt(v['whole'])}", flush=True)
    return rows


# ------------------------------------------------------------------------------------------------ drones: long stationary start
def study_drone():
    """INSANE outdoor_1: 55 s on the ground, then flight. Yaw of the odometry w.r.t. ENU is unobservable until the drone moves. Geo-referenced (un-aligned) error
    of the causal live output (after the alignment at 30 s) in 10 s bins, GNSS alone and batch for reference; also m14 (no stationary start)."""
    rows = {}
    for nm in ('o1d_okvis', 'm14_okvis'):
        c = CASES[nm](); raw = prepared(c); t0 = raw[0, 0]
        gp, ok = c.gt.at(c.gps[:, 0]); eg = np.linalg.norm(c.gps[:, 1:4] - gp, axis=1)
        res = {}
        runs = {'batch robust': run_c(c, 'batch', tag='_drB', odom_tr=raw, cfg=['preset=robust']), 'causal robust v1': run_c(c, 'causal', tag='_dr1', odom_tr=raw, cfg=['preset=robust1']),
                'causal robust': run_c(c, 'causal', tag='_dr2', odom_tr=raw, cfg=['preset=robust'])}
        print(f'{nm}: geo error (no alignment) in 10 s bins; GNSS alone / batch / causal live v1 / causal live now')
        span = raw[-1, 0] - t0
        for a in range(0, int(span), 10):
            row = []
            m = (c.gps[:, 0] >= t0 + a) & (c.gps[:, 0] < t0 + a + 10) & ok; row.append(rms(eg[m]))
            _, e = geo_err(c, runs['batch robust']['out'], t0 + a, t0 + a + 10); row.append(rms(e))
            for lab in ('causal robust v1', 'causal robust'):
                l = runs[lab]['live']; l = l[(l[:, 8].astype(int) & 1) == 1]; _, e = geo_err(c, l[:, :8], t0 + a, t0 + a + 10); row.append(rms(e) if len(e) else float('nan'))
            if a % 20 == 0 or nm == 'o1d_okvis': print(f'  {a:4d} s  ' + '  '.join(fmt(x) for x in row))
        for lab in ('batch robust', 'causal robust v1', 'causal robust'):
            o = runs[lab]['out'] if lab.startswith('batch') else runs[lab]['live'][(runs[lab]['live'][:, 8].astype(int) & 1) == 1][:, :8]
            _, e = geo_err(c, o, t0 + 30); res[lab] = rms(e)
        res['gnss'] = rms(eg[ok & (c.gps[:, 0] >= t0 + 30)])
        rows[nm] = res
        print(f"  whole run from 30 s, geo rms: GNSS {fmt(res['gnss'])}  batch {fmt(res['batch robust'])}  causal live v1 {fmt(res['causal robust v1'])}  causal live now {fmt(res['causal robust'])}", flush=True)
    return rows


# ------------------------------------------------------------------------------------------------ (c)
def study_blackout():
    rows = {}
    for nm in ('complex_rtk', 'complex_sim'):
        base = CASES[nm]()
        t0 = base.t0
        for D in (30, 60, 120, 240):
            g = base.gps; m = ~((g[:, 0] >= t0 + 100) & (g[:, 0] < t0 + 100 + D)); case = with_gps(base, g[m])
            raw = prepared(case)
            res = {}
            for mode, cfg in (('batch', []), ('causal', []), ('causal', ['preset=robust'])):
                r = run_c(case, mode, tag='_blk', odom_tr=raw, cfg=cfg)
                out = r['out'] if mode == 'batch' else r['live'][:, :8]
                out = out[(r['live'][:, 8].astype(int) & 1 == 1)] if mode == 'causal' else out
                bins = {}
                for (a, b) in ((0, 30), (30, 60), (60, 120), (120, 240)):
                    if a >= D: continue
                    t, e = geo_err(case, out, t0 + 100 + a, t0 + 100 + min(b, D)); bins[f'{a}-{b}s'] = rms(e)
                t, e = geo_err(case, out, t0 + 100, t0 + 100 + D)
                pre_t, pre_e = geo_err(case, out, t0 + 40, t0 + 100)
                lbl = mode + ('+robust' if cfg else '')
                res[lbl] = dict(bins=bins, max=float(e.max()) if len(e) else None, rms=rms(e), pre=rms(pre_e))
                if mode == 'causal' and not cfg:   # sigma_h calibration from the live output
                    live = r['live']; tl, el = geo_err(case, live[:, :8], t0 + 100, t0 + 100 + D)
                    sig = np.interp(tl, live[:, 0], live[:, 9])
                    res[lbl]['sigma_ratio_med'] = float(np.median(el / np.maximum(sig, 1e-3))); res[lbl]['sigma_cover'] = float((el < 2 * sig).mean())
            # odometry-only reference: the VIO dead-reckoned from the pose at blackout start (fused state at that time), i.e. what batch/causal degrade to
            key = f'{nm} blackout {D}s'; rows[key] = res
            print(key)
            for k, v in res.items():
                print(f"    {k:15s} err rms: " + ' '.join(f'{b} {x:6.2f}' for b, x in v['bins'].items()) + f" | whole {v['rms']:6.2f} max {v['max']:6.2f} | before blackout (60 s) {v['pre']:5.2f}" +
                      (f" | sigma_h: median err/sigma {v['sigma_ratio_med']:.2f}, within 2 sigma {v['sigma_cover']*100:.0f}%" if 'sigma_ratio_med' in v else ''), flush=True)
    return rows




# ------------------------------------------------------------------------------------------------ velocity input (extension, no python counterpart)
def study_velocity():
    """Doppler-like velocity (true ENU velocity from the RTK reference + 0.1 m/s noise) attached to the simulated SPP fixes"""
    rng = np.random.default_rng(5)
    base = CASES['complex_sim'](); g = base.gps
    gt = base.gt; tt = g[:, 0]
    p1, _ = gt.at(tt + 0.5); p0, _ = gt.at(tt - 0.5); v = (p1 - p0) + rng.normal(0, 0.1, (len(tt), 3))
    raw = prepared(base); res = {}
    for mode in ('batch', 'causal'):
        for lbl, usev in (('position only', False), ('position + velocity', True)):
            WORK.mkdir(exist_ok=True)
            b = WORK / f'vel_{mode}_{int(usev)}'
            write_odom(f'{b}.odom', raw)
            cols = np.c_[g[:, :6], v, np.full(len(g), 0.15)] if usev else g[:, :6]
            np.savetxt(f'{b}.fix', cols, fmt=['%.9f'] + ['%.6f'] * (cols.shape[1] - 1))
            r = subprocess.run([str(GF_RUN), '--odom', f'{b}.odom', '--fix', f'{b}.fix', '--out', f'{b}.out', '--mode', mode, 'rsa=%.6f,%.6f,%.6f' % tuple(base.rsa), 'preset=robust'], capture_output=True, text=True, check=True)
            out = np.loadtxt(f'{b}.out'); s = score_traj(base, restrict(out, raw[:, 0], raw[0, 0] + 30 if mode == 'causal' else None))
            res[f'{mode} {lbl}'] = (s.get('ate_se3'), s.get('ate_noalign'))
            print(f'{mode:7s} {lbl:22s} SE3 {fmt(s.get("ate_se3"))}  geo {fmt(s.get("ate_noalign"))}')
    return res


if __name__ == '__main__':
    which = sys.argv[1]
    res = {'outliers': study_outliers, 'loss': study_loss, 'gaps': study_gaps, 'drone': study_drone, 'blackout': study_blackout, 'velocity': study_velocity}[which]()
    WORK.mkdir(exist_ok=True)
    Path(WORK / f'study_{which}.json').write_text(json.dumps(res, indent=1, default=float))
