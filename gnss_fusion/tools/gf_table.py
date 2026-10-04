#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Results table: per sequence, GNSS alone vs python smoother vs C smoother (python-equivalent settings) vs C with the improvements.
Scores: gnss_eval.score (benchmark.umeyama_alignment): ATE SE3 (aligned) and, for geo-referenced ENU runs (complex), the un-aligned error.
usage: gf_table.py [--cases a,b,c] [--no-py] [--out work/table.json]"""
import sys, json, time, argparse
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa
from compare_py import CASES, py_run

ADVIO = lambda seq, run, cam: (lambda: phone_case(seq, run, cam_only=cam, label=f'{seq}_{run}'))
CASES.update({
    'o1_stella': lambda: phone_case('outdoor1', 'outdoor1_stella_up', cam_only=True, label='o1_stella'),
    'o2_stella': lambda: phone_case('outdoor2', 'outdoor2_stella_up', cam_only=True, label='o2_stella'),
    'a20_okvis': lambda: phone_case('advio20', 'advio20_okvis_default', label='a20_okvis'),
    'a20_orb3mono': lambda: phone_case('advio20', 'advio20_orb3_mono', cam_only=True, label='a20_orb3mono'),
    'a15_okvis': lambda: phone_case('advio15', 'advio15_okvis_default', label='a15_okvis'),
    'm14_okvis': lambda: drone_case('m14', 'okvis2_mono', 'm14_okvis'),
    'o1d_okvis': lambda: drone_case('o1', 'okvis2_mono', 'o1d_okvis'),
    'o1_xrslam': lambda: xrslam_case('outdoor1', 'o1_xrslam'),
    'o2_xrslam': lambda: xrslam_case('outdoor2', 'o2_xrslam'),
    'a20_xrslam': lambda: xrslam_case('advio20', 'a20_xrslam'),
    'a15_orb3mono': lambda: phone_case('advio15', 'advio15_orb3_mono', cam_only=True, label='a15_orb3mono'),
})
GEO_CASES = ('m14_okvis', 'o1d_okvis')
IMPROVED = ['preset=robust1']   # first robust preset (section 10): the baseline of section 11
NEW = ['preset=robust']        # current gf_config_robust() (section 11 features)
INIT_WAIT = 30.0


def restrict(out, raw_t, tmin=None):
    if out is None: return None
    m = np.isin(np.round(out[:, 0], 6), np.round(raw_t, 6))
    if tmin is not None: m &= out[:, 0] >= tmin
    return out[m]


def blackout_windows(case):
    return None


def covered_fixes(case, raw_t):
    k = np.clip(np.searchsorted(raw_t, case.gps[:, 0]), 1, len(raw_t) - 1)
    near = np.minimum(abs(raw_t[k] - case.gps[:, 0]), abs(raw_t[k - 1] - case.gps[:, 0])) < 1.0
    return near


def eval_case(case, do_py=True, cfg_extra=(), verbose=True):
    raw = prepared(case)
    raw_t = raw[:, 0]
    tmin_c = raw_t[0] + INIT_WAIT
    R = {}
    def sc(tr, key, tmin=None):
        s = score_traj(case, restrict(tr, raw_t, tmin))
        R[key] = dict(se3=s.get('ate_se3'), noalign=s.get('ate_noalign'), n=s.get('n_scored'))
    # raw odometry (SE3-aligned: shows how bad the odometry is)
    s0 = score_traj(case, raw[:, :8]); R['odom'] = dict(se3=s0.get('ate_se3'), noalign=None, n=s0.get('n_scored'))
    fx = case.gps[:, :4]
    g = score(fx, case.gt, [0, 0, 0], len(fx), 1, geo=True); R['gnss'] = dict(se3=g.get('ate_se3'), noalign=g.get('ate_noalign'), n=g.get('n_scored'))
    near = covered_fixes(case, raw_t)
    if near.sum() >= 20 and near.sum() < len(near) - 3:
        g2 = score(fx[near], case.gt, [0, 0, 0], int(near.sum()), 1, geo=True); R['gnss_cov'] = dict(se3=g2.get('ate_se3'), noalign=g2.get('ate_noalign'), n=g2.get('n_scored'))
    else: R['gnss_cov'] = R['gnss']
    filled = fill_gaps(raw)
    if do_py:
        for mode, causal in (('batch', 0.0), ('causal', 30.0)):
            out, *_ = py_run(case, causal)
            sc(out, f'py_{mode}', tmin_c if causal else None)
    for mode in ('batch', 'causal'):
        r = run_c(case, mode, tag='_base', odom_tr=filled, lookahead=0.5 if mode == 'causal' else None, cfg=cfg_extra)
        sc(r['out'], f'c_{mode}', tmin_c if mode == 'causal' else None)
        for key, preset in (('ci', IMPROVED), ('cn', NEW)):
            r = run_c(case, mode, tag='_' + key, odom_tr=raw, cfg=list(preset) + list(cfg_extra))
            sc(r['out'], f'{key}_{mode}', tmin_c if mode == 'causal' else None)
            if key == 'cn':   # full time line: also the GNSS-only poses made while the odometry was lost (scored against the same GT)
                s = score_traj(case, r['out'][r['out'][:, 0] >= (tmin_c if mode == 'causal' else -1)])
                R[f'cn_full_{mode}'] = dict(se3=s.get('ate_se3'), noalign=s.get('ate_noalign'), n=s.get('n_scored'))
            if mode == 'causal':
                live = r['live']
                if live is not None:
                    live = live[(live[:, 8].astype(int) & 1) == 1]
                    sc(live[:, :8], f'{key}_live', tmin_c)
                    s = score_traj(case, live[live[:, 0] >= tmin_c][:, :8])
                    if key == 'cn': R['cn_live_full'] = dict(se3=s.get('ate_se3'), noalign=s.get('ate_noalign'), n=s.get('n_scored'))
            R[f'stdout_{key}_{mode}'] = r['stdout'].strip()
    if verbose:
        f = lambda v: '   -  ' if v is None else f'{v:6.2f}'
        print(f"{case.name:18s} odom {f(R['odom']['se3'])} | GNSS {f(R['gnss']['se3'])} (cov {f(R['gnss_cov']['se3'])}) | py b/c {f(R.get('py_batch',{}).get('se3'))}/{f(R.get('py_causal',{}).get('se3'))}"
              f" | C b/c {f(R['c_batch']['se3'])}/{f(R['c_causal']['se3'])} | robust1 b/c/live {f(R['ci_batch']['se3'])}/{f(R['ci_causal']['se3'])}/{f(R.get('ci_live',{}).get('se3'))}"
              f" | robust b/c/live {f(R['cn_batch']['se3'])}/{f(R['cn_causal']['se3'])}/{f(R.get('cn_live',{}).get('se3'))}", flush=True)
    return R


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--cases', default=''); ap.add_argument('--no-py', action='store_true'); ap.add_argument('--out', default=str(WORK / 'table.json'))
    ap.add_argument('--py-from', default='', help='reuse the python-smoother columns of an earlier table.json (the python reference does not change)')
    ap.add_argument('--cfg', default='', help='extra key=value for both variants, comma separated')
    a = ap.parse_args()
    names = a.cases.split(',') if a.cases else ['complex_rtk', 'complex_sim', 'complex_rtk_blk', 'complex_sim_blk', 'o1_okvis', 'o2_okvis', 'a15_okvis', 'a20_okvis',
                                                 'o1_orb3mono', 'o2_orb3mono', 'a15_orb3mono', 'a20_orb3mono', 'o1_stella', 'o2_stella',
                                                 'm14_okvis', 'o1d_okvis', 'o1_xrslam', 'o2_xrslam', 'a20_xrslam']
    cfg = [c for c in a.cfg.split(',') if c]
    res = {}
    for nm in names:
        old = json.loads(Path(a.py_from).read_text()).get(nm) if a.py_from else None
        res[nm] = eval_case(CASES[nm](), do_py=not (a.no_py or old), cfg_extra=cfg)
        if old:
            for k in ('py_batch', 'py_causal'): res[nm][k] = old[k]
    WORK.mkdir(exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=float))
    f = lambda v: '-' if v is None else f'{v:.2f}'
    L = ['| sequence | raw odometry | GNSS alone (all / on covered epochs) | python batch / causal30 | C batch / causal30 (python-equivalent) | C robust v1 batch / causal30 / live | C robust (now) batch / causal30 / live |', '|---|---|---|---|---|---|---|']
    G_ = ['| sequence (geo-referenced ENU only) | GNSS alone | python batch / causal30 | C batch / causal30 | C robust v1 batch / causal30 | C robust (now) batch / causal30 / live |', '|---|---|---|---|---|---|']
    F_ = ['| sequence (odometry partial or lost) | GNSS alone (all fixes) | C robust (now) full time line batch / causal30 (incl. GNSS-only poses) |', '|---|---|---|']
    for nm, R in res.items():
        k = lambda key, fld='se3': f(R.get(key, {}).get(fld))
        L.append(f"| {nm} | {k('odom')} | {k('gnss')} / {k('gnss_cov')} | {k('py_batch')} / {k('py_causal')} | {k('c_batch')} / {k('c_causal')} | {k('ci_batch')} / {k('ci_causal')} / {k('ci_live')} | {k('cn_batch')} / {k('cn_causal')} / {k('cn_live')} |")
        if nm.startswith('complex') or nm in GEO_CASES:
            G_.append(f"| {nm} | {k('gnss','noalign')} | {k('py_batch','noalign')} / {k('py_causal','noalign')} | {k('c_batch','noalign')} / {k('c_causal','noalign')} | {k('ci_batch','noalign')} / {k('ci_causal','noalign')} | {k('cn_batch','noalign')} / {k('cn_causal','noalign')} / {k('cn_live','noalign')} |")
        if R['gnss_cov']['se3'] != R['gnss']['se3']:
            F_.append(f"| {nm} | {k('gnss')} | {k('cn_full_batch')} / {k('cn_full_causal')} |")
    (WORK / 'table.md').write_text('\n'.join(L) + '\n\n' + '\n'.join(G_) + '\n\n' + '\n'.join(F_) + '\n')
    print('\n'.join(L)); print(); print('\n'.join(G_)); print(); print('\n'.join(F_))


if __name__ == '__main__':
    main()
