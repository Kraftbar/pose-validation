#!/usr/bin/env python3
"""Offline experiment: do RTKLIB SPP fixes + Doppler velocity help gnss_fusion (gf_run velocity input, unmodified)?
Case: GVINS complex_environment, odometry = OKVIS2-X mono VIO without GNSS (saved trajectory), fixes = RTKLIB SPP on the raw u-blox observations, 1 Hz.
Scores through gnss_fusion/tools/gf_cases (gnss_eval.score -> benchmark.umeyama_alignment). Nothing in gnss_fusion is edited (write_fixes is
monkey-patched in this process to emit the optional velocity columns).
usage: fusion_doppler_exp.py out.json"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, '/home/nybo/github/pose-validation/gnss_fusion/tools'); sys.path.insert(0, '/home/nybo/github/pose-validation/tools/blocks')
import gf_cases as C
from spp_score import R_enu, load_gt
REPO = Path('/home/nybo/github/pose-validation')
O = json.load(open(REPO / 'external/gnss/seq/complex/origin.json')); LLA0 = O['lla0']

def make_fixes(spp_path, step=10, kpos=0.6, kvel=2.0, vel=True, floor_h=1.0, floor_v=2.0, vel_floor=0.05, pos_off=None, block=None):
    d = np.loadtxt(spp_path); d = d[d[:, 1] >= 0]
    R0 = R_enu(LLA0[0], LLA0[1]); from spp_score import lla2ecef
    e0 = lla2ecef(*LLA0)
    rows = []
    for i in range(0, len(d), step):
        r = d[i]
        t_gps = 315964800.0 + 2140 * 604800.0 + (r[0] - r[9])     # tow minus receiver clock bias = true GPS time of the epoch
        t = t_gps - 18.0 - 0.0262                                  # same UTC/offset mapping as tools/gnss_harness (see docs/gnss_vio_benchmark s.2)
        p = R0 @ (r[3:6] - e0); v = R0 @ r[6:9]
        s3 = np.sqrt(r[10] + r[11] + r[12]); sv3 = np.sqrt(r[16] + r[17] + r[18])
        sh = max(kpos * s3 / np.sqrt(3), floor_h); sz = max(2 * kpos * s3 / np.sqrt(3), floor_v)
        sg = max(kvel * sv3 / np.sqrt(3), vel_floor)
        row = [t, p[0], p[1], p[2], sh, sz]
        if block and block[0] <= t - O['t0'] <= block[1]: row[4] = row[5] = 1e4
        if vel: row += [v[0], v[1], v[2], sg]
        rows.append(row)
    return np.array(rows)

def run(case, fixes, mode, cfg, tag, odom=None):
    case.gps = fixes
    C.write_fixes = lambda path, gps: np.savetxt(path, gps, fmt=['%.9f'] + ['%.6f'] * 3 + ['%.4f'] * 2 + (['%.5f'] * 3 + ['%.4f'] if gps.shape[1] > 6 else []))
    r = C.run_c(case, mode, cfg=cfg, tag=tag, odom_tr=odom)
    return r

def grid(out, spp, step, tag, odoms=(('metric_odom', 1.0), ('scaled0.5_odom', 0.5)), variants=None):
    case = C.complex_case('rtk'); res = {}; base = case.traj
    variants = variants or (('pos', dict(vel=False)), ('pos+vel(k=2)', dict(vel=True, kvel=2.0)), ('pos+vel(k=4)', dict(vel=True, kvel=4.0)),
                            ('pos(blk100-220)', dict(vel=False, block=(100, 220))), ('pos(blk)+vel(k=2)', dict(vel=True, kvel=2.0, block=(100, 220))))
    for odomname, scale in odoms:
        odom = base.copy(); odom[:, 1:4] *= scale; case.metric = scale == 1.0
        for fixname, kw in variants:
            fx = make_fixes(spp, step=step, **kw)
            for mode in ('batch', 'causal'):
                key = f'{odomname}|{fixname}|{mode}'
                try:
                    r = run(case, fx, mode, ['preset=robust'], '_dop_' + tag + str(abs(hash(key)) % 100000), odom=odom)
                    o = r['out']
                    if mode == 'causal': o = o[o[:, 0] >= case.t0 + 30]
                    sc = C.score_traj(case, o); res[key] = dict(se3=sc.get('ate_se3'), noalign=sc.get('ate_noalign'), n=sc.get('n_scored'))
                except Exception as ex: res[key] = dict(error=str(ex)[:200])
                print(tag, key, res[key], flush=True)
    case.gps = make_fixes(spp, step=step, vel=False)
    g = C.gnss_alone(case); res['gnss_alone'] = dict(se3=g.get('ate_se3'), noalign=g.get('ate_noalign'), n=g.get('n_scored'))
    json.dump(res, open(out, 'w'), indent=1, default=float)

if __name__ == '__main__':
    R = REPO / 'runs/blocks'
    if sys.argv[1] == 'main': grid(R / 'fusion_doppler.json', str(R / 'spp/best_s35.txt'), 10, 'a')
    elif sys.argv[1] == 'nomask': grid(R / 'fusion_doppler_nomask.json', str(R / 'spp/l1_GEC.txt'), 10, 'b', variants=(('pos', dict(vel=False, kpos=0.6)), ('pos+vel(k=2)', dict(vel=True, kvel=2.0)), ('pos+vel(k=4)', dict(vel=True, kvel=4.0))))
    elif sys.argv[1] == 'sparse': grid(R / 'fusion_doppler_5s.json', str(R / 'spp/best_s35.txt'), 50, 'c', variants=(('pos', dict(vel=False)), ('pos+vel(k=2)', dict(vel=True, kvel=2.0)), ('pos+vel(k=4)', dict(vel=True, kvel=4.0))))
