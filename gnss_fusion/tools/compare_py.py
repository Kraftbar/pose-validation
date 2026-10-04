#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Python reference (tools/gnss_loose_fusion.py) vs the C library on the same inputs: position differences per pose, node states.
usage: compare_py.py [case ...]   cases: complex_rtk complex_sim complex_rtk_blk complex_sim_blk o1_okvis o2_okvis o1_orb3mono ...
Batch: both run the same iterations (25) on the same nodes. Causal: python replays the sliding window per node; the C library is fed the fixes
0.5 s ahead (--lookahead 0.5) so that every node has its fix when it is solved (python's association uses the fix up to half a node later)."""
import sys, time, json
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa

CASES = {
    'complex_rtk': lambda: complex_case('rtk'), 'complex_sim': lambda: complex_case('sim'),
    'complex_rtk_blk': lambda: complex_case('rtk_blk'), 'complex_sim_blk': lambda: complex_case('sim_blk'),
    'o1_okvis': lambda: phone_case('outdoor1', G / 'out/okvis2x_mobile_nogps_outdoor1/okvis2-vio-final_trajectory.csv', label='o1_okvis'),
    'o2_okvis': lambda: phone_case('outdoor2', 'outdoor2_okvis_default', label='o2_okvis'),
    'o1_orb3mono': lambda: phone_case('outdoor1', 'outdoor1_orb3_mono', cam_only=True, label='o1_orb3mono'),
    'o2_orb3mono': lambda: phone_case('outdoor2', 'outdoor2_orb3_mono', cam_only=True, label='o2_orb3mono'),
}


def prenorm_unit(tr, gps, causal):
    """python-side stand-in for the library's per-frame unit (2D Procrustes scale of the odometry vs the fixes), for non-metric odometry"""
    t = tr[:, 0]
    idx = np.clip(np.searchsorted(t, np.arange(t[0], t[-1], 1.0)), 0, len(t) - 1); tn = t[idx]
    j = np.clip(np.searchsorted(gps[:, 0], tn), 1, len(gps) - 1); j = np.where(np.abs(gps[j - 1, 0] - tn) < np.abs(gps[j, 0] - tn), j - 1, j)
    ok = np.abs(gps[j, 0] - tn) <= 0.5 + 1e-6
    if causal: ok &= (tn - tn[0] <= 30.0)
    P = tr[idx[ok], 1:4]; Z = gps[j[ok], 1:4]
    a = (P - P.mean(0))[:, :2]; c = (Z - Z.mean(0))[:, :2]
    return float(np.sqrt((c ** 2).sum() / max((a ** 2).sum(), 1e-9)))


def py_run(case, causal):
    tr = fill_gaps(prepared(case))
    unit = 1.0
    if not case.metric:
        unit = prenorm_unit(tr, case.gps, causal); tr = tr.copy(); tr[:, 1:4] *= unit
    t0 = time.time()
    out, X, tn = lf.fuse(tr, case.gps, case.rsa, 1.0, causal)
    return out, X, tn, tr, unit, time.time() - t0


def main():
    names = sys.argv[1:] or ['complex_rtk', 'complex_sim', 'o1_okvis', 'o2_okvis', 'o1_orb3mono']
    rows = []
    for nm in names:
        case = CASES[nm]()
        for mode, causal in (('batch', 0.0), ('causal', 30.0)):
            out, X, tn, tr_f, unit, pyw = py_run(case, causal)
            res = run_c(case, mode, tag='_eq', odom_tr=fill_gaps(prepared(case)), lookahead=0.5 if mode == 'causal' else None)
            co = res['out']
            n = min(len(co), len(out))
            if len(co) != len(out):
                print(f'  note: {nm} {mode} pose counts differ C {len(co)} python {len(out)}')
            # python output is in 'unit'-scaled odometry space only through the state; positions are metric already
            d = np.linalg.norm(co[:n, 1:4] - out[:n, 1:4], axis=1)
            nodes = res['nodes']
            nn = min(len(nodes), len(X))
            dX = np.abs(nodes[:nn, 1:6] - X[:nn]) if nn else np.zeros((1, 5))
            # rotation difference
            Rd = [np.degrees(np.arccos(np.clip((np.trace(lf.quat_R(a) @ lf.quat_R(b).T) - 1) / 2, -1, 1))) for a, b in zip(co[:n:50, 4:8], out[:n:50, 4:8])]
            row = dict(case=nm, mode=mode, n=n, pos_max_mm=1e3 * d.max(), pos_rms_mm=1e3 * np.sqrt((d ** 2).mean()), pos_p99_mm=1e3 * np.percentile(d, 99),
                       node_dpsi_max_urad=1e6 * dX[:, 0].max(), node_dpos_max_mm=1e3 * dX[:, 1:4].max(), node_ds_max=dX[:, 4].max(), rot_max_deg=max(Rd), unit=unit, py_s=pyw, c_s=res['wall'])
            rows.append(row)
            print(f"{nm:14s} {mode:6s} n={n:5d} pos diff max {row['pos_max_mm']:.4f} mm  rms {row['pos_rms_mm']:.4f} mm  p99 {row['pos_p99_mm']:.4f} mm | node psi {row['node_dpsi_max_urad']:.3f} urad, pos {row['node_dpos_max_mm']:.4f} mm, s {row['node_ds_max']:.2e} | rot {row['rot_max_deg']:.5f} deg | py {pyw:.1f}s C {res['wall']:.2f}s")
    (WORK / 'compare_py.json').write_text(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main()
