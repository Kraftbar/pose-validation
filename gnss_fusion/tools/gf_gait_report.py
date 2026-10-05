#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""markdown tables for section 12 from work/gait_*.json (run gf_gait_study.py first) -> work/gait_study.md"""
import json, sys
from pathlib import Path
import os
WORK = Path(os.environ['GF_WORK']) if os.environ.get('GF_WORK') else Path(__file__).resolve().parent.parent / 'work'
L = lambda n: json.loads((WORK / n).read_text())


def fm(v, d=2):
    if v is None: return '-'
    if abs(v) >= 100: return '%.0f' % v
    return f'{v:.{d}f}'


def cell(m, keys=('se3',)):
    if m is None or 'error' in m or m.get('se3') is None: return '-'
    return '/'.join(fm(m.get(k)) for k in keys)


def bc(r, key, keys=('se3',)):
    return cell(r.get(key + '_batch'), keys) + ' / ' + cell(r.get(key + '_causal'), keys)


def main():
    acc = L('gait_accuracy.json'); fus = L('gait_fusion.json'); pdr = L('gait_pdr.json'); zu = L('gait_zupt.json'); cal = L('gait_cal.json')
    out = []
    P = out.append
    P('### gait accuracy (3 s epochs, 6 s window, walking = GT speed > 0.5 m/s, ratio = gait speed / GT speed)\n')
    P('| sequence | calibration | GT-walking epochs | detected as WALK | median [IQR] | p10..p90 | distance ratio | rms rel. error | within 1 sigma | k (online) |')
    P('|---|---|---|---|---|---|---|---|---|---|')
    for r in acc:
        iq = '-' if r['median'] is None else f"{r['median']:.2f} [{r['q25']:.2f}, {r['q75']:.2f}]"
        pp = '-' if r['p10_90'] is None else f"{r['p10_90'][0]:.2f}..{r['p10_90'][1]:.2f}"
        P(f"| {r['seq']} | {r['mode']} | {r['n_gt_walk']} | {r['walk_detected']} ({r['n_gt_walk_other']} OTHER) | {iq} | {pp} | {fm(r['dist_ratio'])} | {fm(r['rel_rms'])} | {fm(r['sigma_cover'])} | {r['k_end']:.2f} |")
    P('')
    P('### per-user constants c (v = c cad^2; generic 0.389): ' + ', '.join(f"{k} {v:.3f}" for k, v in cal.items() if k != 'self') + '; leaky self fits: ' + ', '.join(f"{k} {v:.3f}" for k, v in cal['self'].items()) + '\n')

    short = {'o1': 'Outdoor-1', 'o2': 'Outdoor-2', 'i1': 'Indoor-1', 'i2': 'Indoor-2', 'a15': 'ADVIO-15', 'a20': 'ADVIO-20'}
    name = lambda l: short[l.split('_')[0]] + ' ' + l.split('_')[1]
    P('### with GNSS (ATE SE3 m, batch / causal30)\n')
    P('| case | GNSS alone | robust (section 11) | + gait generic | + gait per-user (held-out) / cross-user | + gait online GNSS |')
    P('|---|---|---|---|---|---|')
    for l, r in fus.items():
        if 'gnss' not in r: continue
        us = bc(r, 'gait_user') if 'gait_user_batch' in r else bc(r, 'gait_cross')
        P(f"| {name(l)} | {fm(r['gnss']['se3'])} | {bc(r, 'base')} | {bc(r, 'gait_gen')} | {us} | {bc(r, 'gait_onl')} |")
    P('')
    P('### without GNSS (metric trajectory from odometry + gait only; SE3 / Sim3 ATE m, scale ratio = estimated / true scale, path ratio = path length / GT path length)\n')
    P('| case | raw odometry | + gait generic batch | + gait generic causal30 | + gait per-user / cross-user batch | + gait per-user / cross-user causal30 |')
    P('|---|---|---|---|---|---|')
    keys = ('se3', 'sim3', 'scale', 'path')
    for l, r in fus.items():
        ind = r['indoor']
        g = 'gait_gen' if ind else 'ng_gait_gen'
        u = ('gait_user' if 'gait_user_batch' in r else 'gait_cross') if ind else ('ng_gait_user' if 'ng_gait_user_batch' in r else 'ng_gait_cross')
        P(f"| {name(l)}{' (indoor)' if ind else ''} | {cell(r['odom'], keys)} | {cell(r.get(g + '_batch'), keys)} | {cell(r.get(g + '_causal'), keys)} | {cell(r.get(u + '_batch'), keys)} | {cell(r.get(u + '_causal'), keys)} |")
    P('')
    P('### no odometry: gait speed + gyro-heading PDR (SE3 / Sim3 / scale / path), alone and + GNSS (SE3 batch / causal30)\n')
    P('| sequence | calibration | PDR alone | PDR + GNSS | GNSS alone |')
    P('|---|---|---|---|---|')
    for r in pdr:
        fu = '-' if 'fused_batch' not in r else cell(r['fused_batch']) + ' / ' + cell(r['fused_causal'])
        P(f"| {r['seq']} | {r['mode']} | {cell(r['alone'], keys)} | {fu} | {fm(r['gnss']['se3']) if 'gnss' in r else '-'} |")
    P('')
    P('### zero-velocity factor (drone INSANE outdoor_1)\n')
    P('```\n' + json.dumps(zu, indent=1) + '\n```')
    (WORK / 'gait_study.md').write_text('\n'.join(out) + '\n')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
