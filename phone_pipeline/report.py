#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Result tables of the phone pipeline from runs/phone_pipeline/<seq>/scores.json (score.py) + the reference numbers of the earlier studies.
usage: report.py > runs/phone_pipeline/tables.md        (python with numpy: external/gnss/venv/bin/python)"""
import json, re, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'runs/phone_pipeline'
SEQS = ['indoor1', 'indoor2', 'outdoor1', 'outdoor2', 'advio15', 'advio20']
NAME = {'indoor1': 'Indoor-1', 'indoor2': 'Indoor-2', 'outdoor1': 'Outdoor-1', 'outdoor2': 'Outdoor-2', 'advio15': 'ADVIO-15', 'advio20': 'ADVIO-20'}
SHORT = {'indoor1': 'i1', 'indoor2': 'i2', 'outdoor1': 'o1', 'outdoor2': 'o2', 'advio15': 'a15', 'advio20': 'a20'}
HAS_FIX = {'outdoor1', 'outdoor2', 'advio20'}


def md_rows():
    rows = {}
    for p in ['runs/gnss_compare/phone_more/table.md', 'runs/gnss_compare/robustness/table.md', 'runs/gnss_compare/more_systems/table.md', 'runs/gnss_compare/more_systems2/table.md']:
        f = ROOT / p
        if not f.exists(): continue
        for l in f.read_text().splitlines():
            c = [x.strip() for x in l.strip().strip('|').split('|')]
            if len(c) >= 8 and re.match(r'^[a-z0-9_]+$', c[0]):
                try: rows[c[0]] = dict(sim3=float(c[3]) if c[3] != '-' else None, se3=float(c[4]) if c[4] != '-' else None, scale=float(c[5]) if c[5] != '-' else None, cov=c[7])
                except ValueError: pass
    return rows


def ref_rows():
    d = md_rows(); r = {}
    for s in SEQS:
        r[s] = dict(xrslam=d.get(f'{s}_xrslam'), rdvio=d.get(f'{s}_rdvio_xrsetting'), okvis=d.get(f'{s}_okvis_default'))
    # earlier gnss_fusion section 12 table (same odometry sets, batch / causal30 scored on the odometry sample times)
    gf = json.loads((ROOT / 'gnss_fusion/work/pre13/gait_fusion.json').read_text()) if (ROOT / 'gnss_fusion/work/pre13/gait_fusion.json').exists() else {}
    best = {}
    for s in SEQS:
        cand = []
        for lab, v in gf.items():
            if not lab.startswith(SHORT[s] + '_') or 'orb3mono' in lab: continue   # ORB-SLAM3 mono maps cover only 4-89 % of the epochs on 3 sequences and are GPL
            for k, m in v.items():
                if not isinstance(m, dict) or 'se3' not in m or m['se3'] is None or k.startswith('ng_') or k in ('odom', 'gnss'): continue
                if k.endswith('_batch') or k.endswith('_causal'):
                    cand.append((m['se3'], lab, k))
        if cand:
            bb = min((c for c in cand if c[2].endswith('_batch')), default=None); bc = min((c for c in cand if c[2].endswith('_causal')), default=None)
            best[s] = dict(batch=bb, causal=bc)
        if s in gf or True:
            pass
    return r, best, gf


def f(x, n=2):
    return '-' if x is None else (f'{x:.{n}f}' if abs(x) < 1000 else f'{x:.0f}')


def main():
    sc = {s: json.loads((OUT / s / 'scores.json').read_text()) for s in SEQS if (OUT / s / 'scores.json').exists()}
    refs, best, gf = ref_rows()
    L = []
    P = L.append
    # ---------------------------------------------------------------- headline
    P('### Headline: full pipeline (stella_vio gyro+R-frames+merge, gait, GNSS where usable), ATE SE3 [m], batch / causal\n')
    P('| sequence | full pipeline batch / causal | Sim3 batch / causal | scale ratio (est/true) batch | coverage batch / causal | rms distance to the fixes (geo-referencing check) batch / causal | GNSS alone | XRSLAM | RD-VIO (xrsetting) | OKVIS2-X | best earlier fusion, full-coverage odometry (batch / causal30) |')
    P('|---|---|---|---|---|---|---|---|---|---|---|')
    for s in SEQS:
        if s not in sc: continue
        r = sc[s]; k = 'full|both' if s in HAS_FIX else 'full|gait'
        b, c = r[k]['batch'], r[k]['causal']
        g = r.get('gnss_alone') or {}
        rf = refs[s]
        bst = best.get(s, {})
        bs = '-' if not bst else f"{f(bst['batch'][0])} ({bst['batch'][1]}) / {f(bst['causal'][0])} ({bst['causal'][1]})"
        geo = f"{f(b.get('fix_rms'))} / {f(c.get('fix_rms'))}" if s in HAS_FIX else 'n/a (no fixes)'
        P(f"| {NAME[s]} | **{f(b.get('se3'))} / {f(c.get('se3'))}** | {f(b.get('sim3'))} / {f(c.get('sim3'))} | {f(b.get('scale'))} | {b.get('coverage', 0) * 100:.0f}% / {c.get('coverage', 0) * 100:.0f}% ({c.get('coverage_all', 0) * 100:.0f}% of all frames) | {geo} | "
          f"{f(g.get('se3')) if s in HAS_FIX else 'n/a'} | {f((rf['xrslam'] or {}).get('se3'))} | {f((rf['rdvio'] or {}).get('se3'))} | {f((rf['okvis'] or {}).get('se3'))} | {bs} |")
    # ---------------------------------------------------------------- ablation
    P('\n### Ablation (ATE SE3 [m] batch / causal; first row Sim3 per map because its scale is arbitrary)\n')
    hdr = '| configuration | ' + ' | '.join(NAME[s] for s in SEQS if s in sc) + ' |'
    P(hdr); P('|---|' + '---|' * len([s for s in SEQS if s in sc]))

    def row(label, fn):
        cells = []
        for s in SEQS:
            if s not in sc: continue
            cells.append(fn(sc[s], s))
        P(f'| {label} | ' + ' | '.join(cells) + ' |')

    def bc(key, r, s, gnssneeded=False):
        if key not in r: return 'n/a'
        b, c = r[key]['batch'], r[key]['causal']
        return f"{f(b.get('se3'))} / {f(c.get('se3'))}"
    row('stella_vio default, camera only (Sim3 per map, batch only)', lambda r, s: f(r['raw_default'].get('sim3_per_map')) + f" ({r['raw_default'].get('coverage', 0) * 100:.0f}%)")
    row('+ gait speed prior (no GNSS)', lambda r, s: bc('default|gait', r, s))
    row('+ GNSS fixes (no gait)', lambda r, s: bc('default|gnss', r, s) if s in HAS_FIX else 'n/a (no usable fixes)')
    row('+ gait + GNSS', lambda r, s: bc('default|both', r, s) if s in HAS_FIX else bc('default|gait', r, s))
    row('+ R-frames + merge, + gait + GNSS', lambda r, s: bc('rm|both', r, s) if s in HAS_FIX else bc('rm|gait', r, s))
    row('full = + gyro prior (R-frames, merge, gait, GNSS)', lambda r, s: bc('full|both', r, s) if s in HAS_FIX else bc('full|gait', r, s))
    row('full with the dataset-calibration extrinsic instead of the sequence-fitted one (sensitivity)', lambda r, s: bc('fullcal|both', r, s) if s in HAS_FIX else bc('fullcal|gait', r, s))
    row('full, gait only (no GNSS; metric, not geo-referenced)', lambda r, s: bc('full|gait', r, s))
    row('full, GNSS only (no gait)', lambda r, s: bc('full|gnss', r, s) if s in HAS_FIX else 'n/a')
    P('\nSim3 / scale / coverage of the same rows, and the tracked-only variants (odometry sample times only, no bridged / GNSS-only poses, like the earlier gnss_fusion tables):\n')
    P('| sequence | configuration | SE3 b/c | Sim3 b/c | scale b/c | coverage b/c | tracked-only SE3 b/c30 | dist. to fixes b/c |'); P('|---|---|---|---|---|---|---|---|')
    for s in SEQS:
        if s not in sc: continue
        for v in ('default', 'rm', 'full'):
            for m in (('gait', 'gnss', 'both') if s in HAS_FIX else ('gait',)):
                k = f'{v}|{m}'
                if k not in sc[s]: continue
                r = sc[s][k]; b, c = r['batch'], r['causal']
                tb = r.get('batch_tracked', {}); tc = r.get('causal30_tracked', {})
                geo = f"{f(b.get('fix_rms'))} / {f(c.get('fix_rms'))}" if s in HAS_FIX and m != 'gait' else '-'
                P(f"| {NAME[s]} | {v}+{m} | {f(b.get('se3'))} / {f(c.get('se3'))} | {f(b.get('sim3'))} / {f(c.get('sim3'))} | {f(b.get('scale'))} / {f(c.get('scale'))} | {b.get('coverage', 0) * 100:.0f}% / {c.get('coverage', 0) * 100:.0f}% | {f(tb.get('se3'))} / {f(tc.get('se3'))} | {geo} |")
    # ---------------------------------------------------------------- section 14: slowly varying geo-referencing of the gait stream
    P('\n### Live (causal) fusion against GNSS alone, outdoor sequences: ATE SE3 [m] batch / causal (section 14; `georef` = fix-free gait stream + one slowly varying similarity from all fixes so far, gf_georef)\n')
    outs = [s for s in SEQS if s in sc and s in HAS_FIX]
    P('| configuration | ' + ' | '.join(NAME[s] for s in outs) + ' | mean vs GNSS alone batch / causal |'); P('|---|' + '---|' * (len(outs) + 1))
    def rel(key):
        rb, rc = [], []
        for s in outs:
            if key not in sc[s]: return '-'
            g_ = sc[s]['gnss_alone']['se3']; rb.append(sc[s][key]['batch']['se3'] / g_); rc.append(sc[s][key]['causal']['se3'] / g_)
        return f"{(sum(rb) / len(rb) - 1) * 100:+.1f}% / {(sum(rc) / len(rc) - 1) * 100:+.1f}%"
    P('| GNSS alone | ' + ' | '.join(f(sc[s]['gnss_alone']['se3']) for s in outs) + ' | |')
    for lab, key in (('full, gait only (no fixes; not geo-referenced)', 'full|gait'), ('full + gait + GNSS (section 13 default: fixes in the smoother)', 'full|both'),
                     ('**full, gait stream + georef (section 14)**', 'full|georef'), ('default + georef', 'default|georef'), ('rm + georef', 'rm|georef'), ('fullcal + georef', 'fullcal|georef')):
        P(f'| {lab} | ' + ' | '.join(bc(key, sc[s], s) for s in outs) + f' | {rel(key)} |')
    P('\nSim3 / scale / coverage / distance to the fixes of the georef rows (full):\n')
    P('| sequence | SE3 b/c | Sim3 b/c | scale b/c | coverage b/c | rms distance to the fixes b/c |'); P('|---|---|---|---|---|---|')
    for s in outs:
        if 'full|georef' not in sc[s]: continue
        r = sc[s]['full|georef']; b, c = r['batch'], r['causal']
        P(f"| {NAME[s]} | {f(b.get('se3'))} / {f(c.get('se3'))} | {f(b.get('sim3'))} / {f(c.get('sim3'))} | {f(b.get('scale'))} / {f(c.get('scale'))} | {b.get('coverage', 0) * 100:.0f}% / {c.get('coverage', 0) * 100:.0f}% | {f(b.get('fix_rms'))} / {f(c.get('fix_rms'))} |")
    # ---------------------------------------------------------------- timing
    P('\n### Timing (CPU seconds, shared machine; sv_run = whole stella_vio run incl. ORB extraction at full resolution, single thread)\n')
    P('| sequence | frames | sv_run default CPU s (ms/frame) | sv_run full CPU s (ms/frame) | gait us/IMU sample | gf_run batch s | gf_run causal s (us/frame) |'); P('|---|---|---|---|---|---|---|')
    for s in SEQS:
        if s not in sc: continue
        r = sc[s]; n = r['n_frames']
        d, fu = r['raw_default'].get('timing', {}), r['raw_full'].get('timing', {})
        gt = r.get('gait_timing', {})
        rj = json.loads((OUT / s / ('fuse_full_both' if s in HAS_FIX else 'fuse_full_gait') / 'run.json').read_text())
        wb, wc = rj['runs']['batch']['wall_s'], rj['runs']['causal']['wall_s']
        P(f"| {NAME[s]} | {n} | {f(d.get('cpu_s'), 0)} ({d.get('cpu_s', 0) / n * 1e3:.0f}) | {f(fu.get('cpu_s'), 0)} ({fu.get('cpu_s', 0) / n * 1e3:.0f}) | {f(gt.get('us_per_imu'), 1)} | {wb:.2f} | {wc:.2f} ({wc / n * 1e6:.0f}) |")
    # ---------------------------------------------------------------- SLAM events + timing details
    P('\n### stella_vio events per sequence and variant (from sv_run logs): lost frames / resets / maps / R-frames / bridges / loops accepted (the final trajectory used as odometry contains the SLAM back-end corrections of these loops)\n')
    P('| sequence | default | rm | full | fullcal |'); P('|---|---|---|---|---|')
    for s in SEQS:
        cells = []
        for v in ('default', 'rm', 'full', 'fullcal'):
            lg = OUT / s / f'sv_{v}' / 'log.txt'
            if not lg.exists(): cells.append('-'); continue
            t = lg.read_text()
            m1 = re.search(r'(\d+) loops accepted, (\d+) lost frames, (\d+) resets', t); m2 = re.search(r'maps=(\d+) reinits=(\d+) merges=(\d+) rframes=(\d+) rframes_gyro=(\d+) rbridges=(\d+)', t)
            cells.append(f"{m1.group(2)} / {m1.group(3)} / {m2.group(1)} / {m2.group(4)} / {m2.group(6)} / {m1.group(1)}" if m1 and m2 else '?')
        P(f"| {NAME[s]} | " + ' | '.join(cells) + ' |')
    P('\n### gf_run call latencies (full pipeline, causal run, one thread; us)\n')
    P('| sequence | add_odom without node (mean) | add_odom creating a node (mean / p99 / max) | add_fix (mean / p99) |'); P('|---|---|---|---|')
    for s in SEQS:
        if s not in sc: continue
        k = 'full|both' if s in HAS_FIX else 'full|gait'
        tl = {re.sub(r'\s+n=.*', '', l.replace('timing ', '')).strip(): l for l in sc[s][k].get('causal_timing', [])}
        def g(name, keys):
            l = next((v for kk, v in tl.items() if kk.startswith(name)), None)
            if not l: return '-'
            ms = [re.search(k_ + r'=([\d.]+)', l) for k_ in keys]
            return ' / '.join(m.group(1) for m in ms) if all(ms) else '-'
        P(f"| {NAME[s]} | {g('add_odom (no node)', ['mean'])} | {g('add_odom (new node)', ['mean', 'p99', 'max'])} | {g('add_fix', ['mean', 'p99'])} |")
    print('\n'.join(L))


if __name__ == '__main__':
    main()
