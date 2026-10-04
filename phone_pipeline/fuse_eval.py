#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Fusion-stage-only experiments on the SAVED pipeline inputs (runs/phone_pipeline/<seq>/fuse_full_both/{odom,fix}.txt + speed.txt; stella_vio is not rerun).
Runs gf_run batch + causal with extra config keys in a scratch dir, scores with phone_pipeline/score.py (same code as the section-13 tables), deletes the outputs.

usage: fuse_eval.py [--seqs outdoor1,outdoor2,advio20] [--fm both|gnss] [--workers N] [--csv out.csv] [--rows] "label|key=val key=val" ...
       (label 'base' with empty config = the section-13 configuration; the result must equal runs/phone_pipeline/<seq>/scores.json)
"""
import sys, os, json, shutil, subprocess, tempfile, argparse
from pathlib import Path
from multiprocessing import Pool
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import score as S, run as R  # noqa: E402

_cache = {}


def _case(seq):
    if seq not in _cache:
        some = next(iter(sorted((R.OUT / seq).glob('sv_*/trajectory_maps.tum'))))
        _cache[seq] = (S.phone_case(seq, str(some.resolve()), cam_only=False, label=seq), S.frame_times(seq))
    return _cache[seq]


def one(a):
    label, cfgs, seq, fm, scratch = a
    src = R.OUT / seq / f'fuse_full_{fm}'
    d = Path(tempfile.mkdtemp(dir=scratch))
    try:
        if cfgs.startswith('G:'):   # fix-free (gait-only) stream, then gf_georef_run (section 14)
            gsrc = R.OUT / seq / 'fuse_full_gait'
            for f in ('odom.txt', 'fix.txt'): os.symlink(src / f, d / f)
            for md, sf, out in (('batch', 'batch.out', 'batch.out'), ('causal', 'causal.live', 'causal.live')):
                r = subprocess.run([str(R.ROOT / 'gnss_fusion/c/gf_georef_run'), '--stream', str(gsrc / sf), '--fix', str(src / 'fix.txt'), '--out', str(d / out), '--mode', md] + cfgs[2:].split(), capture_output=True, text=True)
                if r.returncode: return label, seq, dict(err=(r.stderr + r.stdout)[-300:])
                (d / f'{md}.stdout').write_text(r.stdout)
            (d / 'run.json').write_text(json.dumps(dict(init_wait_s=30.0)))
            case, ft = _case(seq)
            res = S.fused(case, d, ft, R.rp(R.cfg_of(seq)['fixes']))
            return label, seq, dict(b=res['batch'].get('se3'), c=res['causal'].get('se3'), bsim=res['batch'].get('sim3'), csim=res['causal'].get('sim3'),
                                    bcov=res['batch'].get('coverage'), ccov=res['causal'].get('coverage'), bfix=res['batch'].get('fix_rms'), cfix=res['causal'].get('fix_rms'))
        for f in ('odom.txt', 'fix.txt'): os.symlink(src / f, d / f)
        sp = R.OUT / seq / 'speed.txt'
        for md in ('batch', 'causal'):
            cmd = [str(R.GF_RUN), '--odom', str(d / 'odom.txt'), '--fix', str(d / 'fix.txt'), '--out', str(d / f'{md}.out'), '--mode', md, '--timing', '--nodes', str(d / f'{md}.nodes')]
            if md == 'causal': cmd += ['--out-live', str(d / 'causal.live')]
            cfg = list(R.GF_BASE)
            if fm == 'both': cmd += ['--speed', str(sp)]; cfg += R.GAIT_CFG
            cfg += [f'loose_k={R.LOOSE_K}'] + cfgs.split()
            r = subprocess.run(cmd + cfg, capture_output=True, text=True)
            if r.returncode: return label, seq, dict(err=(r.stderr + r.stdout)[-300:])
            (d / f'{md}.stdout').write_text(r.stdout)
        (d / 'run.json').write_text(json.dumps(dict(init_wait_s=30.0)))
        case, ft = _case(seq)
        res = S.fused(case, d, ft, R.rp(R.cfg_of(seq)['fixes']))
        return label, seq, dict(b=res['batch'].get('se3'), c=res['causal'].get('se3'), bsim=res['batch'].get('sim3'), csim=res['causal'].get('sim3'),
                                bcov=res['batch'].get('coverage'), ccov=res['causal'].get('coverage'), bfix=res['batch'].get('fix_rms'), cfix=res['causal'].get('fix_rms'))
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seqs', default='outdoor1,outdoor2,advio20'); ap.add_argument('--fm', default='both')
    ap.add_argument('--workers', type=int, default=8); ap.add_argument('--csv'); ap.add_argument('--json')
    ap.add_argument('variants', nargs='+')
    a = ap.parse_args()
    scratch = os.environ.get('GF_SCRATCH', tempfile.gettempdir()); Path(scratch).mkdir(parents=True, exist_ok=True)
    seqs = a.seqs.split(',')
    jobs = []
    for v in a.variants:
        label, _, cfg = v.partition('|')
        for s in seqs: jobs.append((label, cfg, s, a.fm, scratch))
    with Pool(a.workers) as p: out = p.map(one, jobs, chunksize=1)
    tab = {}
    for label, seq, r in out: tab.setdefault(label, {})[seq] = r
    gal = {s: json.loads((R.OUT / s / 'scores.json').read_text())['gnss_alone']['se3'] for s in seqs}
    print('%-34s' % 'variant (batch / causal SE3 m)' + ''.join('%-22s' % s for s in seqs) + 'mean b/c rel. GNSS')
    print('%-34s' % 'GNSS alone' + ''.join('%-22.2f' % gal[s] for s in seqs))
    for label, row in tab.items():
        cells, rb, rc = [], [], []
        for s in seqs:
            r = row[s]
            if 'err' in r: cells.append('ERR'); continue
            cells.append('%.2f / %.2f' % (r['b'], r['c'])); rb.append(r['b'] / gal[s]); rc.append(r['c'] / gal[s])
        print('%-34s' % label + ''.join('%-22s' % c for c in cells) + ('%+.1f%% / %+.1f%%' % (100 * (sum(rb) / len(rb) - 1), 100 * (sum(rc) / len(rc) - 1)) if rb else ''))
    if a.json: Path(a.json).write_text(json.dumps(tab, indent=1))
