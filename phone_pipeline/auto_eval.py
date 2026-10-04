#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Section 15: replay of the phone pipeline fusion inputs through gf_auto_run (smoother with fixes, fix-free gait smoother -> georef, automatic switch) and score
sm / geo / auto with score.py (camera-frame resampling, causal from +30 s).
  source final : runs/phone_pipeline/<seq>/fuse_full_both/{odom,fix}.txt + speed.txt   (odometry of the FINAL stella_vio trajectory)
  source live  : runs/phone_pipeline/<seq>/live_full/pp.{odom,speed}                    (odometry as emitted frame by frame by pp_live)
usage: auto_eval.py [--seqs outdoor1,outdoor2,advio20] [--src final|live|both] [--cfg 'key=val ...'] [--keep DIR] [--json out.json]
"""
import sys, os, json, argparse, subprocess, tempfile, shutil
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'gnss_fusion/tools'))
import run as R, score as S  # noqa: E402
from gf_cases import phone_case  # noqa: E402

AUTO = R.ROOT / 'gnss_fusion/c/gf_auto_run'
BASE = ['preset=robust', 'metric=0', 'rsa=0,0,0', 'speed=1', 'speed_align=1', 'speed_scale_rw_rel=1', 'speed_align_metric=1', 'loose_k=5', 'stream=1', 'g.scale_sigma=0.15']
_cache = {}


def case_of(seq):
    if seq not in _cache:
        some = R.OUT / seq / 'sv_full' / 'trajectory_maps.tum'
        _cache[seq] = (phone_case(seq, str(some.resolve()), cam_only=False, label=seq), S.frame_times(seq))
    return _cache[seq]


def load(path, aligned=True):
    if not Path(path).exists() or Path(path).stat().st_size == 0: return None
    a = np.loadtxt(path, ndmin=2)
    if aligned and a.shape[1] > 8: a = a[(a[:, 8].astype(int) & 1) == 1]
    return a[np.argsort(a[:, 0], kind='stable')]


def inputs(seq, src):
    b = R.OUT / seq
    if src == 'final': return b / 'fuse_full_both/odom.txt', b / 'fuse_full_both/fix.txt', b / 'speed.txt'
    d = b / 'live_full'
    fx = b / 'fuse_full_both/fix.txt'
    return d / 'pp.odom', fx, d / 'pp.speed'


def run(seq, src='final', cfg=(), keep=None):
    case, ft = case_of(seq)
    odom, fix, sp = inputs(seq, src)
    d = Path(keep) if keep else Path(tempfile.mkdtemp(dir=os.environ.get('GF_SCRATCH')))
    d.mkdir(parents=True, exist_ok=True)
    try:
        b = d / f'{seq}_{src}'
        r = subprocess.run([str(AUTO), '--odom', str(odom), '--fix', str(fix), '--speed', str(sp), '--out', f'{b}.auto', '--out-sm', f'{b}.sm', '--out-geo', f'{b}.geo', '--sig', f'{b}.sig', '--timing'] + BASE + list(cfg),
                           capture_output=True, text=True)
        if r.returncode: raise RuntimeError(r.stderr + r.stdout)
        res = dict(stdout=r.stdout.strip())
        fixes = R.rp(R.cfg_of(seq)['fixes'])
        for k in ('sm', 'geo', 'auto'):
            a = load(f'{b}.{k}', aligned=(k != 'geo'))
            res[k] = S.metrics(case, a, ft, tmin=ft[0] + 30.0) if a is not None else dict(coverage=0.0)
        return res
    finally:
        if not keep: shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--seqs', default='outdoor1,outdoor2,advio20'); ap.add_argument('--src', default='both'); ap.add_argument('--cfg', default='')
    ap.add_argument('--keep'); ap.add_argument('--json')
    a = ap.parse_args()
    out = {}
    f = lambda x: '-' if x is None else f'{x:.2f}'
    for s in a.seqs.split(','):
        gal = json.loads((R.OUT / s / 'scores.json').read_text())['gnss_alone']['se3']
        for src in (['final', 'live'] if a.src == 'both' else [a.src]):
            if src == 'live' and not (R.OUT / s / 'live_full/pp.odom').exists(): continue
            r = run(s, src, a.cfg.split(), a.keep); out[f'{s}|{src}'] = r
            print(f"{s:9s} {src:5s} GNSS {f(gal)} | smoother {f(r['sm'].get('se3'))} georef {f(r['geo'].get('se3'))} auto {f(r['auto'].get('se3'))}  ({r['stdout'].splitlines()[-1] if r['stdout'] else ''})", flush=True)
    if a.json: Path(a.json).write_text(json.dumps(out, indent=1, default=float))
