#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Section 15 scoring of the live pipeline (pp_live output in runs/phone_pipeline/<seq>/live_<variant>/) against the final-trajectory pipeline of section 13/14.

  final   : sv_run FINAL trajectory (after later BA / loop corrections) -> run.py make_odom -> gf_run / gf_georef_run (runs/phone_pipeline/<seq>/scores.json, unchanged)
  replay  : the same fusion files replayed with the LIVE odometry (pp.odom, pp.speed): batch = whole-graph smoother over the live poses (isolates what the live odometry costs),
            causal = gf_run sliding window (must equal the streaming pp.sm)
  stream  : what pp_live emitted sample by sample: sm (smoother with fixes), geo (gait stream geo-referenced), auto (the switch)
Scores: ATE SE3 with the same code as score.py (resampled to the camera frames, causal from +init_wait).
usage: live_eval.py <seq> [--variant full] [--no-replay]  -> runs/phone_pipeline/<seq>/live_<variant>/scores.json
"""
import sys, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'gnss_fusion/tools'))
import run as R, score as S  # noqa: E402
from gf_cases import phone_case  # noqa: E402


def load(path, aligned=True):
    if not Path(path).exists() or Path(path).stat().st_size == 0: return None
    a = np.loadtxt(path, ndmin=2)
    if aligned and a.shape[1] > 8: a = a[(a[:, 8].astype(int) & 1) == 1]
    return a[np.argsort(a[:, 0], kind='stable')]


def evaluate(seq, variant='full', replay=True):
    c = R.cfg_of(seq); base = R.OUT / seq; d = base / f'live_{variant}'
    fv = variant if (base / f'sv_{variant}').exists() else 'full'      # variants without a final-trajectory run (section 16 `servo`) are compared with `full`
    ft = S.frame_times(seq)
    case = phone_case(seq, str((base / f'sv_{fv}' / 'trajectory_maps.tum').resolve()), cam_only=False, label=seq)
    fixes = R.rp(c['fixes']) if c['fixes'] else None
    iw = 30.0 if fixes else R.INIT_WAIT_NOFIX
    res = dict(seq=seq, variant=variant, n_frames=int(len(ft)))
    if fixes: res['gnss_alone'] = json.loads((base / 'scores.json').read_text())['gnss_alone']
    old = json.loads((base / 'scores.json').read_text())
    res['final'] = {m: old[f'{fv}|{m}'] for m in ('both', 'gait', 'georef') if f'{fv}|{m}' in old}
    if replay:
        for m in ('both', 'gait'):
            if m == 'both' and not fixes: continue
            R.run_fuse(seq, variant, m, live_dir=d)
        if fixes: R.run_georef(seq, variant, live_dir=d)
        res['replay'] = {}
        for m in ('both', 'gait', 'georef'):
            fd = d / f'fuse_{m}'
            if (fd / 'run.json').exists(): res['replay'][m] = S.fused(case, fd, ft, fixes)
    res['stream'] = {}
    for k in ('sm', 'geo', 'auto'):
        a = load(d / f'pp.{k}', aligned=(k != 'geo'))
        if a is None: continue
        r = S.metrics(case, a, ft, tmin=ft[0] + iw)
        if fixes: r['fix_rms'] = S.fix_rms(a, fixes, tmin=ft[0] + iw)
        r['first_aligned_s'] = float(a[0, 0] - ft[0]); r['coverage_all'] = float(S.resample(a, ft)[1].mean())
        res['stream'][k] = r
    if (d / 'pp.timing').exists(): res['timing'] = (d / 'pp.timing').read_text().strip().splitlines()
    return res


if __name__ == '__main__':
    seq = sys.argv[1]
    var = sys.argv[sys.argv.index('--variant') + 1] if '--variant' in sys.argv else 'full'
    r = evaluate(seq, var, replay='--no-replay' not in sys.argv)
    (R.OUT / seq / f'live_{var}' / 'scores.json').write_text(json.dumps(r, indent=1, default=float))
    f = lambda x: '-' if x is None else f'{x:.2f}'
    for m, v in r['final'].items(): print(f"final   {m:7s} batch {f(v['batch'].get('se3'))} causal {f(v['causal'].get('se3'))}")
    for m, v in r.get('replay', {}).items(): print(f"replay  {m:7s} batch {f(v['batch'].get('se3'))} causal {f(v['causal'].get('se3'))}")
    for k, v in r['stream'].items(): print(f"stream  {k:7s} causal {f(v.get('se3'))} cov {v.get('coverage', 0)*100:.0f}% first {v.get('first_aligned_s', 0):.0f}s")
