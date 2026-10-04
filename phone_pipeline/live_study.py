#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Section 16: paired live-pipeline experiments with start-frame perturbation (the mono front end is chaotic: one run is 40-100 % noise, so every
configuration is run from several start frames (sv_run --skip, a perturbation of the initialisation) and compared paired, start by start).

usage: live_study.py <seq> --cfg "tag|sv_set1 sv_set2|pp_opt1 pp_opt2" [--cfg ...] [--skips 0,10,20,40] [--workers 4] [--stream JPEG_DIR | --fx PGM_DIR] [--keep-fx]
  tag          output dir runs/phone_pipeline/<seq>/study16/<tag>_s<skip>/  (never the canonical live_full/)
  sv_set*      sv_run --set key=val (on top of the pipeline variant `full`); empty = the section-15 default
  pp_opt*      pp_live options verbatim, e.g. "--pp-set a.speed_scale_rw=0.1"
Prints per tag and start: causal ATE SE3 of the streamed AUTO output (scored from +12 s after the first output, +30 s with fixes), Sim3 ATE of the raw live map
(live.tum, one similarity: how consistent the live map is, independent of the fusion), then mean / median over the starts and the paired comparison with the first cfg.
Without --fx the gray PGMs are produced from the JPEG layout of the fetch scripts (--stream) into a scratch dir first (1.4 GB for the 1500-frame sequences).
"""
import sys, os, json, argparse, subprocess, shutil, time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'gnss_fusion/tools'))
import run as R, score as S  # noqa: E402
from gf_cases import phone_case, score_traj  # noqa: E402


def make_fixtures(seq, jdir, fx):
    import cv2
    fx.mkdir(parents=True, exist_ok=True)
    rows = [l.split(',') for l in (Path(jdir) / 'cam0/data.csv').read_text().splitlines()[1:] if l.strip()]
    for i, r in enumerate(rows):
        p = fx / f'{i:06d}.pgm'
        if p.exists(): continue
        img = cv2.imread(str(Path(jdir) / 'cam0/data' / r[1].strip()), cv2.IMREAD_GRAYSCALE)
        R.write_pgm(p, img)
    return len(rows)


def load(path, aligned=True):
    if not Path(path).exists() or Path(path).stat().st_size == 0: return None
    a = np.loadtxt(path, ndmin=2)
    if aligned and a.shape[1] > 8: a = a[(a[:, 8].astype(int) & 1) == 1]
    return a[np.argsort(a[:, 0], kind='stable')]


def run_one(seq, fx, tag, svs, pps, skip):
    c = R.cfg_of(seq); d = R.OUT / seq / 'study16' / f'{tag}_s{skip}'; d.mkdir(parents=True, exist_ok=True)
    (d / 'ext.txt').write_text(' '.join(repr(float(x)) for x in c['imu_ext']) + '\n')
    cmd = [str(R.PP_LIVE), str(R.VOCAB), R.rp(c['rgb_dir']), str(fx), str(d), '--no-snap', '--lean', '--size', c['size'], '--camera', c['camera'],
           '--imu', R.rp(c['imu']), '--imu-ext', str(d / 'ext.txt'), '--imu-toff', str(c['imu_toff']), '--imu-bg', ','.join(str(x) for x in c['imu_bg'])]
    for s in R.SV_VARIANTS['full'] + svs: cmd += ['--set', s]
    if skip: cmd += ['--skip', str(skip)]
    cmd += ['--live-out', str(d / 'live.tum'), '--servo-log', str(d / 'servo.log'), '--pp-out', str(d / 'pp'), '--pp-speed-out', str(d / 'pp.speed')]
    if c['fixes']: cmd += ['--pp-fix', R.rp(c['fixes'])]
    if c['gait']['mode'] == 'user': cmd += ['--pp-gait-c', repr(c['gait']['c'])]
    cmd += pps
    with open(d / 'log.txt', 'w') as f: subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    return d


def score_dir(seq, d, skip=0):
    c = R.cfg_of(seq); case = phone_case(seq, str((R.OUT / seq / 'sv_full' / 'trajectory_maps.tum').resolve()), cam_only=False, label=seq)
    ft = S.frame_times(seq); iw = 30.0 if c['fixes'] else R.INIT_WAIT_NOFIX
    r = {}
    for k in ('auto', 'sm'):
        a = load(d / f'pp.{k}')
        if a is None or len(a) < 20: r[k] = None; continue
        m = S.metrics(case, a, ft, tmin=ft[min(skip, len(ft) - 1)] + iw); r[k] = m.get('se3'); r[k + '_cov'] = m.get('coverage')
    L = load(d / 'live.tum', aligned=False)
    if L is not None and len(L) > 50:
        pose = np.c_[L[:, :8]]
        s = score_traj(case, pose[pose[:, 0] >= ft[min(skip, len(ft) - 1)] + iw]); r['map_sim3'] = s.get('ate_sim3'); r['maps'] = int(len(np.unique(L[:, 8])))
    log = (d / 'log.txt').read_text()
    for key in ('loops accepted',):
        import re
        m = re.search(r'(\d+) loops accepted', log); r['loops'] = int(m.group(1)) if m else None
    m = re.search(r'servo_steps=(\d+)', log); r['servo_steps'] = int(m.group(1)) if m else None
    return r


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('seq'); ap.add_argument('--cfg', action='append', required=True); ap.add_argument('--skips', default='0')
    ap.add_argument('--workers', type=int, default=4); ap.add_argument('--stream'); ap.add_argument('--fx'); ap.add_argument('--keep-fx', action='store_true'); ap.add_argument('--no-run', action='store_true')
    a = ap.parse_args()
    seq = a.seq; skips = [int(x) for x in a.skips.split(',')]
    fx = Path(a.fx) if a.fx else R.OUT / seq / '_fx_study' / seq
    if not a.fx and not a.no_run:
        n = make_fixtures(seq, a.stream, fx); print(f'{n} fixtures in {fx}', flush=True)
    cfgs = []
    for s in a.cfg:
        p = (s.split('|') + ['', ''])[:3]; cfgs.append((p[0], p[1].split(), p[2].split()))
    jobs = [(t, sv, pp, k) for (t, sv, pp) in cfgs for k in skips]
    res = {}
    try:
        def one(j):
            t, sv, pp, k = j
            d = R.OUT / seq / 'study16' / f'{t}_s{k}'
            if not a.no_run: run_one(seq, fx, t, sv, pp, k)
            return j, score_dir(seq, d, k)
        with ThreadPoolExecutor(a.workers) as ex:
            for j, r in ex.map(one, jobs): res[(j[0], j[3])] = r; print(j[0], 's%d' % j[3], json.dumps(r), flush=True)
    finally:
        if not a.fx and not a.keep_fx: shutil.rmtree(R.OUT / seq / '_fx_study', ignore_errors=True)
    out = R.OUT / seq / 'study16' / 'results.json'
    old = json.loads(out.read_text()) if out.exists() else {}
    for (t, k), r in res.items(): old[f'{t}|{k}'] = r
    out.write_text(json.dumps(old, indent=1))
    f = lambda x: '   -  ' if x is None else f'{x:6.2f}'
    base = cfgs[0][0]
    print(f'\n{seq}: causal AUTO ATE SE3 [m] / raw live map Sim3 ATE [m] per start {skips}')
    for t, _, _ in cfgs:
        v = [res[(t, k)].get('auto') for k in skips]; m = [res[(t, k)].get('map_sim3') for k in skips]
        vv = np.array([x for x in v if x is not None]); mm = np.array([x for x in m if x is not None])
        print(f'{t:14s} fused ' + ' '.join(f(x) for x in v) + f' | mean {vv.mean():.2f} median {np.median(vv):.2f} | map ' + ' '.join(f(x) for x in m) + f' | mean {mm.mean() if len(mm) else float("nan"):.2f}')
        if t != base:
            b = np.array([res[(base, k)].get('auto') or np.nan for k in skips]); w = int(np.nansum(np.array(v, float) < b - 0.02)); l = int(np.nansum(np.array(v, float) > b + 0.02))
            print(f'{"":14s} vs {base}: better {w}, worse {l}, equal {len(skips) - w - l} (|d| > 0.02 m); mean ratio {np.nanmean(np.array(v, float) / b):.2f}')


if __name__ == '__main__':
    main()
