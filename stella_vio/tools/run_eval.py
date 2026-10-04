#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""stella_vio evaluation: build, run on the 7 study sequences (parallel), score, print/append a table.
usage: run_eval.py <tag> [--bin PATH] [--seqs a,b,..] [--workers N] [--no-run] [--extra "sv_run args"]
 TUM (fr1_xyz fr1_desk fr1_floor fr2_xyz fr3_long_office): ATE = Sim3 RMSE over tracked frames via tum_eval.ate_over (benchmark.ate_rmse).
 outdoor1 (Mobile-GVIO, 1280x720) / complex (GVINS, 752x480): Sim3 ATE vs GT (gnss_eval.GT, benchmark.umeyama_alignment).
 If the run wrote trajectory_maps.tum (9th column = map id), every map is aligned on its own and the pooled RMS is reported, plus
 the ATE of the largest map. Outputs: runs/stella_vio/<tag>/<seq>/ ; summary runs/stella_vio/<tag>/summary.json"""
import sys, os, re, json, subprocess, time, argparse
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools')); sys.path.insert(0, str(ROOT / 'tools/gnss_harness'))
from benchmark import umeyama_alignment, apply_alignment  # noqa: E402
from gnss_eval import GT  # noqa: E402
import tum_eval  # noqa: E402

G = ROOT / 'external/gnss'; ROB = G / 'rob'; VOCAB = ROOT / 'external/candidates/orb_vocab.fbow'
FIX = ROOT / 'runs/stella_vio/fixtures'
CAM = {'fr1': '517.306408,516.469215,318.643040,255.313989,0.262383,-0.953104,-0.005358,0.002628,1.163314',
       'fr2': '520.908620,521.007327,325.141442,249.701764,0.231222,-0.784899,-0.003257,-0.000105,0.917205',
       'fr3': '535.4,539.2,320.1,247.6,0,0,0,0,0'}
TUM = {'fr1_xyz': 'fr1', 'fr1_desk': 'fr1', 'fr1_floor': 'fr1', 'fr2_xyz': 'fr2', 'fr3_long_office': 'fr3'}
SEQS = {  # name -> (seq dir for rgb/depth.txt, fixtures, camera)
    **{k: (tum_eval.DATA_ROOT / tum_eval.SEQ_DATA_DIRS[k], FIX / k, CAM[v]) for k, v in TUM.items()},
    'outdoor1': (ROB / 'outdoor1/tum', FIX / 'outdoor1', (ROOT / 'tools/gnss_harness/robust_cfg/outdoor1/port_camera.txt').read_text().strip()),
    'complex': (ROB / 'complex/tum', FIX / 'complex', (ROOT / 'tools/gnss_harness/robust_cfg/complex/port_camera.txt').read_text().strip()),
}
# EuRoC mono+IMU (opt-in sequences, not in the default list): Sim3 per map vs the cam0 GT, source layout runs/stella_vio/src/euroc_<seq>/rgb.txt (+depth.txt copy)
EUROC = {'mh01': 'MH_01_easy', 'v102': 'V1_02_medium'}
EUCAM = '458.654,457.296,367.215,248.375,-0.28340811,0.07395907,0.00019359,0.0000176187114,0'
for _k, _v in EUROC.items(): SEQS[_k] = (ROOT / f'runs/stella_vio/src/euroc_{_v}', FIX / _k, EUCAM)
IMU_DIR = ROOT / 'runs/stella_vio/imu'
# --imu: per-sequence IMU arguments (IMU csv, extrinsic from ext_fit / calibration, time offset IMU = cam + toff [s] (0 = best of the gyro_pred offset scans), gyro bias [rad/s] from gyro_pred.md fits)
IMU_CFG = {
    'outdoor1': (G / 'rob/outdoor1/imu0/data.csv', IMU_DIR / 'ext_outdoor1_stella_fit.txt', 0.0, '0.00017,0.00934,0.00074'),
    'complex': (G / 'rob/complex/imu0/data.csv', IMU_DIR / 'ext_complex_stella_up.txt', 0.0, '0.00125,-0.01484,-0.01022'),
    'mh01': (ROOT / 'external/vio/data/MH_01_easy/mav0/imu0/data.csv', IMU_DIR / 'ext_euroc_MH_01.txt', 0.0, '-0.00186,0.02076,0.07884'),
    'v102': (ROOT / 'external/vio/data/V1_02_medium/mav0/imu0/data.csv', IMU_DIR / 'ext_euroc_V1_02.txt', 0.0, '-0.00198,0.02063,0.07610'),
}
GNSS = {'outdoor1': lambda: GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887), 'complex': lambda: GT.from_pvt(G / 'seq/complex')}
def _euroc_gt(name):
    gt = np.loadtxt(ROOT / f'runs/vio_compare/gt/{EUROC[name]}_cam0.tum', ndmin=2)
    class _G:
        def at(self, t):
            j = np.clip(np.searchsorted(gt[:, 0], t), 1, len(gt) - 1); j = np.where(np.abs(gt[j - 1, 0] - t) < np.abs(gt[j, 0] - t), j - 1, j)
            ok = np.abs(gt[j, 0] - t) <= 0.005; return gt[j, 1:4], ok
    return _G()
for _k in EUROC: GNSS[_k] = (lambda k=_k: _euroc_gt(k))


MAPCOL = int(os.environ.get('SV_MAPCOL', '8'))     # 8: map id (merged), 10: segment id (a bridged R-frame part is scored as its own map, like a re-initialized map)
DROP_R = os.environ.get('SV_DROP_R', '0') == '1'   # 1: leave out the R-frames (position extrapolated, not observed) when scoring


def read_maps(d, name='trajectory_maps.tum'):
    p = d / name
    if not p.exists(): p = d / 'trajectory.tum'
    a = np.loadtxt(p, ndmin=2) if p.exists() and p.stat().st_size else np.zeros((0, 8))
    if a.shape[1] < 9: a = np.c_[a, np.zeros(len(a))]
    if a.shape[1] >= 11:
        if DROP_R: a = a[a[:, 9] == 0]
        a = np.c_[a[:, :8], a[:, MAPCOL]] if MAPCOL in (8, 10) else a[:, :9]
    return a  # t x y z qx qy qz qw map


def ate_sim3(est, gt):
    if len(est) < 5: return None
    R, t, s = umeyama_alignment(est, gt, with_scale=True)
    return float(np.sqrt((np.linalg.norm(apply_alignment(est, R, t, s) - gt, axis=1) ** 2).mean())), float(s)


def match(seq, a):
    """returns (est_pos, gt_pos, map ids, times) for the poses with a reference"""
    if seq in TUM:
        gts, gp = tum_eval.read_groundtruth(seq)
        E, P, M, T = [], [], [], []
        for r in a:
            j = int(np.argmin(np.abs(gts - r[0])))
            if abs(gts[j] - r[0]) <= tum_eval.ASSOC_MAX_DIFF: E.append(r[1:4]); P.append(gp[j]); M.append(r[8]); T.append(r[0])
        return np.array(E), np.array(P), np.array(M), np.array(T)
    gt = GNSS[seq](); gp, ok = gt.at(a[:, 0])
    return a[ok, 1:4], gp[ok], a[ok, 8], a[ok, 0]


def score(seq, d, nframes, dur, traj='trajectory_maps.tum'):
    a = read_maps(d, traj); r = dict(n_pose=int(len(a)), coverage=len(a) / nframes)
    log = (d / 'log.txt').read_text(errors='ignore') if (d / 'log.txt').exists() else ''
    m = re.search(r'(\d+) lost frames, (\d+) resets', log)
    if m: r['lost_frames'], r['resets'] = int(m.group(1)), int(m.group(2))
    m = re.search(r'maps=(\d+) reinits=(\d+)', log)
    if m: r['maps'], r['reinits'] = int(m.group(1)), int(m.group(2))
    if len(a) < 5: return r
    r['span'] = float((np.diff(a[:, 0])[np.diff(a[:, 0]) <= 1.0]).sum() / dur)
    E, P, M, T = match(seq, a)
    if len(E) < 5: return r
    err2, parts, tt, tilt = [], {}, [], {}
    for mid in np.unique(M):
        k = M == mid
        x = ate_sim3(E[k], P[k])
        if x is None: continue
        R, t, s = umeyama_alignment(E[k], P[k], with_scale=True)
        tt.append(T[k]); tilt[int(mid)] = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))); err2.append(np.linalg.norm(apply_alignment(E[k], R, t, s) - P[k], axis=1) ** 2); parts[int(mid)] = (int(k.sum()), x[0], x[1])
    if not err2: return r
    e2 = np.concatenate(err2); r['ate'] = float(np.sqrt(e2.mean()))
    big = max(parts, key=lambda q: parts[q][0]); r['ate_main_map'] = parts[big][1]; r['n_main_map'] = parts[big][0]; r['n_maps_scored'] = len(parts); r['tilt_deg'] = {m: round(v, 1) for m, v in tilt.items()}
    if seq in GNSS:  # local accuracy: Sim3 per 60 s window (and per map), pooled RMS and the median window -> independent of long-term scale drift
        w2, wm = [], []
        for mid in np.unique(M):
            for b in range(int((T[M == mid].max() - T[M == mid].min()) // 60) + 1):
                k = (M == mid) & (T - T[M == mid].min() >= 60 * b) & (T - T[M == mid].min() < 60 * (b + 1))
                if k.sum() < 40: continue
                R_, t_, s_ = umeyama_alignment(E[k], P[k], with_scale=True)
                e = np.linalg.norm(apply_alignment(E[k], R_, t_, s_) - P[k], axis=1); w2.append(e ** 2); wm.append(float(np.sqrt((e ** 2).mean())))
        if w2: r['ate_win60'] = float(np.sqrt(np.concatenate(w2).mean())); r['ate_win60_median'] = float(np.median(wm)); r['n_win60'] = len(wm)
    t0 = T.min(); tt = np.concatenate(tt); bins = ((tt - t0) // 30).astype(int)
    r['err30s'] = [round(float(np.sqrt(e2[bins == b].mean())), 2) if (bins == b).sum() > 3 else None for b in range(bins.max() + 1)] if seq in GNSS else None
    for w in (60,):  # accuracy of the start of the run (initialisation quality)
        k = (T - t0 <= w) & (M == M[0])
        x = ate_sim3(E[k], P[k]) if k.sum() >= 20 else None
        if x: r[f'ate_first{w}s'] = x[0]
    return r


def run_one(binp, name, tag, extra, out_root, imu=False):
    d, fx, cam = SEQS[name]; out = out_root / name; out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    cmd = [str(binp), str(VOCAB), str(d), str(fx), str(out), '--no-snap', '--lean', '--camera', cam] + extra
    if imu and name in IMU_CFG:
        f, e, to, bg = IMU_CFG[name]; cmd += ['--imu', str(f), '--imu-ext', str(e), '--imu-toff', str(to), '--imu-bg', bg]
    with open(out / 'log.txt', 'w') as f: rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
    return name, rc, time.time() - t0


def nframes(name):
    if name in TUM: return len(tum_eval.read_tum_list(SEQS[name][0] / 'rgb.txt'))  # README convention: coverage over rgb.txt frames
    return len(list((SEQS[name][1]).glob('*.pgm')))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--bin', default=str(ROOT / 'stella_vio/sv_run'))
    ap.add_argument('--seqs', default=','.join(SEQS)); ap.add_argument('--workers', type=int, default=7); ap.add_argument('--no-run', action='store_true')
    ap.add_argument('--extra', default=''); ap.add_argument('--imu', action='store_true'); ap.add_argument('--traj', default='trajectory_maps.tum')
    a = ap.parse_args(); out_root = ROOT / 'runs/stella_vio' / a.tag; seqs = a.seqs.split(',')
    wall = {}
    if not a.no_run:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(a.workers) as ex:
            for n, rc, w in ex.map(lambda n: run_one(a.bin, n, a.tag, a.extra.split(), out_root, a.imu), seqs):
                wall[n] = w; print(f'{n}: rc={rc} {w:.0f}s', flush=True)
    rows = {}
    for n in seqs:
        dur = {'outdoor1': 393.99, 'complex': 435.65, 'mh01': 184.05, 'v102': 111.0}.get(n)
        if dur is None:
            ts = [float(x.split()[0]) for x in open(SEQS[n][0] / 'rgb.txt') if not x.startswith('#')]; dur = ts[-1] - ts[0]
        r = score(n, out_root / n, nframes(n), dur, a.traj); r['rtf'] = wall[n] / dur if n in wall else None; rows[n] = r
    json.dump(rows, open(out_root / 'summary.json', 'w'), indent=1)
    f = lambda v, k=3: '-' if v is None else f'{v:.{k}f}'
    print(f'\n| seq | ATE m | per-60s-window ATE (pooled / median) | first-60s ATE | coverage | lost frames | resets | maps (reinit) | RTF |\n|---|---|---|---|---|---|---|---|---|')
    for n, r in rows.items():
        if r.get('err30s'): print(f"{n} err by 30 s bin: {r['err30s']}")
        print(f"| {n} | {f(r.get('ate'))} | {f(r.get('ate_win60'))} / {f(r.get('ate_win60_median'))} | {f(r.get('ate_first60s'))} | {r['coverage']*100:.0f}% ({r['n_pose']}) | {r.get('lost_frames','-')} | {r.get('resets','-')} | {r.get('maps','-')} ({r.get('reinits','-')}) | {f(r.get('rtf'),2)} |" + (f" tilt {r['tilt_deg']}" if r.get('tilt_deg') and n in GNSS else ''))


if __name__ == '__main__': main()
