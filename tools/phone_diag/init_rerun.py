# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Re-run stella_vio/tools/imu_init_eval.py (unmodified, C initialiser binary unmodified) on DERIVED inputs.
Variants (all change only the input files / the CLI options that imu_init_eval already exposes):
  base          shipped *_fit inputs
  smoothN       visual positions Gaussian-smoothed with sigma N seconds (per tracking segment), orientation untouched
  floorsX       --floors X,0.1,0.01  (visual position noise floor [m]; default 0.03)
  noacc         accelerometer noise density x0.2 (IMU_NOISE env of imu_init_eval)
  lensL         window lengths extended to 24, 32 s
Outputs go to runs/phone_diag/init_rerun/ (imu_init_eval.OUT is redirected, runs/stella_vio/imu is not touched).
usage: init_rerun.py <variant> [dataset ...]"""
import sys, os, json
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'stella_vio/tools')); sys.path.insert(0, str(Path(__file__).parent))
import imu_init_eval as E
import vis_scale as V

OUT = ROOT / 'runs/phone_diag/init_rerun' / sys.argv[1]      # one directory per variant (datasets() rewrites its derived inputs; parallel runs must not share)
OUT.mkdir(parents=True, exist_ok=True)
E.OUT = OUT
DEFAULT = ['euroc_MH_01', 'outdoor1_orb3_fit', 'outdoor1_stella_fit', 'outdoor2_orb3_fit', 'outdoor2_stella_fit', 'indoor1_orb3_fit', 'indoor1_stella_fit',
           'indoor2_orb3_fit', 'indoor2_stella_fit', 'advio15_orb3_fit', 'advio15_stella_fit', 'advio20_orb3_fit', 'advio20_stella_fit']


def main():
    var = sys.argv[1]; names = sys.argv[2:] or DEFAULT
    D = E.datasets()
    os.environ.pop('IMU_NOISE', None); os.environ.pop('IMU_INIT_EXTRA', None)
    if var.startswith('floors'):
        os.environ['IMU_INIT_EXTRA'] = f'--floors {float(var[6:])},0.1,0.01'
    if var == 'noacc':
        pass
    if var.startswith('lens'):
        E.LENS = [4, 8, 12, 16, 24, 32]
    summ = {}
    for ds in names:
        if ds not in D: print('missing', ds); continue
        d = dict(D[ds])
        if var.startswith('smooth'):
            sig = float(var[6:])
            tr = V.smooth_traj(E.read_tum(d['traj']), sig)
            p = OUT / f'traj_{ds}_{var}.txt'; np.savetxt(p, tr, fmt='%.6f'); d['traj'] = p
        if var.startswith('match'):      # matched Gaussian low-pass of BOTH sides: visual positions and the (body-frame) accelerometer, same sigma
            sig = float(var[5:])
            tr = V.smooth_traj(E.read_tum(d['traj']), sig)
            p = OUT / f'traj_{ds}_{var}.txt'; np.savetxt(p, tr, fmt='%.6f'); d['traj'] = p
            im = np.loadtxt(d['imu'], delimiter=',', comments='#')
            dt = np.median(np.diff(im[:, 0])) * 1e-9; kk = np.arange(-int(4 * sig / dt), int(4 * sig / dt) + 1) * dt
            gk = np.exp(-0.5 * (kk / sig) ** 2); gk /= gk.sum()
            for c in (4, 5, 6): im[:, c] = np.convolve(np.pad(im[:, c], len(kk) // 2, mode='edge'), gk, 'valid')
            q = OUT / f'imu_{ds}_{var}.csv'
            with open(q, 'w') as f:
                f.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
                for r in im: f.write('%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n' % (r[0], *r[1:]))
            d['imu'] = q
        if var == 'noacc':
            d['noise'] = (d['noise'][0], d['noise'][1] * 0.2)
        rows = E.evaluate(ds, d)
        S = E.summarize(ds, rows)
        (OUT / f'init_{var}_{ds}.json').write_text(json.dumps(dict(rows=rows, summary=S)))
        summ[ds] = S
        line = ' '.join(f"L{s['len']}: med ratio {s['med_ratio']:.2f} within20 {100*s['within20']:.0f}% acc {s['n_ok']}" for s in S if s['len'] in (4, 8, 12, 16, 24, 32))
        print(f'{var:10s} {ds:22s} {line}', flush=True)
    (OUT / 'summary.json').write_text(json.dumps(summ))


if __name__ == '__main__':
    main()
