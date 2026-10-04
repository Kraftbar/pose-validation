#!/usr/bin/env python3
"""Sigma calibration of RTKLIB SPP position / Doppler-velocity against RTK GT. usage: spp_calib.py out.txt"""
import sys; sys.path.insert(0, '/home/nybo/github/pose-validation/tools/blocks')
from spp_score import *
o = load_out(sys.argv[1]); gt = load_gt(); x = evaluate(o, gt)
# columns: t, e(3) ve(3) ns qr_xx qr_yy qr_zz qv_xx qv_yy qv_zz hacc
sp = np.sqrt(x[:, 8:11].sum(1)); sv = np.sqrt(x[:, 12:15].sum(1))      # 3-D sigma from ECEF variances (rotation invariant)
e3 = np.linalg.norm(x[:, 1:4], axis=1); v3 = np.linalg.norm(x[:, 4:7], axis=1)
eh = np.hypot(x[:, 1], x[:, 2]); vh = np.hypot(x[:, 4], x[:, 5])
print('position: reported 3-D sigma median %.2f m, actual 3-D err median %.2f rms %.2f; ratio of medians %.2f' % (np.median(sp), np.median(e3), np.sqrt(np.mean(e3 ** 2)), np.median(e3) / np.median(sp)))
print('velocity: reported 3-D sigma median %.3f m/s, actual 3-D err median %.3f rms %.3f; ratio of medians %.2f' % (np.median(sv), np.median(v3), np.sqrt(np.mean(v3 ** 2)), np.median(v3) / np.median(sv)))
# chi: err/sigma_per_axis (ENU via ECEF diag approx: use sigma/sqrt(3) per axis)
for name, err, sig in (('pos', e3, sp), ('vel', v3, sv)):
    r = err / sig
    print(name, 'err/sigma_3d pct 50/68/90/95/99:', np.percentile(r, [50, 68, 90, 95, 99]).round(2), ' (chi3 expected for ideal: 1.54/1.9/2.5/2.8/3.4 x)')
# empirical model: sigma scaled by k so that the 68th percentile of 3-D error equals k*sigma and 95th -> choose k68, k95
for name, err, sig in (('pos', e3, sp), ('vel', v3, sv)):
    r = err / sig; print(name, 'k68(of 3D)=%.2f  k95=%.2f  (use k such that sigma_axis = k*sigma_3d/sqrt(3))' % (np.percentile(r, 68), np.percentile(r, 95)))
# dependence on number of satellites
for lo, hi in ((0, 8), (8, 12), (12, 16), (16, 40)):
    m = (x[:, 7] >= lo) & (x[:, 7] < hi)
    if m.sum() > 20: print('ns [%d,%d): n=%d  H med %.2f p95 %.2f  Vh med %.2f p95 %.2f  rep sigma_pos med %.2f sigma_vel med %.3f' % (lo, hi, m.sum(), np.median(eh[m]), np.percentile(eh[m], 95), np.median(vh[m]), np.percentile(vh[m], 95), np.median(sp[m]), np.median(sv[m])))
# 1 Hz decimation statistics: autocorrelation time of the position error
for k in (10, 30, 100): print('pos err (E) autocorr lag %.1f s: %.2f' % (k / 10, np.corrcoef(x[:-k, 1], x[k:, 1])[0, 1]))
