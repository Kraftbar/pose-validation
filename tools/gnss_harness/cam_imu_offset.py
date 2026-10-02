#!/usr/bin/env python3
"""Camera-vs-IMU time offset from a visual (camera-only) trajectory: cross-correlate the visual angular-speed norm (from consecutive-pose rotation angle,
frame-invariant, scale-free so any mono system works) with the gyro norm. usage: cam_imu_offset.py <traj.txt (t x y z qx qy qz qw)> <imu0/data.csv> [max_gap_s=0.3]
prints t_imu = t_cam + dt (s) at the correlation peak, peak corr, and the correlation at dt=0 / the 2nd-best peak outside +-0.1 s."""
import sys
import numpy as np
tr = np.loadtxt(sys.argv[1]); imu = np.loadtxt(sys.argv[2], delimiter=',', comments='#'); gap = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
if tr[0, 0] > 1e12: tr[:, 0] *= 1e-9
ti = imu[:, 0] * 1e-9; wi = np.linalg.norm(imu[:, 1:4], axis=1)
q = tr[:, 4:8] / np.linalg.norm(tr[:, 4:8], axis=1, keepdims=True)
dt = np.diff(tr[:, 0]); ok = (dt > 0) & (dt < gap)
ang = 2 * np.arccos(np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1))
tv = 0.5 * (tr[1:, 0] + tr[:-1, 0])[ok]; wv = (ang[ok] / dt[ok])
def corr(off):
    sel = (tv + off > ti[0] + 1) & (tv + off < ti[-1] - 1)
    if sel.sum() < 200: return -9
    a = np.interp(tv[sel] + off, ti, np.convolve(wi, np.ones(9) / 9, 'same')); b = np.convolve(wv[sel], np.ones(3) / 3, 'same')
    return np.corrcoef(a, b)[0, 1]
offs = np.arange(-0.5, 0.5, 0.005); c = np.array([corr(o) for o in offs]); k = c.argmax()
far = c[np.abs(offs - offs[k]) > 0.1]
print(f'dt = {offs[k]:+.3f} s (t_imu = t_cam + dt), corr peak {c[k]:.3f}, corr at 0 {corr(0.0):.3f}, best outside +-0.1 s {far.max():.3f}, n={len(tv)}')
if '--curve' in sys.argv:
    print(' '.join(f'{o:+.2f}:{corr(o):.3f}' for o in np.arange(-0.3, 0.301, 0.03)))
