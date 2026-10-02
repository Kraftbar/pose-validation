#!/usr/bin/env python3
"""Apply tools/gnss_loose_fusion.py (4-DoF + scale smoother) to a phone run from external/gnss/rob/out and write a scored-able run dir.
usage: robust_fusion.py <src_run_dir_name> <out_name> [--cam-only] [--sigma S] [--causal W]
  src: external/gnss/rob/out/<src>/traj.txt (IMU/body pose for VIO, camera pose for camera-only)
  --cam-only: the trajectory has no gravity direction (arbitrary camera-frame start): rotate it so that the mean accelerometer specific force
              (R_wb * a_b over the whole run) points along +z, i.e. the IMU is used ONLY for the gravity direction (a camera-only system would need
              a phone's gravity sensor for this, which every phone has); then the smoother estimates yaw + scale + position from the 1 Hz fixes.
  --sigma S: force the GNSS sigma (m) instead of the fixes' reported 14 m (post-hoc sensitivity, as in the main note).
Outputs external/gnss/rob/out/<out_name>/{traj.txt,run.json} (scored by robust_score.py as seq outdoor1)."""
import sys, json, time, argparse
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(Path(__file__).parent.parent))
from gnss_eval import read_traj, quat_to_R  # noqa: E402
import gnss_loose_fusion as lf  # noqa: E402

ROB = Path('/home/nybo/github/pose-validation/external/gnss/rob')
T_BC = np.array([[0.01916709, -0.99980408, -0.00494212], [-0.99955275, -0.01904832, -0.02305337], [0.02295471, 0.00538177, -0.99972202]])  # camera -> IMU rotation (phone)
ap = argparse.ArgumentParser(); ap.add_argument('src'); ap.add_argument('out'); ap.add_argument('--cam-only', action='store_true')
ap.add_argument('--fit', action='store_true', help='no smoother: one global Sim3 (benchmark.umeyama_alignment) of the poses onto the GNSS fixes')
ap.add_argument('--sigma', type=float, default=0.0); ap.add_argument('--causal', type=float, default=0.0); a = ap.parse_args()
tr = read_traj(ROB / 'out' / a.src / 'traj.txt'); tr = tr[np.argsort(tr[:, 0])]
gps = lf.read_gps(ROB / 'outdoor1' / 'gps0' / 'data.csv')
if a.sigma: gps[:, 4] = a.sigma; gps[:, 5] = 2 * a.sigma
if a.cam_only:
    imu = np.loadtxt(ROB / 'outdoor1' / 'imu0' / 'data.csv', delimiter=',', comments='#'); ti = imu[:, 0] * 1e-9
    k = np.clip(np.searchsorted(tr[:, 0], ti), 0, len(tr) - 1); ok = np.abs(tr[k, 0] - ti) < 0.1
    R = quat_to_R(tr[k[ok], 4:8] / np.linalg.norm(tr[k[ok], 4:8], axis=1, keepdims=True))            # R_w<-c
    up = np.einsum('nij,jk,nk->ni', R, T_BC.T, imu[ok, 4:7]).mean(0)                                  # R_wc R_cb a_b
    up /= np.linalg.norm(up); v = np.cross(up, [0, 0, 1.0]); c = float(up @ [0, 0, 1.0]); s = np.linalg.norm(v)
    Ra = np.eye(3) if s < 1e-9 else np.eye(3) + np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) + (np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) @ np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])) * (1 - c) / s ** 2
    tr[:, 1:4] = tr[:, 1:4] @ Ra.T
    tr[:, 4:8] = np.array([lf.R_quat(Ra @ lf.quat_R(q)) for q in tr[:, 4:8]])
    print('gravity (up) in the SLAM frame', up.round(3))
def fill_gaps(tr, maxgap=0.5):
    """the smoother needs a pose at every node: bridge pose gaps (> maxgap s) with constant-velocity positions and held orientation, pseudo-poses every 0.5 s"""
    rows = [tr[0]]
    for i in range(1, len(tr)):
        if tr[i, 0] - tr[i - 1, 0] > maxgap:
            for tt in np.arange(tr[i - 1, 0] + 0.5, tr[i, 0] - 0.25, 0.5):
                w = (tt - tr[i - 1, 0]) / (tr[i, 0] - tr[i - 1, 0]); r = tr[i - 1].copy(); r[0] = tt; r[1:4] = (1 - w) * tr[i - 1, 1:4] + w * tr[i, 1:4]; rows.append(r)
        rows.append(tr[i])
    return np.array(rows)


n_before = len(tr); tr = fill_gaps(tr)
if a.cam_only and not a.fit:   # camera-only has no metric scale: normalise with one global Sim3 scale (Umeyama) against the fixes so the smoother's scale state starts near 1
    from benchmark import umeyama_alignment
    k = np.clip(np.searchsorted(tr[:, 0], gps[:, 0]), 0, len(tr) - 1); ok = np.abs(tr[k, 0] - gps[:, 0]) < 1.0
    sf0 = umeyama_alignment(tr[k[ok], 1:4], gps[ok, 1:4], with_scale=True)[2]; tr[:, 1:4] *= sf0; print('camera-only: pre-normalised by global Sim3 scale %.3f' % sf0)
t0 = time.time()
if a.fit:
    from benchmark import umeyama_alignment, apply_alignment
    k = np.clip(np.searchsorted(tr[:, 0], gps[:, 0]), 0, len(tr) - 1); ok = np.abs(tr[k, 0] - gps[:, 0]) < 1.0
    Rf, tf, sf = umeyama_alignment(tr[k[ok], 1:4], gps[ok, 1:4], with_scale=True)
    out = tr.copy(); out[:, 1:4] = apply_alignment(tr[:, 1:4], Rf, tf, sf); out[:, 4:8] = np.array([lf.R_quat(Rf @ lf.quat_R(q)) for q in tr[:, 4:8]])
    X = np.full((2, 5), sf); tn = []
else:
    out, X, tn = lf.fuse(tr, gps, [0, 0, 0], 1.0, a.causal)
wall = time.time() - t0
d = ROB / 'out' / a.out; d.mkdir(parents=True, exist_ok=True)
np.savetxt(d / 'traj.txt', out, fmt=['%.9f'] + ['%.6f'] * 3 + ['%.8f'] * 4)
json.dump(dict(wall_s=wall, exit_code=0, system='fuse', seq='outdoor1', src=a.src, cam_only=a.cam_only, sigma=a.sigma, causal=a.causal, fit=a.fit, scale_median=float(np.median(X[:, 4]))), open(d / 'run.json', 'w'))
print('fused', a.out, len(out), 'poses, median scale %.3f' % np.median(X[:, 4]))
