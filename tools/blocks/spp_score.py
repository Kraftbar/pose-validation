#!/usr/bin/env python3
"""Score rtklib_spp output against the receiver RTK solution (pvt.txt). usage: spp_score.py out.txt [pvt.txt] [--json f]
Position error in ENU of the GT point, velocity error vs RTK NED velocity; sigma calibration from RTKLIB covariances."""
import sys, json
import numpy as np
A, F = 6378137.0, 1 / 298.257223563; E2 = F * (2 - F)
def lla2ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon); N = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.stack([(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon), (N * (1 - E2) + h) * np.sin(lat)], -1)
def R_enu(lat, lon):
    la, lo = np.radians(lat), np.radians(lon)
    return np.array([[-np.sin(lo), np.cos(lo), 0], [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)], [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])
def load_gt(path='/home/nybo/github/pose-validation/external/blocks/gnss_txt/pvt.txt', hacc=0.1):
    d = np.loadtxt(path, comments='#'); d = d[(d[:, 5] == 3) & (d[:, 8] <= hacc)]
    return d
def load_out(path):
    d = np.loadtxt(path); return d[d[:, 1] >= 0]
def evaluate(o, gt, tmax=0.06, use_dtr=True):
    """returns dict of arrays aligned epochs: t, e_enu(3), v_err(3), qr, qv, ns, stat"""
    tg = gt[:, 1]; res = []
    for r in o:
        t = r[0] - (r[9] if use_dtr else 0)
        i = np.argmin(np.abs(tg - t))
        if abs(tg[i] - t) > tmax: continue
        g = gt[i]; R = R_enu(g[2], g[3]); ge = lla2ecef(g[2], g[3], g[4])
        e = R @ (r[3:6] - ge); vgt = np.array([g[11], g[10], -g[12]])  # E N U
        ve = R @ r[6:9] - vgt
        res.append((r[0], *e, *ve, r[2], r[10], r[11], r[12], r[16], r[17], r[18], g[8]))
    return np.array(res)
def stats(x):
    e = x[:, 1:4]; v = x[:, 4:7]; h = np.hypot(e[:, 0], e[:, 1])
    return dict(n=len(x), h_rms=float(np.sqrt(np.mean(h ** 2))), h_med=float(np.median(h)), h_p95=float(np.percentile(h, 95)), u_rms=float(np.sqrt(np.mean(e[:, 2] ** 2))),
                p3d_rms=float(np.sqrt(np.mean(np.sum(e ** 2, 1)))), vh_rms=float(np.sqrt(np.mean(v[:, 0] ** 2 + v[:, 1] ** 2))), vu_rms=float(np.sqrt(np.mean(v[:, 2] ** 2))),
                v3d_rms=float(np.sqrt(np.mean(np.sum(v ** 2, 1)))), v3d_med=float(np.median(np.linalg.norm(v, axis=1))), nsat_mean=float(np.mean(x[:, 7])))
if __name__ == '__main__':
    o = load_out(sys.argv[1]); gt = load_gt(); x = evaluate(o, gt); s = stats(x)
    if '--row' in sys.argv:
        h = np.hypot(x[:, 1], x[:, 2]); vh = np.hypot(x[:, 4], x[:, 5]); label = sys.argv[sys.argv.index('--row') + 1]
        print('| %s | %d | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.1f |' % (label, s['n'], s['h_rms'], s['h_med'], s['h_p95'], s['u_rms'], np.median(vh), np.percentile(vh, 68), np.percentile(vh, 95), s['v3d_rms'], s['nsat_mean']))
    else: print(json.dumps(s, indent=1))
