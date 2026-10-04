#!/usr/bin/env python3
"""Frame pairs with real ORB matches, GT relative pose, depth for PnP and accelerometer gravity, from the TUM RGB-D sequences held in runs/orb_port/paper_bench/data.
usage: make_pairs.py out.txt [pairs_per_cell=60] ; format documented in tools/blocks/blocks_eval.cpp. Own code."""
import sys, numpy as np, cv2
from pathlib import Path
DATA = Path('/home/nybo/github/pose-validation/runs/orb_port/paper_bench/data')
SEQS = {'fr1_xyz': ('rgbd_dataset_freiburg1_xyz', (517.3, 516.5, 318.6, 255.3, [0.2624, -0.9531, -0.0054, 0.0026, 1.1633])),
        'fr1_desk': ('rgbd_dataset_freiburg1_desk', (517.3, 516.5, 318.6, 255.3, [0.2624, -0.9531, -0.0054, 0.0026, 1.1633])),
        'fr1_floor': ('rgbd_dataset_freiburg1_floor', (517.3, 516.5, 318.6, 255.3, [0.2624, -0.9531, -0.0054, 0.0026, 1.1633])),
        'fr2_xyz': ('rgbd_dataset_freiburg2_xyz', (520.9, 521.0, 325.1, 249.7, [0.2312, -0.7849, -0.0033, -0.0001, 0.9172]))}
GAPS = (4, 8, 16, 32)

def load(p, ncol):
    rows = []
    for l in open(p):
        if l.startswith('#') or not l.strip(): continue
        rows.append([float(x) for x in l.split()[:ncol]] if ncol else l.split())
    return rows

def quat_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def kabsch(a, b):   # R with b ~ R a
    H = a.T @ b; U, S, Vt = np.linalg.svd(H); d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, 1, d]) @ U.T

def main():
    out = open(sys.argv[1], 'w'); npc = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    orb = cv2.ORB_create(nfeatures=1500, scaleFactor=1.2, nlevels=8)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    stats = {}
    for name, (d, (fx, fy, cx, cy, dist)) in SEQS.items():
        D = DATA / d
        if not D.exists(): continue
        rgb = load(D / 'rgb.txt', 0); rgb = [(float(r[0]), r[1]) for r in rgb]
        dep = load(D / 'depth.txt', 0); dep = [(float(r[0]), r[1]) for r in dep]; dt = np.array([x[0] for x in dep])
        gt = np.array(load(D / 'groundtruth.txt', 8)); gtt = gt[:, 0]
        acc = np.array(load(D / 'accelerometer.txt', 4)); acct = acc[:, 0]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]); dco = np.array(dist, float)
        def pose_at(t):
            i = np.searchsorted(gtt, t); i = min(max(i, 1), len(gtt) - 1)
            a, b = gt[i - 1], gt[i]
            if abs(gtt[i - 1] - t) > 0.1 and abs(gtt[i] - t) > 0.1: return None
            w = (t - a[0]) / max(b[0] - a[0], 1e-9); w = min(max(w, 0), 1)
            p = (1 - w) * a[1:4] + w * b[1:4]
            q = (1 - w) * a[4:8] + w * b[4:8] * (1 if np.dot(a[4:8], b[4:8]) > 0 else -1)
            return quat_R(q), p     # camera pose in world: x_w = R x_c + p
        def acc_at(t, win=0.05):
            m = np.abs(acct - t) < win
            return acc[m, 1:4].mean(0) if m.any() else None
        # accelerometer -> camera frame alignment (fixed rotation, Kabsch against GT 'up' in the camera frame) over the whole sequence
        A, U = [], []
        for t, _ in rgb[::10]:
            ps = pose_at(t); a = acc_at(t)
            if ps is None or a is None: continue
            A.append(a / np.linalg.norm(a)); U.append(ps[0].T @ np.array([0, 0, 1.0]))
        A, U = np.array(A), np.array(U); Ral = kabsch(A, U)
        ang = np.degrees(np.arccos(np.clip(np.sum((A @ Ral.T) * U, 1), -1, 1)))
        stats[name] = (float(np.median(ang)), float(np.percentile(ang, 95)), len(A))
        for gap in GAPS:
            idxs = np.linspace(0, len(rgb) - gap - 1, npc).astype(int)
            for i in idxs:
                j = i + gap
                p1, p2 = pose_at(rgb[i][0]), pose_at(rgb[j][0]); a1, a2 = acc_at(rgb[i][0]), acc_at(rgb[j][0])
                if p1 is None or p2 is None or a1 is None or a2 is None: continue
                im1 = cv2.imread(str(D / rgb[i][1]), 0); im2 = cv2.imread(str(D / rgb[j][1]), 0)
                k1, d1 = orb.detectAndCompute(im1, None); k2, d2 = orb.detectAndCompute(im2, None)
                if d1 is None or d2 is None or len(k1) < 30 or len(k2) < 30: continue
                ms = bf.knnMatch(d1, d2, k=2); good = [m[0] for m in ms if len(m) == 2 and m[0].distance < 0.8 * m[1].distance]
                if len(good) < 15: continue
                u1 = cv2.undistortPoints(np.array([k1[m.queryIdx].pt for m in good], np.float64).reshape(-1, 1, 2), K, dco, P=K).reshape(-1, 2)
                u2 = cv2.undistortPoints(np.array([k2[m.trainIdx].pt for m in good], np.float64).reshape(-1, 1, 2), K, dco, P=K).reshape(-1, 2)
                dj = np.argmin(np.abs(dt - rgb[i][0])); dimg = cv2.imread(str(D / dep[dj][1]), cv2.IMREAD_UNCHANGED)
                R1, c1 = p1; R2, c2 = p2
                R21 = R2.T @ R1; t21 = R2.T @ (c1 - c2)
                g1 = Ral @ (a1 / np.linalg.norm(a1)); g2 = Ral @ (a2 / np.linalg.norm(a2))
                gt1 = R1.T @ np.array([0, 0, 1.0]); gt2 = R2.T @ np.array([0, 0, 1.0])
                out.write(f'PAIR {name} {i} {j} {gap}\nK {fx} {fy} {cx} {cy}\nGT {" ".join("%.9f" % v for v in R21.ravel())} {" ".join("%.9f" % v for v in t21)}\n')
                out.write(f'G {" ".join("%.9f" % v for v in np.r_[g1, g2, gt1, gt2])}\nN {len(good)}\n')
                for m, a, b in zip(good, u1, u2):
                    x, y = k1[m.queryIdx].pt; z = 0.0
                    xi, yi = int(round(x)), int(round(y))
                    if 0 <= yi < dimg.shape[0] and 0 <= xi < dimg.shape[1]: z = dimg[yi, xi] / 5000.0
                    out.write(f'{a[0]:.4f} {a[1]:.4f} {b[0]:.4f} {b[1]:.4f} {z:.4f} {k1[m.queryIdx].octave & 255} {k2[m.trainIdx].octave & 255}\n')
        print(name, 'accel->camera alignment residual (median/p95 deg, n):', stats[name], flush=True)
    out.close()
main()
