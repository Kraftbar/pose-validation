#!/usr/bin/env python3
"""Front-end probe (no estimator): for sampled consecutive/3-apart frame pairs, detect + describe + match (Hamming, ratio 0.8 + mutual check) + RANSAC fundamental
matrix, with OpenCV ORB (1000 kp). Reports mean keypoints, matches, RANSAC inliers, inlier ratio and the share of pairs
with < 30 inliers (a tracker has nothing to work with). usage: frontend_probe.py <seq: outdoor1|complex> [step=25] -> runs/gnss_compare/robustness/frontend_<seq>.json"""
import sys, json
from pathlib import Path
import numpy as np, cv2
ROB = Path('/home/nybo/github/pose-validation/external/gnss/rob'); OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare/robustness')
seq = sys.argv[1]; step = int(sys.argv[2]) if len(sys.argv) > 2 else 25; d = ROB / seq
fn = [l.split(',')[1].strip() for l in (d / 'cam0' / 'data.csv').read_text().splitlines()[1:]]
dets = {'ORB': lambda: cv2.ORB_create(1000, 1.2, 8, 31, 0, 2, cv2.ORB_HARRIS_SCORE, 31, 7)}   # cv2 5.0 here has no BRISK (moved to contrib): ORB only
bf = cv2.BFMatcher(cv2.NORM_HAMMING)
res = {}
for name, mk in dets.items():
    det = mk(); rows = {1: [], 3: []}
    for k in range(10, len(fn) - 4, step):
        feats = {}
        for j in (0, 1, 3):
            im = cv2.imread(str(d / 'cam0' / 'data' / fn[k + j]), 0)
            kp, ds = det.detectAndCompute(im, None)
            if name.startswith('BRISK') and ds is not None and len(kp) > 1000:   # keep the 1000 strongest like OKVIS's max_num_keypoints
                o = np.argsort([-p.response for p in kp])[:1000]; kp = [kp[i] for i in o]; ds = ds[o]
            feats[j] = (kp, ds)
        for gap in (1, 3):
            (k0, d0), (k1, d1) = feats[0], feats[gap]
            if d0 is None or d1 is None or len(k0) < 8 or len(k1) < 8: rows[gap].append((len(k0), 0, 0)); continue
            m = [x[0] for x in bf.knnMatch(d0, d1, k=2) if len(x) == 2 and x[0].distance < 0.8 * x[1].distance]
            inl = 0
            if len(m) >= 8:
                p0 = np.float32([k0[a.queryIdx].pt for a in m]); p1 = np.float32([k1[a.trainIdx].pt for a in m])
                F, mask = cv2.findFundamentalMat(p0, p1, cv2.FM_RANSAC, 1.5, 0.99)
                inl = int(mask.sum()) if mask is not None else 0
            rows[gap].append((len(k0), len(m), inl))
    res[name] = {f'gap{g}': dict(n_pairs=len(r), kp=float(np.mean([x[0] for x in r])), matches=float(np.mean([x[1] for x in r])), inliers=float(np.mean([x[2] for x in r])),
                                 inlier_ratio=float(np.sum([x[2] for x in r]) / max(np.sum([x[1] for x in r]), 1)), frac_lt30=float(np.mean([x[2] < 30 for x in r]))) for g, r in rows.items()}
    print(name, json.dumps(res[name]), flush=True)
json.dump(res, open(OUT / f'frontend_{seq}.json', 'w'), indent=1)
