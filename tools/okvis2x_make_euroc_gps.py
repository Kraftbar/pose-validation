#!/usr/bin/env python3
"""EuRoC ground truth -> OKVIS2-X cartesian GNSS file (gps0/data.csv).

    python3 tools/okvis2x_make_euroc_gps.py <seq_dir> [--out <dir>] [--rate 5] [--noise 0.01,0.02]
           [--r_sa 0.05,0.02,0.10] [--blackout 60,30] [--seed 1]

<seq_dir> = EuRoC ASL folder containing mav0/ (needs mav0/state_groundtruth_estimate0/data.csv; fetch with
VIO_DATA_ROOT=<root> python3 tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,imu0,state_groundtruth_estimate0).
Antenna fix = p_RS + R_RS * r_SA (EuRoC GT is the IMU/body pose), taken from the GT sample every >= 1/rate s, plus
Gaussian noise (std H on x,y, std V on z; fixed numpy seed). Blackout = 'start,length' seconds after the first
camera frame (cam0/data.csv; GT start if there is none): no fixes are written in it.
Output: <out>/gps0/data.csv  (default <seq_dir>/mav0/gps0): timestamp[ns], x, y, z, hErr1, hErr2, vErr
(hErr1 = hErr2 = H, vErr = V, as the reported 1-sigma).
The antenna is placed in the GT (Vicon / Leica) frame itself, so a correctly initialised run lives in that frame.
"""
import argparse
from pathlib import Path
import numpy as np


def quat_to_R(q):  # w x y z
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("--out", default=None)
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--noise", default="0.01,0.02")
    ap.add_argument("--r_sa", default="0.05,0.02,0.10")
    ap.add_argument("--blackout", default="60,30", help="start,length [s] after the first camera frame; '' = none")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    mav = Path(a.seq) / "mav0"
    gt = np.loadtxt(mav / "state_groundtruth_estimate0/data.csv", delimiter=",", comments="#")
    t_ns = np.loadtxt(mav / "state_groundtruth_estimate0/data.csv", delimiter=",", comments="#", usecols=0, dtype=np.int64)  # exact ns
    r_sa = np.array([float(v) for v in a.r_sa.split(",")])
    H, V = [float(v) for v in a.noise.split(",")]
    cam = mav / "cam0/data.csv"
    t0 = t_ns[0]
    if cam.exists():
        t0 = int(cam.read_text().split("\n")[1].split(",")[0])
    keep = [0]
    for i in range(1, len(gt)):
        if (t_ns[i] - t_ns[keep[-1]]) * 1e-9 >= 1.0 / a.rate - 1e-3:
            keep.append(i)
    rng = np.random.default_rng(a.seed)
    rows, dropped = [], 0
    b0, b1 = (None, None)
    if a.blackout:
        s, l = [float(v) for v in a.blackout.split(",")]
        b0, b1 = t0 + int(s * 1e9), t0 + int((s + l) * 1e9)
    for i in keep:
        pos = gt[i, 1:4] + quat_to_R(gt[i, 4:8]) @ r_sa + rng.standard_normal(3) * np.array([H, H, V])
        if b0 is not None and b0 <= t_ns[i] <= b1:
            dropped += 1
            continue
        rows.append((int(t_ns[i]), pos))
    out = Path(a.out) if a.out else mav
    (out / "gps0").mkdir(parents=True, exist_ok=True)
    with open(out / "gps0/data.csv", "w") as f:
        f.write("#timestamp [ns], x [m], y [m], z [m], hErr1 [m], hErr2 [m], vErr [m]\n")
        for t, p in rows:
            f.write(f"{t},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{H:.3f},{H:.3f},{V:.3f}\n")
    print(f"wrote {out / 'gps0/data.csv'}: {len(rows)} fixes, {dropped} in blackout "
          f"[{(b0 - t0) * 1e-9 if b0 else 0:.0f},{(b1 - t0) * 1e-9 if b1 else 0:.0f}] s, GT span {(t_ns[-1] - t_ns[0]) * 1e-9:.1f} s, "
          f"first fix {(rows[0][0] - t0) * 1e-9:.2f} s after first frame")


if __name__ == "__main__":
    main()
