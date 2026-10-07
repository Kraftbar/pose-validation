#!/usr/bin/env python3
"""ATE of an okvis2x_port reference run vs EuRoC GT (run with external/gnss/venv/bin/python).

    okvis2x_eval.py <csv> [<seq_dir=runs/okvis2x_port/data/MH_01_easy>] [--compare <other.csv>]
Reports benchmark.ate_rmse (Umeyama with scale, never reimplemented) on IMU positions matched to GT by timestamp (<=5 ms)
and the raw RMS without alignment (meaningful for GNSS runs that are in the GT frame). --compare: RMS position difference
between two trajectories (matched by timestamp, no alignment).
"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from benchmark import ate_rmse  # noqa: E402


def load(p):
    rows = []
    for l in Path(p).read_text().splitlines():
        if not l or l[0] == "#" or l.startswith("timestamp"):
            continue
        f = l.replace(",", " ").split()
        rows.append((int(f[0]), [float(x) for x in f[1:4]]))
    return np.array([r[0] for r in rows], dtype=np.int64), np.array([r[1] for r in rows])


def match(t, tg, tol=5_000_000):
    i = np.clip(np.searchsorted(tg, t), 1, len(tg) - 1)
    j = np.where(np.abs(tg[i - 1] - t) < np.abs(tg[i] - t), i - 1, i)
    return np.abs(tg[j] - t) <= tol, j


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    est_t, est = load(a[0])
    if "--compare" in sys.argv:
        ot, oth = load(sys.argv[sys.argv.index("--compare") + 1])
        ok, j = match(est_t, ot, 1000)
        d = np.linalg.norm(est[ok] - oth[j[ok]], axis=1)
        print(f"compare: {ok.sum()}/{len(est_t)} matched, rms {np.sqrt((d**2).mean()):.4f} m, max {d.max():.4f} m")
        return
    seq = Path(a[1] if len(a) > 1 else "runs/okvis2x_port/data/MH_01_easy")
    g = seq / "mav0/state_groundtruth_estimate0/data.csv"
    gt_t = np.loadtxt(g, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    gt_p = np.loadtxt(g, delimiter=",", comments="#", usecols=(1, 2, 3))
    ok, j = match(est_t, gt_t)
    if "--antenna" in sys.argv:  # global_final.csv holds the GNSS-antenna position p_GA_G: compare with GT p + R r_SA
        q = np.loadtxt(g, delimiter=",", comments="#", usecols=(4, 5, 6, 7))
        r_sa = np.array([0.05, 0.02, 0.10])
        w, x, y, z = q.T
        R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                      np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                      np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)
        gt_p = gt_p + R @ r_sa
    e, p = est[ok], gt_p[j[ok]]
    r = ate_rmse(e, p)
    raw = np.linalg.norm(e - p, axis=1)
    print(f"{a[0]}: matched {ok.sum()}/{len(est_t)}  ATE(Umeyama+scale) rmse {r['ate_rmse']:.4f} median {r['ate_median']:.4f} "
          f"max {r['ate_max']:.4f} scale {r['scale']:.4f}  | raw (no alignment) rmse {np.sqrt((raw**2).mean()):.4f} max {raw.max():.4f}")


if __name__ == "__main__":
    main()
