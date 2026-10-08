#!/usr/bin/env python3
"""Diagnosis tables for the o1 / m14 drone runs from the C GNSS event log (OKVIS_PORT_GNSS_LOG, OKVIS_PORT_FIX_DIAG=1 adds the CD lines).
    external/gnss/venv/bin/python tools/okvis2x_drone_diag.py <seq> <tag> [--bins 0,55,60,70,80,90,100]
Prints: GNSS state-machine transitions with time and distance travelled by then, the window size / yaw sigma timeline, RANSAC rejections, and the
error of the global trajectory (global_final.csv vs the RTK reference, no alignment) per time bin."""
import re, sys, math, json
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools/gnss_harness")); sys.path.insert(0, str(ROOT))
from gnss_eval import GT, antenna


def main():
    seq, tag = sys.argv[1:3]
    out = ROOT / "runs/okvis2x_port/drone/out" / seq / tag
    cam = np.loadtxt(ROOT / f"runs/okvis2x_port/drone/{seq}/mav0/cam0/data.csv", delimiter=",", usecols=0, comments="#") * 1e-9
    t0 = cam[0]
    c = np.loadtxt(out / "causal.csv", delimiter=",", skiprows=1, usecols=range(0, 4))
    ct, cp = c[:, 0] * 1e-9, c[:, 1:4]
    cpath = np.r_[0, np.cumsum(np.linalg.norm(np.diff(cp, axis=0), axis=1))]
    def travelled(t): return float(cpath[min(np.searchsorted(ct, t), len(ct) - 1)])
    now = t0; ev = []; nwin = []; ransac_rej = 0; ci_ok = []; cd = []
    for l in (out / "gnss.log").read_text().splitlines():
        m = re.match(r"AM 0 sid=(\d+) t=(\d+)\.(\d+)", l)
        if m: now = float(m.group(2) + "." + m.group(3)); continue
        if l.startswith("CI 0") and "REJECT" in l:
            ransac_rej += 1; n = int(re.search(r"npts=(\d+)", l).group(1)); nwin.append(n); continue
        m = re.match(r"CI 0 n=(\d+) npts=(\d+) yaw=([\d.e+-]+)", l)
        if m: ci_ok.append((now - t0, int(m.group(2)), float(m.group(3)))); continue
        m = re.match(r"CD 0 n=(\d+) npts=(\d+) spreadW=([\d.]+) spreadG=([\d.]+) pathW=([\d.]+) yawAll=([\d.e+-]+)", l)
        if m: cd.append((now - t0, int(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)))); continue
        if re.match(r"(ST|TG|CR|CS|RE|RI|PA|AL|AF)", l): ev.append((now - t0, l[:160]))
    print(f"== {seq}/{tag}: {len(ev)} state-machine events")
    for t, l in ev: print(f"  t={t:6.1f}s travelled={travelled(now if False else t0 + t):6.1f} m  {l}")
    print(f"RANSAC rejections (rt graph): {ransac_rej}; max npts among them {max(nwin) if nwin else 0}; checks that reached the yaw gate: {len(ci_ok)}")
    if ci_ok:
        a = np.array(ci_ok); print(f"  gate yaw sigma (deg): first t={a[0,0]:.1f}s npts={int(a[0,1])} yaw={a[0,2]:.3g}; min {a[:,2].min():.3f} at t={a[a[:,2].argmin(),0]:.1f}s npts={int(a[a[:,2].argmin(),1])}")
    if cd:
        a = np.array(cd); print("  window diagnostic (CD): t, npts, rms spread of VIO world pts (m), rms spread GNSS (m), path (m), yaw sigma of all-point Umeyama (deg), every 10 s")
        for T in range(0, 101, 10):
            i = np.searchsorted(a[:, 0], T)
            if i < len(a): print(f"   t={a[i,0]:6.1f} npts={int(a[i,1]):4d} spreadW={a[i,2]:8.3f} spreadG={a[i,3]:8.3f} pathW={a[i,4]:8.2f} yawAll={a[i,5]:10.2f}")
        s = a[a[:, 0] < 54]
        if len(s): print(f"  stationary phase (t<54 s): max npts {int(s[:,1].max())}, max spreadW {s[:,2].max():.4f} m, min yawAll {s[:,5].min():.1f} deg ({len(s)} checks)")
        for thr in (5, 1):
            k = np.where(a[:, 5] < thr)[0]
            print(f"  first check with window yawAll < {thr} deg: " + (f"t={a[k[0],0]:.1f}s npts={int(a[k[0],1])} spreadW={a[k[0],2]:.1f}" if len(k) else "never"))
    # global error per bin
    sd = ROOT / "external/drone" / f"ins_{seq}"; gt = GT.from_tum(sd / "gt_enu.tum")
    r_SA = json.load(open(ROOT / "external/drone" / f"ins_{seq}_cfg" / "lever.json"))["r_RTK"]
    rows = []
    for l in (out / "global_final.csv").read_text().splitlines():
        q = [x.strip() for x in l.split(",")]
        if q[0].isdigit(): rows.append([int(q[0]) * 1e-9] + [float(v) for v in q[1:8]])
    g = np.array(rows); gp, ok = gt.at(g[:, 0]); e = np.linalg.norm(g[:, 1:4] - gp, axis=1)
    bins = [0, 55, 60, 70, 80, 90, 1000] if seq == "o1" else [0, 30, 60, 90, 120, 150, 1000]
    print("global (geo) error per time bin, no alignment (rms m): " + "  ".join(
        f"[{bins[i]},{min(bins[i+1],int(g[-1,0]-t0)+1)}) {math.sqrt((e[ok & (g[:,0]-t0>=bins[i]) & (g[:,0]-t0<bins[i+1])]**2).mean()):.2f}"
        for i in range(len(bins) - 1) if (ok & (g[:, 0] - t0 >= bins[i]) & (g[:, 0] - t0 < bins[i + 1])).any()))


main()
