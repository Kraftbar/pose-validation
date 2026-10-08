#!/usr/bin/env python3
"""Generic GNSS validation driver for the OKVIS2-X C port (okvis_port/c, OKVIS_PORT_OKVIS2X=1), unmodified vs opt-in fixes vs loose smoother.

Layout: runs/okvis2x_port/val/<name>/{cam0,imu0}  (EuRoC layout without mav0, as written by gvins_bag_to_euroc.py / insane_to_euroc.py minus mav0)
        runs/okvis2x_port/val/<name>/gps_<variant>/gps0/data.csv    GNSS variants
        runs/okvis2x_port/val/<name>/val.json   {"cfg": yaml with gps_parameters, "r_score": [antenna lever for scoring], "gt": "pvt:<file>" | "tum:<file>"}
    run   <name> <variant> <tag> [--fix A,B] [--nogps] [--log]   -> val/out/<name>/<variant>_<tag>/   (deterministic single-threaded config)
    score <name> <variant> <tag>                                  -> one json line: SE3 / Sim3 / geo RMS / median / max
    loose <name> <variant>                                        -> smoother on val/out/<name>/<variant>_nogps/final.csv with the variant's fixes, scored as geo
python: external/gnss/venv/bin/python (score, loose)."""
import json, os, subprocess, sys, time, hashlib
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools")); sys.path.insert(0, str(ROOT / "tools/gnss_harness")); sys.path.insert(0, str(ROOT))
V = ROOT / "runs/okvis2x_port/val"
VOC = ROOT / "runs/okvis_port/vocabulary/small_voc.bin"


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:12]


def cfgd(name):
    return json.load(open(V / name / "val.json"))


def run(name, variant, tag, fixes, nogps, log):
    import okvis2x_check_gnss as cg
    exe = cg.build()
    j = cfgd(name)
    out = V / "out" / name / f"{variant}_{tag}"
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OKVIS_PORT_OKVIS2X="1")
    for f in fixes:
        env[f"OKVIS_PORT_FIX_{f}"] = "1"
    if log:
        env["OKVIS_PORT_GNSS_LOG"] = str(out / "gnss.log")
    sd = V / name / f"_sd_{variant}_{tag}"
    (sd / "mav0").mkdir(parents=True, exist_ok=True)
    for n, tgt in (("cam0", V / name / "cam0"), ("imu0", V / name / "imu0"), ("gps0", V / name / f"gps_{variant}/gps0")):
        l = sd / "mav0" / n
        if l.is_symlink():
            l.unlink()
        l.symlink_to(tgt.resolve())
    cfg = j["cfg_nogps"] if nogps else j["cfg"]
    t0 = time.time()
    r = subprocess.run([str(exe), str(ROOT / cfg), str(sd), str(VOC), str(out)], env=env, capture_output=True, text=True)
    (out / "stderr.txt").write_text(r.stderr)
    print(f"C {name}/{variant}_{tag} exit {r.returncode} wall {time.time() - t0:.0f}s causal {sha(out / 'causal.csv')} global "
          f"{sha(out / 'global_final.csv') if (out / 'global_final.csv').exists() else '-'}", flush=True)


def load(p):
    import numpy as np
    rows = []
    for l in Path(p).read_text().splitlines():
        q = [t.strip() for t in l.split(",")]
        if q[0].isdigit():
            rows.append([int(q[0]) * 1e-9] + [float(v) for v in q[1:8]])
    a = np.array(rows)
    return a[np.argsort(a[:, 0])]


def gt_of(name):
    from gnss_eval import GT
    kind, f = cfgd(name)["gt"].split(":", 1)
    return GT.from_pvt(V / name) if kind == "pvt" else GT.from_tum(V / name / f)


def fmt(res, g):
    return {"se3": res["ate_se3"], "sim3": res["ate_sim3"], "scale": res["scale"], "geo": g["ate_noalign"], "geo_median": g["noalign_median"], "geo_max": g["noalign_max"]}


def score(name, variant, tag):
    import numpy as np
    from gnss_eval import score as sc
    j = cfgd(name)
    gt = gt_of(name)
    cam = np.loadtxt(V / name / "cam0/data.csv", delimiter=",", usecols=0, comments="#") * 1e-9
    dur = cam[-1] - cam[0]
    d = V / "out" / name / f"{variant}_{tag}"
    res = sc(load(d / "final.csv"), gt, j["r_score"], len(cam), dur)
    g = sc(load(d / "global_final.csv"), gt, j["r_score"], len(cam), dur, geo=True)
    out = fmt(res, g)
    print(json.dumps({"case": f"{name}/{variant}_{tag}", **{k: round(v, 3) for k, v in out.items()}}), flush=True)
    return out


def loose(name, variant):
    import numpy as np
    from gnss_eval import score as sc, read_traj
    j = cfgd(name)
    gt = gt_of(name)
    cam = np.loadtxt(V / name / "cam0/data.csv", delimiter=",", usecols=0, comments="#") * 1e-9
    dur = cam[-1] - cam[0]
    d = V / "out" / name / "base_nogps"
    tum = d / "vio.tum"
    with open(tum, "w") as f:
        for r in load(d / "final.csv"):
            f.write(f"{r[0]:.9f} " + " ".join(repr(float(v)) for v in r[1:8]) + "\n")
    outp = V / "out" / name / f"{variant}_loose.tum"
    # smoother lever = the GNSS antenna in the IMU frame used by the OKVIS2-X config (gps_parameters r_SA), which is what the fixes refer to
    rsa = ",".join(str(x) for x in j["r_sa"])
    r = subprocess.run([sys.executable, str(ROOT / "tools/gnss_loose_fusion.py"), "--traj", str(tum), "--gps", str(V / name / f"gps_{variant}/gps0/data.csv"),
                        f"--rsa={rsa}", "--out", str(outp)], capture_output=True, text=True)
    (V / "out" / name / f"{variant}_loose.log").write_text(r.stdout + r.stderr)
    tr = read_traj(outp)
    res = sc(tr, gt, j["r_score"], len(cam), dur)
    g = sc(tr, gt, j["r_score"], len(cam), dur, geo=True)
    out = fmt(res, g)
    print(json.dumps({"case": f"{name}/{variant}_loose", **{k: round(v, 3) for k, v in out.items()}}), flush=True)


def cmp(name, variant, tag, xdir):
    """C run vs deterministic X reference run (runs/okvis2x_port/reference_runs/<xdir>): causal byte-identical, final cols 1-17, global byte-identical"""
    import okvis2x_check_gnss as cg
    c = V / "out" / name / f"{variant}_{tag}"
    x = ROOT / "runs/okvis2x_port/reference_runs" / xdir
    m = []
    if sha(c / "causal.csv") != sha(x / "causal.csv"):
        m.append("causal: " + str(cg.compare_cols(c / "causal.csv", x / "causal.csv")))
    d = cg.compare_cols(c / "final.csv", x / "final.csv")
    if d:
        m.append("final: " + d)
    if (c / "global_final.csv").exists() and sha(c / "global_final.csv") != sha(x / "global_final.csv"):
        m.append("global differs")
    print(f"{name}/{variant}_{tag} vs X {xdir}:", "PASS (causal byte-identical, final cols 1-17, global byte-identical)" if not m else "FAIL " + "; ".join(m),
          f"causal {sha(c / 'causal.csv')} global {sha(c / 'global_final.csv') if (c / 'global_final.csv').exists() else '-'}")


if __name__ == "__main__":
    a = sys.argv[1:]
    fx = []
    if "--fix" in a:
        i = a.index("--fix"); fx = [f for f in a[i + 1].split(",") if f]; del a[i:i + 2]
    nogps = "--nogps" in a
    lg = "--log" in a
    a = [x for x in a if x not in ("--nogps", "--log")]
    if a[0] == "run":
        run(a[1], a[2], a[3], fx, nogps, lg)
    elif a[0] == "score":
        score(a[1], a[2], a[3])
    elif a[0] == "cmp":
        cmp(a[1], a[2], a[3], a[4])
    elif a[0] == "loose":
        loose(a[1], a[2])
