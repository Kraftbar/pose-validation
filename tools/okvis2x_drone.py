#!/usr/bin/env python3
"""Drone (INSANE o1 / m14) runs of the C port (okvis_port/c, OKVIS_PORT_OKVIS2X=1) and of the deterministic OKVIS2-X reference, plus scoring.

    python3 tools/okvis2x_drone.py run-c   <seq o1|m14> <tag> [--fix NAME,NAME] [--cfg yaml] [--log]   -> runs/okvis2x_port/drone/out/<seq>/<tag>/
    python3 tools/okvis2x_drone.py run-x   <seq> <tag>                                                  (uses tools/okvis2x_run_reference.py)
    python3 tools/okvis2x_drone.py cmp     <seq> <tag>                       C vs X byte comparison (causal, final cols 1-17, global)
    python3 tools/okvis2x_drone.py score   <seq> <tag> [x]                   SE3 / geo scoring with the drone-benchmark scorer (tools/gnss_harness/gnss_eval.py)
Data: runs/okvis2x_port/drone/<seq>/ (tools/drone_harness/insane_to_euroc.py: GNSS geodetic -> local ENU, see docs/okvis2x_yaw_failure_20261007.md).
Run python with external/gnss/venv/bin/python for `score`."""
import hashlib, os, subprocess, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
DR = ROOT / "runs/okvis2x_port/drone"
CFG = ROOT / "okvis2x_port/reference/configs"
VOC = ROOT / "runs/okvis_port/vocabulary/small_voc.bin"


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:12]


def run_c(seq, tag, fixes, cfg, log):
    import okvis2x_check_gnss as cg
    exe = cg.build()
    out = DR / "out" / seq / tag
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OKVIS_PORT_OKVIS2X="1")
    for f in fixes:
        env[f"OKVIS_PORT_FIX_{f}"] = "1"
    if log:
        env["OKVIS_PORT_GNSS_LOG"] = str(out / "gnss.log")
    sd = DR / f"seq_{seq}_{tag}"
    (sd / "mav0").mkdir(parents=True, exist_ok=True)
    for n in ("cam0", "imu0", "gps0"):
        l = sd / "mav0" / n
        if l.is_symlink():
            l.unlink()
        l.symlink_to((DR / seq / "mav0" / n).resolve())
    cfg = cfg or str(CFG / f"okvis2x_mono_drone_{seq}_deterministic.yaml")
    t0 = time.time()
    r = subprocess.run([str(exe), cfg, str(sd), str(VOC), str(out)], env=env, capture_output=True, text=True)
    (out / "stderr.txt").write_text(r.stderr)
    print(f"C {seq}/{tag} exit {r.returncode} wall {time.time() - t0:.0f}s causal {sha(out / 'causal.csv')} global {sha(out / 'global_final.csv')}")


def run_euroc(case, tag, fixes):
    """case = <reference tag>:<gps|gpsb|off>[:<data dir under runs/okvis2x_port>] as in okvis2x_check_gnss.py; output runs/okvis2x_port/drone/out/euroc/<case ref tag>_<tag>/"""
    import okvis2x_check_gnss as cg
    ref, mode, *rest = case.split(":")
    dr = ROOT / "runs/okvis2x_port" / (rest[0] if rest else "data")
    exe = cg.build()
    out = DR / "out/euroc" / f"{ref}_{tag}"
    out.mkdir(parents=True, exist_ok=True)
    sd = cg.seq_dir(f"drone_{ref}_{tag}", str(dr), str(ROOT / "external/vio/data/okvis_brisk_tmp"), "MH_01_easy")
    cfg = CFG / {"gps": "okvis2x_mono_euroc_gps_robustfalse_deterministic.yaml", "gpsb": "okvis2x_mono_euroc_gps_robusttrue_deterministic.yaml",
                 "off": "okvis2x_mono_euroc_deterministic.yaml"}[mode]
    env = dict(os.environ, OKVIS_PORT_OKVIS2X="1", OKVIS_PORT_GNSS_LOG=str(out / "gnss.log"))
    for f in fixes:
        env[f"OKVIS_PORT_FIX_{f}"] = "1"
    t0 = time.time()
    r = subprocess.run([str(exe), str(cfg), str(sd), str(VOC), str(out)], env=env, capture_output=True, text=True)
    (out / "stderr.txt").write_text(r.stderr)
    print(f"EuRoC {ref}/{tag} exit {r.returncode} wall {time.time() - t0:.0f}s causal {sha(out / 'causal.csv')} global {sha(out / 'global_final.csv') if (out / 'global_final.csv').exists() else '-'}")


def run_default():
    """regression gate: default mode (OKVIS_PORT_OKVIS2X unset, OKVIS2 canonical MH_01: final dfe3b58e6a33, causal cc29a746ea6f), all fix switches off"""
    import okvis2x_check_gnss as cg
    exe = cg.build()
    out = DR / "out/default_mh01"
    out.mkdir(parents=True, exist_ok=True)
    sd = cg.seq_dir("drone_default", str(ROOT / "runs/okvis2x_port/data"), str(ROOT / "external/vio/data/okvis_brisk_tmp"), "MH_01_easy")
    env = {k: v for k, v in os.environ.items() if not k.startswith("OKVIS_PORT_")}
    t0 = time.time()
    r = subprocess.run([str(exe), str(ROOT / "okvis_port/reference/configs/okvis_mono_euroc_deterministic.yaml"), str(sd), str(VOC), str(out)],
                       env=env, capture_output=True, text=True)
    print(f"default MH_01 exit {r.returncode} wall {time.time() - t0:.0f}s final {sha(out / 'final.csv')} causal {sha(out / 'causal.csv')} (expect dfe3b58e6a33 / cc29a746ea6f)")


def cmp(seq, tag, xtag=None):
    import okvis2x_check_gnss as cg
    c = DR / "out" / seq / tag
    x = ROOT / "runs/okvis2x_port/reference_runs" / seq / (xtag or tag)
    m = []
    if sha(c / "causal.csv") != sha(x / "causal.csv"):
        m.append("causal: " + str(cg.compare_cols(c / "causal.csv", x / "causal.csv")))
    d = cg.compare_cols(c / "final.csv", x / "final.csv")
    if d:
        m.append("final: " + d)
    if sha(c / "global_final.csv") != sha(x / "global_final.csv"):
        m.append("global differs")
    print(f"{seq}/{tag}", "PASS (causal byte-identical, final cols 1-17, global byte-identical)" if not m else "FAIL " + "; ".join(m),
          f"causal {sha(c / 'causal.csv')} global {sha(c / 'global_final.csv')}")


def score(seq, tag, which):
    import numpy as np
    sys.path.insert(0, str(ROOT / "tools/gnss_harness"))
    sys.path.insert(0, str(ROOT))
    from gnss_eval import read_traj, GT, score as sc
    import json
    sd = ROOT / "external/drone" / f"ins_{seq}"
    gt = GT.from_tum(sd / "gt_enu.tum")
    r_SA = json.load(open(ROOT / "external/drone" / f"ins_{seq}_cfg" / "lever.json"))["r_RTK"]
    cam = np.loadtxt(DR / seq / "mav0/cam0/data.csv", delimiter=",", usecols=0, comments="#") * 1e-9
    dur = cam[-1] - cam[0]
    d = (ROOT / "runs/okvis2x_port/reference_runs" / seq / tag) if which == "x" else (DR / "out" / seq / tag)

    def load(p):
        rows = []
        for l in Path(p).read_text().splitlines():
            q = [t.strip() for t in l.split(",")]
            if q[0].isdigit():
                rows.append([int(q[0]) * 1e-9] + [float(v) for v in q[1:8]])
        a = np.array(rows)
        return a[np.argsort(a[:, 0])]
    res = sc(load(d / "final.csv"), gt, r_SA, len(cam), dur)
    g = sc(load(d / "global_final.csv"), gt, r_SA, len(cam), dur, geo=True)
    out = {"se3": res["ate_se3"], "sim3": res["ate_sim3"], "scale": res["scale"], "geo": g["ate_noalign"], "geo_median": g.get("noalign_median"), "geo_max": g.get("noalign_max")}
    # the same on the causal (non-final-BA) trajectories
    try:
        rc = sc(load(d / "causal.csv"), gt, r_SA, len(cam), dur)
        out["se3_causal"] = rc["ate_se3"]
        gc = load(d / "global_causal.csv") if (d / "global_causal.csv").exists() else None
    except Exception as e:
        out["causal_err"] = str(e)
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in out.items()}))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    fx = []
    cfg = None
    if "--fix" in a:
        i = a.index("--fix"); fx = [f for f in a[i + 1].split(",") if f]; del a[i:i + 2]
    if "--cfg" in a:
        i = a.index("--cfg"); cfg = a[i + 1]; del a[i:i + 2]
    lg = "--log" in a
    a = [x for x in a if x != "--log"]
    if a[0] == "run-c":
        run_c(a[1], a[2], fx, cfg, lg)
    elif a[0] == "run-x":
        sys.exit(subprocess.run([sys.executable, str(ROOT / "tools/okvis2x_run_reference.py"), a[1], "--tag", a[2], "--config",
                                 cfg or str(CFG / f"okvis2x_mono_drone_{a[1]}_deterministic.yaml"), "--data-root", str(DR)]).returncode)
    elif a[0] == "run-default":
        run_default()
    elif a[0] == "run-euroc":
        run_euroc(a[1], a[2], fx)
    elif a[0] == "cmp":
        cmp(a[1], a[2], a[3] if len(a) > 3 else None)
    elif a[0] == "score":
        score(a[1], a[2], a[3] if len(a) > 3 else "c")
