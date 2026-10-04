#!/usr/bin/env python3
"""Run the rdvio_port deterministic reference on a EuRoC sequence and score it.

  python3 tools/run_rdvio_reference.py MH_01_easy --tag run1 [--dump] [--dump-every "prop=1,eval=200"]
Data: runs/rdvio_port/data/<seq>/mav0 (python3 tools/vio_harness/fetch_seq_stream.py with VIO_DATA_ROOT=runs/rdvio_port/data).
Output: runs/rdvio_port/<tag>/<seq>/{traj.tum,log.txt,run.json,dump/}; ATE through tools/vio_eval (benchmark.umeyama_alignment).
"""
import argparse, hashlib, json, os, subprocess, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))
BIN = REPO / "runs/rdvio_port/reference_build/build/rdvio_ref_driver"
V = REPO / "external/vio"


def score(traj, seq):
    import numpy as np
    import vio_eval as ve
    from benchmark import ate_rmse, umeyama_alignment, apply_alignment
    info = json.loads((ve.OUT / "gt" / f"{seq}_info.json").read_text())
    est = ve.read_tum(traj)
    gt = ve.read_tum(ve.OUT / "gt" / f"{seq}_body.tum")
    e, g = ve.associate(est, gt)
    r = ate_rmse(e, g)
    R, t, s = umeyama_alignment(e, g, with_scale=False)
    se3 = float(np.sqrt((np.linalg.norm(apply_alignment(e, R, t, s) - g, axis=1) ** 2).mean()))
    return {"n_pose": int(len(est)), "coverage": len(est) / info["frames"], "ate_sim3": r["ate_rmse"], "scale": r["scale"], "ate_se3": se3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("--tag", default="run1")
    ap.add_argument("--data-dir", default=None)
    ap.add_argument("--sensor", default=str(REPO / "rdvio_port/reference/configs/euroc_sensor.yaml"))
    ap.add_argument("--setting", default=str(REPO / "rdvio_port/reference/configs/setting.yaml"))
    ap.add_argument("--max-seconds", default=None)
    ap.add_argument("--dump", action="store_true", help="set RDVIO_PORT_DUMP_DIR (needs patches 0003-0005 in the build)")
    ap.add_argument("--dump-every", default=None, help="RDVIO_PORT_DUMP_EVERY, e.g. 'eval=200'")
    ap.add_argument("--stock-binary", default=None, help="run another driver binary (e.g. the stock build) with the same arguments")
    a = ap.parse_args()
    data = Path(a.data_dir) if a.data_dir else REPO / "runs/rdvio_port/data" / a.seq
    out = REPO / "runs/rdvio_port" / a.tag / a.seq
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    root = V / "deps/root/usr"; ocv = V / "deps/opencv"
    env["LD_LIBRARY_PATH"] = f"{root}/lib/x86_64-linux-gnu:{root}/lib/x86_64-linux-gnu/openblas-pthread:{ocv}/lib:" + env.get("LD_LIBRARY_PATH", "")
    if a.dump:
        d = out / "dump"; d.mkdir(exist_ok=True); env["RDVIO_PORT_DUMP_DIR"] = str(d)
        if a.dump_every:
            env["RDVIO_PORT_DUMP_EVERY"] = a.dump_every
    traj = out / "traj.tum"
    cmd = [a.stock_binary or str(BIN), a.sensor, a.setting, str(data / "mav0"), str(traj)] + ([a.max_seconds] if a.max_seconds else [])
    t0 = time.time()
    with open(out / "log.txt", "w") as lg:
        rc = subprocess.run(cmd, env=env, stdout=lg, stderr=subprocess.STDOUT).returncode
    wall = time.time() - t0
    sha = hashlib.sha256(traj.read_bytes()).hexdigest() if traj.exists() else None
    res = {"seq": a.seq, "tag": a.tag, "exit": rc, "wall_s": wall, "traj_sha256": sha}
    if traj.exists() and traj.stat().st_size:
        try:
            res.update(score(traj, a.seq))
        except Exception as ex:  # noqa
            res["score_error"] = repr(ex)
    (out / "run.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res))


if __name__ == "__main__":
    main()
