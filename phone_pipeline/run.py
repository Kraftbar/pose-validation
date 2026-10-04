#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""phone_pipeline: end-to-end, deterministic, permissive phone pipeline (all numerics in C99, python only moves files).

  images (+ IMU) --> stella_vio/sv_run  (multi-map ORB mono SLAM, gravity, optional gyro prior, R-frames, map merge)
                          |  trajectory_maps.tum (map id, R-frame flag, segment id), trajectory_gz.tum (gravity-aligned per map)
  IMU --> gnss_fusion/c/gf_gait_run (step cadence -> walking speed)       phone GNSS fixes
                          |                                                  |
                          +--------> gnss_fusion/c/gf_run (robust preset, speed prior, new map = new odometry segment,
                                      R-frame stretch = loose link) --> one continuous metric (geo-referenced) trajectory,
                                      batch (whole-graph smoother) and causal (sliding window, `live` = what an online consumer sees)

usage:
  run.py stages <seq> [--variants a,b] [--fuse a,b] [--stream JPEG_DIR]     sv_run variants (parallel, one lock-step pass over the frames), then every fusion
  run.py sv    <seq> [--variants ..] [--stream JPEG_DIR]
  run.py fuse  <seq> [--variants ..] [--fuse ..]
  run.py live  <seq> [--variants full] [--stream JPEG_DIR] [--pp "--pp-set key=val"]   section 15: the one-process live pipeline pp_live -> live_<variant>/ (pp.auto|sm|geo = live outputs)
<seq> = indoor1 indoor2 outdoor1 outdoor2 advio15 advio20 (configs/<seq>.json). Outputs: runs/phone_pipeline/<seq>/.
--stream: the images are JPEGs in JPEG_DIR (cam0/data + cam0/data.csv of the fetch layout); gray PGMs are produced just in time, shared by all
variants and deleted behind the slowest one (peak disk ~1.5 GB instead of 5-8 GB per sequence). Without it, <out>/_fx/<seq>/*.pgm must exist.
"""
import sys, os, re, json, time, subprocess, argparse, threading, resource, shutil
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SV_RUN = ROOT / 'stella_vio/sv_run'
VOCAB = ROOT / 'external/candidates/orb_vocab.fbow'
GF_RUN = ROOT / 'gnss_fusion/c/gf_run'
GAIT_RUN = ROOT / 'gnss_fusion/c/gf_gait_run'
OUT = ROOT / 'runs/phone_pipeline'

# stella_vio variants (sv_run --set ...). gravity=1 only adds the per-map up vector / trajectory_gz.tum (trajectory unchanged).
SV_VARIANTS = {
    'default': ['gravity=1'],                                  # stella_vio defaults (re-init into a new map, init levels 0..3, confirm 2) + gravity output
    'rm':      ['gravity=1', 'rframe=1', 'merge=1'],           # + R-frames through tracking failures and map merge
    'full':    ['gravity=1', 'rframe=1', 'merge=1', 'gyro=1'], # + gyro rotation prior for tracking and R-frame prediction
    'fullcal': ['gravity=1', 'rframe=1', 'merge=1', 'gyro=1'], # = full with the dataset-calibration extrinsic instead of the sequence-fitted one (calibration honesty)
    'servo':   ['gravity=1', 'rframe=1', 'merge=1', 'gyro=1', 'servo=0.5', 'servo_clip=0.2', 'servo_win=6', 'servo_dmin=2'],   # section 16: full + gait scale servo in the mapping (only meaningful with run.py live: it needs the speeds pp_live pushes)
}
GF_BASE = ['preset=robust', 'metric=0', 'rsa=0,0,0']
GAIT_CFG = ['speed=1', 'speed_align=1', 'speed_scale_rw_rel=1', 'speed_align_metric=1']        # gnss_fusion section 12 settings
INIT_WAIT_NOFIX = 12.0
LOOSE_K = 5.0   # R-frame stretches: link sigma x5 (0.05 m -> 0.25 m + 0.1 m/m): positions there are extrapolated, 'up to a few dm' (stella_vio/RESULTS.md)
# fusion modes: (use fixes, use gait speed, flag R-frame stretches loose)
FUSE_MODES = {'gait': (False, True), 'gnss': (True, False), 'both': (True, True)}


def cfg_of(seq):
    return json.loads((HERE / 'configs' / f'{seq}.json').read_text())


def rp(p):
    return str((ROOT / p) if not str(p).startswith('/') else p)


# ------------------------------------------------------------------------------------------------------------------ stella_vio stage
def last_frame(log):
    try:
        with open(log, 'rb') as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 65536))
            m = re.findall(rb'sv_run: frame (\d+) kfs', f.read())
        return int(m[-1]) if m else -1
    except OSError:
        return -1


def write_pgm(path, img):
    h, w = img.shape
    tmp = path.parent / ('.' + path.name + '.tmp')
    with open(tmp, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h)); f.write(img.tobytes())
    os.replace(tmp, path)


class Feeder(threading.Thread):
    """JPEG -> gray PGM just in time; PGMs (and the JPEGs) behind the slowest consumer are deleted."""
    def __init__(self, jdir, fx, logs, procs, ahead=1000, keep_jpeg=False):
        super().__init__(daemon=True)
        self.jdir, self.fx, self.logs, self.procs, self.ahead, self.keep_jpeg = Path(jdir), Path(fx), logs, procs, ahead, keep_jpeg
        rows = [l.split(',') for l in (self.jdir / 'cam0/data.csv').read_text().splitlines()[1:] if l.strip()]
        self.names = [r[1].strip() for r in rows]
        self.ready0 = threading.Event(); self.n_written = 0; self.error = None

    def progress(self):
        if not self.procs: return -1
        pr = [last_frame(l) for l, p in zip(self.logs, self.procs) if p.poll() is None]
        return min(pr) if pr else 10 ** 9

    def run(self):
        import cv2
        try:
            deleted = 0
            for i, nm in enumerate(self.names):
                while True:
                    prog = self.progress()
                    if prog >= 10 ** 9: return                   # all consumers finished
                    for j in range(deleted, min(prog - 3, i)):   # frames <= prog-3 are done by every consumer
                        (self.fx / f'{j:06d}.pgm').unlink(missing_ok=True)
                        if not self.keep_jpeg: (self.jdir / 'cam0/data' / self.names[j]).unlink(missing_ok=True)
                        deleted = j + 1
                    if i - prog <= self.ahead: break
                    time.sleep(0.3)
                img = cv2.imread(str(self.jdir / 'cam0/data' / nm), cv2.IMREAD_GRAYSCALE)
                if img is None: raise RuntimeError('cannot read ' + nm)
                write_pgm(self.fx / f'{i:06d}.pgm', img); self.n_written = i + 1
                if i == 2: self.ready0.set()
            self.ready0.set()
        except Exception as e:  # noqa
            self.error = e; self.ready0.set()


PP_LIVE = ROOT / 'phone_pipeline/c/pp_live'


def run_sv(seq, variants, stream=None, ahead=1000, live=False, pp_extra=(), keep_jpeg=False, vdir=None, vpp=None):
    """live=True: the same command line through pp_live (one process: stella_vio + gait + fusion, section 15) into live_<variant>/ (its sv outputs are the same files as sv_<variant>/)"""
    c = cfg_of(seq); out = OUT / seq; fx = out / '_fx' / seq; fx.mkdir(parents=True, exist_ok=True)
    cmds, logs, procs, outs = {}, {}, {}, {}
    feeder = None
    if stream:
        # processes need the first frames; the feeder is told about the processes afterwards through shared lists
        pl, ll = [], []
        feeder = Feeder(stream, fx, ll, pl, ahead=ahead, keep_jpeg=keep_jpeg); feeder.start(); feeder.ready0.wait()
    for v in variants:
        d = (vdir or {}).get(v) or out / (f'live_{v}' if live else f'sv_{v}'); d.mkdir(parents=True, exist_ok=True); outs[v] = d
        cmd = [str(PP_LIVE if live else SV_RUN), str(VOCAB), rp(c['rgb_dir']), str(fx), str(d), '--no-snap', '--lean', '--wait-fixtures', '--size', c['size'], '--camera', c['camera'],
               '--imu', rp(c['imu']), '--imu-ext', str(d / 'ext.txt'), '--imu-toff', str(c['imu_toff']), '--imu-bg', ','.join(str(x) for x in c['imu_bg'])]
        (d / 'ext.txt').write_text(' '.join(repr(float(x)) for x in c['imu_ext_cal' if v == 'fullcal' else 'imu_ext']) + '\n')
        for s in SV_VARIANTS[v]: cmd += ['--set', s]
        if live:
            cmd += ['--live-out', str(d / 'live.tum'), '--servo-log', str(d / 'servo.log'), '--pp-out', str(d / 'pp'), '--pp-speed-out', str(d / 'pp.speed')]
            if c['fixes']: cmd += ['--pp-fix', rp(c['fixes'])]
            if c['gait']['mode'] == 'user': cmd += ['--pp-gait-c', repr(c['gait']['c'])]
            cmd += list(pp_extra) + list((vpp or {}).get(v, []))
        cmds[v] = cmd; logs[v] = d / 'log.txt'
    t0 = time.time()
    for v in variants:
        f = open(logs[v], 'w'); p = subprocess.Popen(cmds[v], stdout=f, stderr=subprocess.STDOUT); p._f = f; procs[v] = p
        if feeder: feeder.procs.append(p); feeder.logs.append(logs[v])
    res = {}
    for v in variants:
        p = procs[v]; _, st, ru = os.wait4(p.pid, 0)
        p.returncode = os.waitstatus_to_exitcode(st) if hasattr(os, 'waitstatus_to_exitcode') else st >> 8
        p._f.close()
        nfr = int(re.findall(r'sv_run: frame (\d+) kfs', logs[v].read_text())[-1]) if logs[v].exists() else 0
        res[v] = dict(rc=p.returncode, cpu_s=ru.ru_utime + ru.ru_stime, wall_s=time.time() - t0, max_rss_mb=ru.ru_maxrss / 1024.0)
        (outs[v] / 'run.json').write_text(json.dumps(res[v]))
        print(f'{seq} {"live" if live else "sv"}_{v}: rc={p.returncode} cpu {ru.ru_utime + ru.ru_stime:.0f}s', flush=True)
    if feeder:
        feeder.join(timeout=5)
        for q in fx.glob('*.pgm'): q.unlink()          # fixtures left in the look-ahead window
        for q in fx.glob('.*.tmp'): q.unlink()
    return res


# ------------------------------------------------------------------------------------------------------------------ odometry adaptor + gait + fusion
def make_odom(sv_dir, out_path, loose=True):
    """trajectory_maps.tum (t x y z q map rframe seg) + trajectory_gz.tum (gravity-aligned per map) -> gf_run odometry file with flags:
    new map id -> NEW_FRAME(1); new segment of the same map (a bridged R-frame part, own scale) -> GAP(2)|LOOSE(4); R-frame sample -> LOOSE(4)."""
    m = np.loadtxt(sv_dir / 'trajectory_maps.tum', ndmin=2)
    gzf = sv_dir / 'trajectory_gz.tum'
    if not gzf.exists() or gzf.stat().st_size == 0: raise RuntimeError('no trajectory_gz.tum (run sv_run with gravity=1 and --imu)')
    z = np.loadtxt(gzf, ndmin=2)
    key = {round(r[0], 6): r for r in z}
    rows, prev_map, prev_seg, n_miss = [], None, None, 0
    stats = dict(n_new_frame=0, n_gap=0, n_rframe=0, n_poses=0, n_no_gravity=0)
    for r in m[np.argsort(m[:, 0], kind='stable')]:
        g = key.get(round(r[0], 6))
        if g is None: stats['n_no_gravity'] += 1; continue          # map without a gravity estimate: no usable gravity-aligned pose
        mid, rf, seg = int(r[8]), int(r[9]), int(r[10])
        fl = 0
        if prev_map is not None and mid != prev_map: fl |= 1; stats['n_new_frame'] += 1
        elif prev_seg is not None and seg != prev_seg and loose: fl |= 2 | 4; stats['n_gap'] += 1
        if rf and loose: fl |= 4; stats['n_rframe'] += 1
        prev_map, prev_seg = mid, seg
        if rows and g[0] <= rows[-1][0]: continue
        rows.append([g[0], *g[1:8], fl]); stats['n_poses'] += 1
    with open(out_path, 'w') as f:
        for r in rows: f.write('%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %d\n' % tuple(r))
    return stats


def read_fixes(path, out_path, sigma_k=1.0):
    """gps0/data.csv (ns, E, N, U, hErr1, hErr2, vErr) -> gf_run fixes 't E N U sh sv' (s)"""
    rows = []
    for l in Path(path).read_text().splitlines()[1:]:
        v = [float(x) for x in l.split(',')[:7]]
        rows.append((v[0] * 1e-9, v[1], v[2], v[3], max(v[4], 0.02) * sigma_k, max(v[6], 0.02) * sigma_k))
    with open(out_path, 'w') as f:
        for r in rows: f.write('%.9f %.6f %.6f %.6f %.4f %.4f\n' % r)
    return len(rows)


def run_gait(seq, out):
    """gf_gait_run on the phone IMU -> speed measurements file for gf_run (3 s epochs over a 6 s window); returns timing"""
    c = cfg_of(seq); g = c['gait']
    ep, sp = out / 'gait.ep', out / 'speed.txt'
    cmd = [str(GAIT_RUN), '--imu', rp(c['imu']), '--epochs', str(ep), '--epoch-dt', '3', '--window', '6']
    if g['mode'] == 'user': cmd += ['--model', repr(g['c'])]
    t0 = time.time(); r = subprocess.run(cmd, check=True); wall = time.time() - t0
    n_imu = sum(1 for _ in open(rp(c['imu']))) - 1
    ru = resource.getrusage(resource.RUSAGE_CHILDREN)
    e = np.loadtxt(ep, ndmin=2); n = 0
    with open(sp, 'w') as f:
        for r_ in e:
            if int(r_[3]) == 0: f.write('%.9f %.6f %.6f %.3f 0\n' % (r_[0], r_[4], r_[5], r_[6])); n += 1
            elif int(r_[3]) == 1: f.write('%.9f 0 %.6f %.3f 1\n' % (r_[0], r_[5], r_[6])); n += 1
    return dict(n_epochs=len(e), n_speed=n, wall_s=wall, n_imu=n_imu, us_per_imu=wall / max(n_imu, 1) * 1e6)


def run_fuse(seq, variant, mode, extra=(), tag=None, loose=True, fix_sigma_k=1.0, live_dir=None):
    """live_dir: a pp_live output dir (live_<variant>/): the odometry is its LIVE stream (pp.odom) and the speed measurements its pp.speed; outputs go to live_dir/fuse_<tag|mode>/"""
    c = cfg_of(seq); sv = OUT / seq / f'sv_{variant}'
    d = (live_dir / f'fuse_{tag or mode}') if live_dir else (OUT / seq / f'fuse_{variant}_{tag or mode}'); d.mkdir(parents=True, exist_ok=True)
    use_fix, use_gait = FUSE_MODES[mode]
    if live_dir:
        shutil.copy(live_dir / 'pp.odom', d / 'odom.txt'); ostats = dict(n_poses=sum(1 for _ in open(d / 'odom.txt')))
    else:
        ostats = make_odom(sv, d / 'odom.txt', loose=loose)
    nfix = 0
    if use_fix and c['fixes']: nfix = read_fixes(rp(c['fixes']), d / 'fix.txt', fix_sigma_k)
    else: (d / 'fix.txt').write_text('')
    res = dict(odom=ostats, n_fix=nfix, runs={}, init_wait_s=(30.0 if nfix else INIT_WAIT_NOFIX))
    sp = (live_dir / 'pp.speed') if live_dir else OUT / seq / 'speed.txt'
    for md in ('batch', 'causal'):
        cmd = [str(GF_RUN), '--odom', str(d / 'odom.txt'), '--fix', str(d / 'fix.txt'), '--out', str(d / f'{md}.out'), '--mode', md, '--timing', '--nodes', str(d / f'{md}.nodes')]
        if md == 'causal': cmd += ['--out-live', str(d / 'causal.live')]
        cfg = list(GF_BASE) + list(extra)
        if not nfix: cfg += [f'init_wait_s={INIT_WAIT_NOFIX}']   # no fixes: the alignment comes from the gait speed alone, no need to wait 30 s
        if use_gait: cmd += ['--speed', str(sp)]; cfg += GAIT_CFG
        if loose: cfg += [f'loose_k={LOOSE_K}']
        t0 = time.time(); r = subprocess.run(cmd + cfg, capture_output=True, text=True); wall = time.time() - t0
        if r.returncode: raise RuntimeError(r.stderr + r.stdout)
        (d / f'{md}.stdout').write_text(r.stdout)
        res['runs'][md] = dict(wall_s=wall, stdout=r.stdout.strip().splitlines()[-12:])
    (d / 'run.json').write_text(json.dumps(res))
    return res


GEOREF_RUN = ROOT / 'gnss_fusion/c/gf_georef_run'


def run_georef(seq, variant, extra=(), tag='georef', live_dir=None):
    """section 14: the fix-free (gait only) stream of fuse_<variant>_gait is geo-referenced by ONE slowly varying similarity from the fixes (gf_georef_run);
    causal = live stream of the gait run + the fixes up to now, batch = batch stream + one fit over all fixes. Needs fuse_<variant>_gait."""
    c = cfg_of(seq)
    src = (live_dir / 'fuse_gait') if live_dir else OUT / seq / f'fuse_{variant}_gait'
    d = (live_dir / f'fuse_{tag}') if live_dir else OUT / seq / f'fuse_{variant}_{tag}'; d.mkdir(parents=True, exist_ok=True)
    nfix = read_fixes(rp(c['fixes']), d / 'fix.txt')
    shutil.copy(src / 'odom.txt', d / 'odom.txt')
    res = dict(n_fix=nfix, runs={}, init_wait_s=30.0)
    for md, sf, of in (('batch', 'batch.out', 'batch.out'), ('causal', 'causal.live', 'causal.live')):
        t0 = time.time()
        r = subprocess.run([str(GEOREF_RUN), '--stream', str(src / sf), '--fix', str(d / 'fix.txt'), '--out', str(d / of), '--mode', md] + list(extra), capture_output=True, text=True)
        wall = time.time() - t0
        if r.returncode: raise RuntimeError(r.stderr + r.stdout)
        (d / f'{md}.stdout').write_text(r.stdout)
        res['runs'][md] = dict(wall_s=wall, stdout=r.stdout.strip().splitlines())
    shutil.copy(d / 'causal.live', d / 'causal.out')
    (d / 'run.json').write_text(json.dumps(res))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stage', choices=['stages', 'sv', 'fuse', 'gait', 'georef', 'live'])
    ap.add_argument('seq'); ap.add_argument('--variants', default='default,rm,full,fullcal'); ap.add_argument('--fuse', default='gait,gnss,both')
    ap.add_argument('--stream'); ap.add_argument('--ahead', type=int, default=1000); ap.add_argument('--keep-jpeg', action='store_true'); ap.add_argument('--pp', default='', help='extra pp_live options (live stage), e.g. "--pp-set policy=1"')
    ap.add_argument('--skips', default='0', help='with --cfg: start frames, e.g. 0,20,40')
    ap.add_argument('--cfg', action='append', default=[], help='live stage, section 16: "tag|sv_set1 sv_set2|pp_opt ..." = an extra sv_run configuration on top of `full`, run in the same lock-step pass, output runs/phone_pipeline/<seq>/study16/<tag>_s0/ (the canonical live_full/ is untouched); with --variants none only these')
    a = ap.parse_args()
    vs = a.variants.split(','); fm = a.fuse.split(',')
    if a.stage in ('stages', 'sv'): run_sv(a.seq, vs, stream=a.stream, ahead=a.ahead)
    if a.stage == 'live':
        vdir, vpp = {}, {}
        if a.variants == 'none': vs = []
        for cf in a.cfg:
            t, sv, pp = (cf.split('|') + ['', ''])[:3]
            for k in a.skips.split(','):      # --skips: start-frame perturbations of the initialisation (sv_run --skip), one process each, same lock-step pass
                tk = f'{t}_s{k}'
                SV_VARIANTS[tk] = SV_VARIANTS['full'] + sv.split(); vdir[tk] = OUT / a.seq / 'study16' / tk; vpp[tk] = pp.split() + (['--skip', k] if int(k) else []); vs.append(tk)
        run_sv(a.seq, vs, stream=a.stream, ahead=a.ahead, live=True, pp_extra=a.pp.split(), keep_jpeg=a.keep_jpeg, vdir=vdir, vpp=vpp)
    if a.stage in ('stages', 'gait', 'fuse'):
        g = run_gait(a.seq, OUT / a.seq); (OUT / a.seq / 'gait.json').write_text(json.dumps(g))
    if a.stage == 'georef':
        for v in vs: run_georef(a.seq, v)
    if a.stage in ('stages', 'fuse'):
        for v in vs:
            for m in fm:
                if m == 'gait' or cfg_of(a.seq)['fixes']: run_fuse(a.seq, v, m)
            if cfg_of(a.seq)['fixes'] and 'gait' in fm: run_georef(a.seq, v)


if __name__ == '__main__':
    main()
