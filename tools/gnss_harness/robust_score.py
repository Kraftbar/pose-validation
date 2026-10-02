#!/usr/bin/env python3
"""Phone/handheld robustness scoring (benchmark only). Reads runs from external/gnss/rob/out/<seq>_<system>[_tag]/{traj.txt,run.json,log.txt}
plus the earlier OKVIS2-X outputs, scores them with gnss_eval.score (benchmark.umeyama_alignment, never reimplemented) and writes
runs/gnss_compare/robustness/{table.md,table.json,<id>/{metrics.json,traj.txt}}.
Per run: ATE Sim3 + scale, ATE SE3 (metric-scale systems), pose coverage (poses / camera frames), covered span (sum of pose gaps <= 1 s / duration),
tracking losses / resets / new maps parsed from each system's log, RTF = wall / data duration, error by 30 s bin (Sim3-aligned, whole-run alignment)."""
import sys, json, re
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from gnss_eval import GT, read_traj, score, antenna  # noqa: E402
from benchmark import umeyama_alignment, apply_alignment  # noqa: E402

G = Path('/home/nybo/github/pose-validation/external/gnss')
ROB = G / 'rob'
OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare/robustness')
RSA_OKVIS = [-0.01, -0.03, -0.06]
SEQ = {
    'outdoor1': dict(dir=ROB / 'outdoor1', gt=lambda: GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887), dur=393.99, rsa=[0, 0, 0]),
    'o1half': dict(dir=ROB / 'outdoor1', gt=lambda: GT.from_tum(G / 'seq/outdoor1/gt.tum', dt=-292.887), dur=393.99, rsa=[0, 0, 0]),  # 640x360 copy of outdoor1 (stella port)
    'complex': dict(dir=ROB / 'complex', gt=lambda: GT.from_pvt(G / 'seq/complex'), dur=435.65, rsa=[0, 0, 0]),
}

SEQ['complexs'] = dict(SEQ['complex'], dir=ROB / 'complex')   # same data, accelerometer rescaled x1.075 (diagnostic)


def n_frames(seq):
    return len((SEQ[seq]['dir'] / 'cam0' / 'data.csv').read_text().splitlines()) - 1


def count(log, pats):
    if not log.exists(): return {}
    txt = log.read_text(errors='ignore')
    return {k: len(re.findall(p, txt)) for k, p in pats.items()}


PATS = {
    'stella_up': dict(losses=r'tracking lost', maps=r'new map created', resets=r'resetting system|reset'),
    'orb3': dict(losses=r'Fail to track local map|Frames set to lost', resets=r'Reseting active map', maps=r'New Map created'),
    'okvis': dict(losses=r'TRACKING FAILURE', ransac=r'RANSAC FAIL'),
}
# id -> (system label, sensors, seq, traj file, run dir for run.json/log, pattern family, frame, metric_scale)
LEGACY = {
    ('complex', 'okvis2x_vio_default'): ('OKVIS2-X mono, default (shipped GVINS cfg)', 'cam+IMU', G / 'out/okvis2x_mono_nogps_complex/okvis2-vio-final_trajectory.csv', G / 'out/okvis2x_mono_nogps_timing', 'okvis', 'body'),
}


def runs():
    R = []
    for d in sorted((ROB / 'out').glob('*')):
        if not (d / 'run.json').exists(): continue
        seq, rest = d.name.split('_', 1)
        R.append((d.name, seq, rest, d))
    return R


def load(path):
    a = read_traj(path)
    return a[np.argsort(a[:, 0])] if len(a) else a


LABEL = {
    'stella_up': ('stella_vslam upstream mono (BSD-2)', 'cam', 'cam', 'stella_up'),
    'stella_port': ('stella C port (sv_run, ours)', 'cam', 'cam', 'stella_port'),
    'orb3_mono': ('ORB-SLAM3 mono (GPL, ref)', 'cam', 'cam', 'orb3'),
    'orb3_mi': ('ORB-SLAM3 mono-inertial (GPL, ref)', 'cam+IMU', 'body', 'orb3'),
    'orb3_mi_ds': ('ORB-SLAM3 mono-inertial, dataset cfg (GPL, ref)', 'cam+IMU', 'body', 'orb3'),
    'ov_mono': ('OpenVINS mono (GPL, ref)', 'cam+IMU', 'body', 'ov'),
    'okvis_default': ('OKVIS2-X mono default (BSD-3)', 'cam+IMU', 'body', 'okvis'),
    'okvis_imu10': ('OKVIS2-X mono tuned, IMU noise x10 (BSD-3)', 'cam+IMU', 'body', 'okvis'),
    'okvis_tuned': ('OKVIS2-X mono BRISK thr 12 / 1500 kp (BSD-3)', 'cam+IMU', 'body', 'okvis'),
}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    gts = {s: SEQ[s]['gt']() for s in SEQ}
    rows = []
    for rid, seq, rest, d in runs():
        sysname = re.sub(r'_(try\d+|[a-z]+\d*x\d*)$', '', rest) if rest not in LABEL else rest
        if rest.startswith('fuse'):
            rj = json.loads((d / 'run.json').read_text()); src = rj['src'].split('_', 1)[1]
            mode = 'Sim3 fit to fixes' if rj.get('fit') else ('causal30' if rj.get('causal') else 'batch') + ' smoother'
            LABEL[rest] = (f"own loose fusion ({mode}{', GNSS sigma %g m' % rj['sigma'] if rj.get('sigma') else ', reported sigma'}) on {LABEL.get(src, (src,))[0]}",
                           'cam+GNSS (gravity from accel)' if rj.get('cam_only') else ('cam+GNSS (Sim3 only)' if LABEL.get(src, ('', 'x'))[1] == 'cam' else 'cam+IMU+GNSS'), 'body', 'none')
        key = next((k for k in sorted(LABEL, key=len, reverse=True) if rest == k or rest.startswith(k + '_')), None)
        if key is None: continue
        label, sens, frame, fam = LABEL[key]
        tag = rest[len(key) + 1:]
        tp = d / 'traj.txt'
        if not tp.exists() or tp.stat().st_size == 0:
            tr = np.zeros((0, 8))
        else:
            tr = load(tp)
        run = json.loads((d / 'run.json').read_text())
        nf = n_frames(seq); dur = SEQ[seq]['dur']
        r = dict(id=rid, seq=seq, system=label + (f' [{tag}]' if tag else ''), sensors=sens, exit_code=run.get('exit_code'),
                 wall_s=run.get('wall_s'), rtf=run.get('wall_s', 0) / dur if run.get('wall_s') else None)
        if len(tr) >= 20:
            s = score(tr, gts[seq], SEQ[seq]['rsa'], nf, dur, geo=False)
            r.update({k: v for k, v in s.items() if not k.startswith('_')})
            t = tr[:, 0]; gap = np.diff(t)
            r['covered_span'] = float(gap[gap <= 1.0].sum() / dur)
            r['t_first'] = float(t[0] - (gts[seq].t[0] if seq == 'complex' else 0))
            r['n_gaps_gt_1s'] = int((gap > 1.0).sum())
            if '_t' in s:
                tt = s['_t']; gp, ok = gts[seq].at(t)
                e, p = antenna(tr, SEQ[seq]['rsa'])[ok], gp[ok]
                Rs, ts, ss = umeyama_alignment(e, p, with_scale=True)
                err = np.linalg.norm(apply_alignment(e, Rs, ts, ss) - p, axis=1)
                b0 = tt[0]; bins = ((tt - b0) // 30).astype(int)
                r['err30s_sim3'] = [round(float(np.sqrt((err[bins == k] ** 2).mean())), 2) if (bins == k).sum() > 3 else None for k in range(bins.max() + 1)]
                r['t_scored0'] = float(b0)
        else:
            r.update(n_pose=int(len(tr)), coverage=len(tr) / nf)
        if len(tr) >= 20 and seq == 'outdoor1':   # same-window ATE: ORB-SLAM3's surviving map covers 176-394 s (mono-inertial) / 97-262 s (mono) of 394 s
            t00 = 1778901613.003068
            for w0, w1 in ((176, 394), (97, 262)):
                m = (tr[:, 0] - t00 >= w0) & (tr[:, 0] - t00 <= w1)
                if m.sum() >= 50 and (tr[m, 0].max() - tr[m, 0].min()) > 0.6 * (w1 - w0):
                    sw = score(tr[m], gts[seq], SEQ[seq]['rsa'], int(m.sum()), w1 - w0)
                    r[f'win{w0}_{w1}_sim3'] = sw.get('ate_sim3'); r[f'win{w0}_{w1}_se3'] = sw.get('ate_se3'); r[f'win{w0}_{w1}_n'] = int(m.sum())
        if len(tr) >= 20 and seq == 'outdoor1':   # GNSS fixes alone, restricted to the fixes inside this run's pose coverage (fair comparison for partial coverage)
            fx = np.loadtxt(G / 'seq/outdoor1/gnss_enu.txt'); kk = np.clip(np.searchsorted(tr[:, 0], fx[:, 0]), 1, len(tr) - 1)
            near = np.minimum(np.abs(tr[kk, 0] - fx[:, 0]), np.abs(tr[kk - 1, 0] - fx[:, 0])) < 1.0
            sg = score(fx[near], gts[seq], [0, 0, 0], nf, dur)
            r['gnss_alone_same_times_se3'] = sg.get('ate_se3'); r['gnss_alone_n'] = int(near.sum())
        r.update(count(d / 'log.txt', PATS.get(fam, {})))
        if fam == 'okvis' and (d / 'log.txt').exists():   # RTF of the tracking phase only (until 'Finished!'), the final BA is reported separately
            import datetime
            ts = [(l, re.match(r'[A-Z](\d{8} \d\d:\d\d:\d\d\.\d+)', l)) for l in (d / 'log.txt').read_text(errors='ignore').splitlines() if re.match(r'[A-Z]\d{8} ', l)]
            tt = lambda m: datetime.datetime.strptime(m.group(1), '%Y%m%d %H:%M:%S.%f').timestamp()
            fin = [tt(m) for l, m in ts if 'Finished!' in l]
            if fin: r['rtf_tracking'] = (fin[0] - tt(ts[0][1])) / dur
        if fam == 'stella_port':
            m = re.search(r'(\d+) lost frames, (\d+) resets', (d / 'log.txt').read_text(errors='ignore'))
            if m: r['losses'], r['resets'] = int(m.group(1)), int(m.group(2))
        (OUT / rid).mkdir(exist_ok=True)
        json.dump(r, open(OUT / rid / 'metrics.json', 'w'), indent=1)
        if len(tr): np.savetxt(OUT / rid / 'traj.txt', tr[:, :8], fmt='%.6f')
        rows.append(r)
    json.dump(rows, open(OUT / 'table.json', 'w'), indent=1)
    f = lambda v, n=2: '-' if v is None else f'{v:.{n}f}'
    L = []
    for seq in SEQ:
        L += [f'### {seq}', '', '| run | system | sensors | ATE Sim3 (m) | ATE SE3 (m) | scale | poses / frames | covered span | losses / resets / maps | RTF |', '|---|---|---|---|---|---|---|---|---|---|']
        for r in rows:
            if r['seq'] != seq: continue
            se3 = r.get('ate_se3') if r['sensors'] != 'cam' else None
            L.append(f"| {r['id']} | {r['system']} | {r['sensors']} | {f(r.get('ate_sim3'))} | {f(se3)} | {f(r.get('scale'),3)} | {r.get('n_pose','-')} / {n_frames(seq)} ({r.get('coverage',0)*100:.0f}%) | "
                     f"{f(r.get('covered_span',0)*100,0)}% | {r.get('losses','-')} / {r.get('resets','-')} / {r.get('maps','-')} | {f(r.get('rtf'),2)}{' (track ' + f(r['rtf_tracking'],2) + ')' if r.get('rtf_tracking') else ''} |")
        L.append('')
    (OUT / 'table.md').write_text('\n'.join(L)); print('\n'.join(L))


if __name__ == '__main__':
    main()
