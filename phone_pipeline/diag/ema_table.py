#!/usr/bin/env python3
"""Outdoor-1 blow-up evidence: the bridge speed reference (path-length EMA) vs chord speeds, from a sv_run --diag-log (F and E lines).
usage: ema_table.py diag.log t0_first_frame_ts [t_from t_to bin]"""
import sys, numpy as np
f = sys.argv[1]; t0 = float(sys.argv[2]); a0 = float(sys.argv[3]) if len(sys.argv) > 3 else 200; a1 = float(sys.argv[4]) if len(sys.argv) > 4 else 228; bw = float(sys.argv[5]) if len(sys.argv) > 5 else 2
F, E = {}, {}
for l in open(f):
    v = l.split()
    if v[0] == 'F': F[int(v[1])] = (float(v[2]) - t0, int(v[3]), int(v[4]), np.array(v[10:13], float), int(v[9]), int(v[13]))
    elif v[0] == 'E': E[int(v[1])] = tuple(map(float, v[2:5]))
fr = sorted(F)
T = np.array([F[i][0] for i in fr]); C = np.stack([F[i][3] for i in fr]); ST = np.array([F[i][1] for i in fr]); KF = np.array([F[i][4] for i in fr])
print(' t [s]  state  KF/frame  ema_speed(path)  |ema_vel|  chord speed(bin)  path speed(bin)  max dc/frame[u] (frames dc>0.1)   [map units / s]')
for a in np.arange(a0, a1, bw):
    m = (T >= a) & (T < a + bw) & (ST == 1)
    if m.sum() < 3: print(f'{a:5.0f}   lost/R-frames'); continue
    idx = np.where(m)[0]; P = C[idx]
    chord = np.linalg.norm(P[-1] - P[0]) / (T[idx[-1]] - T[idx[0]])
    path = np.linalg.norm(np.diff(P, axis=0), axis=1).sum() / (T[idx[-1]] - T[idx[0]])
    e = np.array([E[fr[i]] for i in idx])
    print(f'{a:5.0f}   {int(ST[idx[-1]])}      {(KF[idx] >= 0).mean():5.2f}      {e[-1, 0]:8.3f}        {e[-1, 1]:8.3f}   {chord:9.3f}         {path:9.3f}        {e[:, 2].max():7.3f} ({int((e[:, 2] > 0.1).sum())})')
