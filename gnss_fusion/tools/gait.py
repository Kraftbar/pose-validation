#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Gait (step cadence) speed prior from phone IMU: python prototype of c/gf_gait.{h,c}. The two implement the same sample-by-sample algorithm
with the same operation order; tools/check_gait.py compares them.

Stream per IMU sample (t, a = accelerometer specific force [m/s^2], w = gyro [rad/s]):
  an = |a|;  hp = an - EMA(an, tau_slow);  sm = EMA(EMA(hp, tau_sm), tau_sm)
  step = local maximum of sm above max(std_k * running_std(sm), thr_floor), at least min_step_dt after the previous step
  stationary measure: EMA(hp^2, tau_stat) and EMA(|w|, tau_stat)
Epoch estimate at time t over the trailing window W: cadence from the step times, model speed v = k c cad^p, state WALK / STATIONARY / OTHER.
Online calibration (optional): k from GNSS chords vs the gait odometer over straight 30 s windows (gnss_fix()).
Heading: gyro integrated about the filtered gravity direction (straightness test of the calibration windows, and the PDR baseline).
"""
import math, sys
from pathlib import Path
import numpy as np

STATE_WALK, STATE_STATIONARY, STATE_OTHER = 0, 1, 2


class Config:
    tau_slow = 0.8
    tau_sm = 0.03
    tau_std = 15.0
    std_k = 0.35
    thr_floor = 0.3
    min_step_dt = 0.3
    window_s = 6.0          # trailing window of an epoch estimate
    min_steps = 4
    max_last_age = 1.2      # last step must be this recent [s]
    cad_min, cad_max = 1.0, 2.8
    tau_stat = 1.5
    stat_hp_rms = 0.12      # [m/s^2] rms of the high-passed norm below which the phone is still ...
    stat_gyro = 0.12        # ... and the mean |gyro| [rad/s]
    tau_grav = 1.0
    model_c = 0.389         # generic population model v = c * cad^p: step length 0.415 * 1.70 m = 0.70 m at 1.8 Hz (anthropometry, fitted to nothing in this repo)
    model_p = 2.0           # step length grows with cadence (p = 2: v/cad ~ cad); the exponent was picked by leave-one-sequence-out on the 4 Mobile-GVIO sequences
    rel_sigma = 0.20        # relative 1-sigma of the model speed, uncalibrated
    user_rel_sigma = 0.10   # ... after a per-user calibration
    abs_sigma = 0.10        # [m/s]
    # online calibration from GNSS
    online = 0
    on_win = 30.0           # window [s]
    on_edge = 8.0           # edge sets [s]
    on_min_dist = 15.0      # gait distance over the window centre baseline [m]
    on_max_turn = math.radians(20.0)
    on_eval_dt = 5.0
    on_sigma_max = 20.0
    on_prior_m = 150.0      # prior pseudo-distance [m] at k = 1
    k_min, k_max = 0.6, 1.6
    # regularity gate (detector v2, section 17; reg = 0: the section-12 detector): a window whose step intervals / step amplitudes vary too much (coefficient of variation)
    # or whose median amplitude is too small is OTHER, no speed
    reg = 0
    reg_iv_cv = 0.15
    reg_amp_cv = 0.40
    reg_amp_min = 0.0
    reg_sigma_k = 3.0       # reg = 2: irregular windows stay WALK with sigma x this and regular = 0 (a consumer that needs a trustworthy speed skips them)


class Gait:
    def __init__(self, cfg=None):
        self.c = cfg or Config()
        c = self.c
        self.n = 0; self.t_prev = None
        self.slow = 0.0; self.s1 = 0.0; self.s2 = 0.0; self.var_sm = 0.0; self.nstd = 0
        self.hp2 = 0.0; self.gn = 0.0
        self.p1 = 0.0; self.p2 = 0.0; self.tp1 = 0.0   # sm[k-1], sm[k-2], t[k-1]
        self.thr_prev = 0.0
        self.last_step = -1e30
        self.steps = []                                   # (t, amp)
        self.grav = np.zeros(3); self.heading = 0.0
        self.k = 1.0; self.mc = c.model_c
        self.odo = 0.0; self.odo_base = 0.0; self.odo_t = None
        # decimated history for the online calibration: (t, odometer, heading)
        self.hist = []
        self.fixes = []                                   # (t, e, n, sigma)
        self.on_c = 0.0; self.on_d = 0.0; self.on_last = -1e30; self.on_n = 0
        self.rel = c.rel_sigma
        self.walk_odo_extra = 0.0

    # --- model -----------------------------------------------------------------
    def set_model(self, c):
        self.mc = c; self.rel = self.c.user_rel_sigma

    def model_speed(self, cad):
        return self.k * self.mc * cad ** self.c.model_p

    def _ema(self, y, x, tau, dt):
        al = dt / (tau + dt)
        return y + al * (x - y)

    # --- stream ----------------------------------------------------------------
    def push(self, t, a, w):
        c = self.c
        an = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
        wn = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])
        stepped = 0
        if self.t_prev is None:
            self.slow = an; self.s1 = 0.0; self.s2 = 0.0
            self.grav = np.array([a[0], a[1], a[2]], float); self.gn = wn
            self.t_prev = t; self.odo_t = t; self.n = 1
            self.p1 = self.p2 = 0.0; self.tp1 = t
            self._hist(t)
            return 0
        dt = t - self.t_prev
        if not (dt > 0): return 0
        self.t_prev = t
        self.slow = self._ema(self.slow, an, c.tau_slow, dt)
        hp = an - self.slow
        self.s1 = self._ema(self.s1, hp, c.tau_sm, dt)
        self.s2 = self._ema(self.s2, self.s1, c.tau_sm, dt)
        sm = self.s2
        self.n += 1
        # running variance of sm (warm start: plain average for the first samples)
        self.nstd += 1
        al = dt / (c.tau_std + dt); al1 = 1.0 / self.nstd
        if al1 > al: al = al1
        self.var_sm += al * (sm * sm - self.var_sm)
        self.hp2 = self._ema(self.hp2, hp * hp, c.tau_stat, dt)
        self.gn = self._ema(self.gn, wn, c.tau_stat, dt)
        # step: peak at the previous sample
        thr = c.std_k * math.sqrt(self.var_sm)
        if thr < c.thr_floor: thr = c.thr_floor
        if self.p1 > self.p2 and self.p1 >= sm and self.p1 > thr and (self.tp1 - self.last_step) >= c.min_step_dt:
            self.last_step = self.tp1
            self.steps.append((self.tp1, self.p1)); stepped = 1
            if len(self.steps) > 64: self.steps.pop(0)
        self.p2 = self.p1; self.p1 = sm; self.tp1 = t
        # gravity + heading
        self.grav = self.grav + (dt / (c.tau_grav + dt)) * (np.array(a, float) - self.grav)
        gnorm = math.sqrt(float(self.grav @ self.grav))
        if gnorm > 1e-6:
            wz = (w[0] * self.grav[0] + w[1] * self.grav[1] + w[2] * self.grav[2]) / gnorm
            self.heading += wz * dt
        # odometer: model speed from the trailing cadence
        e = self.estimate(t)
        if e['state'] == STATE_WALK: self.odo += e['speed'] * dt; self.odo_base += e['base'] * dt
        self.odo_t = t
        self._hist(t)
        return stepped

    def _hist(self, t):
        if not self.hist or t - self.hist[-1][0] >= 1.0 - 1e-9:
            self.hist.append((t, self.odo_base, self.heading))
            if len(self.hist) > 400: self.hist.pop(0)

    # --- epoch estimate ----------------------------------------------------------
    def estimate(self, t, window=None):
        c = self.c
        W = c.window_s if window is None else window
        sw = [s for s in self.steps if t - W < s[0] <= t]
        st = [s[0] for s in sw]
        out = dict(t=t, window=W, n_steps=len(st), cadence=0.0, state=STATE_OTHER, speed=0.0, sigma=1.0, regular=1)
        if self.n < 2: return out
        stationary = (math.sqrt(self.hp2) < c.stat_hp_rms and self.gn < c.stat_gyro and
                      (len(st) < 2 or t - st[-1] > 2.0))
        if stationary:
            out['state'] = STATE_STATIONARY; out['speed'] = 0.0; out['sigma'] = 0.05
            return out
        if len(st) >= c.min_steps and t - st[-1] <= c.max_last_age and st[-1] > st[0]:
            cad = (len(st) - 1) / (st[-1] - st[0])
            out['cadence'] = cad
            regular = True
            if c.reg and len(sw) >= 3:
                n = len(sw); sa = sa2 = si = si2 = 0.0; ni = 0
                for i, (ts, am) in enumerate(sw):
                    sa += am; sa2 += am * am
                    if i > 0: iv = ts - sw[i - 1][0]; si += iv; si2 += iv * iv; ni += 1
                ma = sa / n; va = max(sa2 / n - ma * ma, 0.0); mi = si / ni; vi = max(si2 / ni - mi * mi, 0.0)
                if math.sqrt(vi) > c.reg_iv_cv * mi or math.sqrt(va) > c.reg_amp_cv * ma or ma < c.reg_amp_min: regular = False
            if not regular and c.reg == 2: regular = True; out['regular'] = 0
            if c.cad_min <= cad <= c.cad_max and regular:
                v = self.model_speed(cad)
                if v > 0.0:
                    out['state'] = STATE_WALK; out['speed'] = v; out['base'] = v / self.k
                    rel = self.rel
                    out['sigma'] = math.sqrt((rel * v) ** 2 + c.abs_sigma ** 2)
                    if not out['regular']: out['sigma'] *= c.reg_sigma_k
        return out

    # --- online calibration (GNSS) -----------------------------------------------
    def _at(self, t):
        """(odometer, heading) at time t from the decimated history (linear)"""
        h = self.hist
        if not h or t < h[0][0] or t > h[-1][0]: return None
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if h[m][0] <= t: lo = m
            else: hi = m
        w = (t - h[lo][0]) / max(h[hi][0] - h[lo][0], 1e-9)
        return (1 - w) * h[lo][1] + w * h[hi][1], (1 - w) * h[lo][2] + w * h[hi][2]

    def gnss_fix(self, t, e, n, sigma_h):
        c = self.c
        if not c.online: return
        self.fixes.append((t, e, n, sigma_h))
        if len(self.fixes) > 120: self.fixes.pop(0)
        if t - self.on_last < c.on_eval_dt: return
        A = [f for f in self.fixes if t - c.on_win <= f[0] <= t - c.on_win + c.on_edge]
        B = [f for f in self.fixes if t - c.on_edge <= f[0] <= t]
        if len(A) < 3 or len(B) < 3: return
        if max(f[3] for f in A + B) > c.on_sigma_max: return
        ta = sum(f[0] for f in A) / len(A); tb = sum(f[0] for f in B) / len(B)
        ea = sum(f[1] for f in A) / len(A); na = sum(f[2] for f in A) / len(A)
        eb = sum(f[1] for f in B) / len(B); nb = sum(f[2] for f in B) / len(B)
        sa = sum(f[3] ** 2 for f in A) / len(A) / len(A); sb = sum(f[3] ** 2 for f in B) / len(B) / len(B)   # variance of the mean (per axis)
        oa = self._at(ta); ob = self._at(tb)
        if oa is None or ob is None: return
        self.on_last = t
        dist = ob[0] - oa[0]                  # distance of the BASE model (k not applied)
        if dist < c.on_min_dist: return
        if abs(ob[1] - oa[1]) > c.on_max_turn: return
        d2 = (eb - ea) ** 2 + (nb - na) ** 2 - 2.0 * (sa + sb)
        chord = math.sqrt(d2) if d2 > 0 else 0.0
        self.on_c += chord; self.on_d += dist; self.on_n += 1
        m0 = c.on_prior_m * (c.on_win - c.on_edge) / c.on_eval_dt
        k = (self.on_c + m0 * 1.0) / (self.on_d + m0)
        self.k = min(max(k, c.k_min), c.k_max)
        self.rel = max(0.10, c.rel_sigma / math.sqrt(1.0 + self.on_d / (m0 * 1.0)))


# ------------------------------------------------------------------------------------------------ helpers (data, epochs, evaluation)
REPO = Path('/home/nybo/github/pose-validation')
sys.path.insert(0, str(REPO / 'tools' / 'phone_diag'))


def load_imu(seq):
    import common
    imu_f = common.SEQ[seq][0]
    im = np.loadtxt(imu_f, delimiter=',', comments='#')
    return im[:, 0] * 1e-9, im[:, 4:7], im[:, 1:4]


def run_stream(t, acc, gyr, cfg=None, model=None, k=1.0, epoch_dt=3.0, fixes=None, window=None):
    """feed a whole recording; returns (Gait, epochs[list of dicts at t0+epoch_dt*n])"""
    g = Gait(cfg)
    if model is not None: g.set_model(model)
    g.k = k
    fx = list(fixes) if fixes is not None else []
    fi = 0
    ep = []; nxt = t[0] + epoch_dt
    for i in range(len(t)):
        while fi < len(fx) and fx[fi][0] <= t[i]:
            g.gnss_fix(*fx[fi]); fi += 1
        g.push(t[i], acc[i], gyr[i])
        if t[i] >= nxt:
            e = g.estimate(t[i], window); e['k'] = g.k; e['odo'] = g.odo; e['heading'] = g.heading; ep.append(e)
            nxt += epoch_dt
    return g, ep


def movavg(x, n):
    n = max(1, int(n)); c = np.cumsum(np.insert(x, 0, 0.0)); y = (c[n:] - c[:-n]) / n
    pad = len(x) - len(y)
    return np.concatenate([np.full(pad // 2, y[0]), y, np.full(pad - pad // 2, y[-1])])


def gt_speed_window(d, t1, W):
    """GT horizontal+vertical path length / W over [t1-W, t1] (0.5 s smoothed positions), or nan"""
    import common
    tg, p = d['tg'], d['p']
    m = (tg >= t1 - W) & (tg <= t1)
    if m.sum() < max(5, int(0.6 * W / np.median(np.diff(tg)))): return float('nan')
    if not common.gt_continuous_mask(tg[m]).all(): return float('nan')
    q = p[m]; rate = 1.0 / np.median(np.diff(tg[m]))
    q = np.stack([movavg(q[:, k], rate * 0.5) for k in range(3)], 1)
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()) * (W / (tg[m][-1] - tg[m][0]))
