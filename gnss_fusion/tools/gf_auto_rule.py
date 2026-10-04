#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Section 15 development tool: simulate candidate switch rules on the SAVED sm / geo streams + signal logs of gf_auto_run (no C re-run) and score them like
gf_auto_table.py / phone_pipeline/auto_eval.py. Used for the leave-one-case-out selection; the chosen rule is then implemented in gf_auto.c and verified end-to-end.
Needs the case streams: gf_auto_table.py --keep (work/auto/<case>.{sm,geo,sig}) and phone_pipeline/auto_eval.py --keep DIR (<seq>_<src>.{sm,geo,sig})."""
import sys, json, itertools
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent.parent / 'phone_pipeline'))
from gf_cases import *  # noqa
from gf_table import CASES, restrict, INIT_WAIT  # noqa

SIG = ['t', 'n_pairs', 'have_fit', 'sres', 'scale', 'psi', 'e_fast', 'e_slow', 'e_all', 'sig_rep', 'sig_white', 'dis', 'n_new', 'n_gap', 'distr', 'rho', 'wt', 'e_last']


def load(path):
    a = np.loadtxt(path, ndmin=2) if Path(path).exists() and Path(path).stat().st_size else np.zeros((0, 10))
    return a


class Item:
    """one case: streams on the odometry sample times + signals + a scoring function"""
    def __init__(self, name, sm, geo, sig, score_fn, gnss=None, metric=True):
        self.name, self.score_fn, self.gnss, self.metric = name, score_fn, gnss, metric
        self.sig = {k: sig[:, i] for i, k in enumerate(SIG)}
        t = sig[:, 0]
        self.t = t
        sm = sm[(sm[:, 8].astype(int) & 1) == 1] if len(sm) else sm
        key = lambda x: np.round(x, 6)
        ds = {k: i for i, k in enumerate(key(sm[:, 0]))} if len(sm) else {}
        dg = {k: i for i, k in enumerate(key(geo[:, 0]))} if len(geo) else {}
        n = len(t)
        self.psm = np.full((n, 3), np.nan); self.pge = np.full((n, 3), np.nan); self.qsm = np.tile([0, 0, 0, 1.0], (n, 1)); self.qge = self.qsm.copy()
        for i, k in enumerate(key(t)):
            if k in ds: self.psm[i] = sm[ds[k], 1:4]; self.qsm[i] = sm[ds[k], 4:8]
            if k in dg: self.pge[i] = geo[dg[k], 1:4]; self.qge[i] = geo[dg[k], 4:8]
        self.cache = {}

    def blend(self, wt, blend_s=20.0):
        """wt: target weight per sample -> w by slew limiting -> blended poses"""
        n = len(self.t); w = np.zeros(n); cur = 0.0
        for i in range(n):
            dt = self.t[i] - self.t[i - 1] if i else 0.0
            step = dt / blend_s if blend_s > 0 and dt > 0 else 1.0
            hs, hg = np.isfinite(self.psm[i, 0]), np.isfinite(self.pge[i, 0])
            tgt = wt[i] if (hs and hg) else (1.0 if hg else 0.0)
            if not hs and hg: cur = 1.0
            else: cur += max(-step, min(step, tgt - cur))
            w[i] = cur
        hs, hg = np.isfinite(self.psm[:, 0]), np.isfinite(self.pge[:, 0])
        p = np.where(hs[:, None] & hg[:, None], (1 - w)[:, None] * np.nan_to_num(self.psm) + w[:, None] * np.nan_to_num(self.pge), np.where(hs[:, None], self.psm, self.pge))
        q = np.where((w > 0.5)[:, None] & hg[:, None], self.qge, self.qsm)
        ok = hs | hg
        return np.c_[self.t, p, q][ok]

    def score(self, wt=None, which=None, blend_s=20.0):
        if which == 'sm': tr = np.c_[self.t, self.psm, self.qsm][np.isfinite(self.psm[:, 0])]
        elif which == 'geo': tr = np.c_[self.t, self.pge, self.qge][np.isfinite(self.pge[:, 0])]
        else: tr = self.blend(wt, blend_s)
        return self.score_fn(tr)


def earlier_items(d=None):
    d = Path(d) if d else WORK / 'auto'
    items = []
    ref = json.loads((WORK / 'table.json').read_text())
    for nm in ref:
        if not (d / f'{nm}.sig').exists(): continue
        case = CASES[nm](); raw = prepared(case); raw_t = raw[:, 0]; tmin = raw_t[0] + INIT_WAIT
        def fn(tr, case=case, raw_t=raw_t, tmin=tmin):
            o = restrict(tr[:, :8], raw_t, tmin)
            if len(o) < 20: return None
            return score_traj(case, o).get('ate_se3')
        items.append(Item(nm, load(d / f'{nm}.sm'), load(d / f'{nm}.geo'), load(d / f'{nm}.sig'), fn, gnss=ref[nm]['gnss']['se3'], metric=case.metric))
    return items


def phone_items(dirpath, srcs=('final', 'live'), seqs=('outdoor1', 'outdoor2', 'advio20')):
    import auto_eval as A, score as S, run as R
    items = []
    for s in seqs:
        case, ft = A.case_of(s)
        gal = json.loads((R.OUT / s / 'scores.json').read_text())['gnss_alone']['se3']
        for src in srcs:
            b = Path(dirpath) / f'{s}_{src}'
            if not Path(f'{b}.sig').exists(): continue
            def fn(tr, case=case, ft=ft):
                m = S.metrics(case, tr, ft, tmin=ft[0] + 30.0)
                return m.get('se3')
            items.append(Item({'outdoor1': 'p_o1', 'outdoor2': 'p_o2', 'advio20': 'p_a20'}[s] + ('' if src == 'final' else 'L'), load(f'{b}.sm'), load(f'{b}.geo'), load(f'{b}.sig'), fn, gnss=gal, metric=True))
    return items


# ---------------------------------------------------------------------------------------------------------------------------------------------------- rules
def rule_wt(it, rho_on, rho_off, q_on, q_off, min_pairs=30, mode='and'):
    """hysteresis state machine on rho = e_slow / max(sig_rep, .5) and the drift ratio q = e_fast / e_slow"""
    s = it.sig; n = len(it.t); wt = np.zeros(n); st = 0.0
    for i in range(n):
        if s['n_pairs'][i] < min_pairs or s['e_slow'][i] <= 0: wt[i] = st; continue
        rho = s['e_slow'][i] / max(s['sig_rep'][i], 0.5)
        q = s['e_fast'][i] / s['e_slow'][i]
        if st < 0.5:
            if rho <= rho_on and q <= q_on: st = 1.0
        else:
            if rho >= rho_off or q >= q_off: st = 0.0
        wt[i] = st
    return wt


def table(items, wt_fn, blend_s=20.0, verbose=True):
    rows = []
    for it in items:
        a = it.score(wt_fn(it), blend_s=blend_s)
        sm, ge = it.score(which='sm'), it.score(which='geo')
        rows.append((it.name, it.gnss, sm, ge, a))
    return rows


def show(rows):
    f = lambda v: '   -  ' if v is None else f'{v:8.2f}'
    for nm, g, sm, ge, a in rows:
        cand = [x for x in (sm, ge) if x is not None]
        best = min(cand) if cand else None
        rg = (a / best - 1) if (a and best) else float('nan')
        print(f'{nm:16s} GNSS {f(g)} sm {f(sm)} geo {f(ge)} auto {f(a)}  vs best {rg:+6.0%}  vs GNSS {(a / g - 1) if a and g else float("nan"):+7.0%}')


if __name__ == '__main__':
    its = earlier_items()
    rows = table(its, lambda it: rule_wt(it, 2.0, 3.0, 1.3, 1.6))
    show(rows)


def rule_wt2(it, rho_on, rho_off, q_on, q_off, floor, min_pairs=30, min_span=0.0):
    s = it.sig; n = len(it.t); wt = np.zeros(n); st = 0.0; t0 = it.t[0]
    for i in range(n):
        if s['n_pairs'][i] < min_pairs or s['e_slow'][i] <= 0 or it.t[i] - t0 < min_span: wt[i] = st; continue
        rho = s['e_slow'][i] / max(s['sig_rep'][i], floor)
        q = s['e_fast'][i] / s['e_slow'][i]
        if st < 0.5:
            if rho <= rho_on and q <= q_on: st = 1.0
        else:
            if rho >= rho_off or q >= q_off: st = 0.0
        wt[i] = st
    return wt


def regret(its, wt_fn, blend_s=20.0, cache={}):
    out = []
    for it in its:
        if it.name not in cache:
            cache[it.name] = (it.score(which='sm'), it.score(which='geo'))
        sm, ge = cache[it.name]
        a = it.score(wt_fn(it), blend_s=blend_s)
        cands = [x for x in (sm, ge) if x is not None]
        out.append((it.name, a / min(cands) if (a and cands) else 1.0, a, sm, ge, it.gnss))
    return out


def sim_w(it, P):
    """full switch simulation -> (blend weight per sample, georef allowed per sample). P: rho_on rho_off floor min_pairs min_span fail_k dwell up_s down_s sigmin metric_only distr_on distr_off"""
    s = it.sig; t = it.t; n = len(t); w = np.zeros(n); st = 0.0; cur = 0.0; hold = -1e18; allow = np.zeros(n, bool)
    for i in range(n):
        dt = t[i] - t[i - 1] if i else 0.0
        el = s['e_last'][i]; es = s['e_slow'][i]
        elig = s['n_pairs'][i] >= P['min_pairs'] and es > 0 and t[i] - t[0] >= P['min_span']
        if elig and (not it.metric and P.get('metric_only', 0)): elig = False
        if elig and s['sig_rep'][i] < P.get('sigmin', 0): elig = False
        if elig:
            noise = max(s['sig_rep'][i], P['floor'])
            rho = es / noise; di = s['distr'][i]
            if st > 0.5 and P['fail_k'] > 0 and el > P['fail_k'] * max(es, 2 * noise) and el > 0:
                st = 0.0; cur = 0.0; hold = t[i] + P['dwell']      # sudden failure of the stream: drop at once
            elif st < 0.5 and t[i] >= hold and rho <= P['rho_on'] and di <= P['distr_on']: st = 1.0
            elif st > 0.5 and (rho >= P['rho_off'] or di >= P['distr_off']): st = 0.0; hold = t[i] + P['dwell']
        else:
            st = 0.0
        hs, hg = np.isfinite(it.psm[i, 0]), np.isfinite(it.pge[i, 0])
        if not hg: cur = 0.0
        elif not hs: cur = 1.0 if st > 0.5 else 0.0
        else:
            d = st - cur
            step = (dt / P['up_s'] if d > 0 else dt / P['down_s']) if dt > 0 else 1.0
            cur += max(-step, min(step, d))
        w[i] = cur
        allow[i] = hs or (hg and st > 0.5)
    return w, allow


def blend_w(it, wa):
    w, allow = wa
    hs, hg = np.isfinite(it.psm[:, 0]), np.isfinite(it.pge[:, 0])
    p = np.where(hs[:, None] & hg[:, None], (1 - w)[:, None] * np.nan_to_num(it.psm) + w[:, None] * np.nan_to_num(it.pge), np.where(hs[:, None], it.psm, it.pge))
    q = np.where((w > 0.5)[:, None] & hg[:, None], it.qge, it.qsm)
    ok = allow
    return np.c_[it.t, p, q][ok]


DEF = dict(metric_only=1, sigmin=4.0, rho_on=1.0, rho_off=1.5, distr_on=0.15, distr_off=0.3, floor=0.1, min_pairs=30, min_span=0.0, fail_k=4.0, dwell=60.0, up_s=20.0, down_s=2.0)


def regret2(its, P, cache={}):
    out = []
    for it in its:
        if it.name not in cache: cache[it.name] = (it.score(which='sm'), it.score(which='geo'))
        sm, ge = cache[it.name]
        a = it.score_fn(blend_w(it, sim_w(it, P)))
        cands = [x for x in (sm, ge) if x is not None]
        out.append((it.name, a / min(cands) if (a and cands) else 1.0, a, sm, ge, it.gnss))
    return out
