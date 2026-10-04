#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Section 15 tables from runs/phone_pipeline/<seq>/live_full/scores.json (live_eval.py) and pp.timing. usage: live_report.py > runs/phone_pipeline/live_tables.md"""
import json, re
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
SEQS = [('indoor1', 'Indoor-1'), ('indoor2', 'Indoor-2'), ('outdoor1', 'Outdoor-1'), ('outdoor2', 'Outdoor-2'), ('advio15', 'ADVIO-15'), ('advio20', 'ADVIO-20')]
f = lambda x: '-' if x is None else f'{x:.2f}'
S = {k: json.loads((ROOT / f'runs/phone_pipeline/{k}/live_full/scores.json').read_text()) for k, _ in SEQS if (ROOT / f'runs/phone_pipeline/{k}/live_full/scores.json').exists()}
print('### Live vs final trajectory (ATE SE3 [m]; batch = whole-graph smoother, causal = sliding-window live output)\n')
print('| sequence | GNSS alone | final traj: batch / causal | live odometry, replay: batch / causal | live, streamed pp_live: smoother / georef / AUTO (causal) | live vs final batch | live vs final causal |')
print('|---|---|---|---|---|---|---|')
for k, nm in SEQS:
    if k not in S: continue
    r = S[k]; fx = 'both' if 'both' in r['final'] and 'gnss_alone' in r else 'gait'
    fb, fc = r['final'][fx]['batch'].get('se3'), r['final'][fx]['causal'].get('se3')
    rp = r['replay'][fx]; rb, rc = rp['batch'].get('se3'), rp['causal'].get('se3')
    st = r['stream']; g = r.get('gnss_alone', {}).get('se3')
    geo = f(st['geo'].get('se3')) if 'geo' in st and st['geo'].get('se3') else '-'
    print(f"| {nm} | {f(g)} | {f(fb)} / {f(fc)} | {f(rb)} / {f(rc)} | {f(st['sm'].get('se3'))} / {geo} / **{f(st['auto'].get('se3'))}** | {rb / fb - 1:+.0%} | {rc / fc - 1:+.0%} |")
print('\n### CPU per stage (pp_live, one process, thread CPU time; shared machine)\n')
print('| sequence | frames | stella_vio ms/frame (mean / p99) | gait us/frame | fusion us/frame (mean / p99 / max) | smoother A | stream smoother B | georef + switch | fix handling |')
print('|---|---|---|---|---|---|---|---|---|')
for k, nm in SEQS:
    p = ROOT / f'runs/phone_pipeline/{k}/live_full/pp.timing'
    if not p.exists(): continue
    L = p.read_text().splitlines()
    n = int(re.search(r'frames (\d+)', L[0]).group(1))
    sv = re.search(r'mean ([\d.]+) p50 ([\d.]+) p99 ([\d.]+) max ([\d.]+)', L[1]); ga = re.search(r'mean ([\d.]+)', L[2]); fu = re.search(r'mean ([\d.]+) p50 ([\d.]+) p99 ([\d.]+) max ([\d.]+)', L[3])
    d = re.search(r'smoother_A ([\d.]+) s\s+stream_smoother_B ([\d.]+) s\s+georef\+switch ([\d.]+) s\s+fix ([\d.]+) s', L[4])
    us = lambda s: 1e6 * float(s) / n
    print(f"| {nm} | {n} | {float(sv.group(1))/1e3:.1f} / {float(sv.group(3))/1e3:.1f} | {ga.group(1)} | {fu.group(1)} / {fu.group(3)} / {fu.group(4)} | {us(d.group(1)):.1f} | {us(d.group(2)):.1f} | {us(d.group(3)):.1f} | {us(d.group(4)):.1f} |")
