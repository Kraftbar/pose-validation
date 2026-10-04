# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Shared loaders / small math for the phone scale diagnosis (numpy only). Run with external/gnss/venv/bin/python."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
G = ROOT / 'external/gnss'
OFFS = json.loads((ROOT / 'tools/gnss_harness/phone_offsets.json').read_text())
OUT = ROOT / 'runs/phone_diag'
SEQ = {  # name: (imu csv, gt tum, gt clock offset: t_imu = t_gt + off)
    'outdoor1': (G / 'seq/outdoor1/imu0/data.csv', G / 'seq/outdoor1/gt.tum', -292.887),
    'outdoor2': (G / 'rob/outdoor2/imu0/data.csv', G / 'rob/outdoor2/gt.tum', OFFS['outdoor2']),
    'indoor1': (G / 'rob/indoor1/imu0/data.csv', G / 'rob/indoor1/gt.tum', OFFS['indoor1']),
    'indoor2': (G / 'rob/indoor2/imu0/data.csv', G / 'rob/indoor2/gt.tum', OFFS['indoor2']),
    'advio15': (G / 'rob/advio15/imu0/data.csv', G / 'rob/advio15/gt.tum', OFFS['advio15']),
    'advio20': (G / 'rob/advio20/imu0/data.csv', G / 'rob/advio20/gt.tum', OFFS['advio20']),
    'euroc_MH01': (ROOT / 'external/vio/data/MH_01_easy/mav0/imu0/data.csv', ROOT / 'runs/vio_compare/gt/MH_01_easy_body.tum', 0.0),
}


def q2R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def q2R_batch(q):
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rotvec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (0.5 if th < 1e-6 else th / (2 * np.sin(th)))


def load(name):
    """returns dict: ti (s, from IMU start), w, f (IMU), tg (GT time on the IMU clock minus t0), p, q, R (GT body->world)"""
    imu_f, gt_f, off = SEQ[name]
    im = np.loadtxt(imu_f, delimiter=',', comments='#')
    t0 = im[0, 0] * 1e-9
    ti = im[:, 0] * 1e-9 - t0
    g = np.loadtxt(gt_f)
    tg = g[:, 0] + off - t0
    o = np.argsort(tg)
    g, tg = g[o], tg[o]
    return dict(name=name, t0=t0, ti=ti, w=im[:, 1:4], f=im[:, 4:7], tg=tg, p=g[:, 1:4], q=g[:, 4:8], R=q2R_batch(g[:, 4:8]))


def gt_window(d, margin=1.0):
    """IMU-time span covered by GT"""
    return max(d['ti'][0], d['tg'][0]) + margin, min(d['ti'][-1], d['tg'][-1]) - margin


def gt_continuous_mask(tg, gap=0.35):
    """per GT sample: True where neighbours are within gap"""
    dt = np.diff(tg)
    m = np.ones(len(tg), bool)
    bad = np.where(dt > gap)[0]
    m[bad] = False; m[bad + 1] = False
    return m


def kabsch(A, B):
    """R minimising |B - A R^T| (rows are vectors): b = R a"""
    U, _, Vt = np.linalg.svd(B.T @ A)
    D = np.diag([1, 1, np.linalg.det(U @ Vt)])
    return U @ D @ Vt
