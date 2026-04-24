# Rejected Python Trials

Log of python-impl experimental branches that were evaluated against the
canonical GT suite and rejected. Kept so future agents don't retry the same
change without new evidence. Baselines below use `simple_slam.py` run via
`benchmark.py --all_gt --impl python --seconds 30`.

Archived sources live in `runs/archive_v1xx/`.

## Canonical baseline (simple_slam.py, 30s)

| sequence | ATE (m) | frames | points |
| --- | --- | --- | --- |
| test_freiburgdesk525 | 0.6842 | 613 | 3143 |
| test_freiburgroom525 | 1.8659 | 750 | 1895 |
| test_freiburgrpy525  | 0.0982 | 723 | 2875 |
| test_freiburgxyz525  | 0.1784 | 750 | 2657 |

## v125 — Improved BA + Keyframe Culling + Baseline-aware Tri
5s desk ATE 0.338 (vs baseline 0.399 at 5s, 0.684 at 30s). Not re-run at 30s;
superseded by later iterations and never beat v127's 5s number.

## v126 — Local Windowed BA + Sub-pixel refinement
5s desk ATE 0.333. Same status as v125 — no 30s evidence of improvement.

## v127 — LK Optical Flow Tracking
Promising at 5s (desk 0.192 vs baseline 0.399), but **ATE-neutral across
all 4 GT sequences at canonical 30s**:

| sequence | baseline 30s | v127 30s | Δ | points Δ |
| --- | --- | --- | --- | --- |
| desk | 0.6842 | 0.6890 | +0.005 | 3143 → 524 |
| room | 1.8659 | 1.8658 | ~0    | 1895 → 751 |
| rpy  | 0.0982 | 0.0982 | 0      | 2875 → 536 |
| xyz  | 0.1784 | 0.1785 | ~0    | 2657 → 873 |

All deltas within the 0.01 m neutral band per the AGENTS.md Promotion Rule,
with a 3–5× map-density drop — the "silent regression" signal the rule warns
about. **Rejected.** The 5s win decays as more frames accumulate.

## v128 — Motion Model Guided Tracking
Broken on Freiburg: produces 0 points at 5s on desk. **Rejected.**

## v129 — SIFT + Final Global BA
Hangs on desk: killed after 193 CPU-minutes / 3 min wall-clock on a nominal
5-second run. The `--timeout 120` arg is not enforced inside the Final Global
BA path. **Rejected** (does not complete on Freiburg at any useful length).

Also note: the README previously claimed `v129` held the Desk record. That
claim was false — v129 does not finish on desk. Removed from README.

## v130 — High-Quality ORB + Final Global BA
Same failure mode as v129: hangs past the 3 min wall-clock cap on a 5s desk
run. **Rejected.**
