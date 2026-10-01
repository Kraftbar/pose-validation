# Monocular SLAM comparison on original TUM RGB-D (2026-09-25)

Scorer: `tools/tum_eval.py` (Sim3 ATE via `benchmark.ate_rmse`, TUM
timestamp association ≤0.02 s). Data: original TUM RGB-D PNG sequences.
Raw: `runs/tum_compare/table.md`, candidate research `runs/tum_compare/CANDIDATES.md`.
ATE-tracked = frames with a pose; coverage = frames with a pose / all frames.
Repo implementations read a lossless FFV1 re-encode of the same images and use
their built-in (fr1-like / heuristic) intrinsics — no real calibration.

## ATE-tracked (m) / coverage

| System | License | fr1_xyz | fr1_desk | fr1_floor | fr2_xyz | fr3_long_office | mean ATE | RTF* |
|---|---|---|---|---|---|---|---|---|
| **stella_vslam** (LC on) | BSD-2 | 0.025 / 99% | 0.020 / 88% | **0.022 / 98%** | **0.004 / 98%** | 0.022 / 99% | **0.019** | 0.4–0.55 |
| stella_vslam (LC off) | BSD-2 | 0.014 / 100% | 0.045 / 91% | 0.173 / 88% | 0.005 / 98% | 0.050 / 99% | 0.057 | 0.4–0.55 |
| ORB-SLAM2 upstream (1-thread, no LC) | GPLv3 | 0.011 / 72% | 0.014 / 64% | never inits | 0.018 / 100% | 0.034 / 99% | 0.019 (4 seqs) | 0.4–4.1 |
| ORB-SLAM2 C port (ours, isolated) | GPLv3-derived | 0.011 / 72% | 0.014 / 64% | never inits | 0.012 / 100% | 0.028 / 99% | 0.016 (4 seqs) | 2.8–6.2 |
| stella_vslam C port (ours, 1-thread deterministic) | BSD-2 / MPL-2.0 | 0.026 / 99% | 0.018 / 89% | 0.022 / 97% | 0.019 / 99% | 0.037 / 99% | 0.024 | 0.37–0.43 |
| stella_vslam (1-thread deterministic reference) | BSD-2 | 0.026 / 99% | 0.018 / 89% | 0.022 / 97% | 0.019 / 99% | 0.037 / 99% | 0.024 | 0.42–0.48 |
| DSO (no photometric calib) | GPLv3 | 0.063 / 18% | 0.211 / 24% | 0.258 / 17% | 0.020 / 3% | 0.089 / 19% | 0.128 | 0.3–1.3 |
| cpp (repo) | own | 0.183 / 100% | 0.681 / 100% | 0.533 / 100% | 0.364 / 100% | 1.336 / 100% | 0.619 | 0.3–0.5 |
| pure_c_plus (repo) | own | 0.184 / 100% | 0.759 / 100% | 0.741 / 100% | 0.344 / 100% | 1.884 / 100% | 0.783 | 0.8–1.25 |

Update 2026-10-01: the pure-C stella port (`stella_port/`) reproduces the
deterministic single-threaded stella reference byte-for-byte on all five
sequences (identical trajectories, so identical ATE); RTF is from
`sv_run` on pre-decoded frames (12.5–14.3 ms/frame) vs the reference with
PNG decode (13.9–15.9 ms/frame). The deterministic rows differ from the
multi-threaded stella_vslam row because mapping runs synchronously.
See `stella_port/HANDOVER.md`.

Other repo impls (python, c, pure_c, pure_c_brief, pure_c_orb): mean 0.82–0.86 m, 100% coverage.
*RTF = wall time / sequence duration (<1 = faster than real time). stella_vslam and DSO
are multi-threaded optimized builds; the ORB-SLAM2 rows are the single-threaded,
deterministic reference setup, so their runtime is not representative of ORB-SLAM2.
stella_vslam/DSO are nondeterministic: median-ATE run of 3 shown.

Not built: ORB-SLAM3, LDSO, DSM, SVO, OV²SLAM, PL-SLAM (GPL or stricter and/or
unmaintained; see CANDIDATES.md). ORB-SLAM3 would be the natural accuracy
reference but is GPL (reference only).

## Findings
- Keyframe/BA-based feature SLAM is 10–60× more accurate than every repo
  implementation (1–3 cm vs 0.2–1.9 m) on these sequences.
- stella_vslam matches ORB-SLAM2 accuracy AND fixes its coverage problems
  (initializes on fr1_floor, 88–99% coverage everywhere) and runs faster than
  real time. Loop closing matters on fr1_floor/fr3 (0.17→0.02, 0.05→0.02).
- DSO without photometric calibration is not competitive on TUM RGB-D.

## Recommendation
Port **stella_vslam** next (BSD-2, ~30k core LOC, same algorithm family as the
validated ORB-SLAM2 port, so the dump-and-compare method and most exactness
lessons carry over). Before any porting:
1. **License provenance audit.** OpenVSLAM (stella_vslam's origin) was withdrawn
   in 2021 over similarity to ORB-SLAM2 code; stella_vslam continued after
   rework. Verify the current code's provenance and its dependencies'
   licenses (g2o core BSD but optional LGPL parts; CSparse LGPL; FBoW + its
   ORB vocabulary file) before treating a port as permissive. The port must be
   written from stella_vslam's source only — never from our GPL ORB-SLAM2 port.
2. Decide scope: tracking + local mapping first (no-LC is 0.057 m mean), loop
   closing second (needed for 0.019 m).
