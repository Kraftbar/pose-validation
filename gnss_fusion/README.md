# gnss_fusion: loosely-coupled GNSS + VIO/VO fusion in C99 (drones / phones)

Own code, MIT licensed (see the header of every file). A C re-implementation of `tools/gnss_loose_fusion.py` (4-DoF + scale pose-graph
smoother, numpy) plus robustness extensions. Library files include only `<stdint.h> <math.h> <stdlib.h> <string.h> <limits.h>`, no
external libraries (own block-tridiagonal Gauss-Newton, 5x5 Cholesky blocks). Clean room: nothing from GVINS / IC-GVINS / ORB-SLAM3 /
OpenVINS was read or used.

```
c/gf_geo.{h,c}      WGS-84 LLA <-> ECEF <-> local ENU
c/gf_fusion.{h,c}   fix + odometry input, causal sliding-window and batch smoother, robust loss, gating, consistency test, segments
c/gf_math.h         internal helpers (3x3, quaternion, 5x5 block Cholesky / block-Thomas)
c/gf_gait.{h,c}     gait (step cadence) speed prior from a phone IMU: step detector, cadence, speed model, still detector, online GNSS calibration
c/gf_run.c          command line driver (stdio, timers): text files in, TUM trajectory out, per-call timing
c/gf_gait_run.c     driver for gf_gait: IMU csv in, per-epoch cadence / speed / state out
c/Makefile          cc -std=c99 -O2 -Wall -Wextra -pedantic (no warnings)
tools/              python validation: compare_py.py (C vs python), gf_table.py, gf_studies.py, test_geo.py, gf_cases.py, run_all.sh;
                    gait.py (python prototype of gf_gait), check_gait.py (C vs python), gf_gait_study.py + gf_gait_report.py (section 12)
```

## Model

Odometry (VIO / VO) is a pose stream in its own frame. A node is created every `node_dt` s (first odometry sample at or after the grid
time). State per node `x = (psi, p, s)`: yaw of the odometry frame w.r.t. ENU, IMU position in ENU, scale. Gravity/roll/pitch are trusted
from the odometry (or from a gravity vector you supply, `gf_set_gravity`).

| factor | residual |
|---|---|
| odometry link i->j | `Rz(-psi_i)(p_j - p_i) - s_i (pL_j - pL_i)`, sigma `0.05 m + 0.02 |d|` |
| yaw / scale random walk | `psi_j - psi_i` (0.5 deg/sqrt(s)), `s_j - s_i` (0.003/sqrt(s)); weak prior `s ~ 1` |
| GNSS fix (antenna lever arm `rsa`) | `p_i + Rz(psi_i) a_i - z_i`, sigma (sh, sh, sv), Huber / Cauchy per axis (IRLS) |
| optional GNSS velocity | `Rz(psi_i) s_i dL_i/dt - v_i` |
| optional gait speed (`gf_add_speed`) | `s_k |dL_h| / T - v` on the scale state (Huber); `|dp_h| / dt - v` on bridged links; `p_j - p_i = 0` when stationary |

Causal mode: after every new node (and every late fix) the last `window_s` = 30 s of nodes are re-linearised (4 Gauss-Newton
iterations), older nodes are frozen. Batch mode: the whole graph, 25 iterations. The initial alignment is a 2D Procrustes (yaw, scale,
translation) on the fixes of the first `init_wait_s` = 30 s (causal; before that the odometry is passed through with `GF_ST_INIT`
clear) or on all fixes (batch).

Extensions that the python reference does not have:

* **Segments** (b): an odometry gap longer than `gap_s`, `GF_ODOM_GAP`, or `GF_ODOM_NEW_FRAME` (VO new map / re-initialisation, unrelated
  yaw / scale / origin) breaks the odometry chain. Position is bridged with a weak link (`1 m + link_speed * dt`), a same-frame gap keeps the
  yaw/scale random walk, a new frame is aligned from its own fixes once it spans `seg_min_extent` (provisional state until then,
  `GF_ST_SEG_PROVISIONAL`). Monocular frames get their own unit.
* **Consistency test** (a), `trust=1`: in a sliding window (30 s) the odometry positions are fitted to the fixes with a trimmed 4-DoF + scale
  similarity. A metric odometry whose window scale leaves [1/1.5, 1.5], a window residual far above the GNSS noise (estimated from second
  differences, capped by the reported sigma), a long-window (120 s) residual above 8 m, or (monocular) a scale jump between the left and right
  half window marks the stretch as distrusted: its odometry links are replaced by the weak position link, so those nodes are bridged by the
  fixes only (smoothed GNSS) and the output between nodes is interpolated instead of propagated with the odometry.
* **chi2 gating** (a), `gate_chi2`: a fix whose normalised residual (sigma inflated by `gate_floor` = 1 m, since the prediction is not exact) is
  above the gate is dropped from the solve; fixes are never gated in a segment with fewer than 5 fixes or when more than half of the window
  would be gated (then the model, not the fixes, is wrong). `robust_init` trims outliers in the initial alignment.
* `gf_config_robust()` sets `trust=1, trust_long_s=120, gate_chi2=16.27, robust_init=1` and a free monocular scale (`scale_rw_mono=0.05`,
  `scale_prior_mono=2`), plus the section-11 features below. The defaults of `gf_config_default()` reproduce the python reference (all new
  features are off there; `preset=robust1` in `gf_run` = the first robust preset without them).
* **Section-11 features** (all C-only, in `gf_config_robust()`): `trust_state_k=1.6` (window similarity scale vs the scale state outside
  [1/k,k] marks the odometry as distrusted: monocular scale collapse / re-scale); `scale_min/max=0.25/4` (scale state cannot flip sign or collapse);
  `trust_start_m=3` (short consistency fit starts from the best contiguous sub-set, so a multipath burst of common-offset fixes does not
  look like bad odometry); `grow_s=120, grow_ratio=20` (metric odometry: no frozen nodes at the start until the fixes span 20x their reported sigma or
  120 s, so yaw / scale are not locked by a poor first alignment, e.g. a long stationary start); `gnss_only_nodes=1` (while the odometry is silent for more than
  `gap_s`, each fix makes a GNSS-only node, `gf_get_pose` returns it with `GF_ST_NO_ODOM`, `gf_node_pose(i)` gives retained ones).

`sigma_h` in `gf_pose` is a heuristic (reported sigma of the last fixes / sqrt(#fixes in the window), grown by `drift_rate` = 2 % of the path
since the last accepted fix), not a marginal covariance.

## Gait speed prior (section 12 of the study; off by default)

`gf_gait` turns the phone accelerometer + gyro into a walking-speed measurement per 3 s epoch (6 s window): `v = k c cad^2`, `c = 0.389` generic
(0.70 m step at 1.8 Hz), a per-user `c` from a held-out sequence (`gf_gait_set_model`), or `k` from the GNSS fixes (`gf_gait_gnss_fix`, did not help
with phone fixes). States WALK / STATIONARY / OTHER (shuffling, running: no measurement). `gf_add_speed()` feeds the measurement to the smoother
(`cfg.speed_on = 1`, `speed=1` in `gf_run`): a factor `s_k * (horizontal odometry path / duration) = v` on the per-node scale state (also across distrusted links),
a horizontal-displacement form on bridged / GNSS-only links, a zero-velocity factor for STATIONARY, and with `speed_align=1` the alignment of a frame
without fixes from the speed alone (indoors, no GNSS: metric trajectories from mono VO; Indoor-1/2 SE3 11 m -> 0.3-0.7 m).

```c
#include "gf_gait.h"
gf_gait *gt = gf_gait_create(NULL);                 /* defaults = generic population model */
gf_gait_set_model(gt, c_user);                      /* optional per-user calibration */
/* every IMU sample: */ gf_gait_push(gt, t, acc /* specific force m/s^2 */, gyro);
/* every 3 s: */ gf_gait_est e; gf_gait_estimate(gt, t, 6.0, &e);
if (e.state == GF_GAIT_WALK)            gf_add_speed(g, e.t, e.speed, e.sigma, e.window_s, 0);
else if (e.state == GF_GAIT_STATIONARY) gf_add_speed(g, e.t, 0.0, e.sigma, e.window_s, GF_SPEED_STATIONARY);
```
`gf_gait_run --imu imu.csv --epochs ep.txt` is the command line version; `gf_run --speed file speed=1 speed_align=1 speed_scale_rw_rel=1` takes lines
`t v sigma window [flags]`. Opt-in additions of the phone pipeline (`phone_pipeline/`): odometry flag `GF_ODOM_LOOSE` (4; the link into the next node gets its position sigma multiplied by `loose_k`, default 1 = no effect; used for rotation-only / extrapolated stretches) and `speed_align_metric=1` (monocular speed alignment tests `seg_min_extent` on the metric path; default 0).
Config keys: `speed speed_k speed_sigma_scale speed_link_sigma zupt_sigma speed_align speed_scale_rw_rel speed_scale_lim speed_scale_rw`.
Limits (section 12.8): walking only (cadence 1.0-2.8 Hz), step length is person dependent (ADVIO-20 person 8 % above the population constant), scale only (no
yaw), scale that decays continuously or whole-map collapses are only partly rescued, causal zero-velocity limited by the measurement cadence.

## API (see `c/gf_fusion.h`)

```c
#include "gf_fusion.h"
#include "gf_geo.h"

gf_config cfg; gf_config_default(&cfg);
cfg.rsa[0] = -0.01; cfg.rsa[1] = -0.03; cfg.rsa[2] = -0.06;     /* GNSS antenna in the IMU frame */
gf_config_robust(&cfg);                                          /* optional */
gf_t *g = gf_create(&cfg, /*causal=*/1);

gf_enu_frame enu; gf_enu_frame_init(&enu, lat0, lon0, h0);       /* LLA -> local ENU for the fixes */
/* odometry (any rate, times increasing); first sample of every frame, if not gravity aligned: gf_set_gravity(g, up_in_odom_frame) */
gf_add_odom(g, t, p, q_xyzw, 0);                                 /* flags: GF_ODOM_NEW_FRAME, GF_ODOM_GAP */
gf_fix f = {0}; f.t = t_fix; gf_lla_to_enu(&enu, lat, lon, h, f.p); f.sigma_h = 3.0; f.sigma_v = 6.0;
gf_add_fix(g, &f);                                               /* before or after the odometry of that time */
gf_pose pose; gf_get_pose(g, &pose);                             /* latest fused pose: p, q, yaw, scale, sigma_h, status */
/* batch: gf_create(&cfg, 0); add everything; gf_solve_batch(g); gf_query(g, t, p_odom, q_odom, &pose) per sample */
gf_destroy(g);
```

Memory: causal mode with `keep_history = 0` keeps about `window_s / node_dt + 8` .. 4x that many nodes (about 400 bytes each), no allocation
per call after warm-up. Not thread safe per object; no globals.

## Build and run

```
make -C gnss_fusion/c
gnss_fusion/c/gf_run --odom odom.txt --fix fixes.txt --out fused.tum --mode causal preset=robust rsa=-0.01,-0.03,-0.06 --timing
```
Input formats are documented at the top of `c/gf_run.c` (TUM odometry with optional flags column, `t E N U sh sv [vx vy vz sv]` fixes or
`--fix-lla`).

## Validation (needs `external/gnss`, the data of the GNSS-VIO study; python with numpy, e.g. `external/gnss/venv/bin/python`)

`tools/run_all.sh` rebuilds and runs everything (including `check_gait.py` and the gait study); outputs go to `work/` (git-ignored). Scoring uses `tools/gnss_harness/gnss_eval.py`
(`benchmark.umeyama_alignment`), nothing re-implemented. Results, differences to the python reference and open issues:
`docs/gnss_vio_benchmark_20261001.md`, sections 10 (C library), 11 (robustness fixes), 12 (gait speed prior).
