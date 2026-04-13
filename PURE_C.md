# Truly Pure C SLAM

This implementation (`simple_slam_c.c`) is a 100% library-free monocular SLAM system. It does not link against OpenCV, BLAS, Eigen, or any external framework.

## Key Technical Features

### 1. Library-Free Computer Vision
- **Shi-Tomasi Corner Detector:** Custom structure tensor calculation with grid-based feature selection.
- **Pyramidal KLT Tracker:** 4-level Lucas-Kanade optical flow with sub-pixel bilinear interpolation.
- **Forward-Backward Consistency:** Robust tracking "sanity check" to prune noisy feature correspondences.
- **Gaussian Smoothing:** 3x3 blur implemented in C to stabilize detection and tracking.

### 2. Pure C SLAM Core
- **Local Bundle Adjustment (V12):** Joint-optimization of the last 3 keyframe poses and shared 3D map points.
- **Levenberg-Marquardt:** Robust non-linear solver with Huber loss for outlier rejection.
- **Global 3D Map:** Persistent storage of triangulated landmarks with observation tracking.
- **PnP Solver (DLT):** Perspective-n-Point pose estimation using a 12x12 linear system.
- **Geometric Loop Closure:** Patch-based NCC verification and trajectory "snapping."

### 3. Math & Infrastructure
- **Jacobi SVD Solvers:** Custom NxN eigensolvers for all geometric systems.
- **FFmpeg Pipeline:** High-performance raw video I/O via standard pipes.
- **Zero-Dependency Build:** Compiles with `gcc -O3 -fopenmp simple_slam_c.c -lm`.

## Benchmarks (April 2026 - 30s Freiburg)

| Sequence | Motion Type | ATE RMSE (V12) | Status |
| :--- | :--- | :--- | :--- |
| **Freiburg RPY** | Pure Rotation | **0.2826 m** | 🚀 Excellent |
| **Freiburg XYZ** | Linear Translation | **0.2405 m** | 🚀 Near-Reference |
| **Freiburg Room** | Wide Area | **1.7892 m** | ⚠️ Moderate |
| **Freiburg Desk** | Fast Rotation / Close-up | **1.5947 m** | ✅ Stable |

## Frozen Baseline

Current frozen universal `simple_slam_c.c` build, using the stronger PnP selection
and accepted LM pose refinement, is kept as the stable Pure C baseline.

| Sequence | Window | ATE RMSE | Notes |
| :--- | :--- | :--- | :--- |
| **Freiburg XYZ** | **5s** | **0.1461 m** | Best verified stable all-around build in this repo. |

## Recent Exploration

Recent post-baseline `pure_c` experiments were evaluated but not promoted into the
active source, because they either regressed outright or only produced marginal
single-sequence changes without a convincing all-GT win.

The recovery log and exploratory trial summary live in `PURE_C_RECOVERY.md`.
That note records:

- ideas that looked promising,
- trials that were measured and rejected,
- and the reasoning for keeping `simple_slam_c.c` on the restored healthy baseline.

## BRIEF Relocalization in `pure_c_brief`

The `pure_c_brief` variant (`simple_slam_c_brief.c`) gained an in-tree
BRIEF-256 relocalization path — still zero external dependencies. Each `MapPoint`
stores a 256-bit BRIEF descriptor computed from its birth keyframe. When fewer
than 50 tracked corners carry a map-point link in the current frame, unmatched
tracked corners are described and matched by Hamming distance against the map
(thresholds: `best < 35`, Lowe ratio `0.60`). Accepted matches set the corner's
`pt_idx` and flow into the existing PnP path.

Effect on the 30-second all-GT sweep (vs the prior `pure_c_brief` baseline):

| Sequence | Before | After | Δ |
|----------|--------|-------|---|
| `test_freiburgroom525` | 1.8414 | **1.7934** | **−0.0480** |
| `test_freiburgdesk525` | 0.7412 | 0.7410 | −0.0002 |
| `test_freiburgrpy525`  | 0.0980 | 0.0979 | −0.0001 |
| `test_freiburgxyz525`  | 0.1760 | 0.1788 | +0.0028 |

The room gain is the primary signal; other sequences move within the ~0.01 m
noise floor. The xyz delta is a small regression and cost `pure_c_brief` its
runner-up position on that sequence by 0.0006 m.

## Comparison with State-of-the-Art (SOTA)

Typical monocular ATE RMSE results for the Freiburg sequences:

| Sequence | ORB-SLAM2 (SOTA) | Truly Pure C (V12) | Notes |
| :--- | :--- | :--- | :--- |
| **fr1/xyz** | ~0.010 m | **0.240 m** | SOTA uses Global Bundle Adjustment. |
| **fr1/desk** | ~0.016 m | **1.594 m** | Desk features high-speed rotational blur. |

While SOTA systems achieve centimeter precision, they typically require >50,000 lines of C++ and heavy dependencies (OpenCV, Eigen, g2o). The **Truly Pure C** implementation achieves sub-meter performance in only **~320 lines** with **zero dependencies**.

## Final Version Summary

| Version | Feature / Tweak | Performance / Accuracy Gain |
| :--- | :--- | :--- |
| **V12** | **Local Bundle Adjustment** | **XYZ RMSE dropped to 0.24m** (62% gain). |
| **Frozen** | **Stronger PnP + accepted LM pose refinement** | **XYZ 5s RMSE improved to 0.1461m**. |
| **V11** | **Forward-Backward Tracking**| **Desk RMSE dropped to 1.59m** (53% gain). |
| **V9/V10**| **Robust LM + Loop Closure**| Stabilized trajectory; enabled global consistency. |
| **V7** | **PnP + Global Map** | Anchored trajectory to landmarks; prevented drift. |
| **V5/V6** | **Bilinear + Pyramids** | Sub-pixel precision; enabled high point density. |

## Build and Run

```bash
# Compile
gcc -O3 -march=native -fopenmp simple_slam_c.c -o simple_slam_pure_c -lm

# Run
./simple_slam_pure_c --video_path test_freiburgxyz525.mp4 --seconds 30 --metrics_out runs/pure_c_metrics.json
```
