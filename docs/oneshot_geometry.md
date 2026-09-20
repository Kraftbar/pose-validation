# Geometry-core one-shot (2026-09-19)

Status: **experimental, not promoted**. Branch `feat/oneshot-geometry-core`.
The default tracker is unchanged; `--geometry_core` enables the numerical
corrections. Two more extensive tracker rewrites were rejected and removed
from the active source.

## What was found

The essential-matrix implementation assumes descending singular values and
uses column 2 of U as the null-space direction, while `svd_3x3` provides neither
sorted values nor an orthonormal completion for a rank-deficient input.
`enforce_essential_constraints` also multiplies by V instead of V transpose.
An already valid synthetic E fails the legacy projection-invariance test.

The camera-coordinate pose Jacobian differentiates a left perturbation of
both R and t, but the legacy update rotates only R. The new retraction applies
`R' = exp([w]x) R`, `t' = exp([w]x) t + v`. Its Jacobian passes finite
differences, and refinement converges on a synthetic camera with a distant
world origin.

The opt-in path also replaces the fixed 100-rotation DLT eigensolver with
converged cyclic Jacobi sweeps, normalizes each six-point world sample, removes
duplicate sample indices, resolves the projective sign through the rotation
determinant, and scores the resulting rigid pose. The old sign convention uses
translation.z, which is not constrained to be positive. A synthetic PnP case
with negative translation, a distant world origin, and 20% outliers fails the
legacy support assertion and passes with the corrected path.

These corrections are a coherent numerical experiment, not an ablation that
attributes the live result to a single fix. They do not solve initialization,
map consistency, or recovery after lost tracking. The existing smoother and
map/backend policy remain active with `--geometry_core`.

## Full validation and reproducibility

This checkout initially lacked `runs/benchmark/`, three GT sequences, Python
OpenCV, and native OpenCV development files. The missing sequences were
restored from `geohot/twitchslam` at
`c52a14fe1034426b6dfc1e6222984a9a06e40b6a`.
Dependencies were installed in an isolated Python environment and an extracted
Ubuntu OpenCV prefix, without system package installation.

Three clean, serial `python3 benchmark_native.py --all_gt --force` sweeps,
each with a dedicated `--out_dir`, completed all 28 runs:

| Sweep | Artifact directory |
|-------|--------------------|
| Original HEAD, separate source snapshot | `runs/oneshot/baseline/full_gt/` |
| Final source, default mode | `runs/oneshot/default_full_gt/` |
| Final numerical experiment | `runs/oneshot/candidate_full_gt/` |

The wrapper does not forward implementation-specific arguments. The last
sweep used `runs/oneshot/candidate_source/`, an isolated source copy with only
the `geometry_core` config default set to 1. Its four `pure_c_plus` timelines
also exactly match the earlier candidate diagnostic build. All final default
timelines, including every `xyz`, `raw_xyz`, method and map count, exactly
match the original-HEAD baseline.

Environment: GCC 13.3.0, native OpenCV 4.6.0, `OMP_NUM_THREADS=4`,
`OPENBLAS_NUM_THREADS=1`, standard 30-second/120-second-timeout settings.
The local environment activation is saved in `runs/oneshot/environment.sh`;
its dependency paths are under `/tmp` and must be recreated if removed.
The complete comparison is `runs/oneshot/comparison.json`. All ATE values were
calculated with `benchmark.ate_rmse` and matched frame IDs.

The fresh original-HEAD `pure_c_plus` mean is **0.6543 m**, not the historical
README value of 0.5050 m. C++ reproduces the historical table to four decimal
places. The cause of the pure-C difference has not been isolated. This
experiment therefore compares against the fresh same-machine baseline, and
does not replace the historical canonical table. Timings are not compared:
the final two serial sweeps overlapped in wall-clock time.

## Results

All runs processed 613/750/723/750 frames for desk/room/rpy/xyz respectively.
Reported ATE includes the existing output smoother; raw ATE uses `raw_xyz`.

| Sequence | Default ATE | Geometry ATE | Default raw ATE | Geometry raw ATE | PnP frames, default → geometry |
|----------|-------------|--------------|-----------------|------------------|--------------------------------|
| desk | 0.6916 | 0.7500 | 0.7500 | 0.7441 | 335 → 457 |
| room | 1.6539 | 1.4635 | 1.7786 | 1.5248 | 219 → 549 |
| rpy | 0.0972 | 0.0956 | 0.0993 | 0.0958 | 67 → 531 |
| xyz | 0.1747 | 0.1761 | 0.1750 | 0.1755 | 272 → 632 |
| mean | **0.6543** | **0.6213** | **0.7007** | **0.6351** | |

Map points move from 11344/17040/16001/9942 to
15972/21316/20397/13271. There is no hidden frame truncation or map-density
collapse in this numerical candidate. Nevertheless, the 0.0585 m reported desk
regression prevents treating the mean improvement as a clean replacement.
Keep it opt-in.

## Rejected structural rewrites

The rewrites used persistent first observations for landmark birth,
reprojection and parallax checks against the actual accepted world poses,
PnP tracking with inlier refinement, and E only for bootstrap. They exported
raw centers directly and did not run the old BA/loop/smoothing path. These
were `pure_c_plus`-only 30-second diagnostics, not canonical sweeps.

| Variant | desk ATE | room ATE | rpy ATE | xyz ATE | Failure |
|---------|----------|----------|---------|---------|---------|
| Persistent-track map (`geometry_v4`) | 0.6645 | 1.8685 | 0.0998 | 0.1593 | Last accepted PnP at frames 199/224/12/395; then held pose |
| Stricter bootstrap + descriptor relinking (`geometry_v5`) | 0.7294 | 1.8428 | 0.0990 | 0.1720 | Delayed initialization and failed recovery; rejected |

Sources, binaries and traces remain under `runs/oneshot/geometry_v4/` and
`runs/oneshot/geometry_v5/`. Holding a pose still emits frames, so frame-count
completion alone must not be interpreted as successful tracking.

The earlier numerical ablations are also saved: `geometry_room.json` changes
E/SVD and pose updates only; `geometry_v2/` adds eigensolver convergence;
`geometry_v3/` adds normalized, rigid-scored PnP and is the retained opt-in
candidate. These exploratory outputs did not overwrite canonical summaries.

## Reproduce and continue

```bash
# Numerical regression tests, including rank-zero/rank-one/rank-two SVD,
# full eigensystems, finite-difference pose Jacobian, two-view recovery,
# outlier PnP and large-origin pose refinement.
gcc -O2 -fopenmp tools/test_plus_geometry.c -lm -o /tmp/test_plus_geometry
/tmp/test_plus_geometry

# Opt-in diagnostic across all four GT sequences.
python3 benchmark.py --all_gt --impl pure_c_plus --force \
  --extra_args='--geometry_core' --out_dir runs/oneshot/recheck

# Required full validation of the active defaults.
python3 benchmark_native.py --all_gt --force \
  --out_dir runs/oneshot/default_recheck
```

The numerical tests also pass AddressSanitizer and UndefinedBehaviorSanitizer
with leak detection enabled.

Another focused attempt is warranted: the corrected geometry substantially
increases PnP support on room and improves its raw trajectory. Start from the
tested opt-in path, diagnose desk's remaining map/pose feedback, and establish
bootstrap and recovery coverage before replacing the tracking lifecycle.
Judge both reported and raw ATE on every sequence, plus periods without an
accepted pose. The failed rewrites show why bounded scale alone is insufficient.
