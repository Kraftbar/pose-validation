# Pose Validation & SLAM Shootout

A lightweight SLAM benchmark repo comparing Python, C++, OpenCV-linked C, and library-free pure C variants on local monocular sequences.

## Key Implementations
- **`simple_slam.py`**: Python baseline using OpenCV and SciPy.
- **`simple_slam_opt.cpp`**: Optimized C++ port using OpenCV.
- **`simple_slam_c.c`**: Current standalone pure C implementation (~1,300 LOC, ~1,260 code-only).
- **`simple_slam_c_brief.c`**: Promoted BRIEF pure C snapshot (~1,300 LOC, ~1,260 code-only).
- **`simple_slam_c_orb.c`**: Full library-free ORB pipeline (~1,440 LOC, ~1,400 code-only) with scale pyramids, FAST-9, ORB descriptors, PnP, and Motion-only BA.
- **`simple_slam_c_plus.c`**: Pure C with local BA + loop closure (~1,420 LOC, ~1,365 code-only) — currently the best pure-C by mean ATE.

LOC counts above reflect the current "math-on-paper" layout (one statement per line,
matrices laid out as grids). Earlier history notes reference the pre-format compact
counts (~1/3 the current size) — those are not directly comparable.

## Current GT Tracking
The benchmark suite tracks the full **4 GT datasets × 7 implementations = 28 runs** across:
- `test_freiburgxyz525`
- `test_freiburgrpy525`
- `test_freiburgroom525`
- `test_freiburgdesk525`

Current highlights (30s canonical run):
- **`cpp`**: Best overall, **3 / 4 GT wins** (room, rpy, xyz).
- **`python`**: Best on `test_freiburgdesk525`.
- **`pure_c_plus`**: Best pure-C by mean ATE (0.691 m), runner-up on room and xyz. Beats `pure_c_brief` on 3 / 4 sequences.
- **`pure_c_brief`**: Best pure-C on desk only (0.720 m); otherwise ranks 3rd–4th among pure-C variants.

The generated tracking artifacts live in `runs/benchmark/`:
- `gt_tracking.json`
- `gt_tracking.csv`
- `gt_tracking.md`

For the full tables and LOC-aware comparison, see [BENCHMARKS.md](BENCHMARKS.md).

## Development Philosophy

This repo is built in stages. The goal is not to beat ORB-SLAM2 or RTAB-Map
directly — those are 30K-LOC production systems, with depth sensors, DBoW
vocabularies, and years of tuning. The goal is to build a small monocular
SLAM stack from scratch, in plain C with no external dependencies, where
every algorithmic step is visible and modifiable.

The project favors:

- **Small, readable implementations.** Math should look like the math on paper:
  matrices laid out as grids, one equation per line, references to the source
  texts (Hartley & Zisserman, etc.) where appropriate.
- **Correctness before performance.** A formatting pass or refactor is
  acceptable churn if it leaves behavior unchanged (verified by `--all_gt`).
  Speed work comes after the algorithm is right.
- **Benchmarks on real GT, not synthetic toys.** All claims are anchored to
  the four TUM Freiburg sequences with absolute trajectory error — see
  `BENCHMARKS.md`.
- **Gradual expansion.** Feature ladder is documented in
  `BENCHMARKS.md` "Improvement roadmap": local BA → global BA → loop closure
  → pose-graph optimization. Pick one structural change at a time, validate,
  promote.

The project does **not** optimize only for lowest line count. After a brief
"truly pure C" phase that produced unreadable single-statement files, the
codebase was reformatted to math-on-paper style; LOC roughly tripled and the
algorithms became much easier to reason about. Code-golf wins are welcome
once a feature works, never as a substitute for clarity.

Benchmarks are guidance, not a scoreboard. We do not chase ATE numbers by
hiding behavior behind tuned thresholds, dataset-specific magic constants, or
silently skipping frames. The roadmap targets — like ~0.09 m on `fr1/xyz`
with global BA — are stated up-front so the cost (in LOC and complexity) is
honest.

## Quick Start
```bash
# Rebuild and benchmark all GT-backed datasets
python3 benchmark_native.py --all_gt --force
```

## Frame-by-Frame GT Plots
For paper-style error curves, use the saved per-run metrics JSON plus the GT `.npz`.

```bash
# Compare a few implementations on the xyz sequence
python3 tools/plot_frame_errors.py \
  --gt test_freiburgxyz525.npz \
  --output runs/plots/test_freiburgxyz525_compare.svg \
  --csv runs/plots/test_freiburgxyz525_compare.csv \
  python=runs/benchmark/test_freiburgxyz525.json \
  cpp=runs/benchmark/test_freiburgxyz525_cpp.json \
  pure_c_orb=runs/benchmark/test_freiburgxyz525_pure_c_orb.json
```

This produces an SVG with per-frame translation error, plus rotation error when the trace contains rotations. Keep generated plots under `runs/plots/`. For single-run diagnosis and worst-frame inspection, keep using `tools/diagnose_trace.py`.

## Pure C Binaries
```bash
# Current pure C
gcc -O3 -march=native -fopenmp simple_slam_c.c -o simple_slam_pure_c -lm

# BRIEF pure C snapshot
gcc -O3 -march=native -fopenmp simple_slam_c_brief.c -o simple_slam_pure_c_brief -lm
```

## Dataset Notes
Top-level videos cover the standard local datasets. The extra Freiburg GT-backed sequences are discovered from a local checkout under `external/twitchslam/videos/` when present.

For technical details on the library-free implementations, see [PURE_C.md](PURE_C.md). For benchmark tables and historical comparisons, see [BENCHMARKS.md](BENCHMARKS.md).
