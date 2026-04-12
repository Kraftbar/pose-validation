# Pose Validation & SLAM Shootout

A lightweight SLAM benchmark repo comparing Python, C++, OpenCV-linked C, and library-free pure C variants on local monocular sequences.

## Key Implementations
- **`simple_slam.py`**: Python baseline using OpenCV and SciPy.
- **`simple_slam_opt.cpp`**: Optimized C++ port using OpenCV.
- **`simple_slam_c.c`**: Current standalone pure C implementation.
- **`simple_slam_c_brief.c`**: Promoted BRIEF pure C snapshot (`~363 LOC`), now tracked as a first-class benchmark variant.

## Current GT Tracking
The benchmark suite now tracks the full **4 GT datasets × 5 implementations = 20 runs** across:
- `test_freiburgxyz525`
- `test_freiburgrpy525`
- `test_freiburgroom525`
- `test_freiburgdesk525`

Current highlights:
- **`cpp`**: Best overall tradeoff, with **3 / 4 GT wins**.
- **`python`**: Best on `test_freiburgdesk525`.
- **`pure_c_brief`**: Best small-code pure C reference, runner-up on **3 / 4 GT datasets**.

The generated tracking artifacts live in `runs/benchmark/`:
- `gt_tracking.json`
- `gt_tracking.csv`
- `gt_tracking.md`

For the full tables and LOC-aware comparison, see [BENCHMARKS.md](BENCHMARKS.md).

## Quick Start
```bash
# Rebuild and benchmark all GT-backed datasets
python3 benchmark_native.py --all_gt --force
```

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
