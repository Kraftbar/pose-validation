# Pose Validation & SLAM Shootout

A lightweight SLAM system featuring multiple implementations ranging from Python prototypes to highly optimized, library-free C code. This project serves as a "shootout" to compare accuracy and performance across different levels of architectural density.

## Key Implementations
- **`simple_slam.py`**: The original Python baseline using OpenCV and SciPy.
- **`simple_slam_opt.cpp`**: Optimized C++ port using OpenCV.
- **`simple_slam_c.c`**: The **"Truly Pure C"** engine. A 100% library-free implementation (no OpenCV, Eigen, or BLAS) featuring sub-pixel KLT, PnP, and Local Bundle Adjustment in only **~320 lines of code**.

## Benchmarks (Freiburg XYZ Sequence)
| Implementation | Libraries | ATE RMSE (30s) |
| :--- | :--- | :--- |
| OpenCV-Linked C | OpenCV | 0.1627 m |
| Python Baseline | OpenCV, SciPy | 0.1787 m |
| **Truly Pure C (V12)** | **None** | **0.2405 m** |

## Quick Start (Truly Pure C)
```bash
# Compile
gcc -O3 -march=native -fopenmp simple_slam_c.c -o simple_slam_pure_c -lm

# Run
./simple_slam_pure_c --video_path video.mp4 --seconds 30 --metrics_out results.json
```

For technical deep-dives into the library-free architecture, see [PURE_C.md](PURE_C.md). For detailed results across all datasets, see [BENCHMARKS.md](BENCHMARKS.md).
