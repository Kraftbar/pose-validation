# Native `simple_slam` ports

The repository now includes two native variants:

- `simple_slam_opt.cpp` — optimized C++ port of the headless `simple_slam.py` core.
- `simple_slam_c.c` — C baseline implementation of the same rough pipeline,
  built through a small C ABI shim in `simple_slam_c_shim.cpp` for OpenCV I/O,
  resize, corner extraction, and SVD.

Why the C shim exists: OpenCV 4 no longer provides a fully usable pure-C build
surface for the needed functionality, so the `.c` implementation keeps its core
logic in C and calls a minimal C-compatible bridge for the OpenCV-dependent parts.

## Build

```bash
cmake -S . -B build-native
cmake --build build-native -j
```

## Run

```bash
./build-native/simple_slam_opt \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 1 --timeout 20 \
  --metrics_out runs/_smoke_simple_slam_opt.json

./build-native/simple_slam_c \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 1 --timeout 20 \
  --metrics_out runs/_smoke_simple_slam_c.json

# Compare Python vs C++ vs C and report speedup vs Python
python benchmark_native.py \
  --video test_freiburgxyz525 \
  --seconds 5 --timeout 60

# Outputs:
# - runs/benchmark/native_comparison_5s.json
# - runs/benchmark/native_comparison_5s.csv
```

## Current scope

- `simple_slam_opt`: uses OpenCV ORB + BF matching + PnP + essential-matrix fallback + triangulation.
- `simple_slam_c`: uses a simpler C baseline with patch-matched corners, essential-matrix pose recovery, and triangulation.
- Both support the core headless benchmark path and emit JSON metrics compatible with the local tooling.
- Neither native version includes plotting, threaded UI mode, or SciPy-based bundle adjustment yet.
