# Truly Pure C SLAM

`simple_slam_c.c` is the standalone pure-C monocular SLAM core in this repo. Tracking, geometry, optimization, and map management live in one C translation unit with no OpenCV, Eigen, BLAS, or g2o dependency inside the core itself. Video decode is handled through the `ffmpeg` CLI.

## Current Feature Set

### Front-End
- **Bucketed Shi-Tomasi corners:** Keeps features spatially distributed instead of clustering on a few strong regions.
- **Pyramidal LK tracking:** 4-level Lucas-Kanade with bilinear interpolation and a forward-backward check.
- **Motion-seeded LK:** Uses the previous frame's mean optical flow as the next LK initialization.
- **Lightweight preprocessing:** 3x3 blur in C before detection and tracking.

### Geometry and Mapping
- **Hartley-normalized 8-point estimation:** More stable essential matrix fitting than the earlier raw normalized-coordinate DLT.
- **DLT PnP + LM refinement:** Uses mapped landmarks when available, then refines pose with a Huber-weighted optimizer.
- **Adaptive keyframes:** Inserts keyframes from track-ratio loss, low inlier count, rotation since the last KF, or age.
- **Map growth at every KF:** Re-observed landmarks get their observation count bumped and unmatched tracked pairs can triangulate new points.
- **Map-point culling + compaction:** Weak or stale landmarks are removed and the map is compacted so live point count can be capped.

### Optimization and Loop Handling
- **Local BA:** Alternating pose/point refinement over the latest `BA_WINDOW` keyframes.
- **O(window) point accumulation:** Avoids the older map-growth blow-up from repeatedly scanning the full map per point.
- **Patch-based loop verification:** Thumbnail retrieval + NCC matching + essential-matrix pose recovery.

## Useful Runtime Flags

- **`--kf_min_inliers`:** Marks pose estimation as weak when inliers fall below this threshold.
- **`--kf_max_rot_deg`:** Forces a keyframe when rotation from the last KF grows too large.
- **`--max_points`:** Caps the live map after culling/compaction.
- **`--metrics_out`:** Writes timeline and summary JSON for benchmarking.

## Build and Run

```bash
# Standalone pure-C build of the core file
gcc -O3 -march=native -fopenmp simple_slam_c.c -o simple_slam_pure_c -lm

# Native project build used by benchmark.py
cmake --build build-native -j4 --target simple_slam_c

# Short sanity run
./build-native/simple_slam_c \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 5 --timeout 30 \
  --metrics_out runs/pure_c_iter/dbg.json

# Example with an explicit live-map cap
./build-native/simple_slam_c \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 30 --timeout 120 \
  --kf_min_inliers 20 \
  --kf_max_rot_deg 5 \
  --max_points 15000 \
  --metrics_out runs/pure_c_metrics.json
```

## Benchmarks

The canonical Freiburg and comparison tables live in `BENCHMARKS.md`. The latest runtime work has focused on stability and bounded map growth, so re-run the benchmark suite after algorithm changes instead of treating older numbers in this file as current.
