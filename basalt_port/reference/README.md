# basalt_port reference build (single-threaded, deterministic, headless)

Reference for the Basalt port. Same method as `rdvio_port/reference` and `okvis_port/reference`: a pristine upstream tree,
a short list of patches, one script, byte-identical repeat runs.

## Build

    python3 tools/build_basalt_reference.py --jobs 4        # ~2m50s wall with 4 jobs, from scratch
    # -> runs/basalt_port/reference_build/{src,build,deps_root,provenance.json}, binary build/basalt_ref_driver

* Source: `external/vio/basalt_src` = GitLab master `0f3b2b52` (2026-03-22, version string 0.1.7, the same version as the
  prebuilt release in `external/vio/basalt`) plus pinned third-party checkouts in `external/vio/basalt_src/_deps`
  (the exact refs of Basalt's own vcpkg overlay ports): basalt-headers `aa441ba3`, Sophus 1.24.6 `d0b7315a`, cereal v1.3.2
  `ebef1e92`, magic_enum v0.9.7 `e046b69a`, opengv `91f4b19c`. Not fetched (not needed headless): Pangolin 0.9.4, RealSense,
  ros_comm / roscpp_core (rosbag), vcpkg itself, CLI11 (only for the upstream executables; the .deb is unpacked anyway).
* Upstream cannot be built as is: its `CMakeLists.txt` needs vcpkg (network), Pangolin, RealSense, rosbag and uses
  `-O3 -march=native`. `CMakeLists.reference.txt` replaces it: opengv (static, from source, globbed) + libbasalt (static, 16
  .cpp files: dataset_io, linearization, optical_flow, utils, vi_estimator) + `driver/basalt_ref_driver.cpp`.
* Flags: `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math -std=c++17`, no `-march` (SSE2 baseline), `-DEIGEN_DONT_PARALLELIZE`,
  `BASALT_INSTANTIATIONS_{DOUBLE,FLOAT}`. g++ 13.3.0.
* Dependencies: Eigen **3.4.0** from `external/vio/deps` (upstream's vcpkg baseline pins 5.0.1, and basalt-headers'
  CMake asks for `find_package(Eigen3 5.0)`, but the headers and Basalt compile and run with 3.4.0; the same Eigen as the other
  ports, so the evaluation-order models transfer), oneTBB 2021.11 (vcpkg baseline: 2022.3), OpenCV 4.6.0 (calib3d, features2d,
  imgcodecs, imgproc, core; only `cv::imread` and `cv::FAST` are on the VIO path), fmt 9.1.0 header-only, nlohmann-json 3.11.3
  (Ubuntu .debs fetched with `apt-get download`, unpacked without root into `reference_build/deps_root`).

## Patches (`patches/`, applied -p1 on the copied tree; sha256 in `provenance.json`)

| patch | what | observe-neutral? |
|---|---|---|
| 0001-dataset-io-drop-rosbag | `DatasetIoFactory` without the `bag` type (drops the ros_comm dependency; EuRoC/KITTI/UZH loaders untouched) | yes |
| 0002-opengv-ransac-fixed-seed | `CentralRelativePoseSacProblem(..., randomSeed=false)` in `findInliersRansac` (default seeds `mt19937` from `time(0)+clock()`) | defensive only: `findInliersRansac` is called solely from `nfr_mapper.cpp` (mapper), never from the VIO path |
| 0003-port-dump-instrumentation | module M0 observe-only dump, env-gated (`BASALT_PORT_DUMP_DIR`, `_EVERY`, `_FULL`): new header `include/basalt/utils/bs_port_dump.h`, FLOW records in `frame_to_frame_optical_flow.h`, IMU / OPT / ITER / MARG records + executed-path counters in `sqrt_keypoint_vio.cpp`, counters in `marg_helper.cpp`, `linearization_base.cpp`. Layouts: `basalt_port/PLAN.md` section 5, `basalt_port/c/check_bs_dump.c` | yes: trajectory sha256 `a7d3e7a3...` with the dump off, on (every record) and on (full arrays, sampled); it deliberately does not call `get_sqrt_cov_inv()` (ODR hazard, PLAN.md 4d) |

Not patches but part of the reference: `driver/basalt_ref_driver.cpp` (vio.cpp with all Pangolin/GUI code removed; sets
`tbb::global_control max_allowed_parallelism = 1`, `cv::setNumThreads(0)`, forces `vio_enforce_realtime = false`, writes TUM),
and `CMakeLists.reference.txt`.

## Nondeterminism sources (all of them found by reading the VIO path; `grep` for tbb/thread/rand/clock over src + include)

| source | where | answer |
|---|---|---|
| TBB `parallel_for` / `parallel_reduce` (float/double sums in linearisation, error evaluation) | `linearization_abs_qr.cpp` (11 sites), `ba_base.cpp:215`, `*_sc.cpp`, `sc_ba_base.cpp`, optical flow `parallel_for` over points/cameras | `global_control(1)`. Measured with this oneTBB (probe `runs/basalt_port/probe/tbb_probe.cpp`): with 1 thread and default grain 1, `parallel_reduce` calls the body once per element in index order and never splits/joins (N=1..400), so every reduction is a plain sequential left-to-right accumulation. The C port can use serial loops. |
| `tbb::concurrent_unordered_map` iteration order (tracks, `Corners`, `Matches`) | frontend, `common_types.h` | with 1 thread insertion order and hash are fixed; the order is a function of `std::hash<int64_t>` / `TimeCamId` hash buckets, which a port must reproduce where iteration order feeds numerics. Checked in M0 (PLAN.md 4b): the frontend map is copied into an ordered map (no hazard); the estimator's libstdc++ `unordered_*` order does change the trajectory bytes (but not the ATE) |
| producer/consumer threads (`feed_images`, `feed_imu`, optical-flow thread, estimator thread, state consumer) | `vio.cpp`, `*optical_flow.h`, `sqrt_keypoint_vio.cpp:272` | FIFO bounded queues, each consumer waits for its input; results do not depend on scheduling (4 runs identical, two of them concurrent). The only timing-dependent branch, `vio_enforce_realtime` (drops frames), is forced off by the driver (upstream vio.cpp does the same). |
| opengv RANSAC seeding | `SampleConsensusProblem(randomSeed=true)` | patch 0002; not on the VIO path |
| timers (`std::chrono`, `Timer`, `ExecutionStats`) | `accumulator.h`, `time_utils.hpp` | statistics only, never feed a decision |
| OpenCV threading / CPU dispatch | `cv::FAST`, `imread` | `cv::setNumThreads(0)`; FAST/PNG results do not depend on the dispatched SIMD path (to be re-verified against a C FAST in M3) |
| Eigen multithreading | GEMM | `-DEIGEN_DONT_PARALLELIZE` (upstream also sets it) |
| `-march=native`, `-O3`, FMA contraction | upstream CMake | removed / `-ffp-contract=off` |

No `rand`/`srand`/`random_device` in `src` or `include` on the VIO path (grep).

## Determinism evidence (EuRoC MH_01_easy stereo, `runs/okvis2x_port/data/MH_01_easy`, euroc_config.json + euroc_ds_calib.json, float estimator, ABS_QR, frame_to_frame flow)

    runs/basalt_port/out/mh01_run{1,2,3,4}.tum   3683 lines each (3682 states)
    a7d3e7a334591ce25157aed3777b17967269e1d67a2fa03c489cda11be32f1bf  (all four)
    basalt_ref_driver sha256 a5c2ecb14b5b45067cf2d82e937c014e6e517402b356c10c48c446c2959c0db7

Runs 1+2 sequential on the first build, runs 3+4 concurrently (both on the same machine at once) on a second, from-scratch
rebuild by the script: all four byte-identical.

## Accuracy and runtime

ATE via `benchmark.ate_rmse` (`external/gnss/venv/bin/python`), IMU position vs `state_groundtruth_estimate0`, nearest GT
within 5 ms (3638 of 3682 states matched):

| metric | value |
|---|---|
| ATE RMSE Umeyama + scale (`benchmark.ate_rmse`) | 0.0337 m (median 0.0254, max 0.0927, scale 1.0155) |
| ATE RMSE SE3 (`umeyama_alignment(with_scale=False)`) | 0.0740 m |
| wall time, 1 thread, 3682 frames (~184 s of data) | 61.3 - 62.0 s sequential (3 runs), 63.8 / 64.1 s two concurrent runs; RTF ~0.34 |
| peak RSS | ~78 MB |

For comparison `docs/vio_candidates_20261001.md` lists the prebuilt release (4 threads, -O3 -march=native) on MH_01 as
0.066 / 0.021 (SE3 / Sim3 columns of that table) at RTF 0.14. This build is single-threaded, -O2 and a different OpenCV/TBB, so
numbers are close but not identical; the port targets this reference, not the prebuilt.

Eval snippet: match `est` (TUM, seconds) to the GT csv (ns) by nearest timestamp <= 5 ms, then
`benchmark.ate_rmse(est_xyz, gt_xyz)`.

## Run

    LD_LIBRARY_PATH=external/vio/deps/root/usr/lib/x86_64-linux-gnu:external/vio/deps/opencv/lib \
    runs/basalt_port/reference_build/build/basalt_ref_driver --dataset-path runs/okvis2x_port/data/MH_01_easy \
      --cam-calib runs/basalt_port/reference_build/src/data/euroc_ds_calib.json \
      --config-path runs/basalt_port/reference_build/src/data/euroc_config.json --out traj.tum

## M0: dump, reader, runner, experiments

    python3 tools/check_basalt_port.py                 # m0: builds basalt_port/c/check_bs_dump.c, makes runs/basalt_port/dumps/{all,full} if missing
                                                       # (instrumented reference, ~75 s each), verifies the trajectory hash, runs the reader,
                                                       # enforces the executed-path facts (float, ABS_QR, SqrtToSqrt only, no SC / nullspace)
    python3 tools/check_basalt_port.py --also-off --regen      # also re-check the dump-off hash, regenerate the dumps

* Last verified (2026-10-07): dump off, `all`, `full` trajectories all `a7d3e7a334591ce2...`; reader on `all`: 41,887 records, 0 violations; on `full`: 1,159 records, 0 violations.
* ODR guard: `tools/build_basalt_reference.py` fails if the linked driver's `IntegratedImuMeasurement<float>::compute_sqrt_cov_inv` is not the float (`sqrtss`) copy
  (an instrumented first draft instantiated the `::sqrt(double)` copy in another translation unit and changed the trajectory to `68d99d40...`; PLAN.md 4d).
* `experiments/exp_unordered_order.patch` (applied on top of the patched tree in a scratch copy, NOT part of the reference): env `BASALT_EXP_ORDER` bitmask perturbs the
  iteration order of the unordered containers (1 reverse / 2 sort / 4 reserve for `unconnected_obs0`; 8 sort / 16 reverse `landmark_ids`; 32 reserve for `kpts`;
  64 sort / 128 reverse the `computeError` host frames); results in PLAN.md 4b (every mask changes the trajectory bytes, max deviation 3 mm, ATE unchanged).
* `reference_tools/ate_eval.py est.tum <seq dir> [--interp]`: Sim3 / SE3 / yaw-only ATE through `benchmark.ate_rmse` (used for PLAN.md 4c; prebuilt release runs: `runs/basalt_port/prebuilt/`).
