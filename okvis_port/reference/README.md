# OKVIS2 deterministic single-threaded reference

Mirrors `stella_port/reference/`. Never edits `external/vio/okvis2`; `tools/build_okvis_reference.py` copies it
(without `.git`/`build/`) to `runs/okvis_port/reference_build/src`, applies `patches/*.patch` in order, builds
`okvis_app_synchronous` with `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math` (Release layout, SSE2 baseline) against the
`external/vio/deps` prefix (Eigen 3.4.0, glog, gflags, Boost, OpenBLAS, OpenCV 4.6.0 with a link-only highgui stub),
and writes `runs/okvis_port/reference_build/provenance.json` (upstream + submodule shas, patch sha256, compiler,
flags, Eigen version).

| patch | content |
|---|---|
| `0001-deterministic-single-threaded.patch` | background (loop-closure) optimisation and realtime optimisation are joined inside the frame that starts them; blocking publication queue; matching-thread segments run sequentially (same partition); OpenGV RANSAC RNG seeded 12345 instead of `time(0)+clock()` |
| `0002-zero-init-descriptor-pool.patch` | `Frontend::matchToMap` descriptor pool is uninitialised upstream and a landmark with no usable observation keeps one garbage row that the matcher compares to the keypoints (run-to-run nondeterminism under load); zero-initialised |
| `0003-imu-error-dump.patch` | `OKVIS_PORT_DUMP_DIR` instrumentation of `ImuError` (propagation, redoPreintegration, append, Evaluate, initPose) used by `okvis_port/c/check_ok_imu.c`; no numerical effect |
| `0004-solver-trace.patch` | `OKVIS_PORT_TRACE_DIR`: per-solve trace (input checksums, Ceres cost/iteration summary) and graph-mutation / per-frame detection / matcher-input event log (used to localise divergences; `events.txt` is ~150 MB per run) |
| `0005-kinematics-camera-dump.patch` | `OKVIS_PORT_KIN_DUMP_DIR` / `OKVIS_PORT_KIN_EVERY` instrumentation (module M2): every Transformation operation, `sinc`/`deltaQ`/`rightJacobian`, all non-batch `PinholeCamera` project/backProject variants (public entry points wrap renamed `*PortImpl` bodies) and the `NCameraSystem::computeOverlaps` result; call counts of every kind are printed to stderr at exit; no numerical effect (record layouts in `okvis_port/c/ok_kin.h`, `ok_cam.h`) |
| `0006-drain-publication-queue.patch` | `ThreadedSlam::stopThreading` waits for the publication queue to drain (under load the last states of `causal.csv` were dropped) |
| `0007-error-terms-manifolds-dump.patch` | `OKVIS_PORT_ERR_DUMP_DIR` / `OKVIS_PORT_ERR_EVERY` instrumentation (module M3): `EvaluateWithMinimalJacobians` of `ReprojectionError` (all camera variants), `PoseError`, `SpeedAndBiasError`, `RelativePoseError`, `HomogeneousPointError` (public entry wraps a renamed `*PortImpl` body, `Evaluate` calls it), every `setInformation` (LLT) and the variance/diagonal constructors, `PoseManifold` / `HomogeneousPointManifold` `plus`/`plusJacobian`/`minus`/`minusJacobian`; call counts printed at exit; no numerical effect (record layouts in `okvis_port/c/ok_err.h`, `ok_param.h`) |
| `0008-solver-dump.patch` | `OKVIS_PORT_SOLVE_DUMP_DIR` / `OKVIS_PORT_SOLVE_EVERY` / `OKVIS_PORT_SOLVE_FULL_EVERY` / `OKVIS_PORT_SOLVE_SPARSE_FULL_EVERY` instrumentation (module M4): per `::ceres::Solve` called from `ViGraph::optimise` a framed record stream `solve.bin` (options, Problem snapshot with cost-function payloads, program / reduced / reordered block order, per-iteration summary + minimizer vectors or their hashes, Dogleg and Gauss-Newton internals, Schur structure, reduced dense system, sparse `J^T J` system, raw outputs of not-yet-ported cost functions, termination); hooks live in a new header of the Ceres copy (`external/ceres-solver/include/ceres/okvis_port_solve_hooks.h`) and are armed only inside `ViGraph::optimise`; no numerical effect (layouts in `okvis_port/c/ok_solve.h`) |
| `0009-graph-twopose-problem-dump.patch` | `OKVIS_PORT_GRAPH_DUMP_DIR` instrumentation (module M5): `graph.bin` (new header `okvis_ceres/include/okvis/ceres/OkvisPortGraphDump.hpp`: every `TwoPoseStandardGraphError::compute` with its observations, parameter snapshots and outputs, every `convertToReprojectionErrors`, sampled `TwoPose{Standard,Extrinsics}GraphError{,Const}::EvaluateWithMinimalJacobians` (`OKVIS_PORT_GRAPH_TPEVAL_EVERY`), sampled `ViGraph::updateLandmarks` landmarks (`OKVIS_PORT_GRAPH_LM_EVERY`, `OKVIS_PORT_GRAPH_LM_SUB`)), `problem.bin` (Ceres copy `ProblemImpl` / `solver.cc`: every AddParameterBlock / SetManifold / AddResidualBlock / RemoveResidualBlock / RemoveParameterBlock / SetParameterBlockConstant / Variable, Problem construction and destruction, program-order hash at every `Solve()`, the full order every `OKVIS_PORT_GRAPH_PROBLEM_FULL_EVERY`-th), and the `portDump` payloads of the TwoPose* cost functions in the PROBLEM record of patch 0008; no numerical effect (layouts in `okvis_port/c/ok_graph.h`, `ok_problem.h`, `ok_solve.h`) |

`configs/okvis_mono_euroc_deterministic.yaml` = `external/vio/okvis_mono_euroc.yaml` with
`realtime_num_threads = full_graph_num_threads = 1` and `parallelise_detection = false`
(`num_matching_threads = 4` is kept: the segmentation is semantic, only its scheduling is made sequential).

Run: `python3 tools/run_okvis_reference.py MH_01_easy --tag run1 --dump --dump-every "prop=1,preint=1,append=5,eval=200"`
(dataset: `python3 tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,imu0`, ~1.3 GB, delete the PNGs after the runs).
Error-term dumps: add `--err-every "reproj=1000,pose=10,..."` (`all=N` sets the default first, 0 = count only).
Solver dumps (module M4): `python3 tools/run_okvis_reference.py MH_01_easy --tag m4 --solve-dump --solve-every 20 --solve-full-every 4 --solve-sparse-full-every 4`
(mono) and the same with `--tag s4 --data-dir MH_01_easy_s --config okvis_port/reference/configs/okvis_stereo_euroc_deterministic.yaml` (stereo, on a
symlinked copy of the dataset so both can run in parallel); every 20th realtime solve and every full-graph solve gets a snapshot, every 4th of those the full vectors.
Graph / Problem dumps (module M5): add `--graph-dump` to the solver-dump command above (tags `m5` / `s5`; sampling `--graph-tpeval-every 50 --graph-lm-every 2
--graph-lm-sub 8 --graph-problem-full-every 50` are the defaults): `graph.bin` (every TwoPoseStandardGraphError::compute and convertToReprojectionErrors, sampled TwoPose*
Evaluate and updateLandmarks records) and `problem.bin` (every ceres::Problem mutation and the program order at every Solve) next to `solve.bin`, whose PROBLEM record
then also carries the TwoPose* payloads.
Check the C port: `python3 tools/check_okvis_port.py --tag m3,m3cov,s3,m4,s4,m5,s5 --eigen-tests` (every harness skips the tags that lack its dump files).
Outputs, dumps and data are EuRoC-derived (non-commercial licence): keep them under gitignored `runs/` and
`external/vio/data/`, never commit them.
