# stella_vslam pure-C port — handover

Goal: a permissively licensed (BSD-2-compatible) library-free C port of
stella_vslam (monocular), validated against a deterministic single-threaded
reference, same method as the (GPL, isolated, uncommitted) ORB-SLAM2 port.
Why stella: `docs/slam_candidates_comparison_20260925.md` (1.9 cm mean ATE,
88–99% coverage on original TUM, BSD-2). License audit + porting rules:
`docs/stella_vslam_license_audit.md`.

## Rules
- Clean room: port only from stella_vslam ≥0.3 source (pinned e445b545) and
  OpenCV BSD sources. Never read/copy `orb_port/` or `external/orb_slam2_reference/`.
- Keep BSD-2 (AIST 2019, stella-cv 2022) / BSD-3 (OpenCV, OpenGV) / MIT
  (libmv, FBoW) notices on derived files.
- Own trained ORB vocabulary before any public release (stella's vocab
  provenance is undocumented).
- No g2o csparse_extension / CHOLMOD in the port (reference build may link
  them; it is never distributed).
- Nothing committed yet.

## Coordination — read before resuming workers

Codex completed the independent **landmark representative ORB descriptor**
leaf (`landmark::compute_descriptor`, selection only). Reserved new paths:
`stella_port/c/sv_landmark_descriptor.{h,c}`, `check_sv_landmark_descriptor.c`,
`stella_port/reference_landmark_descriptor/`,
`tools/*_stella_landmark_descriptor.py`, and
`runs/stella_port/landmark_descriptor/`. No map-model, initializer, shared
type, shared-runner or existing-reference changes. Native selection, selected
observation identity, descriptor bytes and all row medians match in 11,488
cases (442,004 values); two-process dumps deterministic, normal and ASan/UBSan
checks pass, plus 14 API checks and negative fixtures. Real-image inputs cover
all xyz/desk frames but use controlled observation clouds, not tracked map
histories. See `stella_port/reference_landmark_descriptor/README.md` for the
map caller's contract and remaining cache/lifecycle responsibilities.

Codex completed the separate **BoW descriptor matcher** leaf:
`match::bow_tree::match_frame_and_keyframe` and `match_keyframes` (ORB only).
Reserved: `stella_port/c/sv_match_bow.{h,c}`, `check_sv_match_bow.c`,
`stella_port/reference_match_bow/`, `tools/*_stella_match_bow.py`, and
`runs/stella_port/match_bow/`. No triangulation matcher, initializer,
shared types, reference build, or shared runner edits. Published harness
passes 24,152 real-library matcher calls (13,213,632 counts/output slots),
on all 798 xyz / 598 desk frame inputs plus synthetic cases. Native outputs
are deterministic across two processes; normal and ASan/UBSan checks pass,
as do 18 API checks and negative fixtures. The landmark attachments are
controlled fixtures, not continuous-map replay. See
`stella_port/reference_match_bow/README.md` for integration and reproduction.

### Reserved for Codex (2026-09-30): own ORB vocabulary

Codex owns building a permissively licensed replacement for stella's
`orb_vocab.fbow` (provenance undocumented): own k-means++ vocabulary-tree
trainer, training-data choice + license record, FBoW-compatible output, and an
A/B of the stella reference with the new vocabulary on the TUM comparison set.
Reserved new paths: `tools/vocab/**`, `stella_port/vocab/**`,
`runs/stella_port/vocab/**`, `docs/stella_vocab.md`. The canonical dumps and
harnesses keep using the original vocabulary. Claude works on the top-level
system loop / continuous replay (`sv_system*`, `check_sv_system*`,
`tools/run_stella_port_replay.py`) and does not touch these.

### Reserved for Codex (2026-09-29): relocalization leaf

Codex owns stella's `module::relocalizer` (BoW candidate query via existing
`sv_bow_db`, `match::bow_tree`, `solve::pnp_solver` EPnP+RANSAC, pose
optimization, projection re-search) and the tracking-side `relocalize` glue.
Reserved new paths: `stella_port/c/sv_pnp.{h,c}`, `sv_relocalizer.{h,c}`,
`check_sv_pnp.c`, `check_sv_reloc.c`, `stella_port/reference_reloc/**`,
`runs/stella_port/reference_reloc/**`. Claude does loop closing
(loop detector, Sim3 solver, pose graph, loop BA glue) in parallel and does not
touch these.

Codex completed this leaf (2026-09-30). `sv_pnp` and `sv_relocalizer`
plus the automatic tracking-relocalization branch are available. The two
new harnesses are auto-discovered: **90 PnP cases and 88 relocalization cases,
0/9,689,738 comparisons**, normal and ASan/UBSan (leak detection disabled).
Fixtures span xyz and desk. Shared tracking/loop files and runner are untouched.

Isolated patches **0015–0018** live under `reference_reloc/patches/`;
include them when choosing future patch numbers. Two guards define upstream
uninitialized-output behavior for degenerate PnP; robust relocalization uses
a fixed seed. Desk550 is excluded because its snapshot references an absent
keyframe. These are component replays, not continuous-run ATE. The caller
still performs the subsequent local-map tracking step. Eigen-derived helpers
are MPL files in `reference_reloc/c/`. Reproduction, exact counts, provenance,
and limitations: `reference_reloc/README.md`. Nothing committed.

### Reserved for Codex (2026-09-29): essential 5-point RANSAC leaf

Codex owns `essential_solver::find_via_ransac` + `essential_5pt.h` (5-point
minimal solver: `FullPivLU` kernel, 10x10 solve, `EigenSolver<Matrix<double,10,10>>`)
and the matching-side `match::robust`. Reserved new paths:
`stella_port/c/sv_eigen_fullpivlu.{h,c}`, `sv_eigen_eigensolver.{h,c}` (MPL),
`sv_solve_essential_5pt.{h,c}`, `sv_solve_essential_ransac.{h,c}`,
`check_sv_essential_5pt.c`, `stella_port/reference_essential/**`,
`runs/stella_port/reference_essential/**`. Codex may replace the stub
`sv_match_robust.c` (and only that file among module-5 sources) once exact.
Claude does not touch these; Claude continues with the mapping module.

**Completed by Codex (2026-09-29).** Five-point solver + RANSAC + tracking
robust fallback are now exact on all 28 force-BoW fallback frames (1 xyz,
27 desk): 0/1,057,531 and 0/25,285,393 solver-trace comparisons. Two fresh
reference processes agree for every fixture. `check_sv_track_bow` no longer
skips robust frames: 0/989,673 (785 frames) and 0/679,624 (541 frames).
Normal tracking remains 0/989,176 and 0/679,504. Solver and forced-BoW
tracking pass ASan/UBSan; LeakSanitizer is unavailable under sandbox ptrace.
New Eigen code stays in MPL `sv_eigen_*` files. Mapping harness edits are
limited to source dependencies. Nothing committed. See
[reference_essential/README.md](reference_essential/README.md) for commands,
proof artifacts and scope limits (notably 6/7-sample variants are unported;
tracking uses five). This is component replay, not an autonomous ATE result.

Codex has reserved the independent **BoW database / candidate lookup** leaf
(`data::bow_database`: add, erase, clear, shared-word filtering and scoring).
The leaf is now implemented and validated: 2,941 native-reference queries,
all 798 xyz / 598 desk frame vectors plus synthetic cases, zero membership,
score-bit, threshold or posting-snapshot mismatches. Normal and ASan/UBSan
checks pass, as do 20 API checks and negative fixtures. Run
`python3 tools/check_stella_bow_db.py` (fresh `--out` for repeat runs).
The published harness uses the existing shared-runner ABI without runner
edits. See
`stella_port/reference_bow_db/README.md` for its API, results and the
candidate-order integration gate (ID-sorted C results versus upstream's
pointer-hashed vector). Claude's
module 3 initializer, area matching, H/F, triangulation, RNG and Eigen-derived
decompositions remain Claude's work. Skip the reserved leaf when assigning
later modules; the surrounding relocalization/loop detector is not reserved.

Reserved new paths:
- `stella_port/c/sv_bow_db.{h,c}`
- `stella_port/c/check_sv_bow_db.c`
- `stella_port/reference_bow_db/`
- `tools/build_stella_bow_db.py`, `tools/dump_stella_bow_db.py`,
  `tools/check_stella_bow_db.py`
- `runs/stella_port/bow_db/` (isolated builds, fixtures and results)

Codex will reuse `sv_bow` and existing upstream/reference inputs read-only.
It will not modify initializer files, shared types, the existing reference
build/dumps, or `tools/check_stella_port.py` without coordinating first.
Use a dedicated checker while the leaf is incomplete so automatic discovery
by the shared runner does not consume unfinished harness files. Add the
shared `check_sv_bow_db.c` harness only once its fixture contract is ready.
This note is advisory, not a filesystem lock; resume workers must reread it.

## State (2026-09-25)
- Reference: `stella_port/reference/` (README, patches 0001–0007, driver,
  deterministic config), builder `tools/build_stella_reference.py`, dumper
  `tools/dump_stella_reference.py`, outputs `runs/stella_port/`.
  Single-threaded (synchronous mapping + global optimization), deterministic:
  all dump files byte-identical across fresh runs on full fr1_xyz/fr1_desk.
  ST vs MT accuracy: fr1_xyz 2.61 vs 2.46 cm, fr1_desk 1.79 vs 2.02 cm.
- Dumps (19 files/frame, all byte-identical across runs; patches 0001–0005):
  per-frame tracking detail (keypoints, descriptors, matches, stage poses,
  local map, keyframe decision, RNG draw counts), map state after mapping,
  mapping-pass detail (culling reasons, triangulation, fusion, local BA
  iterations/chi2/outliers, keyframe culling), loop-detector/global-opt
  step summary, erased keyframes/landmarks with replaced-by ids.
  `--snapshot-interval N` trims map snapshots; `--light` minimal mode.
  Timing trap: with synchronous mapping, keyframe insertion drains INSIDE
  `feed_monocular_frame()`; the driver resets traces before that call.
  Not dumped: per-call cost trace of the shared loop/global BA optimizer.
- Port: `stella_port/c/`, runner `python3 tools/check_stella_port.py`
  (builds every `check_*.c` from its `/* SV_PORT_SOURCES: */` line, runs on
  every sequence in `runs/stella_port/reference_dumps/`).
  Module 1 DONE — ORB extraction (`sv_extract`, `sv_fast`, `sv_image`,
  `sv_undistort`, `orb_point_pairs.h`): byte-exact keypoints (incl.
  undistorted) + descriptors on every frame of fr1_xyz (798) and fr1_desk
  (598 — stella's TUM driver uses rgb/depth-associated frames).
  Fixture images come from `tools/dump_stella_fixtures.cc` linked against the
  reference's own OpenCV 4.6 (Python cv2 4.14 resize differs by 1 LSB).
  Exactness notes: OpenCV 8U INTER_LINEAR vertical pass has an extra `>>4`
  before the multiply (CV_8U specialization); undistortPoints denominator
  includes k3; stella keeps K/dist as float32; descriptor angle converted in
  double (`angle * M_PI / 180.0`); `response` is not preserved by stella's
  undistort (always 0 in dumps).
  Module 2 DONE — frame grid + area queries (`sv_frame`) and FBoW
  (`sv_bow`: .fbow load from memory, transform at level 4, score): exact on
  every frame of both sequences (BoW + score on all frames and on (i,i+1),
  (i,i+50) pairs). Reference tool `stella_port/reference_tools/` (links the
  built stella library; `tools/dump_stella_frame_bow.py`). Exactness notes:
  grid math subtracts in float then widens to double; FBoW weights float,
  norm accumulated in double; score multiplies in float, accumulates double.
  Module 3 (initializer) IN PROGRESS: `sv_rng` DONE — mt19937 + GCC 13
  libstdc++ `uniform_int_distribution` (Lemire path) + `std::shuffle`
  (paired-swap) + stella's `create_random_array`, 0/9880 vs real libstdc++
  (`stella_port/reference_tools/dump_rng.cc`, `check_sv_rng.c`, own runner).
  Findings: initializer triangulation uses the closed-form 2x2 overload (no
  SVD); H/F estimation + decompositions need Eigen `JacobiSVD`.
  `sv_eigen_svd` + `sv_eigen_qr` DONE (MPL-2.0 files, Eigen-derived):
  JacobiSVD<Matrix3d> (full U/V/σ) and JacobiSVD<Matrix<double,Dynamic,9>>
  with ColPivHouseholderQR preconditioning (V/σ/rank), 0/5215 values vs real
  Eigen over 85 fixtures (3x3 incl. degenerate; N×9 N=8..500);
  `stella_port/reference_tools/dump_eigen_svd.cc`, `check_sv_eigen_svd.c`
  (run from repo root). Findings: dynamic-offset blocks reduce with
  alignedStart=0; a 1-column Householder update dispatches to dot()/Redux
  (stride-4 two-accumulator order), not GEMV; U rotation uses the direct
  (c,s) because of JacobiSVD's extra transpose.
  Module 3 (initializer) DONE: `sv_match_area` (area matcher, margin=100),
  `sv_solve_homography`/`sv_solve_fundamental`/`sv_solve_essential`
  (compute_H21/F21, decompose, check_inliers, find_via_ransac RANSAC loop),
  `sv_triangulate` (closed-form 2x2 overload + camera reproject), `sv_init`
  (try_initialize_for_monocular + perspective::initialize orchestration,
  find_most_plausible_pose/triangulate replica), `sv_linalg` (3x3/vec3
  double-precision primitives, MPL-2.0, Eigen-derived evaluation order --
  see below). Standalone dumper `stella_port/reference_tools/
  dump_stella_init.cc` (drives match::area + real homography_solver/
  fundamental_solver/perspective directly through public API, no patches
  to the main reference; self-checks its own replicated find_most_plausible_pose
  against a real end-to-end `initialize::perspective::initialize()` call,
  0 mismatches on both full sequences) + `tools/dump_stella_init.py`.
  Harness `check_sv_init.c`: 0/316621 (fr1_xyz), 0/201296 (fr1_desk) --
  every attempt's matches, RNG-driven RANSAC (cost/H21/F21/inlier masks),
  model selection, all H/F decompose hypotheses (rot/trans/parallax/valid
  counts) and the final selected pose, bit-exact.
  Eigen 3x3 double evaluation-order findings (see "Eigen 3x3 double
  evaluation-order rules" below for the full measured table): critically,
  `A.transpose() * B` (matrix*matrix, LHS transpose used inline, e.g.
  `cam_matrix_2.transpose() * F_21` in fundamental_solver::decompose) is
  ALL rows left-associative -- NOT the mixed rows-0-1-L/row-2-R rule that
  plain `A*B` and `A*B.transpose()` (RHS transposed) both use. This was
  the actual source of the last residual (1-2 ULP in E_21's row 2 cascading
  through JacobiSVD into visibly different rotation hypotheses); bisected
  via `stella_port/reference_tools/debug_decompose.cc` (prints every
  decompose() intermediate -- E21, JacobiSVD U/sv/V, W, U*W*Vt products,
  det-sign flips, trans normalize -- from the real library on one dumped
  bit-exact F21, diffed against the same steps in the C port).
  Next: map data model + initial map + global BA (own LM solver, no
  CSparse) -- create_map_for_monocular is NOT covered by module 3 (global
  BA / map scaling only).
  Module 4a (map data model + monocular initial-map creation) DONE:
  `sv_map.{h,c}` ports `data::keyframe`/`data::landmark`/`data::graph_node`
  (spanning-tree subset only) and
  `module::initializer::create_map_for_monocular()` up to but NOT
  including the global BA call, split into `sv_map_build_pre_ba()` (the
  pre-BA keyframe/landmark/spanning-tree construction, from module 3's
  already-exact rot/trans/triangulated-points/matches, taken as input)
  and `sv_map_apply_post_ba()` (everything AFTER BA: mean normal + ORB
  scale variance, `keyframe::compute_median_depth`, and
  `module::initializer::scale_map()`'s median-depth scaling + wrong-init
  verdict, given an INJECTED post-BA keyframe pose + landmark positions --
  the real g2o global bundle adjuster is a separate, concurrently
  developed module (`sv_g2o*`), not ported here). Key finding:
  covisibility connections are NEVER populated during
  create_map_for_monocular -- `graph_node::update_connections()` is only
  called later, from the mapping module (out of this module's scope), so
  `connected_keyfrms_and_num_shared_lms_`/`ordered_covisibilities_` stay
  empty for both initial keyframes at this stage; only the spanning tree
  (parent/child/root, set directly by create_map_for_monocular) is
  populated. Reference tool `stella_port/reference_tools/
  dump_stella_map_init.cc` (+ `tools/dump_stella_map_init.py`) replicates
  create_map_for_monocular by calling the real library's own PUBLIC
  methods in its exact program order (`keyframe::make_keyframe`,
  `landmark`'s public ctor/`connect_to_keyframe`/`compute_descriptor`/
  `update_mean_normal_and_obs_scale_variance`, `graph_node`'s public
  spanning-tree setters, `map_database::add_keyframe`/`add_landmark`, and
  a REAL `optimize::global_bundle_adjuster::optimize_for_initialization`
  call) -- no reimplemented map/BA math, no patch to the shared reference
  build. The only private method reimplemented is `scale_map()` itself
  (pure scalar-multiply orchestration, no interesting math). Dumps
  `runs/stella_port/reference_map_init/<seq>/`: `keyframes_pre.tsv`/
  `landmarks_pre.tsv` (pre-BA), `keyframes_postba.tsv`/
  `landmarks_postba_pos.tsv` (real BA output, the harness's injected
  fixture), `keyframes_post.tsv`/`landmarks_post.tsv`/`scale.tsv`
  (final, post-BA+scale), `init_state.tsv`/`matches.tsv` (module-3 input,
  reused verbatim). Harness `check_sv_map_init.c`: 0/2668 (fr1_xyz),
  0/4096 (fr1_desk) -- every keyframe pose/pose_wc-derived field,
  spanning-tree link, and landmark (position, representative descriptor,
  mean normal, min/max valid distance, observed/observable counters,
  ref keyframe, observations) bit-exact in both the pre-BA and post-BA+
  scale dumps, plus the scale/verdict values; ASan/UBSan clean; two-process
  reference-tool dumps byte-identical (determinism check). Each sequence
  initializes exactly once (fr1_xyz: frame 7-8, 76 landmarks; fr1_desk:
  frame 55-56, 118 landmarks) -- module 4a only ever builds one initial
  map, so this harness doesn't yet exercise a second/failed init or
  landmark culling (out of scope; landmarks are never culled inside
  create_map_for_monocular itself). Trap found+fixed while building the
  reference tool: `initialize::perspective::initialize()` does NOT
  invalidate non-triangulated `init_matches_` entries itself -- that
  invalidation loop belongs to `create_map_for_monocular` and must be
  replicated explicitly, and `init_triangulated_pts` entries for
  non-triangulated matches are left as uninitialized `Vec3_t`s (not
  zeroed) by the real library, so they must never be read.
  Next: global bundle adjustment itself (g2o LM solver -- separate
  module, in progress) → wiring it into `sv_map_apply_post_ba`'s caller.
- Module 4b (g2o optimization core, pose-optimizer slice) DONE, bit-exact:
  `sv_eigen_quaternion.{h,c}` (MPL-2.0), `sv_g2o_se3.{h,c}` (BSD, SE3Quat
  exp/compose/map + shot_vertex/landmark_vertex oplus), `sv_g2o_edge.{h,c}`
  (BSD, mono_perspective_pose_opt_edge: error/Jacobian/Huber/
  constructQuadraticForm), `sv_g2o_pose_optimizer.{h,c}` (BSD, full
  `pose_optimizer_g2o::optimize` orchestration + a faithful
  `OptimizationAlgorithmLevenberg::solve` port), `sv_eigen_llt.{h,c}` +
  `sv_eigen_amd.{h,c}` (MPL-2.0, real `SimplicialLLT`/`AMDOrdering`
  algorithms specialized to this leaf's one real input shape). Real-library
  validation: `stella_port/reference/patches/
  0006-pose-optimizer-g2o-trace.patch` adds an opt-in global trace buffer
  inside `pose_optimizer_g2o.cc` itself (no public API exposes what edges
  it builds internally, and every real call is deep inside private
  tracking-module state -- patching the library, not reconstructing frame
  state, was the documented choice); `stella_port/reference_tools/
  dump_stella_g2o_pose.cc` + `tools/dump_stella_g2o.py` run the same
  synchronous driver loop as the main reference and dump every real call
  to `runs/stella_port/reference_g2o/<seq>/{calls,obs,outliers}.tsv`:
  1570 calls on fr1_xyz, 1082 on fr1_desk, all from `module::
  frame_tracker`'s three call sites via `tracking_module.cc:447` (no
  relocalization or loop-closure Sim3 calls occurred on either sequence),
  byte-identical across two full runs. `check_sv_g2o_pose.c` replays every
  call: **0/1570 (fr1_xyz), 0/1082 (fr1_desk) mismatches at zero
  tolerance** -- final pose, outlier-flag set, and `num_valid_obs` all
  bit-exact.

  **Bit-exact closure (bisected via a standalone instrumented-g2o build,
  2026-09-25):** `runs/stella_port/reference_g2o/g2o_instrumented/` (own
  copy of `external/candidates/g2o` @ the same pinned commit, SVTRACE-gated
  `fprintf` added to `OptimizationAlgorithmLevenberg::solve` -- lambda,
  H/b via the public `Vertex::hessian(i,j)`/`b(i)`, dx, tempChi, rho,
  accept/reject -- never changing a computed value; built standalone,
  never touching the shared reference build) plus
  `runs/stella_port/reference_g2o/g2o_instrumented/driver/replay_pose.cc`
  (a small driver that rebuilds ONE captured call's g2o problem -- real
  `shot_vertex.h`/`perspective_pose_opt_edge.h`/`terminate_action.{h,cc}`
  copied in verbatim, edges built directly instead of through
  `pose_opt_edge_wrapper` since that needs `camera::base` -- and runs it
  against the instrumented g2o with the SAME algorithm loop copied from
  `pose_optimizer_g2o.cc`). Comparing this against a matching
  SVTRACE-instrumented copy of `sv_g2o_pose_optimizer.c`, bisecting
  call-by-call on the worst-residual call each time, found and fixed, in
  order:
  1. **H/b and chi2 term-grouping order.** g2o's
     `BaseFixedSizedEdge::constructQuadraticForm` computes
     `AtO = J^T*omega` then `H += AtO*J`, `b += J^T*weightedError` (each
     entry a sum of two ALREADY-SCALED products) -- not
     `H(r,c) += s*(J0[r]*J0[c] + J1[r]*J1[c])` (scale applied to an
     already-summed pair). Mathematically equal, not bit-equal. Likewise
     `chi2() = _error.dot(information()*_error)` computes
     `(isq*e0)*e0 + (isq*e1)*e1`, not `isq*(e0*e0+e1*e1)`. Measured the
     exact real-Eigen grouping for these fixed 2x6/2x2 shapes with a tiny
     g++ test (0/500000 mismatches) before fixing `sv_g2o_edge.c`.
  2. **Missing `SE3Quat::normalizeRotation()`.** Every real SE3Quat
     constructor (from a rotation matrix OR quaternion) and `operator*`
     calls `normalizeRotation()` (sign-canonicalize `w>=0`, then
     `coeffs().normalize()`) -- this port's `sv_se3_exp`/`sv_se3_compose`
     and the initial-pose reconstruction from a dumped `Mat44_t` did not.
     `squaredNorm()` on the 4-vector of coeffs is ALSO a new shape: the
     SSE2 packet-pairwise grouping `(x*x+z*z)+(y*y+w*w)`, not sequential
     `((x*x+y*y)+z*z)+w*w` (measured ~30% mismatch) or `(x*x+y*y)+(z*z+w*w)`
     (~27% mismatch) -- 0/500000 for the cross-pair grouping. Added
     `sv_quat_normalize`/`sv_se3_normalize_rotation`, called everywhere a
     rotation is constructed or composed. This was the single biggest
     fix (found by comparing the initial quaternion the two replays
     reconstructed from the identical dumped `Mat44_t` -- all 4
     components off by a small, consistent amount, the signature of a
     missing normalize rather than a rounding-order bug).
  3. **Wrong solver entirely.** `g2o::LinearSolverEigen<BlockSolver_6_3::
     PoseMatrixType>` wraps `Eigen::SimplicialLLT<SparseMatrix, Upper>`
     (plain Cholesky, sqrt on the diagonal) -- NOT `SimplicialLDLT` as
     originally assumed. Found by comparing `dx` after the linear solve
     (matched H/b, mismatched dx) then reading
     `linear_solver_eigen.h` directly. `sv_eigen_ldlt.{h,c}` renamed to
     `sv_eigen_llt.{h,c}` and rewritten for the real
     `factorize_preordered<DoLDLT=false>` algorithm -- which has its OWN
     internal subtlety: `yi = l_ki = yi / Lx[Lp[i]]` REASSIGNS `yi` to
     the already-divided value before the scatter/diagonal-update use it
     (`y[r] -= L(r,i)*l_ki`, `d -= l_ki*l_ki`), unlike the LDLT branch's
     undivided accumulator -- using the undivided value (copy-pasted from
     the LDLT draft) caused actual numerical blow-up (negative `d`,
     `sqrt`->NaN) on real captured H, not just a rounding mismatch. Also:
     g2o's `fillCCS(Cx, upperTriangle=true)` sources column k's values
     from `H[row][k]` for `row<=k` (the upper triangle) -- this port's H
     is a full, not-quite-symmetric-to-the-last-bit array (like g2o's
     own), so reading the wrong triangle silently fed the factorization
     slightly wrong numbers.
  4. **Float constants.** `pose_optimizer_g2o.cc` declares
     `constexpr float chi_sq_2D = 5.99146;` and
     `const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);` -- FLOAT, not
     double. `(double)5.99146f` != `5.99146` (5.991459846496582... vs
     5.99146 exactly), and `sqrtf` of the float value differs from
     `sqrt` of the double literal at the 7th significant digit. This
     constant feeds every edge's chi2 threshold AND Huber weight, so a
     ~1e-7 relative error compounds over repeated LM iterations into a
     visible final-pose difference. Fixed via
     `#define CHI_SQ_2D ((double)5.99146f)` and
     `(double)sqrtf((float)CHI_SQ_2D)`.
  5. **Missing early-termination propagation.** `OptimizationAlgorithmLevenberg::
     solve()` returns `Terminate` (not `OK`) when
     `qmax==_maxTrialsAfterFailure || rho==0 || !isfinite(lambda)`, and
     `SparseOptimizer::optimize(iterations)`'s outer for-loop condition
     includes `&& ok` (`ok = result==OK`) -- so a `Terminate` result
     stops the outer per-call iteration loop immediately (after that
     iteration's own postIteration/gain-check still runs once). This
     port's `lm_solve` kept iterating past that point. Fixed by computing
     the same `should_terminate` condition and breaking the outer loop.
  Each fix was verified independently (a tiny g++ test against real
  Eigen, or a targeted single-call replay diff) before moving to the
  next; the SE3Quat product formula from the prior session (quat_product
  SSE2 grouping, 0/2,000,000) and the Jacobian (central-difference,
  ~8.6e-10) needed no changes.
  (Pose-optimizer leaf: the "NOT ported" note that used to be here is superseded by the module 4b part 2 entry directly below.)
- **Module 4b part 2 (local BA + global BA) DONE (2026-09-29): local BA
  bit-exact on every real call, initial-map global BA bit-exact, loop-BA
  entry point bit-exact against the real g2o `LinearSolverEigen`.**
  Files (all new, none of Codex's reserved leaves touched):
  `sv_g2o_ba.{h,c}` (BSD: landmark/shot vertices, binary mono reprojection
  edge with Huber, `BlockSolver<6,3>` structure + Schur + back-substitution,
  Levenberg loop, `SparseOptimizer::optimize` driver with stella's
  `terminate_action`), `sv_bundle_adjuster.{h,c}` (BSD: orchestration of
  `local_bundle_adjuster_g2o::optimize`, `global_bundle_adjuster::
  optimize_for_initialization` and `::optimize`, over a read-only *map view*
  because `sv_map` has no covisibility/observation/keypoint model yet; it
  returns poses/positions/outlier observations instead of mutating a map),
  `sv_umap_order.{h,c}` (BSD behavioural model of libstdc++
  `unordered_map<unsigned,T>` iteration order), extended `sv_eigen_amd.{h,c}`
  (real Eigen AMD, MPL-2.0; the old identity function stays for the pose
  leaf) and `sv_eigen_llt.{h,c}` (general sparse `SimplicialLLT<Upper>` with
  permutation: `sv_sllt_*`; the dense 6x6 `sv_llt6_*` stays for the pose
  leaf), harness `check_sv_g2o_ba.c`. Reference side: patch
  `reference/patches/0007-ba-trace.patch` (+ `optimize/ba_trace.h`),
  `reference_tools/dump_stella_g2o_ba.cc`, `replay_ba_eigen.cc`,
  `tools/dump_stella_g2o.py --ba / --ba-replay`, standalone checks
  `reference_tools/{umap_order_test,eigen_amd_llt_test,eigen_shape_tests_ba}.cc`.
  Licence note: Eigen's `Amd.h` and `SimplicialCholesky_impl.h` are MPL-2.0
  files that carry a notice that Davis (CSparse / LDL) licensed the adapted
  code to Google for distribution under MPL-2.0 inside Eigen; the C files are
  transliterations of those Eigen files. No CSparse, CHOLMOD or
  `csparse_extension` code was read or used; the g2o files used are core BSD
  ones.

  **Captures** (`runs/stella_port/reference_g2o_ba/<seq>/ba_calls.bin`,
  binary, 27 MB / 38 MB, two-process runs byte-identical): fr1_xyz 39 calls
  (36 local BA + 1 initial global BA + 2 real `global_bundle_adjuster::
  optimize()` loop-BA-entry calls on the final map, huber on/off), fr1_desk
  61 (58 + 1 + 2). Every local BA call the pipeline makes and the one
  initial-map BA are covered; no loop closure occurs in either sequence, so
  the loop-BA entry point is driven directly on the final map (54 keyframes
  on fr1_desk). Coverage: 168,776 / 227,587 edges, 216 / 153 outlier
  observations. Each call records the map view read, the g2o graph as built
  (vertex ids/owners/initial estimates, edge order/measurement/information/
  Huber delta), per stage the edge levels at stage start and per iteration
  the robust chi2, lambda, Levenberg trials and stop flag, final estimates,
  per-edge chi2/error/depth/level, outlier list, applied Mat44 poses.
  Adding the patch does not change the existing dumps: fr1_xyz reference
  dumps regenerate byte-identically; fr1_desk too except the loop-candidate
  id column of `global_opt_step.tsv`, which varies from run to run of the
  same binary (23/28/29 seen; no loop is ever accepted, nothing downstream
  changes; pre-existing pointer-hash order in the loop-candidate path).

  **Results** (`python3 tools/check_stella_port.py --seqs fr1_xyz,fr1_desk`,
  `check_sv_g2o_ba` rows): local BA **0/36 and 0/58 calls differ at
  tolerance 0** (graph, every iteration's chi2/lambda/trials/flags, stage
  edge levels, final estimates, edge state, outliers, applied poses);
  initial global BA 0/1 and 0/1 at tolerance 0 (SimplicialLLT path, not
  CSparse -- identical, the reduced system is a single free 6x6 pose block);
  loop-BA entry point on fr1_xyz 0/2 at tolerance 0; on fr1_desk 2/2 differ
  from the **CSparse** reference by at most: chi2 4.1e-15 relative (4 of
  ~20 iteration values, 2.6e-11 absolute on chi2 ~6.5e3), vertex estimates
  7.8e-15 absolute, applied poses 2.2e-15, edge errors 2.0e-12 absolute;
  every structural quantity (graph, iteration counts, trial counts, stop
  decisions, edge levels, outliers) is identical. `SV_BA_GLOBAL_TOL`
  (default 1e-9, measured as |d|/max(1,|ref|)) is what the runner uses for
  kind-1/2 calls; `SV_BA_GLOBAL_TOL=0` demands bit equality. Because
  CSparse is the only difference, `replay_ba_eigen` rebuilds every captured
  global graph with stella's own vertex/edge headers and the **real g2o
  `LinearSolverEigen`** (SimplicialLLT + AMDOrdering): the port equals it
  bit for bit on all 6 global graphs, including the fr1_desk loop BA whose
  Schur pattern (54 poses, 1125 of 1485 block entries) yields a genuinely
  permuted AMD ordering. That replay also isolates the deviation above as
  the CSparse-vs-Eigen Cholesky difference (max vertex dev 7.8e-15).
  Standalone checks against real Eigen 3.4 / real libstdc++: `eigen_amd_llt_test`
  (sv_amd_order and the sparse LLT analyze/factorize/solve, driven as
  `LinearSolverEigen` drives them, 4000 random block patterns incl. dense,
  banded, arrow, sparse: 0 mismatches), `eigen_shape_tests_ba` (300k random
  cases per kernel, kernels below, real `g2o::internal::axpy/atxpy`: 0
  mismatches), `umap_order_test` (60,000 random insert/erase sequences: 0
  order mismatches).

  **Exactness findings.** No bisection with the instrumented standalone g2o
  copy was needed this time (the first end-to-end run of all 94 local calls
  and 2 initial BAs matched); the failures we did hit were harness/dump
  bugs (see 3). What made it exact:
  1. *`terminate_action` writes through the caller's `force_stop_flag`.*
     `local_bundle_adjuster_g2o::optimize` passes mapping's
     `abort_local_BA_`; when the first optimization stops on the 1e-3 gain
     criterion the flag is left `true`, so step 6's `if (*force_stop_flag)
     run_robust_BA = false` skips the second (outlier-removal + kernel-free)
     stage. Only 9/36 (fr1_xyz) and 1/58 (fr1_desk) local calls run the
     second stage (the others used all 5 first-stage iterations or were
     flagged); the outlier list of step 7 is nevertheless always computed
     and applied. The port reproduces this with an explicit
     `int* stop_flag` (external flag, else the terminate action's own aux
     flag installed by iteration -1).
  2. *Vertex/edge order comes from libstdc++.* Local keyframes, fixed
     keyframes and local landmarks are `std::unordered_map<unsigned,...>`;
     vertex ids (= Hessian block order = Schur accumulation order) and edge
     insertion order follow their iteration order. `sv_umap_order` models it
     (1 bucket empty; first insert -> 13; growth to the next tabulated prime
     >= max(n+2, 2*buckets); new node at the front of its bucket group, or
     of the whole list for an empty bucket; rehash re-inserts in list order;
     prime table measured via `rehash(n)`/`bucket_count()` for n < 400000,
     small-n table measured too).
  3. *g2o dump-side traps:* `SparseOptimizer::vertices()` is an
     `unordered_map<int,Vertex*>` (sort by id when tracing), and
     post-iteration actions live in a pointer-ordered
     `std::set<HyperGraphAction*>`, so a second recording action can run
     before `terminate_action`; the trace hooks into `terminate_action`
     itself instead.
  4. *Active structure:* `_activeEdges`/`_activeVertices` are sorted by
     id (creation order), edges with level 1 are dropped from the next
     `initializeOptimization()`, vertices without a level-0 edge leave the
     index mapping, and the Schur block pattern is built from
     `vertex->edges()` -- ALL edges of a landmark, including level-1 ones --
     so stage 2 carries explicit zero Schur blocks that still steer AMD.
  5. *Stale errors:* `chi2()` of an inactive (level-1) edge is the error
     from the last `computeActiveErrors()` in which it was active; step 6/7
     decisions use those (the port stores `err` per edge and only refreshes
     active edges). `depth_is_positive()` uses the current estimates.
  6. *Local BA Schur systems are block-dense* (all keyframe pairs share
     landmarks), so AMD is the identity there; only the big loop BA
     exercises a real permutation. `sv_amd_order` is validated against real
     Eigen on random patterns and end to end via the replay above.
  7. Float constants as in the pose leaf (`(double)5.99146f`,
     `(double)sqrtf(5.99146f)` for chi2 threshold/Huber delta; information =
     `Identity * inv_sigma_sq(float)`).

  Eigen evaluation-order rules measured for this module (SSE2,
  `-ffp-contract=off`; same flags as the table below; asserted by
  `eigen_shape_tests_ba` against real Eigen/g2o helper templates):
  - `Matrix<6,3> * Matrix3d` into a 6x3 (`BDinv`): all rows left-associative
    `((a0*b0)+a1*b1)+a2*b2` (3 full packets).
  - `Map<Matrix<6,1>> += Matrix<6,3> * Vector3d`: dst + `((a0*b0)+a1*b1)+a2*b2`.
  - `Matrix<6,6> -= Matrix<6,3> * Matrix<6,3>.transpose()`: dst - L-sum.
  - `y.segment<3> += A(6x3).transpose() * x.segment<6>` (`atxpy`): every
    coefficient is a complete-unrolled linear-vectorized redux over 6
    products: `(a0+(a2+a4)) + (a1+(a3+a5))` (packet split, then lane add).
  - `y.segment<3> += Matrix3d * x.segment<3>` (`axpy`, Dinv * cl): rows 0,1
    left-associative (packet), row 2 `a0+(a1+a2)` (scalar tail); no
    runtime-alignment dependence for a fixed-size-3 destination.
  - Depth-2 products (`AtO = A^T*omega`, `H += AtO*A`, `b += A^T*we`,
    `Hpl += B^T*AtO^T`) for the 3-dim landmark and the 6x3 off-diagonal
    block: `dst + (p0 + p1)` exactly as for the pose leaf.
  - `Matrix3d::inverse()` and `Matrix3d * Vector3d` (Dinv, db): existing
    `sv_mat3_inverse` / `sv_mat3_mulv` rules confirmed again.

  Residuals / not covered: markers and stereo/fisheye/equirectangular edges
  (harness skips calls with markers; none occur), the
  `use_additional_keyframes_for_monocular` branch (implemented, never
  exercised: config default false), an externally *set* force-stop flag at
  entry (only the false-at-entry path occurs), real loop closures (none in
  fr1_xyz/fr1_desk; the loop-BA entry point is validated on the final map
  only), and the map-side application of the results
  (`erase_landmark`/`erase_observation`/descriptor updates) which belongs to
  the map module. The sequence-level global_opt_step candidate column is
  nondeterministic (see above) but outside BA.
- **Module 5 (tracking side) DONE including the robust fallback (2026-09-29),
  0 mismatches at tolerance 0 on every tracked frame of fr1_xyz (785) and
  fr1_desk (541).**
  Files (all new; Codex's reserved leaves used read-only: `sv_match_bow`,
  `sv_landmark_descriptor`): `sv_track.h` (data model + API), `sv_track_frame.c`
  (config/ORB tables, frame/keyframe pose setters, `reproject_to_image`,
  hamming, `angle::diff`), `sv_frame_tracker.c` (`match::projection::
  match_current_and_last_frames`, pose-optimizer glue = edge construction from
  a frame, `motion_based_track`, `bow_match_based_track`, `discard_outliers`),
  `sv_local_map.c` (`local_map_updater`), `sv_tracking.c` (`feed_frame`
  tracking half: `update_last_frame`, `track_current_frame`, `update_local_map`,
  `search_local_landmarks` incl. `can_observe`/`predict_scale_level` and
  `match_frame_and_landmarks`, `optimize_current_frame_with_local_map`,
  `update_motion_model`, `keyframe_inserter::new_keyframe_is_needed`,
  `finish_frame` = state transition + `last_cam_pose_from_ref_keyfrm` +
  `last_frm`), `sv_kf_insert.c` (`create_new_keyframe` = `make_keyframe` +
  `update_landmarks`: add_observation, mean normal / scale variance,
  representative descriptor), `sv_match_robust.c` (validated tracking fallback; see reserved-leaf completion above),
  `sv_eigen_mat4.{h,c}` (MPL-2.0: Matrix4d product). Harnesses:
  `check_sv_track.c` (canonical dumps), `check_sv_track_bow.c` (fault-injection
  dumps, below).
  Reference side: patch `0008-pre-track-state.patch` (read-only getter
  `tracking_module::get_pre_track_state()`: tracking state, twist,
  last_cam_pose_from_ref_keyfrm, last-frame pose/ref keyframe/landmark hash,
  last reloc, last inserted keyframe, keyframe count -> `track_pre.tsv`;
  driver also writes `keyframe_meta.tsv` = keyframe id -> exact timestamp),
  `0009-kf-insert-trace.patch` (state of the new keyframe and of every landmark
  it observes right after `update_landmarks()` -> `kf_insert.tsv`,
  `kf_insert_lms.tsv`), `0010-force-track-path.patch` (OPT-IN fault injection,
  default off: env `STELLA_PORT_FORCE_PATH=bow:N|robust:N` makes every N-th
  frame skip the motion tracker / motion+BoW trackers; only used through
  `tools/dump_stella_reference.py --force-path bow:5`, writing to
  `runs/stella_port/reference_dumps_force_<kind>/`; canonical dumps and
  `runs/tum_compare` are untouched). Rebuilt with
  `tools/build_stella_reference.py`; canonical dumps regenerated: all old files
  byte-identical (except stderr.log and the known fr1_desk `global_opt_step.tsv`
  candidate column), new files byte-identical across two runs.
  Method: teacher forced per frame t. Map at the start of t = snapshot after
  frame t-1 (`keyframes.tsv`/`landmarks.tsv`, existing dumps; keyframe
  keypoints/descriptors come from the keyframe's source frame via timestamp),
  tracker state from `track_pre.tsv`, last-frame associations from
  `matches.tsv[t-1]` (hash-checked). Compared per frame: track path, pose after
  `track_current_frame` (bits), final returned pose_wc (bits), every keypoint's
  landmark id, local keyframe list and local landmark list (order), num_tracked /
  num_reliable, keyframe decision + all 16 sub-fields, new keyframe id / pose_cw
  / pose_wc / trans_wc / every landmark's post-`update_landmarks` state, and
  (after `finish_frame` on snapshot t) the next frame's twist,
  `last_cam_pose_from_ref_keyfrm`, last-frame pose and ref keyframe.
  Ordering findings: the reference is `-DDETERMINISTIC=ON`, so
  `keyframe_to_num_shared_lms_t`, landmark observations and spanning children
  are id-ordered `std::map/set` (no libstdc++ hash order; `sv_umap_order` not
  needed); `greater_number_and_id_object_pairs` is a total order, so
  partial_sort == sort for the used prefix. `unordered_*` lookups in
  `search_local_landmarks` / `match_frame_and_landmarks` are never iterated.
  Semantic traps found: `feed_frame` RETURNS `get_pose_wc()` (the dump column
  named pose_cw is pose_wc); the returned/`last_cam_pose_from_ref_keyfrm`
  uses the new keyframe's pose AFTER mapping/local BA (snapshot t); frame and
  keyframe compute `trans_wc` differently (`-R^T*t` lazy transpose, all-L, vs
  `-rot_wc*t` materialized, rows 0-1 L / row 2 R) and both are used;
  `num_observations()` counts observations (mono); mapper is never
  paused/skipping in the synchronous reference so those flags are constants;
  `lms_ratio_thr_view_changed` is 0.5 (YAML-ctor default), not the header's 0.8;
  pose optimizer in tracking is {2,2,10}.
  Results (`check_sv_track`): fr1_xyz 0/989176 (785 frames: 784 motion, 1 bow,
  36 keyframe insertions / 7,736 landmark updates), fr1_desk 0/679504 (541
  frames: 540 motion, 1 bow, 58 insertions / 6,196 landmark updates).
  `check_sv_track_bow` (bow:5 dumps, including every robust fallback):
  fr1_xyz 0/989673 (785 frames, 1 robust), fr1_desk 0/679624 (541 frames,
  27 robust). See the reserved-leaf completion above and
  `reference_essential/README.md` for full solver traces and sanitizers.
  Open issues: (1) Nondefault essential-solver sample sizes 6/7 are not ported;
  the tracking fallback uses the validated five-point path. (2) Not ported /
  not exercised: initialization, relocalization, tracking loss and reset,
  temporal keyframes (fixed_keyframe_id_threshold>0), markers, stereo/depth.
  (3) The motion tracker's `2*margin` retry and the `assume_forward/backward`
  branches (non-monocular) are unverified by data; the retry is a straight
  transcription. (4) Landmark `num_observed/num_observable` increments made
  by tracking are applied in the C map but only observable through later
  snapshots that also include mapping effects (checked indirectly through
  `kf_insert_lms.tsv`, which records them after the increments).
- Next modules: mapping module (triangulation, fusion, local BA glue,
  culling; consumes `sv_tr_map`/`sv_tr_kf` records) → relocalization → loop
  detection + Sim3 + pose graph. Robust tracking fallback is now validated.

## Eigen 3x3 double evaluation-order rules (measured, 2026-09-25)
Measured against real Eigen 3.4 (`-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`,
SSE2 baseline) on 20 000 random cases each, bit-exact. L = `(a0+a1)+a2`,
R = `a0+(a1+a2)`. Scratch tests: `$SCRATCH/eigtest/t*.cc` (re-creatable).
- `Matrix3d * Matrix3d` and `Matrix3d * Matrix3d.transpose()`: rows 0–1 L,
  row 2 R (packet rows + scalar remainder row). Triple products are pairwise
  `(A*B)*C`, each with this rule.
- `Matrix3d * Vector3d`: rows 0–1 L, row 2 R.
- `Matrix3d.transpose() * Vector3d`: ALL rows L.
- `Matrix3d.transpose() * Matrix3d` (LHS transpose, matrix*matrix; e.g.
  `cam_matrix_2.transpose() * F_21`): ALL rows L -- **not** the mixed
  rows-0-1-L/row-2-R rule (that's specific to a plain-ColMajor LHS; a
  materialized `.transpose()` result used as LHS, i.e. stored in a named
  `Mat33_t` first, goes back to being a plain matrix and uses the mixed
  rule for any *later* product). Bisected 2026-09-25 against real Eigen
  via `stella_port/reference_tools/debug_decompose.cc` on a real dumped
  F21/E21 (fundamental_solver::decompose's `cam2.transpose()*F21`); this
  was the actual fix that closed the `check_sv_init` hyp.rot residual --
  the earlier guess (mixed rule for both transpose directions) was wrong
  specifically for LHS-transpose matrix*matrix.
- `determinant()`: row-0 cofactor expansion, L:
  `(m00*(m11*m22-m12*m21) + m01*(m12*m20-m10*m22)) + m02*(m10*m21-m11*m20)`.
- `inverse()` (3x3): `cof<i,j> = m(i1,j1)*m(i2,j2) - m(i1,j2)*m(i2,j1)`
  (i1=(i+1)%3 …); `det = L(cof<0,0>*m00, cof<1,0>*m10, cof<2,0>*m20)`;
  `invdet = 1.0/det`; `result(r,c) = cof<c,r> * invdet`.
- `dot`, `squaredNorm`, `norm` (= sqrt of L sum): L. `normalized()` =
  `x / sqrt(L(x·x))` (division, not multiply by reciprocal). `cross`: plain
  formula.
- `Quaternion<double> * Quaternion<double>` (module 4b, 2026-09-25):
  dispatches to `quat_product<Architecture::Target,...,double>`
  (`Geometry_SIMD.h`), the SSE2/ARM64 packet specialization, NOT the
  generic 16-multiply/12-add scalar formula in `Quaternion.h`, on any
  Eigen build with `EIGEN_VECTORIZE_SSE` (true for this repo's reference
  build flags: `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`, SSE2
  baseline -- same flags used to measure every other rule in this table).
  Unrolled to scalar (coeffs order x,y,z,w) and verified against real
  Eigen 3.4, 0/2,000,000 random-quaternion mismatches
  (`g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -msse2`):
  ```
  t1x = aw*bx + ay*bz        t2x = az*bx - ax*bz
  t1y = aw*by + ay*bw        t2y = az*by - ax*bw
  x = t1x - t2y              y = t1y + t2x
  u1x = aw*bz - ay*bx        u2x = az*bz + ax*bx
  u1y = aw*bw - ay*by        u2y = az*bw + ax*by
  z = u1x + u2y              w = u1y - u2x
  ```
  (derived from the `addsub`/`preverse` SSE2 lane algebra -- see
  `stella_port/c/sv_eigen_quaternion.c`). `toRotationMatrix` and the
  matrix->quaternion Shoemake assign (`quaternionbase_assign_impl<Other,3,3>`)
  are plain scalar `coeffRef`/branch code in Eigen's own source (no packet
  dispatch to unpick), ported directly in the same file -- not yet
  independently diffed against real Eigen the way the product was, but
  lower risk since there is no hidden vectorized specialization to miss.
- Module 4b part 2 additions (BlockSolver<6,3> Schur / back-substitution
  shapes, `sv_ba_k_*` in `sv_g2o_ba.c`, 300k random cases each vs real
  Eigen/g2o, 0 mismatches): `Matrix<6,3>*Matrix3d`, `Map<Matrix<6,1>> +=
  Matrix<6,3>*Vector3d`, `Matrix<6,6> -= Matrix<6,3>*Matrix<6,3>^T`: all
  rows L. `segment<3> += Matrix<6,3>^T * segment<6>`: `(a0+(a2+a4)) +
  (a1+(a3+a5))`. `segment<3> += Matrix3d * segment<3>`: rows 0-1 L, row 2 R.
  Depth-2 products: `dst + (p0+p1)`. Details in the module 4b part 2 entry.
- Module 5 additions (`eigen_shape_tests_track.cc`, 300k random cases each,
  0 mismatches vs real Eigen 3.4, same flags): `Matrix4d * Matrix4d` (frame
  poses, velocity): every row left-associative `((a0*b0+a1*b1)+a2*b2)+a3*b3`
  (two full SSE2 packets, no scalar remainder row); `-R.transpose() * t`
  (`frame::set_pose_cw`): all-L sum then negated; `-R * t` with R
  materialized (`keyframe::set_pose_cw`): rows 0-1 L, row 2 R, then negated;
  `R * p + t` (`reproject_to_image`): mixed matrix*vector rule, then add;
  `(a-b).norm()` = sqrt((x²+y²)+z²); `a.dot(b)` L. Float paths: `logf/ceilf`
  for `predict_scale_level`, float margins/scale factors, doubles widened only
  where the source widens.

- Module 6 additions (`reference_tools/eigen_shape_tests_map.cc`, 300k random cases each, 0 mismatches vs
  real Eigen 3.4, same flags): `rot_cw.block<1,3>(2,0).dot(pos_w) + trans_cw(2)` (`check_depth_is_positive`, a
  strided row block => non-vectorized redux) is **R**: `a0*b0 + (a1*b1 + a2*b2)` (the contiguous
  `Vec3.dot(Vec3)` stays L; the L variant mismatches 24 % of cases); `create_E_21` =
  `rot_21 = rot_2w * rot_1w.transpose()` (mixed rule), `-rot_21 * trans_1w + trans_2w` (mixed, negate, add),
  `skew(trans_21) * rot_21` (mixed) -- all existing `sv_mat3_*` primitives; `JacobiSVD<Matrix4d>` full U/V
  (`solve::triangulator::triangulate(bearing, bearing, Mat44, Mat44)`): the square no-preconditioner path of
  `sv_eigen_jacobisvd_*` at n = 4 (`sv_eigen_jacobisvd_4x4`, 0/300000 for U, V and sigma).

## Open item before porting loop detection (2026-09-29): RESOLVED by patch 0012
Root cause: `data::bow_database::acquire_keyframes()` gathered its result in a
pointer-hashed `std::unordered_set<shared_ptr<keyframe>>` and returned a vector
copied from it; `loop_detector::find_continuously_detected_keyframe_sets()`
consumes that vector in order (first intersecting keyframe becomes the set's
`lead_keyfrm_`), so ASLR-dependent pointer order picked the lead candidate.
`loop_candidates_to_validate_` (also pointer-hashed) is iterated by
`select_loop_candidate_via_Sim3` and the top-n covisibility expansion.
Fix: `stella_port/reference/patches/0012-id-ordered-loop-candidates.patch`
switches those containers to the `nondeterministic::unordered_set` alias
(id-ordered under DETERMINISTIC). Order only, no math. Original description:
The deterministic reference was NOT fully deterministic in the loop detector:
on fr1_desk, `global_opt_step.tsv`'s loop-candidate id varies across runs of
the same binary (23 / 28 / 29 observed). No loop is ever accepted on
fr1_xyz/fr1_desk, so nothing downstream changes today, but the source
(likely a pointer- or hash-ordered container / score tie in
`module/loop_detector.cc` or the BoW database candidate ranking) must be
found and fixed in the reference (recorded patch) before the loop-closing
module is ported or validated.

## Module 6 -- mapping module (2026-09-29): DONE, 0 mismatches at tolerance 0 on all 96 mapping passes
`mapping_module::mapping_with_new_keyframe()` for every keyframe of fr1_xyz (37 passes, 36 with local BA) and
fr1_desk (59 passes, 58 with local BA): `check_sv_mapping` 0/296,849 (fr1_xyz) and 0/588,336 (fr1_desk).
Compared per pass, against `snapshot(t)` (keyframes.tsv / landmarks.tsv) and the mapping-pass dumps: culled
landmark ids + order, triangulation per neighbor (match pairs, accepted landmark ids, position bits),
fused (replaced, by) pairs, culled keyframes, local BA invoked, every alive keyframe (pose bits, ordered
covisibilities + weights, spanning parent / children, per-keypoint landmark slots), every alive landmark
(position, descriptor, mean normal, min/max valid distance, observed/observable, reference keyframe, observation
list), and every alive keyframe's raw connected map (`conn.tsv`, incl. expired keys). Breakdown (stderr of the
harness): xyz 3689 keyframe / 223,240 landmark / 1054 connected-map / 1226 culled-landmark / 15,120
triangulation / 1927 fused / 53 culled-keyframe / 74 local-BA items; desk 12,124 / 480,320 / 3464 / 1646 /
16,894 / 1255 / 64 / 118. Totals over both: 7,934 accepted triangulated landmarks, 3,086 fused pairs, 2,776 culled
landmarks, 21 culled keyframes.

**Files (all new unless noted)** -- `stella_port/c/`: `sv_mapping.{h,c}` (store_new_keyframe, local_map_cleaner,
create_new_landmarks + two_view_triangulator, update_new_keyframe / fuse_landmark_duplication, graph_node,
keyframe/landmark erasure and merging, local BA glue = builds the `sv_bav_view` from the map and applies steps 7/8
of `local_bundle_adjuster_g2o::optimize`), `sv_map_match.c` (`bow_tree::match_for_triangulation`,
`check_epipolar_constraint`, `create_E_21`, Mat44-pose `triangulator::triangulate`, `compute_median_depth`),
`sv_rbtree.{h,c}` (behavioural model of the std::map red-black tree, see below), `check_sv_mapping.c` (harness).
Additions to module 3/5 files: `sv_eigen_svd.{h,c}` (+`sv_eigen_jacobisvd_4x4`), `sv_track.h` (+`sv_tr_obs.bearings`,
`sv_tr_kf.is_root`, prototypes), `sv_track_frame.c` (`sv_tr_obs_ensure_bearings`, free), `sv_kf_insert.c` (two static
helpers made public as `sv_tr_lm_*`), `check_sv_track.c` (`is_root` at snapshot load, `SV_MAPPING_HARNESS`
hooks, `SV_MAX_REPORT` env). `check_sv_mapping.c` `#include`s `check_sv_track.c` (main renamed) and runs the mapping
pass on top of its teacher-forced per-frame tracker replay; tracker items count too. Reference side:
`reference/patches/0011-lifetime-and-graph-raw-state.patch`, driver additions (`main.cc`, appended blocks marked
`patch 0011`), `reference_tools/eigen_shape_tests_map.cc`, `reference_tools/rbtree_test.cc`.
No reserved Codex path touched (`sv_match_bow`, `sv_landmark_descriptor` used read-only; `sv_match_robust.c`
untouched).

**Method.** Pre-state of the pass for the keyframe inserted at frame t = snapshot(t-1) + module 5's replay of
frame t (visibility counters) + `sv_tr_create_new_keyframe` (verified against kf_insert*.tsv). State that
snapshots do not expose is carried in the C `sv_mapping` context across passes and rebuilt from scratch at the
start: `local_map_cleaner::fresh_landmarks_` (a `std::list` WITH duplicates, appended per pass in keypoint order),
every keyframe's full `connected_keyfrms_and_num_shared_lms_`, `landmark::first_keyfrm_id_`,
`map_database::next_landmark_id_`. Reference changes: patch 0011 (instrumentation only, no computation change) adds
(a) `graph_node::get_raw_state()` and `conn.tsv` (per mapping pass, every alive keyframe: raw connected map in map
order + raw ordered list, expired keys as id -1), (b) a keyframe-destructor log with a phase marker
(`kf_destroyed.tsv`: frame, phase, kf id; phase 0 = tracking, 1 = mapping pass up to
`remove_redundant_keyframes`, 2 = from there on incl. global optimization, 3 = after synchronize). Rebuilt with
`tools/build_stella_reference.py --skip-copy` after `patch -p1` on the existing src copy (`diff -rq` against a from-
scratch build of all 11 patches: identical). Determinism: canonical dumps regenerate byte-identically for every
pre-existing file (only stderr.log; fr1_desk `global_opt_step.tsv` candidate column as before), new files identical
across two runs (xyz: all files; desk: all but `global_opt_step.tsv`). `conn.tsv` / `kf_destroyed.tsv` were copied
into `runs/stella_port/reference_dumps/<seq>/` (other files untouched); `tools/dump_stella_reference.py` regenerates them.

**Traps found (each was a large block of mismatches until fixed).**
1. `tracking_module::initialize()` passes ALL initial keyframes to the mapper in BFS order: **keyframe 0's pass runs
   first** and really works (it triangulates keyframe 0 <-> keyframe 1: reference keyframe 0, first_keyfrm_id 0, ids
   continue after the initializer's); the reference only dumps the last pass (keyframe 1). The harness rebuilds the
   pre-state by dropping every landmark from the first id created by a pass (smallest id with reference keyframe 0 /
   first id of the keyframe-1 triangulation row) and replays both passes. Fresh list order depends on it (keyframe
   0's landmarks are id-ordered).
2. Covisibility bookkeeping: `add_connection` re-sorts the neighbor's ordered list from its FULL connected map (so low
   counts appear); an own `update_connections` sets the map to all counts but the ordered list to counts > 15 (or the single
   nearest). Order = (count desc, id desc). The graph is not symmetric (a pair below the threshold is only in one map).
3. **Dangling weak_ptr keys.** `connected_keyfrms_and_num_shared_lms_` is `std::map<weak_ptr<keyframe>, ..., id_less>`
   whose comparator dereferences the keys; an expired key compares as +infinity while the tree was built with its
   id. When an erased keyframe is destroyed while a neighbor's (asymmetric) map still holds it, lookups for larger
   keys fail: `get_num_shared_landmarks` (dumped weights) returns 0, `add_connection` inserts, `erase_connection`
   does nothing, and the entry disappears from `get_covisibilities()` (expired). Visible in the dumps as `26:0` weights
   and vanished entries (fr1_xyz kf 25 from frame 522, fr1_desk kfs 15..29 from frame 522). Reproduced exactly by
   `sv_rbtree` (libstdc++ red-black tree: `_M_get_insert_unique_pos`, `_M_get_insert_hint_unique_pos`, `operator[]` =
   lower_bound + emplace_hint, `equal_range`+`erase`, `insert_and_rebalance`, `rebalance_for_erase`, header
   leftmost/rightmost), 0/702,758 operations vs the real `std::map` with expiring keys incl. full pre-order structure
   and colour comparison (`reference_tools/rbtree_test.cc`; a one-line mutation of the recolouring is caught in 22
   operations). `get_top_n_covisibilities` / `get_covisibilities` skip expired entries WITHOUT counting them.
4. **Object lifetime is an input, not derivable.** An erased keyframe expires when the last `shared_ptr` drops; the
   holders are outside the mapping module (the loop detector retains BoW candidates, e.g. fr1_desk keyframe 3 -- erased
   at frame 489 -- is never destroyed in the whole run; keyframe 6 erased at 432 dies at frame 522 phase 2). xyz: every
   erased keyframe dies in the same frame, phase 2. The harness injects `kf_destroyed.tsv` via `sv_mapping_set_expired`:
   at the start of the pass of frame t everything destroyed at frame < t or (frame t, phase <= 1) is expired; at the
   dump of frame t everything destroyed at frame <= t. Porting the loop detector's retention is out of scope; the C
   library takes the schedule from the caller.
5. `remove_invalid_landmarks` culls by `observed_ratio < 0.3` BEFORE the age test, per list entry (duplicates
   are dropped as `will_be_erased`); `first_keyfrm_id_ + 2 < cur` needs the creation keyframe (kept in the context).
6. Refresh conditionals (`if (!has_representative_descriptor()) compute_descriptor(); if (!has_valid_prediction_parameters())
   update_...`) are modelled as unconditional recomputes after `replace()`/`connect_to_keyframe()`: the values are
   pure functions of the current state and never stale (0 `has not ...` warnings in stderr.log, 0 mismatches).
7. `new_connections` is an `unordered_map<idx, lm>`: applied in ascending idx (order-insensitive: no landmark is connected
   twice to a keyframe, counter `n_dup_connect` = 0); `duplicated_lms_in_keyfrm` is id-ordered (patch 0003).
8. Local BA glue: outlier observations are erased with the OLD keyframe poses (descriptor + mean normal refreshed), then
   `set_pose_cw` for the local keyframes, then position + `update_mean_normal...` for local landmarks
   (independent per landmark, so unordered_map order is irrelevant). BA is invoked iff the map has > 2 keyframes.
9. Float/double: `residual_rad_thr = (float)((double)0.2f * M_PI / 180.0)`; `check_epipolar_constraint`'s two float args
   are swapped at the call site (commutative product); ratio test `0.95f * (float)second < (float)best`; epipole
   `cos_dist_thr = 0.99862953475`; reprojection tests `chi_sq_2D = 5.99146f` times a float sigma^2 table
   (`level_sigma_sq`, not in `sv_tr_config`; computed in `sv_mapping_init`); `check_scale_factors` mixes float octave
   ratio and double distance ratio; `ratio_factor = 2.0f * scale_factor`; `median_depth` uses `(float)trans_cw_z`.

**Mutation checks (constant perturbed once in a scratch copy, then discarded; xyz / desk mismatches):**
observed_ratio_thr 0.3 -> 0.31: 86,445 / 98,452; triangulation chi_sq_2D 5.99146 -> 5.9: 52,558 / 53,188; fuse margin
3.0 -> 2.9 (pass 1): 4,762 / 3,980; baseline_dist_thr_ratio 0.02 -> 0.2: (crash) / 125,753; epipole cos_dist_thr
0.99862953475 -> 0.9986: 14 / 0; connected-map comparator ignoring expiry: 16 / 72.

**Not exercised / open.** (1) Stale neighbors: a covisibility entry whose keyframe was erased but is not yet destroyed
would be a valid neighbor upstream (erased keyframe with cleared observations); the port skips and counts them
(`n_stale_neighbor` = 0 on both sequences). (2) `recover_spanning_connections` iterates a `std::set<shared_ptr<keyframe>>`
(pointer order); the port uses ascending id and counts equal-count candidates (`n_span_ties` = 0). (3) Not ported:
`tracker_->replace_landmarks_in_last_frm` (the harness reports the fused pairs; the tracker replay is teacher-forced
from `matches.tsv`), `bow_db_->erase_keyframe`, `erase_temporal_keyframes`, markers, stereo / equirectangular paths,
`use_baseline_dist_thr` (absolute) branch, mapping with a queue (skipping local BA), interruption flags. (4) The
`match::robust` fallback of `create_new_landmarks` (used only without a BoW vocabulary) is not ported.
(5) Loop closure interacts with this module through keyframe lifetimes (item 4 above) and the pose graph; the
loop detector, Sim3 and pose-graph optimization are next.

## Module 7 -- loop closing (2026-09-30): DONE, 0 mismatches at tolerance 0 on 21 replay sets (bit-exact vs the Eigen-solver reference)
`global_optimization_module::run_step()` for every keyframe step of fr3_long_office (161 steps, 1 real accepted loop kf 161 <-> 6),
fr1_desk, fr1_xyz, fr1_floor, fr2_xyz, plus stress variants. Only fr3_long_office contains a real accepted loop (the other
TUM sequences here have none); rejection at stages 4-8 and 10 and the final-match threshold are provoked with opt-in knobs.
Runner rows (`python3 tools/check_stella_port.py`, harness `check_sv_loop`, dumps `runs/stella_port/reference_loop_eigen/<name>`):
fr3_long_office 0/71164, fr3_long_office_gA 0/69191, _gB 0/68389, _rej4 0/206, _rej5 0/207, _rej6 0/208, _rej7 0/209, _rej8 0/210,
_rej10 0/216, _vfinal 0/217 (rej*/gA/gB/vfinal replay steps >= 150 only, trace of earlier steps is in the dump), fr1_desk 0/635,
fr1_desk_st 0/12363 (spurious accepted loop), fr1_xyz 0/232, fr1_xyz_st 0/260, fr1_floor 0/2245, fr1_floor_st 0/5952 (1314 candidate
validations), fr2_xyz 0/533, fr2_xyz_st 0/18133 (spurious accepted loop). Existing modules' rows unchanged (fr1_xyz/fr1_desk).
vs the canonical CSparse reference (`reference_loop/fr3_long_office`, harness in tolerance mode, default SV_LOOP_TOL=1e-6):
0/71164 at 1e-6; 170 at 1e-7; pose-graph results differ from CSparse by up to ~1e-8 relative (ill-conditioned graph), so bit
equality needs the Eigen solver (like loop BA in module 4b). BA/pose graph use `STELLA_PORT_EIGEN_SOLVER=1` in the reference.

**Reference (patch 0013, `0013-loop-closing-determinism-trace-eigen-solver.patch`; builds byte-identically from fresh copy)**:
1) determinism: pointer-ordered containers in the loop path made to id order (`keyframe_Sim3_pairs_t`, `get_loop_edges`,
`extract_new_connections`, graph_optimizer `loop_connections`); 2) dump-only trace (`util/loop_trace.h`, hook phases 0/1/2,
`bow_database::get_all_keyframe_ids`); 3) opt-in env `STELLA_PORT_EIGEN_SOLVER` (graph optimizer + global BA use LinearSolverEigen);
4) opt-in env `STELLA_PORT_LOOP_THR_OPT1/_A/_B` for the three hard-coded thresholds (10/25/40). Canonical fr1_xyz/fr1_desk
dumps regenerate byte-identically (all 27 files but stderr.log). Driver: `--loop-dump [--loop-snap-from N]`; tool
`tools/dump_stella_loop.py` (`--eigen-solver`, `--yaml-set Sec.key=v`, `--env K=V`, `--suffix`). Dumps are deterministic
(two runs, cmp of every file identical). Disk: reference_loop_eigen 824 MB, reference_loop 146 MB. Codex owns patches 0015/0016.

**Port files**: `sv_sim3.{h,c}` (g2o::Sim3), `sv_eigen_lu3.{h,c}` (MPL, PartialPivLU<3x3> solve), `sv_g2o_sim3.{h,c}` (numeric-
Jacobian LM core for the 7-dof pose graph and transform_optimizer, Eigen SimplicialLLT+AMD), `sv_loop.{h,c}` (detector, candidate
validation, correct_loop, graph glue, loop BA glue; `#include`s sv_mapping.c to reuse its landmark/covisibility statics -- list
sv_loop.c, not sv_mapping.c, in SV_PORT_SOURCES), `check_sv_loop.c`. Reference tools: `eigen_shape_tests_sim3.cc` (1.5M cases
exp/log/inverse/mul/map/from_rot vs real g2o: 0 mismatches), `replay_sim3_opt.cc` (3000 random pose graphs + 3000 transform-
optimizer problems vs real g2o/stella classes: 0 differ; target added to reference_tools/CMakeLists.txt; sum-order mutation
caught in 299/300). Runner: `check_stella_port.py` runs `check_sv_loop*` over LOOP_DUMP_ROOT (additive; `--seqs` may name loop dumps).
Harness compares the exact text of ~30 event types (candidates, continuity sets, matches, PnP-fed pose optimizations, projection
re-search, scale, mutual matching, transform optimizer, accepted Sim3, corrected Sim3s/landmarks/poses, fuse pairs, replaced
landmarks, new connections, pose-graph vertices/edges/result) and the map after the pose graph and after loop BA.
Mutations (scratch copies, reverted): fuse margin 4->3.9, graph gain 1e-3->1e-4, transform chi 10->9.9, projection margin 10->9,
numeric delta 1e-9->1e-8, LM tau, bow ratio .75->.95, cos_parallax_thr, mutual margin 7.5->2: all caught (14k-30k mismatches);
tiny changes (ratio .76, thr 0.99996, margin 7.4) do not alter this data. ASan/UBSan clean on desk/xyz/floor/fr2 (fr3 not re-run).

**Eigen rules measured (300k+ cases, real Eigen 3.4)**: Sim3 exp/log as written in g2o with `(B*Omega)*Omega` = scalar-scaled
matrix then Matrix3 product (mixed rule); `W.lu().solve(t)` on 3x3 = unblocked LU (division by pivot) + complete-unrolled
triangular solves `rhs(i) -= (p0+p1)`, then `/= diag`. 7-vector dot/chi2 and `dst += A^T*v` (7x7): `((p0+(p2+p4))+(p1+(p3+p5)))+p6`;
`H += AtO*A` (7x7 Map, any alignment): rows 0-5 `((((((p0+p1)+p2)+p3)+p4)+p5)+p6)`, row 6 `(p0+(p1+p2))+((p3+p4)+(p5+p6))`;
transposed off-diagonal block `B^T*AtO^T`: all elements the row-6 pattern; 2-dim unary edges: `dst+(p0+p1)`. Traps: `fuse::detect_duplication`
defaults `do_reprojection_matching=false` in correct_loop (mapping passes true) -> separate port function; `s` in graph_optimizer's
pose update is a `float`; `optimized_pose` is uninitialized when the pose optimizer has < 5 observations (trace prints it only if
num_valid_obs>0); outliers rejected in `lms_in_cand` do not touch `curr_match_lms_observed_in_cand` (copy); `projection_matcher` in
validate_impl has check_orientation=true but it is unused there; landmark cache flags (`has_valid_prediction_parameters_`,
`set_pos_in_world` clears it) are modelled explicitly because correct_loop refreshes conditionally; keyframe 0 is never queued to the
loop detector (BoW DB holds 1..cur-1 alive).

**Retention (task 3)**: offline analysis on all five sequences (114 erased keyframes): the destruction step/frame of every erased
keyframe (phase 2) equals the first loop-detector step after erasure whose new `cont_detected_keyfrm_sets_` (lead+members) no longer
contains it (never destroyed if held to the end, e.g. fr1_desk kf 3): 114/114. So the detector part of the retention is modelled
by the continuity sets the port already produces; module 6's harness still uses the injected `kf_destroyed.tsv` (not modified;
switching it needs a detect call per keyframe step inside that harness).

**Open / not covered**: PnP RANSAC result injected from the trace (Codex's `sv_pnp` is in progress; swap via `sv_loop.pnp`);
reject stage 9 (no scale reference) and brute-force matching (`num_matches_thr_robust_matcher>0`) not exercised; expired keys in
connected maps: tree shape not dumped (counter `n_expired_in_loop`=0 everywhere); `tracker_->replace_landmarks_in_last_frm`, set_not_to_be_erased flags, merging two spanning trees, and post-loop mapping/culling not ported; snapshot-based teacher forcing (per-step map from `loop_snap.tsv`), not an autonomous run.

## Top-level system / continuous replay (2026-09-30): DONE, exact to the end on fr1_xyz, fr1_desk and fr3_long_office
`sv_system` runs the whole pipeline with NO teacher forcing: gray image + timestamp in, everything else (map, tracker, mapper
hidden state, loop detector state, BoW database, keyframe lifetimes) from its own previous state. Continuous replay against the
canonical reference dumps is **bit-exact on every compared item of every frame** (details below): no divergent frame on any of
the three sequences. Full runner `python3 tools/check_stella_port.py`: **56 rows, 0 mismatches** (the previous 54 plus
`check_sv_system` on fr1_xyz / fr1_desk); the earlier rows are unchanged.

**Files** (`stella_port/c/`): `sv_system.{h,c}` (top-level system), `sv_run.c` (driver; the ONLY port file with stdio; also
compiled into the harness), `check_sv_system.c` (runner harness, auto-discovered), `tools/run_stella_port_replay.py` (deep
diagnostics + ATE). Additive edits to shared module files (every new hook is NULL / 0 by default, so module 5-7 harnesses are
untouched): `sv_track.h` (+ `SV_TR_PATH_RELOC_AUTO/BY_POSE`, `sv_tr_map.kf_pool`, `sv_tracker.{reloc_hook,mapper_paused,
optimize_ran}`), `sv_track_frame.c` (`sv_tr_map_kf_any`), `sv_tracking.c` (Lost state -> reloc hook, pool lookup of erased
reference keyframes, paused-mapper decision), `sv_mapping.{h,c}` (`is_protected` / `on_erase` hooks in
`keyframe::prepare_for_erasing`), `sv_loop.{h,c}` (`pnp_ransac` hook with the solver inputs, `replaced_hook`). No Codex-reserved
file was modified: `sv_pnp` / `sv_relocalizer` / `sv_bow_db` / `sv_match_bow` / `sv_landmark_descriptor` / robust + essential files
are used read-only; nothing under `tools/vocab`, `stella_port/vocab`, `runs/stella_port/vocab`.

**Step order per frame** (= reference driver: `feed_monocular_frame()` then `synchronize_background_modules()`):
1. `sv_orb_extract` -> `sv_undistort_point` -> `sv_tr_obs` (grid). Image input: the driver reads the exact gray frames of the
   reference (8-bit PGM fixtures from `tools/dump_stella_fixtures.cc`, `runs/stella_port/fixtures/<seq>/%06d.pgm`; PNG decoding +
   `cvtColor` need OpenCV so it is not reimplemented). Timestamps: `rgb.txt` / `depth.txt` association exactly like
   `tum_rgbd_util.cc` ((rgb + nearest depth) / 2, 0.1 s threshold), so frame ids equal the reference's.
2. Initializing: `create_initializer` / `match::area` (`prev_matched_coords_` updated in place across attempts, a failed attempt
   with < 50 matches resets and the NEXT frame becomes the reference) / `sv_init_try_monocular` / `sv_map_build_pre_ba` / initial
   global BA (`sv_bav_global_init`, 100 iterations, Huber, gain 1e-5f) / `sv_map_apply_post_ba` (median scaling, wrong-init
   verdict) / records installed into the `sv_tr_map` pool / frame statistics of the two initial frames (PRE-BA poses) / mapping
   passes for keyframe 0 THEN keyframe 1 (`tracking_module::initialize` hands the spanning tree in BFS order) / `last_frm`.
3. Tracking / Lost: `sv_tracker_track` (motion -> BoW -> robust; in Lost state Codex's `sv_reloc_tracking_glue` through
   `sv_tracker.reloc_hook`, then the normal local-map step) -> `update_frame_statistics` -> keyframe decision -> insertion
   (`sv_tr_create_new_keyframe`, mapping pass inline, keyframe queued for the global optimizer unless it is a spanning root)
   -> state transition (Lost within `init_retry_threshold_time` = 5 s of the initialization -> `reset()`) -> `finish_frame`.
4. `synchronize`: for every queued keyframe: `sv_loop_detect` (BoW database ids = the persistent database's content), the
   keyframe is then registered in the database, `sv_loop_validate` (PnP through the real `sv_pnp_ransac`: 30 iterations, 10 min
   inliers, GN 10, no recompute, default-seeded engine per solver instance = `use_fixed_seed: true`), `sv_loop_correct` (pose
   graph + loop BA), then the keyframe-lifetime update.

**Gaps closed (each per stella source)**
- `tracker_->replace_landmarks_in_last_frm`: implemented (`sv_loop.replaced_hook`). During a mapping pass it is a no-op (the
  mapper runs at the end of `feed_frame` where `last_frm_` is still the PREVIOUS frame and tracking has finished); it matters only
  in `correct_loop`, where `last_frm_` is the current frame (single-hop `replaced_lms` map, id-ordered, `add_landmark` after
  `erase_landmark(replaced_lm)`).
- `bow_db erase_keyframe`: `sv_mapping.on_erase` hook (fires inside `prepare_for_erasing`, after the spanning tree is repaired,
  where upstream calls `replace_reference_keyframe` and `bow_db->erase_keyframe`); the same hook does
  `frame_statistics::replace_reference_keyframe` (rel pose `old_rel * old_pose_cw * inv(new_pose_cw)`, plain 4x4 inverse: only
  the saved 9-digit trajectory depends on it).
- Erasure protection: `keyframe::cannot_be_erased_` == "has a loop edge" at every point a mapping pass can observe it (the
  flags raised during a global step -- current keyframe, validated candidates -- are lowered again by `set_to_be_erased()`
  unless a loop edge exists). `prepare_for_erasing` is then a no-op but the culled-keyframe trace still lists the id.
- Spanning-tree merge: upstream `correct_loop` only warns and returns when the two roots differ ("not yet implemented");
  `sv_loop_correct` already returns 1 there. A single map has a single root (a new root only exists after `reset()`, which clears
  the map), so it is unreachable.
- Post-loop mapping resume: the deterministic reference NEVER resumes the mapper. `correct_loop` calls `mapper_->async_pause()`
  (returns immediately, `is_terminated_` defaults true, but sets `pause_is_requested_`) and `mapper_->resume()` returns early on
  `is_terminated_`. Consequence in the reference: after the first accepted loop `keyframe_inserter::new_keyframe_is_needed`
  returns false forever (`mapper_paused_or_pausing`), no keyframe is inserted, no mapping pass / culling runs.
  `sv_system_params.resume_mapper_after_loop = 0` (default) reproduces that, `1` gives the threaded upstream behaviour.
  On fr3_long_office the two modes are indistinguishable (the loop is accepted at the last keyframe, id 161, frame 2407, and
  `--resume-mapper` inserts the same 160 keyframes and ends with the same map counts).
- Keyframe destruction: no `kf_destroyed.tsv`. An erased keyframe is destroyed at the first loop-detector step after the
  erasure whose `cont_detected_keyfrm_sets_` (lead + members) no longer contain it (`update_lifetimes`); it then becomes an
  expired key of the connected maps (`sv_mapping_set_expired`). Compared with the reference schedule: 16/16 (xyz), 5/5 (desk;
  keyframe 3 is held to the end in both) and 21/21 (fr3) destruction frames equal.
- Erased keyframes referenced by a frame: upstream keeps the object (and its pose) alive; `sv_tr_map.kf_pool` gives the tracker
  the record of an erased keyframe (`sv_tr_map_kf_any`) for `update_last_frame` / `track_current_frame` / `finish_frame`.
- Persistent trace state: the reference dump prints `tracker->last_track_path_` / `last_pose_after_initial_track_` /
  `last_num_tracked_lms_` / `keyframe_inserter::last_decision_` even for frames in which they were not recomputed;
  `sv_frame_result` reproduces that persistence (initial frames print `none`, identity, 0/0 and the default decision).
- Ported but NOT validated (unreachable in the deterministic reference): `tracking_module::reset()` (upstream deadlocks in the
  synchronous reference: `mapper_->async_reset().get()` waits for a mapping thread that never runs; the port resets the
  initializer, mapper hidden state, loop detector, BoW database, map ids and frame statistics but -- like upstream -- keeps
  `cont_detected_keyfrm_sets_`, `twist_`, `last_frm_`) and Lost -> relocalization inside a continuous run. Both are exercised
  by `sv_run ... --blank A-B` (frames A..B replaced by a flat image): fr1_xyz `--blank 300-320` goes Lost at 300 and
  `relocalize_auto` succeeds on the first real frame (321), tracking continues; `--blank 30-36` (Lost < 5 s after init 12) resets
  at frame 30 and re-initializes at frame 54 with keyframe ids restarting at 0. ASan/UBSan (leak check on): clean on xyz,
  desk, fr3_long_office (through the accepted loop, ~35 min under ASan) and both blank scenarios.

**Traps found on the way**: (1) BoW database entries must be pointer stable (`sv_bow_db` keeps pointers to the caller's
`sv_bow_db_keyframe` and its BoW vector; the first version indexed a `realloc`ed array -> use-after-realloc crash at the first
keyframe erasure after the table grew, fr3 frame ~1714). (2) `map_db_->last_inserted_keyfrm_->get_trans_wc()` is read live
(local BA / loop BA move it): the map view refreshes it from the record at the start of every frame. (3) Initial-map records
carry `first_keyfrm_id_ = 1` and `next_landmark_id_ = #initial landmarks`; keyframe 0's pass runs first. (4) The init frame's
frame statistics use the PRE-BA keyframe poses (`rel = pose_cw * ref->pose_wc` is computed before the BA).

**Results** (`python3 tools/run_stella_port_replay.py`; 0 differences everywhere; single thread, -O2: xyz 28 s, desk 27 s,
fr3 2 min; every number is "differences / compared items"; a mutation check -- `observed_ratio_thr` 0.3 -> 0.31 in a scratch
copy -- gives 232,777 / 975,657 mismatches with the first at frame 46):

| sequence | compared against | items |
|---|---|---|
| fr1_xyz (798 frames) | `reference_dumps/fr1_xyz` | frame_trace (path, initial + final pose bits, ref keyframe, tracked/reliable) 0/798; kf_decision (all flags) 0/798; frames_before 0/798; frames_after (kf + lm counts) 0/798; matches (landmark of every keypoint) 0/970,888; keyframes snapshot after every frame 0/12,491 rows; landmarks snapshot after every frame 0/640,402 rows; keyframe destruction frames 0/16; harness `check_sv_system` 0/975,657 |
| fr1_desk (598) | `reference_dumps/fr1_desk` | same set: 0/598 x4, matches 0/712,111, keyframes 0/18,037, landmarks 0/592,783, destruction 0/5; harness 0/715,592 |
| fr3_long_office (2585) | `reference_loop_eigen/fr3_long_office` (Eigen-solver reference, `--light`, no per-frame trace) | frames_before (pose text) 0/2585, frames_after 0/2585, destruction 0/21, accepted loops (kf 161 <-> 6, frame 2407) 0/1, ALL 141 keyframes right after the loop BA (pose bits, spanning tree) 0/141, ALL 4649 landmarks right after the loop BA (position, descriptor, normal, valid range, counters, reference keyframe, observations) 0/4649, `trajectory.tum` byte-identical |

The loop of fr3 is found with the REAL relocalization PnP (`sv_pnp_ransac`), not the injected trace: same inliers, same pose
graph, same loop BA (this also closes module 7's "PnP injected" open item). `trajectory.tum` of the port is byte-identical to the
reference's on all three sequences (`diff` = 0 lines).

**ATE** (`tools/tum_eval.py`'s own association + `benchmark.ate_rmse`, Sim3-aligned RMSE, m; numpy is not installed system-wide,
use a venv):

| sequence | poses | port vs GT | reference vs GT | port vs reference |
|---|---:|---:|---:|---:|
| fr1_xyz | 787 | 0.026058 | 0.026058 | 0.000000 (max position deviation 0.0) |
| fr1_desk | 543 | 0.017877 | 0.017877 | 0.000000 (0.0) |
| fr3_long_office (loop closed, Eigen solver) | 2560 | 0.036990 | 0.036990 | 0.000000 (0.0) |

**Reproduce**: `python3 tools/run_stella_port_replay.py --seqs fr1_xyz,fr1_desk,fr3_long_office [--keep]` (builds `sv_run`, generates
missing fixtures -- fr3 needs `dump_stella_fixtures`; on this machine its OpenCV needs `libtbb` from `/tmp/pose-opencv` and
gdcm / gdal / armadillo from `runs/stella_port/vocab/deps` (read-only), the tool sets `LD_LIBRARY_PATH` -- runs, compares,
prints the first divergent frame of every file and the ATE table; outputs in `runs/stella_port/replay/<seq>/`, the big
snapshot / matches files are deleted unless `--keep`; `--no-run` re-compares an existing run). The stand-alone driver:
`sv_run <vocab.fbow> <tum_seq_dir> <fixtures_dir> <out_dir> [max_frames] [--snap-every N|--no-snap|--snap-loop] [--snap-from F]
[--resume-mapper] [--no-loop] [--blank A-B]`. `runs/stella_port/fixtures/fr3_long_office` (794 MB of PGMs) can be deleted;
it is regenerated on demand.

**Open / not covered**: (1) reset and Lost -> relocalization inside a continuous run have no reference (see above); their
components (reloc, PnP) are validated in isolation by Codex's harnesses, the glue (`sv_reloc_tracking_glue` through the tracker
hook, `sv_tr_map_kf_any`, reset bookkeeping) only by the `--blank` stress runs. (2) The kf-lifetime rule is the module-7
retention rule (holders = the continuity sets); other holders (a frame's reference keyframe, the queue) never mattered on these
sequences (21/21, 16/16, 5/5) but are not modelled. (3) A single global-queue slot array of 8 keyframes (the synchronous mapper
queues at most one per frame). (4) Monocular / perspective / no markers / no temporal keyframes / ORB only, as before.
(5) Trajectory bookkeeping uses a plain 4x4 double inverse; it reproduces the reference file bytes on all three runs but is not
proven bit-exact against Eigen's `Matrix4d::inverse()`.

### Continuous replay on all 5 comparison sequences + ATE + speed (2026-10-01)
No port code was changed (only `tools/run_stella_port_replay.py`: a sequence without a full dump in `reference_dumps/` now
uses the light Eigen-solver reference in `reference_loop_eigen/<seq>`; default `--seqs` lists all five). `check_stella_port.py`
untouched / not rerun.

**Exactness** (port `sv_run` vs deterministic reference, no teacher forcing):
- fr1_floor (1242 frames, init at 24, **20 Lost frames, 19 relocalization attempts, no reset, no loop**, 139 keyframes, 23 erased/destroyed):
  against the light reference (`reference_loop_eigen/fr1_floor`): frames_before 0/1242, frames_after 0/1242, kf destruction 0/23,
  trajectory byte-identical. Additionally against a freshly generated FULL canonical dump (`run_stella_reference` without
  `--light`, 1.3 GB, deleted afterwards): frame_trace 0/1242, kf_decision 0/1242, matches 0/1,388,261, keyframes snapshot after
  every frame 0/74,735 rows, landmarks snapshot after every frame 0/3,018,481 rows. So Lost -> relocalization (Codex's
  `sv_reloc_tracking_glue` + the tracker hook) is bit-exact inside a continuous run: the "no reference" caveat only applies to
  `reset()`. The reference does NOT lose tracking within 5 s of the init on this sequence, so no reset is reached and the
  reference does not deadlock; exact comparison is possible to the end.
- fr2_xyz (3669 frames, init at 24, no loss, no loop, 79 keyframes, 49 erased/destroyed): light reference: frames_before 0/3669,
  frames_after 0/3669, destruction 0/49, trajectory byte-identical. (No full dump: ~2 GB, not generated; the light compare covers
  every frame's pose and the keyframe / landmark counts.)
- Neither sequence closes a loop, so canonical CSparse == Eigen-solver reference there (the loop-dump references are used).

**ATE** (`tools/tum_eval.py` association + `benchmark.ate_rmse`, Sim3-aligned RMSE, m; port == reference everywhere, max position
deviation 0.0):

| sequence | poses | port vs GT | reference vs GT |
|---|---:|---:|---:|
| fr1_xyz | 787 | 0.026058 | 0.026058 |
| fr1_desk | 543 | 0.017877 | 0.017877 |
| fr1_floor | 1199 | 0.021725 | 0.021725 |
| fr2_xyz | 3646 | 0.018721 | 0.018721 |
| fr3_long_office (loop, Eigen) | 2560 | 0.036990 | 0.036990 |

**Speed** (this machine, 16 cores but both single threaded, idle, mean of 2 runs, gcc -O2 `-ffp-contract=off`; port = `sv_run
--no-snap` on pre-decoded PGM fixtures, i.e. no PNG decoding; reference = `run_stella_reference --light` with OMP_NUM_THREADS=1,
OpenCV ORB + PNG decode, Eigen solver env; duration = rgb.txt time span, TUM is 30 Hz; the reference has a few small light-trace
files written too):

| sequence | frames | duration s | port wall s | port ms/frame | port RTF | ref wall s | ref ms/frame | ref RTF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| fr1_xyz | 798 | 26.6 | 25.9 | 32.5 | 1.02 | 12.4 | 15.6 | 2.14 |
| fr1_desk | 598 | 20.4 | 24.1 | 40.2 | 0.85 | 9.1 | 15.3 | 2.24 |
| fr1_floor | 1242 | 46.3 | 82.4 | 66.3 | 0.56 | 17.3 | 13.9 | 2.68 |
| fr2_xyz | 3669 | 122.3 | 108.5 | 29.6 | 1.13 | 58.4 | 15.9 | 2.10 |
| fr3_long_office | 2585 | 87.1 | 122.2 | 47.3 | 0.71 | 37.9 | 14.6 | 2.30 |

The port is 1.9x - 4.8x SLOWER than the reference (not real time on 3 of 5). Stage breakdown (seconds, scratch timing build of
`sv_system.c` with `clock_gettime` around the stage calls, NOT in the repo; trajectories byte-identical to the default build):

| sequence | extract (ORB) | tracking (incl. reloc) | mapping (incl. local BA, culling) | loop detect (BoW query) | loop validate + correct | rest (undistort, bookkeeping, I/O) |
|---|---:|---:|---:|---:|---:|---:|
| fr1_xyz (23.4 wall) | 19.2 | 0.9 | 1.1 | 1.1 | 0 | 1.1 |
| fr1_desk (23.1) | 14.1 | 0.4 | 1.4 | 5.6 | 0.002 | 1.6 |
| fr1_floor (80.8) | 26.2 | 1.6 (reloc 0.6, 19 calls) | 2.8 | **46.7** | 0.01 | 3.5 |
| fr2_xyz (101.7) | 85.9 | 4.6 | 3.9 | 3.8 | 0 | 3.5 |
| fr3_long_office (117.9) | 57.8 | 2.1 | 3.2 | **50.2** | 0.3 | 4.3 |

Findings: (1) `sv_orb_extract` is 70-85 % of the time on the sequences without a loop-detect hot spot (~24 ms/frame at 640x480,
vs the reference's whole frame in ~15 ms) -- the main target for a speed phase. (2) `sv_loop_detect` is 330-370 ms per
keyframe on fr1_floor / fr3 (and 95 ms on desk) vs ~13-48 ms on xyz / fr2: it grows with database size and revisits (it sits in
the BoW database query / candidate scoring path, `sv_bow_db.c` is Codex-reserved; not investigated or touched). (3) Tracking,
mapping, loop correction are negligible. perf is unavailable here (`perf_event_paranoid=4`), so no function-level profile.
Temporary dumps, PGM fixtures of floor / fr2 / fr3 and the venv were deleted; `reference_dumps/fr1_floor` removed.

## Speed phase (2026-10-01): 2.2x - 5.2x faster, still bit-exact
Goal: faster without changing a single bit. `check_stella_port.py`: **56 rows, 0 mismatches** (all PASS, incl. `check_sv_extract`
0/970,888 + 0/712,111, `check_sv_system`, `check_sv_loop` x20). Continuous replay (`tools/run_stella_port_replay.py`, all 5 sequences, numpy venv):
every one of the 30 comparison rows 0, trajectories byte-identical, ATE unchanged (0.026058 / 0.017877 / 0.021725 / 0.018721 / 0.036990).
Flags unchanged (`-std=c99 -O2 -ffp-contract=off -fno-fast-math`), no intrinsics, same headers only.

**Profiling method.** `perf` and `gdb -p` are blocked; `valgrind` is absent. Used `gcc -pg` + `gprof` in a scratch dir, plus `clock_gettime`
timers in scratch copies. Caveat: gprof does not see libc (malloc/realloc/memmove/qsort) -- the loop-detect hot spot was INVISIBLE in it
(the gprof total was 22 s of an 80 s run) and had to be found by reading the code path + a scratch timer.

**Top functions before (gprof flat, self time)**, fr1_xyz (21.8 s profiled; run 31.7 s): `sv_gaussian_blur7_u8` 49.3 %, `sv_fast_detect` 17.2 %,
`sv_bow_transform` 8.3 %, `sv_corner_score16` 6.2 %, `sv_resize_linear_u8` 6.0 %, `sv_orb_extract` 4.7 %, `sv_frame_get_keypoints_in_cell` 1.4 %,
`sv_cv_round_f` 1.0 % (998 M calls, an out-of-line `nearbyintf` each), then all < 0.5 % (`sv_umap_insert`, `sv_quat_map`, `sv_pose_opt_edge_*`,
`sv_undistort_point`, `sv_tr_hamming`, `sv_ba_k_schur_sub`, `build_system`, `nodes_find_or_create`, `sv_bow_db_add`, ...). fr1_floor (22 s profiled of 80 s):
`sv_bow_transform` 30 %, `sv_fast_detect` 17.6 %, blur 15.3 %, `sv_orb_extract` 12 %, resize 9 %, `sv_corner_score16` 3.5 %, `sv_bow_db_add` 2.8 %, rest < 1 %.

**Top functions after** (same method, fr1_xyz 10.5 s / fr3_long_office 31 s profiled): fr1_xyz: `sv_fast_detect` (score inlined) 38.9 %, blur 20.9 %,
`sv_orb_extract` (IC angle, descriptors, grid) 15.3 %, resize 11.4 %, `sv_frame_get_keypoints_in_cell` 3.1 %, everything else < 1 % each (`sv_bow_transform` 0.9 %).
fr3_long_office: `sv_fast_detect` 37.2 %, blur 23.4 %, `sv_orb_extract` 15.2 %, resize 11.1 %, `sv_frame_get_keypoints_in_cell` 1.6 %, `sv_bow_transform` 1.0 %,
rest < 1 %. Extraction is now ~95 % of the profiled time; tracking + mapping + loop closing together are ~1.5 ms/frame.
Scratch stage split of `sv_orb_extract` on fr1_xyz (ms/frame): pyramid 1.45, FAST 4.6 (of which score ~1), distribute 0.07, IC angle 0.32, blur 2.87,
descriptors 1.75 (total 12.1; 24 before).

### Where the 330-370 ms/keyframe of `sv_loop_detect` went (answer: NOT in Codex-reserved code)
`sv_loop_detect` (Claude-owned `sv_loop.c`) **rebuilt the whole BoW database from scratch for every query**: `sv_bow_db_create` + `sv_bow_db_add` for all
`n_db` keyframes (hundreds of words each; each new word is a sorted-array insert with `memmove` of the bucket array and each posting a `realloc`-grown
list), then queried it once and destroyed it. O(total postings) allocation + memmove per keyframe, all inside libc, which is why gprof showed nothing.
It was also why the cost grew with database size (floor / fr3: 140-160 keyframes; desk: 58 with many words). `sv_system` already keeps a persistent
database (`s->db`) with identical content (`in_db`), so the rebuild was pure waste. Fix: `sv_loop` got an optional `ext_db` + `ext_dbk(user, id)` callback
(`sv_loop.h`), `sv_system.c` `wire_hooks` points them to its persistent database; `sv_loop_detect` queries it directly and resolves the reject set through
the callback (same keyframe entries, same ids, same candidate order -- the query sorts by id anyway). The old rebuild path stays when `ext_db` is NULL
(used by `check_sv_loop`, which feeds `db_ids`). Result: loop detect 1.4 ms/keyframe on fr1_floor (0.18 s total over 140 keyframes, from 46.7 s).

### Speed: for Codex (no change made; `sv_bow_db.c` is reserved)
Not a hot spot any more, but `sv_bow_db_query` is O(postings x distinct-candidates) and O(postings x n_reject): for every posting of every query word it
linearly scans `reject[]` and `r.matches[]` (the candidate list built so far) to find/insert the keyframe. With a persistent database of N keyframes this is
~(words x bucket size x N) per query (a few ms at 160 keyframes, quadratic growth in map size). Exact-preserving fix: give each `sv_bow_db_keyframe`
a dense slot (e.g. a per-query `uint32_t slot_of[max_id+1]` / generation-stamped array indexed by `keyframe->id`, or a small open-addressing table)
for both the reject test and the match lookup; the final `qsort` by id and all counts are unchanged, so results (including order) stay identical.
`sv_bow_db_add` could likewise skip the per-call `calloc(created)` and use `lower_word` once per word (it already does) -- minor.

### Changes (all exact; files re-read fresh before editing, license headers untouched)
- `sv_loop.{h,c}`, `sv_system.c`: persistent-database query (above). The biggest single win on fr1_floor / fr3_long_office.
- `sv_image.c`: `sv_gaussian_blur7_u8` -- the same fixed-point 7-tap separable blur, but: horizontal pass on a padded (BORDER_REFLECT_101) line copy so no
  per-tap border tests, uint16 intermediate (max 255*256), vertical pass over 7 row pointers (reflect once per row, not per pixel), int32 instead of int64
  accumulation (max 255*65536 fits), symmetric kernel pairs added first; all integer so reordering is exact. `sv_resize_linear_u8`: row0/row1 temporary
  arrays removed, horizontal and vertical formula fused per output pixel (same integer expressions). Removed the unused `sv_saturate_u8`.
- `sv_image.h`: `sv_cv_round_f` is now `static inline`: for |v| < 2^22 it uses `(v + 12582912.f) - 12582912.f` (round-half-even by the float add itself,
  guarded by `__FLT_EVAL_METHOD__ == 0`), else the old `nearbyintf`. Identical to `nearbyintf` in the default rounding mode; removes ~1 G out-of-line calls per run.
- `sv_fast.c`: replaced the 25-step data-dependent `count > K` verification loop by two 16-bit circle masks (darker than `v - t` / brighter than `v + t`)
  and `sv_has_run9` (cyclic run of >= 9 set bits via shift-and); a dark and a bright run cannot coexist. `sv_corner_score16` takes the known side and skips
  the other side's loop (provably a no-op: two 9-arcs on the 16-ring share a pixel). The early antipodal rejects are unchanged.
- `sv_bow.c`: `hamming32` uses 4 x 64-bit XOR + SWAR popcount (was 32 per-byte `while (x)` popcounts).
- Tried and rejected (no gain or worse, reverted): branch-free first FAST stage (+4..12 % slower), hoisting the 8 offsets into locals, blur without the padded
  line copy. FAST is ~12 cycles/pixel scalar; the rest needs SIMD, which the rules forbid.

### Speed (this machine, idle, single run each, `sv_run --no-snap` on pre-decoded PGMs, same method as 2026-10-01; before -> after)
| sequence | frames | port ms/frame | port RTF | port wall s | speed-up | reference ms/frame (OpenCV ORB + PNG decode) |
|---|---:|---:|---:|---:|---:|---:|
| fr1_xyz | 798 | 32.5 -> 14.3 | 1.03 -> 2.32 | 25.9 -> 11.4 | 2.3x | 15.6 |
| fr1_desk | 598 | 40.3 -> 14.2 | 0.85 -> 2.41 | 24.1 -> 8.5 | 2.8x | 15.3 |
| fr1_floor | 1242 | 66.3 -> 12.7 | 0.56 -> 2.94 | 82.4 -> 15.8 | 5.2x | 13.9 |
| fr2_xyz | 3669 | 29.6 -> 13.7 | 1.13 -> 2.44 | 108.5 -> 50.2 | 2.2x | 15.9 |
| fr3_long_office | 2585 | 47.3 -> 12.5 | 0.71 -> 2.70 | 122.2 -> 32.3 | 3.8x | 14.6 |
The port is now faster than the reference on all five (reference numbers are the earlier measurement, with PNG decode; the port reads PGM) and runs real time
with ~2.3-2.9x margin. Remaining cost is ~95 % ORB extraction: FAST ~40 %, blur ~21 %, descriptors + IC angle ~15 %, pyramid ~11 %.
Possible further (exact) ideas: the BoW vector of a keyframe is computed twice (`sv_loop` `kf_bow` and `sv_system` `db_add`; ~1 ms per keyframe, negligible now);
blur only the tiles that contain keypoint patches (probably ~20 % of blur at best).
