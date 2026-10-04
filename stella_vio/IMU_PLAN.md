# IMU integration plan for stella_vio

2026-10-02. Design only; no IMU implementation or new accuracy result is
claimed. `stella_vio/` is the development fork described in [README](README.md).
Its owner is changing the visual frontend and initialization concurrently;
this task reads its sources and writes only this plan. The exact
`stella_port/` baseline and the paused OKVIS2 port remain separate.

## Decision and first deliverable

Keep the ORB frontend and add inertial support in two measurable stages:

1. **Rotation assistance:** calibrated gyro prediction for matching and
   motion compensation, with gravity alignment only when justified. This
   remains monocular visual SLAM with arbitrary scale.
2. **Metric VIO:** jointly estimate poses, velocities, gyro/accelerometer
   biases, initial gravity direction and scale, using image observations
   and IMU factors in a bounded temporal window.

Start with the data/time contract and an adapter around the already validated
OKVIS IMU leaf. Do not launch another full OKVIS module. BRISK is already
complete; its detector/descriptor is not needed by this plan.

The main uncertainty is successful initialization and continuous operation on
phones, not whether we can integrate acceleration. Low-parallax or unexcited
motion can leave scale and biases poorly observable. Rotation assistance may
help tracking but cannot manufacture metric scale or missing visual geometry.

## Evidence and prerequisite baseline

[Phone/handheld study, section 8](../docs/gnss_vio_benchmark_20261001.md#8-phonehandheld-robustness-2026-10-02)
reports weak phone initialization, large scale drift, and a rotation burst
that defeats visual tracking on the handheld rig. Full-resolution Stella
tracks much more of Outdoor-1 than its reduced-resolution variant. The same
study found a 720p candidate-buffer overflow, fixed-size driver assumptions,
and incorrect handling of headerless timestamp files in the old driver.

The fork README now describes variable-size input and comment-aware parsing;
its source already exposes new visual initialization/restart options. Treat
the study's defect list as regression cases, not proof that today's fork is
still broken. Before IMU integration, have the visual owner freeze a source
hash and publish:

- ASan/UBSan results for dense 1280x720 images, several supported dimensions,
  allocation failures, and calibration/image-size mismatch rejection.
- Frame-to-timestamp identity with blank/comment/headerless lists; matching
  image, exposure timestamp and fixture index; unchanged intended TUM behavior.
- Complete camera-only runs and configuration hashes on the phone set below,
  including restarts, map IDs, missing poses, initialization latency and scale
  diagnostics. Freeze the vocabulary choice across each comparison.

The old Outdoor-1 result is one run with scratch driver changes and approximate
GT synchronization. It motivates experiments; it is not a clean baseline for
the current fork. Its 5.73 m GNSS-only score is SE3-aligned on that recording,
not an absolute-position guarantee or a target to tune every sequence against.

## Existing code and integration boundaries

| Read-only starting point | What it supplies | Required extension in the fork |
|---|---|---|
| `c/sv_system.{h,c}`, `sv_tracking.c`, `sv_track_frame.c` | Image/timestamp feed, tracking, resets, relocalization | IMU ingestion, sensor state, mode and validity reporting |
| `c/sv_init.c`, `sv_solve_*`, `sv_match_area.c` | Visual initialization and tracks | Multi-frame visual-inertial initialization with observability gates |
| `c/sv_g2o_pose_optimizer.{h,c}` | A single 6-DoF pose with fixed landmarks | A 15-DoF current inertial state and a correctly accounted prior |
| `c/sv_g2o_ba.{h,c}`, `sv_bundle_adjuster.{h,c}` | Specialized 6/3 pose/landmark Schur system | 15/3 state/landmark system and binary IMU factors |
| `c/sv_mapping.c`, `sv_map.*`, `sv_loop.*` | Keyframe/map lifetime and visual loop corrections | Temporal links, inertial lifetime, metric loop consistency |
| `../okvis_port/c/ok_imu.{h,c}` | Propagation, preintegration, append, whitened residuals and Jacobians | Thin convention adapter and independent regression coverage |
| `../okvis_port/c/ok_eigen.{h,c}` | Exact numerical kernels used by the IMU leaf | Preserve MPL provenance when reused |

These are integration locations, not permission to overwrite concurrent work.
Proposed new filenames below are provisional until the fork owner reserves
them. Changes to shared state types, optimizers and the feed loop should be
integrated by one owner after leaf interfaces are agreed.

Keep C99 and libm-only production code. A development reference/test tool may
use the existing permitted C++ libraries or independently written numerical
checks. No ORB-SLAM, OpenVINS, VINS, GVINS, `orb_port/`, GPL glue or other GPL
implementation sources are input to this work. Existing benchmark reports
and dataset/calibration records can be used without reading those sources.

Licensing: `ok_imu.h` and the present implementation declare
`BSD-3-Clause AND MPL-2.0`, and use the MPL `ok_eigen` helpers. Reuse is not a
BSD-only addition. Retain the OKVIS2/ROS/Eigen notices in
[OKVIS NOTICE](../okvis_port/NOTICE), preserve upstream-derived file boundaries,
and update the fork's notices/inventory when code is imported. Existing
[Stella provenance qualifications](../stella_port/NOTICE) also continue to
apply; this plan does not resolve them.

## Sensor, state and time contract

Use `T_AB` to mean the transform taking coordinates in B into A. Let B be
the IMU body, C the camera, and W the gravity-aligned local world:

```text
state_i = (R_WB, p_WB, v_WB, b_g, b_a)       # 15 local degrees of freedom
T_BC    = fixed calibrated camera-to-body transform
T_WC    = T_WB * T_BC
T_CW    = inverse(T_WC)                      # existing Stella pose convention
g_W     = (0, 0, -g)                        # physical downward acceleration
t_imu   = t_camera + delta_t                # explicit offset sign
```

Positions and extrinsic translations are metres, time seconds, angular rate
rad/s and accelerometer readings specific force in m/s². Velocity is in W;
biases and raw measurements are in B. Preserve the metric camera/IMU lever
arm when converting or scaling a visual map. Do not multiply the calibrated
extrinsic translation by the estimated visual scale.

Document quaternion order, multiplication and perturbation conventions at
the adapter boundary. OKVIS stores `[p, q_xyzw]` for `T_WS` and
`sb=[v_W,b_g,b_a]`; it subtracts a positive-Z `g_W` internally. That is compatible
with the downward physical gravity above after an explicit convention mapping.
Stella stores world-to-camera poses and left-multiplies the SE3 update in
`[omega, upsilon]` order. Storage transposes alone do not convert Jacobians.

Use integer nanoseconds on a documented clock internally, with a sequence
origin for conversion to OKVIS `sec/nsec`; never truncate an epoch timestamp
to a floating-point frame index. Store the original stamp, clock domain,
exposure reference (start/mid/end), image orientation and crop/resize transform.
Do not confuse camera–IMU offset with camera–GT or GNSS clock offsets.

Buffer raw IMU samples in timestamp order with bounded retention. Reject or
explicitly flag duplicates, backwards stamps, missing brackets, saturation,
and excessive gaps; do not silently interpolate across a long outage. Retain
both endpoint-bracketing samples. Integrate every IMU sample between image
exposure times even when frames are skipped. Replay waits for the final
bracketing sample; live operation uses an explicit latency budget and marks
unsupported prediction as such. Ingest raw accelerometer data including
gravity, not an OS 'linear acceleration' channel with gravity removed.

Calibration config records intrinsics/distortion at the actual resolution,
`T_BC`, gyro/accelerometer scale and axis conventions, noise densities, bias
random walks, initial bias uncertainty, gravity magnitude, timing parameters
and optional sensor readout time. Begin with fixed calibrated extrinsics and
noise; estimating all calibration variables during a weak phone startup is
not the first milestone. Continuous noise density and per-sample variance
must not be interchanged. See the primary
[Kalibr noise-model documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
for these units and discretization conventions.

## Preintegration and rotation assistance

Wrap the existing leaf before changing its arithmetic:

- `ok_imu_propagation`: predict pose/velocity from a known metric state.
- `ok_imu_error_init`, `ok_imu_redo_preintegration`: build an interval at
  reference biases, retaining covariance, bias Jacobians and raw samples.
- `ok_imu_append`: validate interval extension/merge semantics before using
  it for non-keyframes or keyframe culling.
- `ok_imu_evaluate`: obtain the 15-component residual and ambient pose/bias
  Jacobians. Its residuals and Jacobians are already whitened. Apply whitening
  exactly once, and preserve the native residual/tangent order.

Chain the ambient 15x7 pose Jacobians through the chosen 7x6 local update and
the camera/body transformation. Verify the resulting tangent Jacobians by
central differences using exactly the solver's update rule. The reference's
orientation-error formula and operation ordering remain authoritative for
this reusable leaf; do not replace them with an assumed SO(3) log formula
while claiming bit identity.

Reintegrate after material bias changes, changed interval endpoints/time
offset, or stale covariance. Define and validate thresholds with recorded
error bounds; do not silently apply first-order bias correction arbitrarily
far from its reference. Keep raw samples until every active factor and prior
that may need rebuilding is finalized. Test append versus fresh integration
against the native behavior; do not assume floating-point associativity.

First use the gyro integration to rotate predicted bearings and seed existing
projection matching. Keep geometric verification and visual outlier tests.
Report this mode as rotation-assisted, nonmetric. On static segments, the
accelerometer can estimate gravity direction and the gyro mean can seed bias;
`ok_imu_init_pose` assumes no acceleration and is not a dynamic initializer.
Accelerometer magnitude alone does not prove stationarity. Do not tilt the
map on every walking-frame accelerometer reading.

The mathematical background for preintegrated motion, uncertainty and bias
correction is [Forster et al., On-Manifold Preintegration](https://arxiv.org/abs/1512.02363).
Our initial implementation reference is the pinned OKVIS leaf, whose formulas
and conventions need not be bit-identical to another paper's formulation.

## Visual-inertial initialization for phone starts

Use explicit states: `WAITING_FOR_IMU`, `ROTATION_ONLY`, `VISUAL_NONMETRIC`,
`INITIALIZING_VI`, `TRACKING_METRIC`, and `LOST`. A nonmetric visual map may
exist while the metric initializer waits. Publish its mode and uncertainty;
do not present arbitrary units as metres.

1. Collect a sliding multi-frame window, long-lived visual tracks and fully
   bracketed IMU intervals. Use gyro-assisted rotation to keep tracks across
   early turns. Estimate translation only when the observations constrain it.
   Reject unstable triangulation rather than lowering parallax thresholds
   until a two-frame initializer accepts a nearly planar scene.
2. If a static interval is detected from gyro, acceleration variation and
   visual motion together, initialize gyro bias and coarse gravity. A dynamic
   start must proceed without that assumption. Accelerometer bias and gravity
   cannot generally be separated from a single static orientation.
3. Obtain a provisional multi-view visual reconstruction in arbitrary units.
   Retain well-conditioned points; deferred/near-infinite tracks can still
   constrain rotation. Use visual rotations and preintegrated gyro rotations
   to refine gyro bias, then redo affected integrations.
4. Fit scale `s`, per-frame velocities, gravity direction and accelerometer
   bias using position/velocity constraints over the whole window, with
   calibrated camera/body lever arm and explicit bias priors. Constrain
   `|g_W|=g` and refine its direction on a two-dimensional tangent plane;
   parameterize positive scale with `log(s)` in nonlinear refinement.
5. Jointly refine the window's poses, landmarks, velocities and biases with
   visual and IMU residuals. Fix translation and yaw gauge without falsely
   fixing observable roll/pitch during gravity estimation. Estimate initial
   gravity in a provisional frame, then rotate into the OKVIS gravity-aligned
   frame before using its fixed-gravity evaluator in normal tracking.
6. Admit the metric solution only if the gauge-projected, properly scaled
   system has adequate rank/conditioning, finite plausible bias estimates,
   bounded scale uncertainty, visual depth/reprojection support, and stable
   estimates across successive windows. Specify these gates before held-out
   evaluation. A positive scale or a low final residual alone is insufficient.
7. Apply scale/rotation/translation atomically to map points, camera/body poses,
   velocities and cached motion state; transform/rebuild covariances and
   priors consistently. Preserve physical extrinsics. Continue checking scale
   and bias stability during an initial probation interval.

Pure rotation, constant-velocity low-parallax motion and some restricted
accelerations cannot supply all needed information. Remain in a declared
nonmetric/rotation mode, extend the window within memory limits, or restart
when geometry improves. Do not force metric initialization after a fixed
timeout, use GT to pick the scale, or infer scale just from gravity. GNSS is
excluded from this initializer so the VIO experiment remains independently
measurable. More advanced structureless initialization is a later hypothesis
if the multi-view approach still fails, not an implied solved capability.

## Tracking optimizer, local BA and retained information

Start a separate visual-inertial optimizer path; keep the visual-only path
available for controlled A/B runs. Existing 6/3 arrays in `sv_g2o_ba` cannot
be made inertial by attaching an extra scalar pose weight.

For frame tracking, optimize the current 15-DoF state against existing map
points plus the preceding state's inertial constraint and a prior. A first
diagnostic may fix the previous state, but that is conditional optimization
and understates its uncertainty. Before using the result as an estimator
prior, account for previous-state uncertainty by joint optimization and
elimination. A gyro pose seed alone is only a predictor, not a VIO optimizer.

For local BA, use a **contiguous temporal core of inertial states**, augmented
by visual covisibility neighbors. Add reprojection edges and one IMU interval
factor between each adjacent retained temporal pair. Include velocity/bias
blocks even for visually weak states needed to preserve the chain. Use the
full preintegration covariance, including its cross-correlations; don't add
a second bias random-walk factor if already represented by the OKVIS factor.

Implement a small dense 15-state-block reference first (for example, 5–10
keyframes); compare normal equations, accepted LM steps and final cost with
the sparse implementation. Eliminate independent 3D landmarks by Schur
complement while retaining pose–velocity–bias and inter-state couplings. Keep
robust image residuals, explicit IMU saturation/gap handling, and tested
Jacobian signs. A robust loss must not conceal a bad extrinsic or clock offset.

A bounded estimator needs a defined information-retention policy. Prototype
a fixed temporal boundary first, label its limitations, then implement
marginalization before claiming a consistent sliding-window estimator.
Eliminate outgoing states and connected landmarks with a rank-aware
square-root/Schur prior; retain its linearization point and tangent convention.
Specify a fixed/first-estimate linearization policy for marginalized terms
and test its gauge nullspace. Reuse of a marginal prior plus its original
factors double-counts information: every measurement must have exactly one
active representation. Conversely, discarding old states without their
information changes the estimator and must be measured.

Keep the first implementation's time/extrinsic calibration fixed once its
prior is formed. If calibration subsequently changes, either rebuild all
affected factors/prior from retained history or deliberately restart the
window; stale marginalized factors cannot simply be retimed.

## Culling, loss, relocalization and loop closure

Keyframe culling must preserve the temporal IMU chain. When removing an
intermediate keyframe, reconstruct the spanning interval from retained raw
samples using validated append/reintegration semantics. If those factors
are already absorbed into a prior, update the factor ownership first; never
both retain the old prior contribution and add the same measurements again.

During a brief visual dropout, propagate with growing covariance and report
`predicted`, not `visually tracked`. Use a configurable time/uncertainty limit;
IMU-only output must not silently inflate successful tracking coverage. After
a long gap or device clock reset, reinitialize rather than carrying old bias,
velocity, timestamps or interval factors across unrelated maps.

PnP relocalization recovers pose, not velocity or biases. Reacquire a short
inertial window and validate those states before declaring metric tracking
restored. A restart has its own map ID and scale-valid flag. Existing fork
restart behavior can create disconnected maps; do not concatenate them into
one apparently continuous trajectory or align each separately as a single
successful run.

Initially disable loops for VIO development and state that scope explicitly.
After metric gravity-aligned initialization, visual Sim3 scale corrections
must not arbitrarily rescale the inertial map. Implement gravity-preserving
yaw/translation loop correction (4 DoF), or a fully consistent inertial
global optimization. Rotate world velocities with the map, keep body-frame
biases in their body convention, and transform or rebuild affected priors.
For nonuniform corrections, refresh the active window rather than rotating
one global prior blindly. Metric/nonmetric map merging is a later explicit
feature. Validate loop continuity and inertial residuals before re-enabling
the existing visual loop path.

## Camera–IMU time offset and phone rolling shutter

First use calibrated, fixed timing. Estimate a coarse offset from vector
visual angular increments and gyro integration in a common frame, then fit
it on an excited calibration segment using rotational/reprojection residuals.
The [Kalibr camera–IMU calibration documentation](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
describes joint spatial/temporal calibration; use its documented conventions
as a cross-check, not a substitute for recording our own offset sign.

The study's broad angular-speed correlation peak does **not** establish
millisecond synchronization: it cannot exclude roughly 10–30 ms on the phone.
The approximately 293 s GT clock shift is unrelated. Freeze GT alignment
metadata independently of the estimator; do not tune the sensor offset by ATE.

After fixed-offset VIO passes, optionally add a bounded scalar `delta_t` with
a calibration prior. Its updates change integration endpoints, sample
interpolation, exposure poses and Jacobians; rebuild/relinearize those terms.
Test derivatives at sample-boundary changes, which are piecewise smooth.
Freeze the estimate when excitation is weak, and report uncertainty. If logs
show clock drift, diagnose it before adding a clock-rate parameter; a constant
offset model cannot correct changing clock rates or variable pipeline latency.

For rolling shutter, preserve each feature's original **sensor row** before
undistortion, resizing, cropping or image rotation. Define the timestamp
reference row and signed first-to-last-row readout interval explicitly:

```text
t_feature = t_camera + delta_t
          + (row_sensor / (sensor_height - 1) - reference_row_fraction) * readout
```

Require `sensor_height > 1`. A mid-readout timestamp uses reference fraction
0.5; this must follow sensor metadata rather than an assumption about file
names. Row-center/exposure conventions and readout direction belong in config.
The [Kalibr rolling-shutter calibration guide](https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration)
provides a primary reference for measuring shutter parameters.

Begin with measured readout and gyro rotation compensation for bearing
prediction/initialization. This is an approximation: translation, changing
depth and exposure blur remain. The complete model evaluates camera pose at
each feature's row time using IMU propagation/interpolation and includes that
dependence in residual Jacobians. Avoid correcting a pixel once and again
inside the residual. Keep raw ORB descriptors initially; evaluate descriptor
warping separately if needed.

Do not simultaneously free time offset, readout, extrinsics and biases at a
low-excitation startup. Calibrate readout offline first; estimate it online
only with bounded priors and observable motion. A fitted readout can absorb
other modeling errors. Include readout=0, positive/negative direction, rotated
images and known synthetic readout tests. Rolling shutter is a plausible
phone contributor in the study, not an established cause of its failures.

## Module order and completion gates

| Order | Proposed scope | Completion gate |
|---|---|---|
| M0 | Frozen visual baseline and sensor/calibration contract | Full phone baseline, parser and 720p memory checks recorded by visual owner |
| M1 | `sv_imu_buffer.*`, `sv_imu_adapter.*`, conversion checks | Time brackets, gaps, units, poses and perturbations verified; unchanged OKVIS leaf exactness |
| M2 | Gyro prediction and optional static gravity seed | Rotation-only tests and complete gyro-assisted A/B; output explicitly nonmetric |
| M3 | `sv_vi_factor.*`, small dense `sv_vi_optimizer.*` | Residual/Jacobian/whitening, gauge and dense normal-equation checks |
| M4 | `sv_vi_init.*`, multi-frame VI initializer | Excited starts recover scale/gravity/bias; degenerate starts refuse false confidence |
| M5 | Tracking and temporal local BA integration | Continuous metric replay without injected reference states; sparse/dense agreement |
| M6 | Prior/marginalization, culling, reset and relocalization | Bounded memory; no factor duplication; dropout/restart/map-ID regression suite |
| M7 | Fixed calibrated rolling-shutter model; optional online time offset | Known offset/readout recovery and improved held-out residuals without EuRoC regressions |
| M8 | Metric loop correction and final multi-sequence evaluation | Consistent state/prior corrections; complete acceptance tables and limitations |

Timing metadata and offline calibration begin in M1; M7 is the later model
extension, not permission to ignore timing earlier. M3 and M4 may be developed
as separate leaves after conventions are frozen, but shared tracking/BA
integration stays with one owner. M5–M6 and phone initialization are the likely
hard parts. No calendar estimate is justified until M1 and a small end-to-end
window expose the remaining interface and conditioning issues.

## Validation plan

### Numerical and lifecycle checks

Use two distinct standards: bit-exact native comparison for reused OKVIS
kernels on the pinned toolchain, and independently checked numerical accuracy
for the new Stella+IMU estimator. There is no upstream combined estimator
whose entire trajectory this fork can honestly claim to reproduce exactly.

Retain the existing `check_ok_imu` propagation/preintegration/append/evaluate
fixtures and deterministic two-process native dumps. Record reference/source
hashes, flags (`-ffp-contract=off`, no fast-math), and every compared field.
Add adapter tests with identity and nontrivial extrinsics, nonzero lever arm,
quaternion sign equivalence, nonzero biases, and endpoint interpolation.

Use independent analytic/synthetic checks: stationary sensor (no translation
drift in a noiseless case), pure rotation, known acceleration, biased motion,
nonuniform sampling, clipped samples, missing packets and timestamp resets.
Check covariance symmetry/positive semidefiniteness and Monte Carlo residual
consistency. Compare tangent Jacobians with central differences over an
epsilon sweep; predeclare absolute/relative tolerances and scaling before
accepting results, and report maxima and worst cases rather than averages.

Compare a small dense solve against Schur elimination, LM accept/reject steps,
and full-batch versus marginalized linearized solutions at their common
linearization point. Test unobservable translation/yaw directions and
initialization's additional degenerate modes. Exercise culling, reset,
relocalization, loop transforms, bias reintegration and calibration changes.
Run ASan/UBSan including long 720p replays; distinguish unavailable leak
checking from a successful leak check.

### Sequence matrix and controlled experiments

| Data | Development role | Evaluation obligation |
|---|---|---|
| EuRoC cam0 + IMU: MH_01_easy, V1_01_easy | Calibrated global-shutter integration and initialization diagnosis | Entire recordings; record early moving starts as well as static windows |
| Remaining EuRoC MH_02–05, V1_02–03, V2_01–03 | Held-out motion, blur, difficult starts | Full mono+IMU sweep before a broad VIO claim; report missing/failed sequences |
| Mobile-GVIO Outdoor-1 | Known low-parallax, scale-drift case | Original full-resolution frames; match the study's 15 fps setup first, then separately test native rate |
| Mobile-GVIO Outdoor-2, Indoor-1, Indoor-2 | Held-out phone scenes | Confirm availability/calibration/GT; full runs, with no per-sequence ATE tuning |
| ADVIO 15 and 20 | Different phone/indoor motion | Audit camera/IMU timestamps, extrinsics and GT quality before scoring |
| `complex_environment` handheld recording | Fast-rotation/dropout stress test, not a phone | Use dataset records only; audit accelerometer scale and camera frame convention |
| Existing five TUM visual sequences | Camera-only regression | No synthetic claim of full gyro+accelerometer data where unavailable |

The current phone tools already name indoor1/indoor2/outdoor2/advio15/advio20
and clock offsets. Read their latest generated results before freezing the
manifest; section 8 predates that ongoing work. They are candidate validation
inputs, not a claim that all have already been run or calibrated for this fork.
Reserve Outdoor-1 for development and the additional phone recordings for
held-out evaluation. Freeze calibration on separate calibration segments;
record any use of a dataset author's supplied calibration.

Run ablations in order: frozen camera-only, gyro assistance, metric VI with
fixed timing/global-shutter model, calibrated timing, rolling-shutter model,
then metric loop closure. Use identical frames, calibration inputs, vocabulary
and feature settings wherever that comparison allows. Report independent
effects; don't change resolution, vocabulary, initializer and sensor noise at
once and attribute the result to IMU. Repeat nondeterministic references at
least three times and retain the full spread. Test deterministic replay in
fresh processes. Measure runtime serially on the same machine with build
flags, preprocessing costs and frame counts recorded.

### Metrics and promotion

- Primary metric for metric VIO: **SE3-aligned ATE RMSE, scale fixed to one**.
  Reuse the existing scorer's `benchmark.umeyama_alignment(...,
  with_scale=False)` path. For monocular Sim3 diagnostics use
  `benchmark.ate_rmse`; it estimates scale and must not be labeled SE3 ATE.
  Do not introduce another alignment implementation.
- Report Sim3 ATE and its fitted scale separately to expose scale error,
  alongside local scale stability, translational/rotational RPE, initialization
  latency, bias/gravity estimates and their uncertainty diagnostics.
- Publish both emitted-pose coverage and visually constrained coverage,
  with duration, prediction-only stretches, failures, resets and disconnected
  maps. Pair full-sequence reports with same-timestamp-window comparisons;
  a short accurate surviving map is not full-sequence success.
- Match GT clock, body/camera frame and lever arm before scoring. For phone
  LiDAR-rig GT, report unresolved calibration and synchronization uncertainty;
  keep existing estimated offsets fixed across candidates. Never choose an
  offset or independently align each reset map to hide a failure. Report
  pre-initialization and post-initialization spans separately.
- Save causal online output separately from loop-corrected/offline trajectories.
  Report latency, wall time/data duration, peak memory and frame drops. GNSS
  remains a separate baseline and later fusion experiment; do not feed its
  fixes into a supposedly IMU-only scale evaluation.

Predeclare acceptance thresholds from the frozen baseline and measurement
uncertainty. Require stable metric scale and initialization, no unexplained
coverage regression, and complete per-sequence ATE tables before promotion.
Short windows are diagnostic. Record rejected variants with numbers in the
fork's results record; do not overwrite canonical benchmark artifacts.

Use dedicated `runs/stella_vio/imu/<experiment>/` outputs with manifest,
source/config hashes, calibration, input timestamps, seed, trajectory, mode
trace and metrics. Follow the repository's data policy: EuRoC images/dumps
stay ignored and uncommitted; preserve phone dataset license/provenance and
never assume the TUM vocabulary's CC BY license covers other recordings.
Fetch only the recordings needed for the current stage and coordinate disk
space with the other worker.

The repository's mandatory `python3 benchmark_native.py --all_gt --force`
applies when changing its listed SLAM implementations or benchmark plumbing,
including registering/promoting this fork. The proposed dedicated VIO suite
is additional evidence, not a replacement for that required sweep. This
documentation-only task changes neither algorithms nor benchmark plumbing.

## Implementation status (2026-10-02, standalone leaves, not wired into tracking)

New files only (`stella_vio/c/sv_imu.{h,c}`, `sv_imu_gyro.c`, `sv_imu_init.{h,c}`, drivers `check_sv_imu*.c`, `sv_imu_gyro_run.c`, `sv_imu.mk`;
`stella_vio/tools/{imu_init_eval,gyro_pred_eval,ext_fit}.py`; results in `runs/stella_vio/imu/`). Build/run: `make -f sv_imu.mk -C stella_vio/c check checksyn`.

- Preintegration is an own MIT implementation (Forster 2017, midpoint rule, int64-ns IMU buffer with duplicate/backwards/gap handling), NOT the
  OKVIS leaf; `check_sv_imu` cross-checks against `ok_imu.c` (dR agrees to 5e-16, dv/dp to 1e-5, J_Rbg to 1e-15 up to the left-perturbation sign).
  Tests: analytic trajectories (second-order convergence), first-order bias correction, Jacobians by finite differences (1e-9), Monte Carlo covariance
  (chi2 mean 8.9 vs 9), gyro prediction, outage refusal. EuRoC MH_01/V1_02 vs GT: `euroc_*_preint.txt`.
- `sv_vi_init`: gyro-bias GN, closed-form scale/gravity/velocity, then |g|-constrained refinement with sigma_logS / sigma_grav_deg gates.
  Synthetic: `check_sv_imu_initsyn`. Real data: `imu_init_eval.py` -> `init_summary.md`. Handheld/EuRoC works (gate-accepted windows 85-100% within 20%
  from about 3-4 s); phone results depend on camera-IMU time offset (ADVIO needs -0.32 s) and the Mobile-GVIO phone scale remains unresolved.
- Gyro rotation prediction: `sv_imu_gyro_predict_cam`, `gyro_pred_eval.py` -> `gyro_pred.md` (includes the 3.7 rad/s burst and a time-offset scan).
