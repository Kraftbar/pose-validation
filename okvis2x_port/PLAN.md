# OKVIS2-X GNSS path: port plan (2026-10-06, Claude)

Goal: extend the bit-exact C99 OKVIS2 port (`okvis_port/`) with OKVIS2-X's GNSS fusion (ethz-mrl/OKVIS2-X, BSD-3, commit
38043e4, read-only copy in `external/gnss/OKVIS2-X`). Same method as okvis_port: deterministic single-threaded reference,
observe-only dump patches, C replay harnesses, tolerance 0.

## Work split
- Reference (agent): `tools/build_okvis2x_reference.py`, `okvis2x_port/reference/` (patches, configs, README). Checks:
  - the base drift of X against our OKVIS2 canonical hashes (GNSS off);
  - GNSS-on runs on MH_01 with generated fixes (`tools/okvis2x_make_euroc_gps.py`: 5 Hz, 1 / 2 cm, seed 1, a 30 s
    blackout at 60 s, r_SA = [0.05, 0.02, 0.10]).
- Leaf (Codex): `okvis_port/c/ok_gps.{h,c}`: GpsErrorSynchronous / GpsErrorAsynchronous (IMU-propagated to the GNSS
  stamp) and PoseManifold4d, with an oracle against the real classes.
- Integration (Claude): ViGraph / ViSlamBackend / ThreadedSlam GNSS parts in `ok_vigraph`, `ok_vslam`, `ok_system`, plus
  new dump patches for the GNSS graph events.

## Staging
1. `robust_gps_init: false`. T_GW init by umeyamaTransform: a closed-form yaw (atan2 of H(0,1) - H(1,0) and
   H(0,0) + H(1,1)) plus the centroid translation, not an SVD.
2. `robust_gps_init: true`, adding:
   - estimateRigidRansac: std::mt19937(42) with std::uniform_int_distribution<int>, so libstdc++'s distribution
     algorithm must be modelled;
   - Align4DoF_Ceres: a separate Ceres problem (DENSE_QR, default Levenberg-Marquardt, AutoDiffCostFunction<3,7> over
     Jets, Cauchy(3), PoseManifold4d). These are new solver paths and Jet arithmetic for our Ceres port.

## The GNSS state machine (ViGraph.cpp, upstream line numbers)
- States: Off -> Idle -> Initialising -> Initialised -> ReInitialising (gpsStatus_).
- checkValidGpsMeasurements (1128):
  - Off / Idle / Initialising: reject sigma_h > 6 m or sigma_v > 10 m.
  - Initialised: reject 3-sigma outliers against the estimate.
  - ReInitialising (or a fixed last GNSS state): accept everything.
  - Quirks: the output deque is filled in REVERSE order; the state search dereferences before its end check.
- addGpsMeasurements (1216): attaches each fix to the latest state with timestamp <= fix and records state.gpsMode.
  - Off: first fix, Reset of the LocalCartesian frame (geodetic only), -> Idle.
  - Idle / Initialising: the fix goes into gpsInitMap_ and gets a factor.
  - ReInitialising: also gpsReInitStates_.
  - Each factor: GpsErrorAsynchronous on (pose, speedAndBias, T_GW) with Cauchy(3).
- initializationStrategy (1313):
  - Idle: checkForGpsInit with yaw error < 5 deg -> Initialising.
  - Initialising: checkForGpsInit (yaw sigma < yaw_error_threshold) -> Initialised, needsInitialAlignment_.
  - ReInitialising: re-check plus a drift heuristic on the angular distance per step
    (budget 0.03 + 0.004 / sqrt(steps)) -> needsFullAlignment_.
- checkForGpsInit (1014):
  - the antenna points of the IMU-propagated states (applyPreInt);
  - with robust init, the last 100 states;
  - the 4x4 Hessian of (translation, yaw) from the per-fix covariances, inverted with Matrix4d::inverse (reuse
    rdvio_port's rd_m4_inverse model) and the 3x3 covariance inverses;
  - yaw sigma = sqrt(P(3,3)) in degrees.
- addGpsMeasurement (973):
  - a GpsErrorAsynchronous(position, covariance.inverse(), the IMU deque, the state and fix stamps) goes into
    state.GpsFactors;
  - the Ceres residual is added ONLY in Initialising / Initialised / ReInitialising. Idle factors are added later by
    addGpsInitFactors, which walks gpsInitMap_ in StateId order.
- T_GW is ONE block, the T_GW of states_.begin() (freeze / unfreeze / set).
- needsGpsReInit (852):
  - Initialised and the last GNSS state's pose is fixed (marginalised): a dropout, so gpsDropoutId_ is set and a
    position alignment is needed;
  - ReInitialising and the pose at positionAlignedId_ is fixed: likewise.
- needsPosGpsAlignment (891):
  - only when robust init is off;
  - evaluates a GpsErrorAsynchronous of the last init fix (gpsInitImuQueue_) at the state found by the same
    deref-before-end search;
  - posError = C_GW^T * error().
- resetFullGpsAlignment: back to Initialised and the init buffers are cleared.
- Still to read:
  ViSlamBackend addGpsMeasurementsOnAllGraphs / tryGpsAlignment / attemptFull/PosGpsAlignment / addGpsAlignmentFrame,
  and the ThreadedSlam per-frame handover.

## Status (2026-10-06 evening)
- Reference built and deterministic (agent): `okvis2x_port/reference/README.md`.
  - GNSS off, on MH_01: X DIFFERS from OKVIS2 from the first output row, so our okvis_port must first be brought to X's
    numerics.
    - Mono final ATE: X 0.038 vs OKVIS2 0.054.
    - Stereo final position RMS difference to OKVIS2: 2.6 cm.
  - GNSS on, `robust_gps_init: false`: the init fires (state 338), then the dropout, then the re-init.
    - Global trajectory RMS against the GT antenna: 3.9 cm.
    - Final ATE: 0.034 m.
  - `robust_gps_init: true`: the init never completes on MH_01. The cause is not found; the rejections are DLOGs,
    compiled out.
- Init helpers ported (agent): `okvis_port/c/ok_gps_init.{h,c}`, oracle `okvis_gps_init_test.cc`.
  - umeyama, RANSAC with mt19937 and libstdc++ 13's Lemire uniform_int, the yaw Hessian, the 3x3 / 4x4 inverses.
  - 0 mismatches over 4 seeds x 20k cases. Claude re-ran seeds 11 and 12: 0 / 85 MB.
- GNSS error terms + PoseManifold4d DONE: `okvis_port/c/ok_gps.{h,c}`, oracle `okvis_gps_test.cc`.
  - Codex's draft stopped at the usage limit; the Codex resume then failed with a model-not-supported error. A Claude
    agent rewrote the draft.
  - 0 mismatches over 4 seeds x 20k scenarios. Claude re-ran seed 21 from a fresh build: 0 / 142 MB.
  - Details: `okvis2x_port/HANDOVER_gps_leaf.md`. Build and run: `runs/okvis2x_port/gps_leaf/{build_oracle,run,run_all}.sh`.
  - TODO: `tools/check_okvis_port.py` builds oracles with the okvis2 headers. Under it okvis_gps_test prints SKIP and
    exits 0, which can be misread as a pass. Add X include support, or a fail-on-SKIP flag, when the X reference joins
    the runner.
  - The `T*Pd*T^T` association is not observable in the outputs (one mutant survives). This is inherited from ok_imu.c.
- Drift localisation (agent, running): our dump patches 0003-0014 ported onto X, then the C harnesses run against X's dumps.
- Drift localisation (agent): our dump patches ported to X as `okvis2x_port/reference/patches/0005-0015`.
  - Numerics are unchanged: compare columns 1-17 of final.csv. X writes uninitialised values into columns 18-21.
  - The C harnesses against X's GNSS-off MH_01 mono dumps:
    - cam / err / imu / kin / param / problem / solve PASS, so the leaf code is the same as OKVIS2's;
    - graph / vigraph / vslam / frontend FAIL.
  - Cause 1 (pinned): ViGraph::updateLandmarks in X uses a different landmark quality.
    - The norm of the standard deviation of the observation ray directions, initialised above 0.04, with no "behind"
      zeroing.
    - OKVIS2 used (minD - 3/sqrt(lambda_min)) / minD with the 0.15 threshold.
    - With OKVIS2's rule toggled back in, X's causal.csv matches OKVIS2 up to row 31 instead of row 2.
  - Cause 2 (open, agent continuing): at frame 32, matchMotionStereo creates 6 extra landmarks in X with identical
    prior state.
  - Numerically neutral but structural: X adds a T_GW parameter block in addStatesInitialise (11 vs 8 graph events,
    4 vs 3 parameter blocks).
  - The current runs/okvis2x_port/reference_build contains env-gated experiment toggles (default off). It must be
    rebuilt cleanly with patches 0001-0015 only; the agent was asked to do it.
- Drift localisation: cause 2 found by the agent. X no longer clears the images of frames converted to the pose graph
  (applyStrategy), so overlapFraction keeps seeing them and matchMotionStereo matches against them.
  - With both causes toggled, X equals OKVIS2 over all of MH_01 mono.
  - The X reference was rebuilt cleanly with patches 0001-0015 only; the hashes are unchanged.
- OKVIS2-X mode in the C port (Claude), env `OKVIS_PORT_OKVIS2X=1`; default 0 = OKVIS2:
  - `ok_graph_update_landmark_x`: the direction-std quality, threshold 0.04.
    - Eigen measured against the real library: the rowwise mean is a partial redux into a 3-vector temporary whose packet
      rows start at row 1 (`ok_graph_x_mean_pkt`); the outer rowwise sum is a left fold in every row.
    - Oracle `okvis_port/reference_tools/okvis_graph_x_test.cc`: 0 / 80,000.
  - `ok_vsb_okvis2x`: keep the images of pose-graph frames.
  - `ok_dbow_okvis2x`: DBoW cut-off 0.375.
  - Result on MH_01 mono: the C app equals X clean_off (causal 01dd5ae3b8e6 / final 77e4e218837d over columns 1-17).
  - Default mode still equals the canonical OKVIS2 hashes (final dfe3b58e, causal cc29a746).
  - Stereo: the C app with OKVIS_PORT_OKVIS2X=1 equals X gpsoff_stereo1 (causal 0dba14369b74 / final a19c6f0a1754 over
    columns 1-17).
- GNSS integration (agent, running): stage (a), target clean_gps_a byte for byte.

## Status (2026-10-07): stage (a) integrated, bit-exact
- GNSS ON, `robust_gps_init: false`: the C app reproduces the X reference byte for byte (causal.csv, final.csv columns 1-17, global trajectory) on
  `clean_gps_a` (= `gps_a_mono1`) and on two further GNSS scenarios (`gnss_v2`, `gnss_v3`); GNSS off with `OKVIS_PORT_OKVIS2X=1` = `clean_off`; default mode =
  the OKVIS2 canonical hashes. Details, code map, kept quirks, not-exercised branches and next steps: `HANDOVER_gnss_integration.md`.
- Check: `python3 tools/okvis2x_check_gnss.py` (or `tools/check_okvis_port.py --gnss-e2e`).
- Next: stage (b) `robust_gps_init: true` (checkValidGpsMeasurements, Align4DoF_Ceres), geodetic input, stereo + GNSS.

## Status (2026-10-07, later): stage (b) `robust_gps_init: true` bit-exact
- Cause of the never-firing robust init on MH_01 5 Hz: RANSAC needs >= 40 points, the realtime window holds <= 27 (HANDOVER_gnss_robust.md).
- New datasets `data_r1` / `data_r2` (20 Hz) and X reference runs `gps_b_r1_{1,2}`, `gps_b_r2_{1,2}` (deterministic); the C app reproduces them (causal, final cols 1-17, global)
  and `gps_b_mono1`. New ported pieces: `ok_align4.{h,c}` (Jet residual, Eigen HouseholderQR, `Align4DoF_Ceres`), `DENSE_QR` in `ok_solve*.c`, `checkValidGpsMeasurements`
  (`ok_vggps.c`, called from `ok_vsb_add_gps_measurements`); oracle `okvis_align4_test.cc`. Check: `python3 tools/okvis2x_check_gnss.py --stage-b`.
