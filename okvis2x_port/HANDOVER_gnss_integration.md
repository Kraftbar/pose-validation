# OKVIS2-X GNSS fusion in the C app (stage a, `robust_gps_init: false`), 2026-10-07

Result: with `OKVIS_PORT_OKVIS2X=1` and `gps_parameters` in the config the C app `okvis_c_euroc` reproduces the OKVIS2-X deterministic
reference with GNSS ON byte for byte, on three different GNSS scenarios (below): `causal.csv` (whole file), `final.csv` columns 1-17 and
the global trajectory (`global_final.csv`). Run it: `python3 tools/okvis2x_check_gnss.py` (or `tools/check_okvis_port.py ... --gnss-e2e`).

## Numbers (MH_01_easy mono, one thread, `-O2 -ffp-contract=off -fno-fast-math`, 3,6xx frames each, C wall 5.0-5.5 min)
| case (reference run) | rows | causal.csv sha256 | global sha256 | final.csv cols 1-17 |
|---|---|---|---|---|
| `clean_gps_a` (5 Hz, 1/2 cm, seed 1, 30 s blackout at 60 s) = `gps_a_mono1` | 3658 | `deabab007e3d..` identical | `0f711351270e..` identical | identical |
| `gnss_v2` (10 Hz, 5/10 cm, seed 2, 8 s blackout at 40 s) | 3660 | `86d2e74bbac7..` identical | identical | identical |
| `gnss_v3` (2 Hz, 2/4 cm, seed 3, 3 s blackout at 100 s) | 3652 | identical | identical | identical |
| GNSS off, `OKVIS_PORT_OKVIS2X=1` = `clean_off` | 3682 | `144d71718396..` identical | - | identical |
| default mode (OKVIS2) | 3682 | causal `cc29a746ea6f..`, final `dfe3b58e6a33..` = canonical | - | - |
`gnss_v2` / `gnss_v3` are extra X reference runs made for this work (`tools/okvis2x_make_euroc_gps.py` variants in
`runs/okvis2x_port/data_v{2,3}`, `tools/okvis2x_run_reference.py ... --tag gnss_v{2,3} --data-root ...`); the first full run of the C code
matched `clean_gps_a` with no debugging iteration, so no divergence had to be localised and patch 0016 (X event log) was not needed.
The X reference build and the hashes in `reference/README.md` are untouched.

Paths exercised (C log `OKVIS_PORT_GNSS_LOG`, one line per event): Off -> Idle (first fix, only the newest fix of a batch takes the `Off`
branch), Idle -> Initialising (yaw sigma < 5 deg, `addGpsInitFactors` on both graphs, T_GW set), Initialising -> Initialised (yaw sigma <
`yaw_error_threshold`), initial alignment (`addGpsAlignmentFrame(1)` + full-graph optimisation with the GPS factors, then `T_GW` fixed in
both graphs), dropout (`needsGpsReInit`), ReInitialising, position alignment (`attemptPosGpsAlignment`), the drift heuristic, full alignment
(`attemptFullGpsAlignment` + a 50-iteration realtime optimisation + `addGpsAlignmentFrame`), `resetFullGpsAlignment`, and the
`addGpsBacklog_` route (fixes arriving while a loop closure / alignment result is pending: 4 backlog pushes in `gnss_v3`).
NOT exercised by any run: the second branch of `needsGpsReInit` (state ReInitialising and the pose at `positionAlignedId_` fixed), the
`IMUOLD` / `IMUTOONEW` early returns of `addGpsMeasurement(s)`, a measurement older than every state (upstream dereferences `rend()`).

## What was ported where (upstream line numbers: external/gnss/OKVIS2-X, commit 38043e4)
| upstream | C |
|---|---|
| `GpsParameters`, `getGpsCalibration` (ViParametersReader.cpp:632) | `ok_config.{h,c}`: `has_gps`, `gps_type`, `gps_r_SA`, `gps_yaw_error_threshold`, `gps_robust_init` |
| DatasetReader.cpp:473-591 (`gps0/data.csv`, header skipped, `std::stof`, `t_gps - start + 1 s > 0`, EOF ends streaming) | `okvis_c_euroc.c`; the order is IMU, GPS (`while t_gps <= t`), frame |
| ThreadedSlam.cpp:346 `addGpsMeasurement`, :506 first-frame drop, :623 per-frame pull (initialised branch only), :842 handover, :845 pop | `ok_system.c`: `ok_sys_add_gps`, queue `gq`, deque `gdq`, `ok_vsb_add_gps_measurements` after `setKeyframe` |
| ViGraph: T_GW block (:339, created at `addStatesInitialise` after the extrinsics, `PoseManifold4d`, variable), `state.T_GW = lastState.T_GW`, `GpsFactors`, `gpsMode`, `addGpsMeasurement` (:973 incl. the residual only in Initialising / Initialised / ReInitialising, `CauchyLoss(3)`), eliminate cleanup (`gpsStates_`, `gpsReInitStates_`, `gpsInitMap_`) | `ok_vigraph.{h,c}` (the graph side: `ok_vg_gps_*`; GNSS-off graphs are unchanged: the block is only created when `ok_vg_gps_enable` was called) |
| ViGraph state machine: `checkForGpsInit` (:1014), `addGpsMeasurements` (:1216), `initializationStrategy` (:1318), `addGpsInitFactors` (:1401), `needsGpsReInit` / `reInit` / `needsFull/Pos/InitialGpsAlignment` / `reset*` (:835-972), freeze / unfreeze / set | `ok_vggps.{h,c}` (new; uses `ok_gps_init_core`, `ok_gps_async_*`) |
| ViSlamBackend: `addGps`, `addGpsMeasurementsOnAllGraphs`, `tryGpsAlignment`, `attemptFull/PosGpsAlignment`, `addGpsAlignmentFrame`, `optimiseRealtimeGraph` T_GW copy / freeze block (:938-962), `synchroniseRealtimeAndFullGraph` backlog + T_GW fixing (:1709-1733) | `ok_vslam.c` (`ok_vsb_add_gps`, `ok_vsb_add_gps_measurements`, statics `try_gps_alignment`, ...) |
| Ceres side: GPS residual (3 residuals; pose, speed-and-bias, T_GW; row-major ambient Jacobians x PlusJacobian), `PoseManifold4d` (ambient 7, tangent 4), `CauchyLoss(3)` | `ok_solve.{h,c}` (`OK_SV_T_GPS`, `OK_SV_KIND_POSE4`, `OK_SV_LOSS_CAUCHY3`), `ok_vsolve.c`; the term is evaluated LIVE (it re-preintegrates, as the C++ object does) |
| writeFinalCsvTrajectory (NrGps, SID, gpsMode), writeGlobalCsvTrajectory (:2325) | `ok_system.c` (`ok_sys_write_final_csv`, `ok_sys_write_global_csv`) |

Quirks kept: fixes and states are walked in reverse and `sids` is filled with `push_front`; in `Off` only the newest fix of the batch is
inserted into `gpsInitMap_` (no factor) and the older ones of the same batch already see `Idle`; `gpsInitMap_` is a multimap (insert after the
equal range) and `addGpsInitFactors` adds the residuals of a state once PER ENTRY; `gpsInitImuQueue_` appends the boundary IMU sample twice
(`<` test); `needsPosGpsAlignment` builds a throw-away `GpsErrorAsynchronous` from `gpsInitImuQueue_` and the last map entry; the first
`needsFullGpsAlignment` call is on the full graph and its outputs are overwritten by the realtime one; `fullGraph_.setGpsStatus` happens BEFORE
the full graph adds the batch (so the full graph runs the batch with the status of before the realtime graph's transition); the T_GW copy
realtime -> full runs on every `optimiseRealtimeGraph` while the loop-closure flags are clear and until observable, then both are frozen; a
fix only enters the deque of the initialised branch of `processFrame` (the first two frames never pull any).

## Not ported / next steps
- `robust_gps_init: true` (stage b): `checkValidGpsMeasurements` (:1128), the RANSAC branch of `checkForGpsInit` is in `ok_gps_init_core` but
  `Align4DoF_Ceres` (Ceres DENSE_QR, LM, Jets, Cauchy(3), PoseManifold4d) is not; `ok_sys_new` rejects the config. The X reference never
  initialises on MH_01 in this mode (reference README), so there is nothing to compare on that data yet; the cause of that is still open.
- geodetic / geodetic-leica data (`LocalCartesian` of GeographicLib), stereo with GNSS (not run), `doFinalBa` GNSS unfreeze, multi-session.
- Replay harness for GNSS (dump patches): not needed so far. If a future change diverges: `OKVIS_PORT_GNSS_LOG=<file>` (C; status transitions `ST`,
  fixes `AM`, init checks `CI`, strategy `IS`/`IF`, re-init `RE`/`RI`, alignments `PA`/`AL`/`AF`, T_GW writes `TG`, backlog `BL`) is the C half; the X half
  would be an env-gated `fprintf` at the same places of ViGraph.cpp / ViSlamBackend.cpp as `0016-gnss-event-log.patch` (observe only).
- `tools/check_okvis_port.py` compiles `okvis_gps_test.cc` / `okvis_gps_init_test.cc` with the OKVIS2 include set (prints SKIP, exit 0): open item
  from HANDOVER_gps_leaf.md, unchanged.

## Files
`okvis_port/c/`: new `ok_vggps.{h,c}`; changed `ok_config.{h,c}`, `ok_solve.{h,c}`, `ok_vsolve.c`, `ok_vigraph.{h,c}`, `ok_vslam.{h,c}`,
`ok_system.{h,c}`, `okvis_c_euroc.c`; the `OK_PORT_SOURCES` lines of `check_ok_{solve,vigraph,vslam,frontend,system}.c` gained `ok_gps.c ok_gps_init.c`
(+ `ok_vggps.c` for the last three). Tools: `tools/okvis2x_check_gnss.py`, `--gnss-e2e` in `tools/check_okvis_port.py`.
Build of the app by hand: the sources of the first line of `check_ok_system.c` (minus `check_ok_system.c`) + `okvis_c_euroc.c` + `ok_png.c`.
Work dir `runs/okvis2x_port/gnss_int/` (outputs, logs, symlinked sequence dirs, `build.py`, `cmp.py`).
