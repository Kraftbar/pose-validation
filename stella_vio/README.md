# stella_vio

Development fork of the exact stella_vslam C port. **It diverges on purpose.**
The exact baseline is `stella_port/` @ tag `stella-c-port-v1` (bit-exact to the
deterministic stella reference; never edited for features). Do not expect the
harnesses in `stella_port/` to pass against this directory.

- `c/`: copy of `stella_port/c/` sources and headers (no `check_*` harnesses)
  plus `sv_eigen_pnp.{c,h}` (from `stella_port/reference_reloc/c`). Licence
  headers, `NOTICE` and `LICENSES/` are kept; see `NOTICE` for the fork note.
- Build: `make` (gcc, C99, libm only) -> `./sv_run`; `make asan` -> `./sv_run_asan`.
- Run: `sv_run <vocab.fbow> <seq_dir> <fixtures_dir> <out_dir> [max_frames] [--no-snap] [--size WxH] [--camera ...]`.
  `seq_dir` needs `rgb.txt` and `depth.txt` (`#` lines are skipped); frame size is read from the first PGM fixture.
- Starting point (2026-10-02): identical trajectory to `stella_port` on fr1_xyz (byte-compared).
- Every change, accepted or rejected, with numbers: `RESULTS.md`.
- Camera only for now; IMU is a later step.
- Opt-in robustness switches (defaults unchanged, see `RESULTS.md` "R-frames and map merge"): `--set rframe=1` (rotation-only frames through tracking failures, deferred
  initialization bridged into the existing map; `rf_*` tuning keys; `c/sv_rot.{c,h}`), `--set merge=1` (keep the old map on re-initialization and merge maps by Sim3 on place
  recognition; `c/sv_loop.c` `merge_components`). Extra outputs: `trajectory_maps.tum` columns 10 (R-frame flag) and 11 (segment id).
- PoseLib-style blocks, opt-in (`c/sv_poselib.{c,h}`, BSD-3 notice in `NOTICE`; see `RESULTS.md` "PoseLib-style blocks"): `--set init_refine=1` (Sampson LM refinement of the initializer pose), `--set init_lo=1` (+`init_lo_thr`) (5pt LO-RANSAC initializer), `--set pnp_lo=1` (P3P LO-RANSAC + LM for relocalization / loop PnP). Defaults unchanged, none promoted. Unit test: `make check_sv_poselib`, `reloc_study.py`.
- `--live-out F` (opt-in, off by default; all other output files are unchanged): the LIVE pose of every tracked frame, written the moment it is computed (the tracking pose of that frame before any later local BA / loop correction), one line `t x y z qx qy qz qw map_id rframe seg loop_accepted scale_cal up_n ux uy uz` (map id as the trajectory file would label it now, R-frame flag, segment id, a loop was accepted / a bridged-part scale calibration applied in this frame, gravity up vector of the map accumulated so far). `sv_frame_result` carries the same `live_*` fields. `sv_run.c` can be included with `SV_RUN_NO_MAIN` + `SV_RUN_HOST` by a host program that sets `sv_run_frame_hook` (per-frame callback) and calls `sv_run_main()` (`phone_pipeline/c/pp_live.c`).
- Gait scale servo (opt-in, section 16 of the study, `RESULTS.md` "Gait scale servo"; **off by default, the trajectory is byte-identical with it off**): `--set servo=G` (gain; 0 = off) `servo_win=S` (window, s) `servo_dmin=M` (metres for the reference) `servo_clip=C` (max |ln f| per keyframe) plus explored variants `servo_mode=0|1|2 servo_gate servo_href servo_dead servo_k`; the host feeds the walking speed with `sv_system_push_speed(sys, t, v)` (`phone_pipeline/c/pp_live.c` does; plain `sv_run` never does, so the switch does nothing there). `--servo-log F` writes the decision trace. Loop-detector knobs explored and rejected: `--set loop_cont=N loop_matches=N` (0 = stella's 3 / 20).
- `--wait-fixtures` (opt-in): a missing fixture PGM is waited for (up to 10 min) instead of ending the run, so that a streaming feeder can write and delete the PGMs behind the run (`phone_pipeline/run.py --stream`). Default behaviour and the exact-port check are unchanged.
- Outdoor-1 blow-up study (`docs/phone_outdoor1_scale_20261008.md`, all opt-in, defaults byte-identical): `--set rf_speed=1 [rf_speed_win=12]` takes the pre-gap speed of the R-frame bridge (prior and `rf_calib` re-fit) from the median of 2 s chord speeds instead of the per-frame path-length EMA, which a jittery tracked pose inflates 3.5x (Outdoor-1 bridge unit 4.4x too large); `--diag-log F` (per-frame / per-keyframe trace), `kf_min_interval` / `kf_enough_lms` (keyframe throttling, rejected), `SV_BRIDGE_DEBUG=1` (prints the bridge prior).
