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
- `--wait-fixtures` (opt-in): a missing fixture PGM is waited for (up to 10 min) instead of ending the run, so that a streaming feeder can write and delete the PGMs behind the run (`phone_pipeline/run.py --stream`). Default behaviour and the exact-port check are unchanged.
