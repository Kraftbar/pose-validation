# Status (2026-10-05)

Entry point for new sessions. Short on purpose; details live in the linked files.

## What this repo is

A hands-on SLAM/VO/VIO survey and lab:

- Benchmark systems.
- Port the best permissive ones to bit-exact C99, to read how they really reach their numbers.
- Build our own phone/drone stack from the best pieces.

Learned methods are out of scope. GPL code may be read for understanding, but never translated into permissive folders.

## Parts

| Folder | What | State | Read first |
|---|---|---|---|
| `stella_port/` | stella_vslam mono, bit-exact | complete | `stella_port/HANDOVER.md` (top) |
| `okvis_port/` | OKVIS2 (drone winner), bit-exact | complete for EuRoC: M1–M8 done; `okvis_c_euroc` runs MH_01 mono + stereo from images + IMU csv, trajectories byte-identical to the reference | `okvis_port/HANDOVER.md`, `PLAN.md` |
| `rdvio_port/` | RD-VIO/XRSLAM (phone winner), bit-exact | M1–M7 done; window solve, image functions and map layer native | `rdvio_port/HANDOVER.md`, `PLAN.md` |
| `stella_vio/` | our modified copy of stella_port | re-init, IMU, R-frames, merge, PoseLib-style solvers, gait scale servo (all opt-in beyond defaults) | `stella_vio/README.md`, `RESULTS.md` |
| `gnss_fusion/` | our C GNSS fusion | smoother, gait speed prior, georef, auto switch, gait regularity gate | `gnss_fusion/README.md` |
| `phone_pipeline/` | stella_vio → gait → fusion, live | beats GNSS alone outdoors (live 4.31 / 11.33 / 11.82 vs 5.73 / 14.66 / 12.00 m); indoor 0.6–0.8 m with servo | `phone_pipeline/README.md` |
| `orb_port/` | ORB-SLAM2 port (GPL) | private, never committed | `docs/orb_port_handover.md` |

## Benchmarks and survey

- **VIO candidates and EuRoC:** `docs/vio_candidates_20261001.md`, about 30 systems surveyed and 17 run.
  - Stereo: OKVIS2 0.023 m.
  - Mono+IMU: ORB-SLAM3 0.053 (GPL), OKVIS2 0.064.
- **Phones and GNSS:** `docs/gnss_vio_benchmark_20261001.md`. Sections 8–17 cover the phone studies, gnss_fusion, the pipeline, live mode and gait.
- **Drones:** `docs/drone_benchmark_20261002.md`. OKVIS2 is best, and OKVIS2-X has a yaw-init failure.
- **Diagnosis and planning:**
  - Phone scale: `docs/phone_scale_diagnosis_20261003.md` (errors-in-variables bias).
  - Roadmap: `docs/roadmap_research_20261003.md`.
  - Building blocks: `docs/building_blocks_20261003.md`.
- **Repo's own 30 s benchmark:** `runs/benchmark/`. The README matrix is stale against the regenerated numbers; it has not been updated.

## How to verify (all must pass before committing)

- `python3 tools/check_stella_port.py`: 56/56 PASS.
- `python3 tools/check_okvis_port.py --tag m8,s8 --native-solve 1 --eigen-tests --data external/vio/data/okvis_brisk_tmp`: all PASS. The dumps live in `runs/okvis_port/reference_runs/MH_01_easy/{m8,s8}`; `--data` (images + imu csv from `tools/okvis_port_images.py MH_01_easy`) adds native BRISK and the end-to-end `check_ok_system`.
- `python3 tools/check_rdvio_port.py --modules m4,m5 --oracle --seeds 1,2,3 --tag4 m5 --tag5 m5`.
  - The m1 row uses tag `m1`.
  - m2/m3 need the `m23` dump regenerated (about 2 min).
- `bash phone_pipeline/check_baselines.sh <tmpdir>`: the stella_vio exact port must be cmp-identical, and the fusion tables must show 0 differences.
- `gnss_fusion/tools/test_auto.py`: 6/6 PASS.
- Use `external/gnss/venv/bin/python` for any numpy script.

## Rules that bit before

- Never commit:
  - `orb_port/`
  - `simple_slam_c_plus.c` and `simple_slam_c_plus_config.h` (they include a GPL header)
  - `run_orbslam_benchmark.py`
  - the ORB row in `docs/rejected_trials.md`
  - binaries, data, dumps, `gnss_fusion/*.whl`
  - the large `.fbow`
  - `tools/*/gpl_glue`

  `.git/info/exclude` covers most of these.
- A global `Makefile` ignore rule exists. New Makefiles need `git add -f`.
- Agents must use their own image directories and never delete data they did not create. Disk is tight; keep about 20 GB free.
- The `$5` monthly spend cap kills agents mid-task. Resume them with SendMessage.

## Open decisions and next steps

1. **Push blocked:** local commit `a2dc6c8` (OKVIS2 M7c–M7d) contains `okvis_port/c/ok_dbow.*`, a DBoW2 source port. DBoW2's licence requires notifying the author on redistribution, and the repo is public. Choose one: notify the author and then push, push without `ok_dbow`, or push and notify afterwards.
2. **OKVIS2:** done for EuRoC (BRISK hooked up, system driver M8; full suite with `--data` all PASS on 2026-10-05, committed 08a6b38). The app also reads EuRoC PNGs directly (`ok_png`, bit-exact with cv::imread). Open: runtime 2–3x the reference, parameter blocks never freed, other sequences.
3. **RD-VIO:**
   - M7 (OpenCV CLAHE / LK / GFTT): **done 2026-10-06**. Codex wrote it and Claude verified it: 70 fixture cases and a full MH_01 stream of 12.15 G bytes, 0 mismatches. The bit-exactness holds for this machine's OpenCV CPU dispatch.
   - M6 map layer: **done 2026-10-06**, 38.5 M events of MH_01 replayed bit-exact (patch 0009 map log).
   - M9 initializer: **done 2026-10-06** (5cb3df0); the one MH_01 initialization replays bit-exact.
   - M10 sliding-window tracker (`track()`): **done 2026-10-06**. 776 logged calls of MH_01 replay bit-exact stage by stage (patch 0012), with the PARSAC masks taken from the log.
   - M8 + M11, the whole C system: **done 2026-10-06**. `rdvio_c_euroc` writes the reference trajectory byte for byte on seven EuRoC sequences (MH_01, MH_03, MH_04, MH_05, V1_01, V1_03, V2_01). Run it with `tools/check_rdvio_port.py --modules sys`.
   - Two OpenCV pieces are still outside the port: the undistortion (the system reads a pack of undistorted images) and EPnP (the IMU-PARSAC around it is native C, with EPnP behind a callback). Both are reserved for Codex (M7b).
4. **Phone stack:**
   - The Outdoor-1 map-unit blow-up at about 200–260 s is unexplained, and it blocks the servo outdoors.
   - Indoor-2 live error is still twice the final error.
5. **Codex:** at most one job at a time, with a distilled brief; resume a warm session where possible (see the `codex-delegation` memory). The RD-VIO M7 job is done and verified. Leftovers from the cancelled 2026-10-05 fan-out: `runs/okvis_port/png/` and `runs/phone_pipeline/o1_diag/` (27 MB), both safe to delete.
6. **Optional:** a Basalt port (BSD, fast stereo); your own Android recordings with raw GNSS and RTK ground truth.
