# stella_vio results (camera only)

Baseline = exact port (`stella_port/` @ `stella-c-port-v1`, frame-size fixes only; `reinit_sec=0 init_max_level=0 init_confirm=1`).
Date 2026-10-02. All numbers single runs of deterministic code (identical numbers on every rerun), on a heavily loaded shared machine (RTF is meaningless, not reported).
Datasets (not in the repo, never committed): TUM RGB-D fr1_xyz / fr1_desk / fr1_floor / fr2_xyz / fr3_long_office (per-sequence shipped camera), Mobile-GVIO `Outdoor-1`
(1280x720, 15 fps, 394 s, 5898 frames, Sim3 vs the LiDAR-rig GT with the study's 292.887 s offset), GVINS `complex_environment` (752x480, 20 Hz, first 436 s, 8721 frames, RTK GT, camera = `/cam0`).

## Tools

- `tools/run_eval.py <tag> [--seqs ..] [--extra "--set k=v .."]`: runs `sv_run` on the 7 sequences in parallel, scores (outputs `runs/stella_vio/<tag>/`). TUM: Sim3 ATE over tracked frames
  (`tum_eval.ate_over`, `benchmark.ate_rmse`), coverage over `rgb.txt` frames. GNSS sequences: Sim3 ATE (`gnss_eval.GT`, `benchmark.umeyama_alignment`), **one alignment per map**
  (`trajectory_maps.tum`, 9th column = map id) and pooled RMS, so a re-initialized map (new, arbitrary scale) is scored fairly; "per-60s-window ATE" = Sim3 per 60 s window and map, pooled RMS / median window
  (local accuracy, independent of the long-term scale drift that dominates the whole-run number of a loop-free monocular run).
- `tools/init_study.py`, `tools/study_all.sh`: **multi-start robustness windows** (`sv_run --skip S`): the same configuration started at many frames (TUM: 300-frame windows every 100 frames, 75 windows;
  Outdoor-1: 450 frames every 150, 37 windows; complex: 600 frames every 300, 28 windows; 140 in total). A window is a success if coverage >= 60% and Sim3 ATE < 0.05 m (TUM) / 2 m (Outdoor-1) / 1 m (complex).
  This measures initialisation + early tracking, which is what is chaotic (see below); a single whole-run ATE of one configuration is not a reliable ranking signal.
- `tools/spread.sh`: the same configuration under 7 perturbations of the initializer Hamming threshold (50..64) on one sequence: spread of the whole-run ATE.
- `tools/scale_probe.py`, `tools/make_fixtures.py`.
- New `sv_run` options: `--set key=value` (`reinit_sec init_parallax init_tri init_min_valid init_hamm init_ratio init_max_level init_par_frac init_confirm orb_*`), `--skip N`, `--lean`, `--size WxH`;
  extra output `trajectory_maps.tum`; log line `maps=N reinits=M`.

## Accepted changes (defaults of `sv_system_params_default`)

1. **Re-initialization into a new map** (`reinit_lost_sec = 2.0`). When tracking has been Lost (relocalization failing) for 2 s of data time, the poses of the current map are archived with a map id,
   the system is reset (the old map, its BoW database and keyframes are dropped: "keep the newest map and log it"; no merge, relocalization against an old map is not possible after this) and a new map is initialised.
2. **Initializer matcher on pyramid levels 0..3** (`init_max_level = 3`; port: level 0 only). The area matcher of the initializer only matched reference keypoints of octave 0 against candidates of the same octave:
   on textureless / blurry footage (grass, phone video) that yields 60-100 matches, too few valid points (< 50) for most attempts, so the initialisation takes many seconds or never happens.
   Octave 4+ is too inaccurate (noisier pairs), 1-2 too few.
3. **Initialisation confirmation** (`init_confirm = 2`; port: 1): an initialization pair must succeed on 2 consecutive frames (the later, wider-baseline pair is used). Removes the early, eager inits that step 2 makes possible
   (fr3_long_office whole-run ATE spread over the 7 perturbations: 0.018-0.597 without, 0.018-0.035 with).

Tracking, mapping, loop closing, relocalization are unchanged (still the exact port code).

### Whole-run table (ATE m / coverage / lost frames; 2nd line for GNSS runs: per-60s-window ATE pooled / median, maps)

| sequence | baseline (exact port) | + re-init (1) | + init levels 0..3 (2) | + confirm 2 (3) = defaults |
|---|---|---|---|---|
| fr1_xyz | 0.026 / 99% / 0 | 0.026 / 99% / 0 (unchanged, never Lost 2 s) | 0.011 / 99% / 0 | 0.011 / 99% / 0 |
| fr1_desk | 0.018 / 89% / 0 | 0.018 / 89% / 0 | 0.020 / 98% / 0 | 0.019 / 97% / 0 |
| fr1_floor | 0.022 / 97% / 20 | 0.022 / 97% / 20 | 0.022 / 98% / 19 | 0.024 / 98% / 21 |
| fr2_xyz | 0.016 / 98% / 0 | 0.016 / 98% / 0 | 0.005 / 100% / 0 | 0.012 / 100% / 0 |
| fr3_long_office | 0.025 / 99% / 0 | 0.025 / 99% / 0 | **0.071** / 99% / 0 (end loop missed) | 0.022 / 99% / 0 |
| Outdoor-1 (Sim3, 1 map) | 37.5 / 100% / 0; win60 4.96 / 1.68 | 37.5 (no Lost 2 s, unchanged) | 16.6 / 100% / 6; win60 5.17 / 0.85 | 45.7 / 100% / 3; win60 3.41 / 1.89; first 60 s 2.1 (baseline 11.8) |
| complex_environment | 0.114 / **7%** (631 poses) / 8022; win60 0.114 | 3.27 / **97%** / 41, 2 maps; win60 0.29 / 0.25 | 3.32 / 99% / 41, 2 maps; win60 0.29 / 0.25 | 2.73 / 97% / 43, 2 maps; win60 0.29 / 0.25 |

Notes: README numbers reproduced by the baseline column (fr1_xyz 0.026, desk 0.018, floor 0.022, fr2 0.016, fr3 0.025). The whole-run Outdoor-1 number is dominated by monocular scale drift (scale ratio
0.3x..7x over the run, no loop closure on this path): it moved 37.5 / 16.6 / 45.7 between configurations that differ only in the first seconds, i.e. it is not a ranking signal. The windowed numbers and the
multi-start success counts below are. complex: the first map covers 0-34 s (0.11 m), the second map (a new scale) the remaining 400 s (whole-map Sim3 3.7 m, drift to ~10 m in the last 30 s bin, no loop closure).
fr3 "0.071" in column (2) is a missed end-of-sequence loop closure from a poor early init; column (3) fixes it. TUM numbers must stay within noise of the baseline: they do (fr1_xyz and fr2_xyz improve, fr1_floor
0.022-0.024).

### Multi-start robustness (successful windows / windows)

| configuration | fr1_xyz /5 | fr1_desk /3 | fr1_floor /10 | fr2_xyz /34 | fr3 /23 | Outdoor-1 /37 | complex /28 | total /140 |
|---|---|---|---|---|---|---|---|---|
| baseline (exact port) | 4 | 3 | 4 | 31 | 20 | 21 | 9 | 92 |
| init levels 0..3 | 5 | 3 | 7 | 33 | 21 | 30 | 27 | 126 |
| + confirm 2 (**accepted**) | 5 | 3 | 7 | 34 | 22 | 33 | 28 | **132** |

## Rejected / explored (numbers: windows successes /140 unless stated, or the stated metric)

| change | result | verdict |
|---|---|---|
| init parallax threshold 2 deg (port 1) | TUM: fr1_xyz 92% cov, fr1_floor 48% cov (124 Lost frames), fr2_xyz 86%, fr3 ATE 0.050; 3 deg: fr1_floor never initialises, desk 43%; Outdoor-1 first 80 s: 3.8 m (1 deg: 15.0 m) but 25 s without a map at 2.5 deg | rejected: late / no init |
| init parallax 2 + min triangulated 100 | no sequence initialises | rejected |
| parallax measured at the median instead of the 50th point (`init_par_frac` 0.2-0.7) | identical to baseline (the bad Outdoor-1 pair at frame 3 has median parallax > 1 deg) | rejected, no effect |
| init_confirm 3 / 4 / 5 (with level-0 matcher) | Outdoor-1 first 80 s: 10.7 m / never initialises | rejected (confirm 2 only works with the wider matcher: 132; confirm 3: 120) |
| init matcher levels 0..1 / 0..2 / 0..4 / 0..5 / 0..7 | 12/18 (vs base 11/18; 600-frame protocol), 121, 119, 117 (vs 126 for 0..3); fr1_desk 0.588 m outlier with 0..2 | rejected, 0..3 best |
| init Hamming 64 + ratio 0.95 (alone / with levels 0..3) | 25/37 (base 21) / 24/37 (levels 0..3: 30) ; 120/140 (levels 0..3 + ratio 0.95) | rejected |
| init Hamming 60 + ratio 0.92 with levels 0..3 | 125/140 (vs 126) | neutral, rejected |
| init min valid points 30 / 60 / 70 (min triangulated 20-50) | 9/18 and 10/18 vs 11/18 (30); 60 and 70 identical to the unchanged result | rejected |
| re-init after 1 s vs 3 s Lost (complex) | 3.77 m / 22 Lost frames vs 3.52 m / 61: equivalent | 2 s kept |
| keyframe-insertion / local-mapping changes for fast rotation | not tried: kf_decision shows a keyframe on almost every frame during the 3.7 rad/s burst (`frame_trace` / `kf_decision` of the baseline complex run); the camera simply turns into an unmapped area and tracked landmarks drop 119 -> 17 in 8 frames, so there is nothing to map yet; re-initialisation is the recovery | n/a |

Failure analysis (what the data say):
- complex_environment: tracking degrades over frames 690-699 (tracked landmarks 119 -> 17) as the rig turns into unmapped space, then 8022 Lost frames, relocalization never succeeds (map of 80 keyframes, no overlap
  with later views). With re-initialisation 97-99% coverage.
- Outdoor-1: the first map is built from far features (median landmark distance equals tens of metres while the path is 1.3 m/s); translation is weakly observed, the est/GT path-length ratio over the run drifts 0.3x..7x. This is
  inherent to monocular without IMU; camera-only changes only move the chaos around. Fix = IMU (below).
- Initialisation is chaotic: nearby configurations give the same whole-run quality only on average; judge by the multi-start tables.

## 720p handling

The heap overflow (candidate cap) is fixed in the exact port (growing array, `HANDOVER.md`); stella_vio inherits it (ASan/UBSan clean on 1280x720 Outdoor-1 frames and on a re-init stress run).
Feature-setting trials on Outdoor-1 (37 multi-start windows, default config = 33/37):

| change | windows | verdict |
|---|---|---|
| `orb_min_area` 400 (about 2x keypoints) | 29/37 | rejected |
| 10 pyramid levels, scale 1.15 | 28/37 | rejected |
| `orb_min_area` 1600 (fewer keypoints) | 27/37 | rejected |

The 720p problem was the initializer's matcher (accepted change 2), not the feature count or pyramid.

## Next steps (IMU)

Gyro prior for the motion model and for ranking initialization pairs; gravity from the accelerometer to align every map (re-initialized maps then share roll/pitch); gyro dead-reckoning rotation
through Lost bursts + relocalization with a pose prior; visual-inertial scale alignment (loose, `tools/gnss_loose_fusion.py` style) per map; GNSS Sim3 fit to merge maps. An IMU-side start (`sv_imu*`) was added by another agent.

## IMU wiring (2026-10-03). Fixtures regenerated (TUM: C++ OpenCV `pgm_cache`, frames numbered after the rgb/depth association, as sv_run does); baseline numbers reproduce the table above exactly.

Exact-port check (all new switches off: `--set reinit_sec=0 init_max_level=0 init_confirm=1`, no `--imu`, init_seeds=1): fr1_xyz `trajectory.tum` is `cmp`-identical to `stella_port`'s sv_run (787 poses).
New `--set` keys: `init_seeds`, `gyro` (bit0 tracking prior, bit1 Lost dead-reckoning), `gravity`, `dr_sec`; options `--imu --imu-ext --imu-toff --imu-bg`.

### Step 1: init RANSAC from several fixed seeds, best by valid points (`init_seeds=4`, seeds 5489,1,2,7) - REJECTED as default (kept opt-in)

| | fr1_xyz | desk | floor | fr2_xyz | fr3 | windows /75 (TUM) |
|---|---|---|---|---|---|---|
| default (1 seed) ATE / cov | 0.011 / 99% | 0.019 / 97% | 0.024 / 98% | 0.012 / 100% | 0.022 / 99% | 71 (5,3,7,34,22) |
| init_seeds=4 | 0.010 / 100% | 0.019 / 91% (1 reset) | 0.022 / 98% | **0.005** / 100% | 0.022 / 99% | 69 (4,3,8,33,21) |

fr2_xyz improves as the HANDOVER predicted, but multi-start windows are 2 worse and desk gets a reset: selection by valid points does not pick a better map reliably. Not promoted.

### Steps 2-4 (gyro prior, gravity alignment, Lost dead-reckoning); IMU config: `run_eval.IMU_CFG` (ext_fit extrinsics, toff 0, gyro bias from gyro_pred.md fits)
Whole run ATE (Sim3 per map) / per-60s-window pooled / median / first-60s / coverage / Lost frames / maps. Windows = multi-start successes.

| sequence | camera-only default | gyro=1 (step 2) | gyro=3 (steps 2+4) |
|---|---|---|---|
| complex | 2.73 / 0.29 / 0.25 / 0.12 / 97% / 43 / 2 maps | 3.57 / 0.29 / 0.25 / 0.12 / 97% / 43 / 2 | identical to gyro=1 (dead-reckon 2 ok of 40 tries, burst at 34.9 s still splits the map) |
| complex, gyro=3 reinit 6 s dr 6 s | | | 4.04 / 0.31 / 0.24 / 0.12 / 97% / 122 Lost / 2 maps (2 ok of 119 tries): no recovery |
| mh01 (EuRoC) | 0.036 / 0.031 / 0.023 / 0.016 / 100% / 0 / 1 | 0.047 / 0.036 / 0.034 / 0.023 / 100% / 0 / 1 | same as gyro=1 |
| outdoor1 | 45.7 / 3.41 / 1.89 / 2.09 / 100% / 3 / 1 | 2.82 / 0.67 / 0.45 / 0.35 / 99% / 31 / 2 maps | 45.5 / 3.12 / 0.42 / 0.35 / 100% / 25 / 1 map (dead-reckon 19 ok of 25 tries, no reinit) |
| windows (complex, mh01, outdoor1) | 28/28, 10/11, 33/37 | 28/28, 10/11, 33/37 | outdoor1 33/37 |

- Step 2 (gyro rotation prior; motion-model, BoW and robust initial pose; centre by world-frame constant velocity): large local gain on Outdoor-1 (first 60 s 2.09 -> 0.35 m, median window 1.89 -> 0.45 m), neutral on windows everywhere, but complex whole-run 2.73 -> 3.57 and mh01 0.036 -> 0.047 (worse, chaotic). It does not prevent the complex burst loss at 34.9 s (the camera turns into unmapped space). **Not promoted** (mixed); opt-in `--set gyro=1` with `--imu`.
- Step 3 (gravity from accelerometer, causal running sum per map, written as `gravity.txt` and `trajectory_gz.tum`; trajectory/ATE unchanged): residual tilt of each map's Sim3 to GT, before -> after: complex 114.4/100.8 -> 0.6/0.3 deg, mh01 108.5 -> 0.9, outdoor1 117 -> 4.8 (GT rig frame may itself be tilted). **Accepted** (output-only, ATE-neutral by construction); enabled with `--set gravity=1` when an IMU is given.
- Step 4 (Lost dead-reckoning, `gyro=3`): outdoor1 recovered 19 of 25 Lost frames without re-init (1 map, median window 0.42), but whole/window counts equal and complex fails to relocalize (2/40). **Not promoted**, opt-in.
- Step 5 (per-map metric scale) and EuRoC V1_02 not done (V1_02 data not present; no time). Step 1 window counts: see above.
Defaults unchanged: init_seeds=1, gyro=0, gravity=0. Exact-port check passes (fr1_xyz cmp-identical to stella_port).

## R-frames and map merge (2026-10-03, roadmap item 6; own implementation, idea of RD-VIO arXiv:2310.15072, no code taken)

Both features are opt-in (`--set rframe=1`, `--set merge=1`); **defaults are unchanged and the unmodified default is bit-identical**: `trajectory.tum` of the new binary with all switches off is `cmp`-identical to the previous
binary on the 5 TUM sequences, mh01 and complex (tables below show the default next to the modified numbers), and the exact-port check (fr1_xyz, `--no-snap --set reinit_sec=0 init_max_level=0 init_confirm=1`) is `cmp`-identical to
`stella_port`'s sv_run (787 poses; re-run after the last code change). New code: `c/sv_rot.{c,h}` (rotation-only two-view estimation), `c/sv_loop.c` (`merge_components`), `c/sv_system.c` (R-frame chain, bridge, soft reset, labels).
Tools: `tools/burst_study.py`, `tools/merge_study.py`; `run_eval.py` / `init_study.py` / `study_all.sh` gained segment scoring (`SV_MAPCOL=10`, windows print `[segment-scored ...]`) and `SV_DROP_R=1`.

### What was built

**R-frames (`rframe=1`)**. When `track()` fails (state Tracking, not within 5 s of an initialization) the frame is not declared Lost: a rotation chain starts at the last good frame.
Each frame is matched to the previous one under a predicted rotation (gyro when `--imu` is given, else the previous relative rotation; window 40 / 90 px, Hamming <= 64, ratio 0.9), a 2-pair rotation RANSAC on the bearing
pairs (4 px threshold, 150 hypotheses, deterministic LCG) and a closed-form (Horn quaternion) refit give `R_ab`; weak vision (< 40 inliers) that contradicts the gyro by > 10 deg loses against the gyro, and with no usable vision the frame is
gyro-only for up to `rf_gyro_max` = 40 frames. The camera centre is the last good centre plus the smoothed pre-loss velocity, faded to a hold over `rf_hold_sec` = 1 s. The frame gets that pose (reference = last keyframe, flagged R-frame in
`trajectory_maps.tum` column 10, not counted as Lost), the Lost-state relocalizer still runs, and the landmarks of the last good frame are projected into the chain pose every frame (like the gyro=3 dead-reckoning but with the chain pose), so a
camera that turns back into the map re-locks immediately. Triangulation is deferred to the existing initializer: it runs on the chain frames, starting `rf_init_after` = 1 s after the chain began; when it succeeds its two keyframes
are **installed into the existing map** (ids and landmark ids continue, spanning-tree parent = the anchor keyframe, young-map rules count only the new keyframes via `kf_floor`) with the reference frame placed at the chain pose and the map
scaled by a speed prior (`rf_scale=1`: smoothed pre-gap speed x elapsed time / initial baseline; 0 = median-depth prior, 2 = geometric mean). A bridged part is its own *segment* (column 11 of the trajectory) of the same map, and its scale is re-fitted once, `rf_calib` = 4 s
after the bridge, by matching the mean speed of the part to the pre-gap speed (similarity about the bridge point; `rf_calib=0` disables).

**Map merge (`merge=1`)**. A re-initialization (Lost for `reinit_sec`) keeps the old map (keyframes, landmarks, BoW entries, loop/mapping state, frame statistics; only tracker + initializer are reset; new ids continue) instead of
dropping it. stella's loop closer refuses a loop whose keyframes lie in different spanning trees ("merge two spanning trees: not yet implemented"); `merge_components` implements it: the Sim3 `S` of the validated loop (old world -> camera of the
current keyframe, with the scale between the two maps) moves every keyframe pose (`(T_k T_cur^-1) S`) and landmark (`S^-1 T_cur p`) of the current tree into the old map's frame and units, hooks the root below the candidate keyframe, the tracker's motion
state is rescaled, the mapper is resumed (the deterministic port pauses it for good after any loop), and the ordinary `correct_loop()` then fuses the duplicated landmarks, runs the pose graph and the loop BA over the joined tree. Place recognition while
tracking the new map is the existing BoW + Sim3 loop detector; relocalization against the old map's keyframes also works (the BoW database is kept).

### complex_environment, whole run (8721 frames, `--imu` given in every row, gyro prior off). "one" = a single Sim3 for everything that is one map, "seg" = one Sim3 per segment (a bridged part is scored like a re-initialized map)

| configuration | ATE one / seg [m] | win60 pooled / median (one; seg) | first-60 s (one; seg) | coverage | Lost frames | maps | R-frames / bridges / merges |
|---|---|---|---|---|---|---|---|
| default (unmodified) | 2.73 / 2.73 (2 maps) | 0.29 / 0.25 | 0.12 | 97% | 43 | 2 | - |
| `rframe=1` | 4.75 / 4.12 (3 segments) | 0.50 / 0.31; 0.31 / 0.30 | 0.48; 0.13 | 99% | **1** | **1** | 45 / 2 / - |
| `merge=1` | 3.49 / 3.49 (2 maps) | 0.43 / 0.29 | 0.12 | 98% | 43 | 2 | - / - / 0 |
| `rframe=1 merge=1` | identical to `rframe=1` (the chain never fails, no re-init) | | | | | | |

The two rotation bursts (34.0-35 s, 3.7 rad/s, and 41-43.5 s, 3 rad/s) are carried by 45 R-frames in two chains (vision inliers 600-900 per frame; the gyro was never needed here), the first map keeps its 80 keyframes, and tracking
resumes in the same map; the default drops the map and spends 43 Lost frames. The single-alignment ATE is worse (4.75 vs 2.73) because the scale of a bridged part is a prior: its error is what "one" measures, not local accuracy (win60 per segment 0.31 vs 0.29).
`merge=1` cannot do anything on this sequence: the path moves away monotonically (distance of the second map to the first map's path grows 8 m -> 400 m), so there is nothing to recognize; its row differs from the default only by the changed young-map rules after the soft reset (chaotic mono drift, 3.49 vs 2.73).
The rest of the long run (400 s, no loop closure) is dominated by monocular scale drift as before (last 30 s bins 5-10 m).

### Burst study (`tools/burst_study.py`): complex started at 7 different frames before the two bursts, 1200 frames (60 s) each; mean / median over the starts

| configuration (starts 150,250,300,350,450,500,550) | ATE one | ATE seg | coverage | Lost frames |
|---|---|---|---|---|
| default (unmodified) | 0.13 / 0.10 (per map) | 0.13 / 0.10 | 89% | 36.4 |
| `rframe=1` (**final defaults**: `rf_scale=1 rf_calib=4 rf_init_after=1 rf_hold_sec=1`) | 1.12 / 0.88 | 0.19 / 0.10 | 99% | 0.4 |
| same, `--imu` (gyro fused) | identical (vision always won) | | | |
| `rf_calib=0` | 2.75 / 2.31 | 0.17 / 0.11 | 99% | 0.4 |
| `rf_init_after=1.5` / `2.0` | 3.00 / 2.53; 1.93 / 1.68 | 0.25 / 0.25; 0.36 / 0.37 | 99% | 0.4 |
| `rf_hold_sec=2` | 1.61 / 1.39 | 0.12 / 0.09 | 99% | 0.4 |

| configuration (starts 200,300,400,500; earlier sweep) | ATE one | ATE seg | coverage | Lost |
|---|---|---|---|---|
| default | 0.26 / 0.10 | 0.26 / 0.10 | 90% | 31.5 |
| `rf_scale=0` (median-depth prior), `rf_calib=0`, `rf_init_after=0.5` | 5.22 / 5.01 | 1.08 / 1.16 | 97% | 0.8 |
| `rf_scale=1`, `rf_calib=0`, `rf_init_after=0.5` | 4.71 / 4.74 | 0.31 / 0.19 | 99% | 0 |
| `rf_scale=1`, `rf_calib=4`, `rf_init_after=0.5` | 3.57 / 3.56 | 0.31 / 0.19 | 99% | 0 |
| `rf_scale=1`, `rf_calib=4`, `rf_init_after=1.0` | 1.34 / 1.05 | 0.26 / 0.10 | 99% | 0 |

R-frames remove the Lost stretches (Lost 36 -> 0.4 frames, coverage 89 -> 99%) at unchanged local accuracy (seg median 0.10 m, same as the default's per-map 0.10). The whole-window single-alignment error 1.1 m (default
rows: per map) is the scale uncertainty of the bridges; the speed prior beats the median-depth prior (the scene depth changes completely across a turn: scale errors 2x), the deferred 4 s speed re-fit and waiting 1 s before the bridge halve it. The parameters
were chosen on these same starts (7 per sweep, overlapping windows), so the exact optimum is not proven; direction and size of the effect were stable across both sweeps. Not tried: a visual-inertial scale comparison
(`sv_vi_init`, 10-12 s windows, 85-93% of gate-accepted windows within 20% on complex): the segments between the bursts are 6 s long, shorter than the window, so it could not have been applied here (the code was written, never fired in the sweep, and removed again).

### Blank-out test with the gyro (complex, started at frame 200, 700 frames, frames 450-480 = 1.5 s replaced by a flat gray image)

| configuration | Lost frames | maps | coverage | ATE one (seg) |
|---|---|---|---|---|
| default | 73 | 2 | 75% | 0.09 (per map) |
| `rframe=1`, no IMU (vision cannot carry the blank, the chain dies at once) | 31 | 1 | 95% | 0.41 |
| `rframe=1 --imu` (31 gyro-only R-frames over the blank, the map projected into the chain pose re-locks at frame 481) | **0** | 1 | 100% | 0.51 (0.19) |

### TUM + EuRoC MH_01, whole run (ATE m / coverage / Lost frames); mh01 and complex use `--imu` with `gyro=0`

| sequence | default | `rframe=1` | `merge=1` | `rframe=1 merge=1` |
|---|---|---|---|---|
| fr1_xyz | 0.011 / 99% / 0 | same | same | same |
| fr1_desk | 0.019 / 97% / 0 | same | same | same |
| fr1_floor | 0.024 / 98% / 21 | 0.033 / 100% / 1 (0.024 without the 20 R-frames) | same as default | 0.033 / 100% / 1 |
| fr2_xyz | 0.012 / 100% / 0 | same | same | same |
| fr3_long_office | 0.022 / 99% / 0 | same | same | same |
| mh01 | 0.036 / 100% / 0 | same | same | same |

Nothing else triggers an R-frame or a re-init on these sequences; fr1_floor's only change is the 20 frames at the very end of the sequence (R-frames with extrapolated position, 0.2-0.3 deg residual parallax, so a 0.009 m ATE loss from 20 approximate poses; excluding them: 0.024).

### Multi-start windows (successful windows / windows; same protocol and thresholds as above, `study_all.sh`, `--imu`); "seg" count scores a bridged part as its own map

| configuration | fr1_xyz /5 | fr1_desk /3 | fr1_floor /10 | fr2_xyz /34 | fr3 /23 | complex /28 | mh01 /11 | total /114 |
|---|---|---|---|---|---|---|---|---|
| default (unmodified binary, re-run today) | 5 | 3 | 7 | 34 | 22 | 28 | 10 | **109** |
| `rframe=1` | 5 | 3 | 7 | 34 | 22 | 28 | 10 | 109 (seg 109) |
| `rframe=1 merge=1` | 5 | 3 | 7 | 34 | 22 | 28 | 10 | 109 (seg 109) |
| `merge=1` | 5 | 3 | 7 | 34 | 22 | 27 | 10 | 108 |

### Outdoor-1 (Mobile-GVIO, 1280x720 15 fps, 5898 frames; `--imu` given) and its multi-start windows (37 windows of 450 frames, ATE < 2 m, coverage >= 60%)

| configuration | ATE one / seg [m] | win60 pooled / median | first-60 s | coverage | Lost frames | maps | R-frames / bridges | windows /37 |
|---|---|---|---|---|---|---|---|---|
| default (unmodified) | 45.7 | 3.41 / 1.89 | 2.09 | 100% | 3 | 1 | - | **33** |
| `rframe=1` | 43.6 / 43.3 | 3.64 / 2.49 | 2.09 | 100% | **0** | 1 | 66 / 1 | **34** |
| `merge=1` | identical to default (no re-init happens) | | | | | | | 33 |
| `rframe=1 merge=1` | identical to `rframe=1` | | | | | | | 34 |
| `gyro=1` (step 2) | 2.82 | 0.67 / 0.45 | 0.35 | 99% | 31 | 2 | - | 33 |
| `gyro=1 rframe=1 merge=1` | 29.2 / 2.97 (2 segments) | 1.51 / 0.51; 0.85 / 0.36 | 0.35 | 100% | **0** | **1** | 29 / 1 | 33 |

The single extra window of `rframe=1` is the start-150 window: the default loses the map there (coverage 26%, 31 Lost frames), the R-frames carry it (coverage 96%, ATE 1.87 m); every other window is identical.
With `gyro=1` the default splits Outdoor-1 into two maps (31 Lost frames); with R-frames the run stays one map without Lost frames, and scored per segment it matches the two-map numbers (2.97 vs 2.82; win60 0.85 / 0.36 vs 0.67 / 0.45); the single-alignment 29 m is the scale drift of
a 394 s monocular run that the default hides by aligning each map separately (the default one-map run is 45.7).

### Map merge: blank-out stress test (`tools/merge_study.py`; 3 s of flat gray frames at three places per sequence, Lost -> re-init after 2 s -> the camera later sees the old scene again)

ATE / coverage / Lost frames / maps for the default (old map dropped, pooled per-map Sim3) and for `merge=1` (one Sim3 for everything that merged); last column = merges.

| sequence | blank at | default | `merge=1` | merges |
|---|---|---|---|---|
| fr1_xyz | 199 / 399 / 558 | 0.012 / 83%, 0.011 / 87%, 0.012 / 87% (61-62 Lost, 2 maps) | 0.011, 0.011, 0.012 (same coverage, 1 map each) | 1 / 1 / 1 |
| fr1_desk | 153 / 306 / 429 | 0.029 / 57%, 0.018 / 81%, 0.020 / 81% | identical | 0 / 0 / 0 |
| fr1_floor | 310 / 621 / 869 | 0.012 / 82% (123 Lost, 3 maps), 0.016 / 67%, 0.021 / 70% | 0.012 / 90% (82 Lost, 2 maps), 0.142 / 79%, 0.221 / 89% (main map 0.018 / 0.021; the extra is a short third map) | 0 / 0 / 0 |
| mh01 | 920 / 1841 / 2577 | 0.023, 0.033, 0.037 (97%, 41 Lost, 2 maps) | 0.019, 0.530, 0.143 (1 map when merged) | 0 / 1 / 1 |
| fr2_xyz | 917 / 1834 / 2568 | 0.015, 0.011, 0.011 (96-97%, 61 Lost, 2 maps) | 0.020, 0.012, 0.054 | 1 / 1 / 1 |
| fr3_long_office | 646 / 1292 / 1809 | 0.024, 0.022, 0.035 (95%, 61 Lost) | 0.219, 0.022, 0.035 | 1 / 0 / 0 |

9 of 18 blank-outs end in a merge (the loop closer finds the old map from the new one 9 to ~1600 frames after the re-initialization; fr1_desk, fr1_floor and two fr3 / one mh01 positions never recognize it again before the sequence ends). The merged map is one map with one scale: after a merge the single-alignment ATE is
0.011-0.012 (fr1_xyz), 0.012-0.054 (fr2_xyz), 0.14-0.53 (mh01, merge only at the very end of the sequence, after a 90 s second map whose scale had drifted by ~8x relative to the first), 0.22 (fr3, the merge comes 1500 frames after the re-init; the second map's drift is spread over both maps by the loop BA; 4x more BA iterations change nothing). Per-map alignment of the default hides exactly this mismatch (it fits a free scale to each map), so the default's smaller numbers are not a like-for-like win; what merge changes is that the maps become one consistent, reusable map and
tracking can relocalize against both. A first version applied the scale of the loop twice (merged mh01 ATE 3.0 / 2.7 m, fr3 0.38); fixed by re-expressing the loop with the current keyframe's own rigid pose after the move (see `merge_components`). ASan/UBSan clean on a fr1_xyz merge run and a
complex `rframe=1 merge=1` run.

### Verdicts

| feature | verdict | why |
|---|---|---|
| R-frames, `rframe=1` | **accepted as opt-in, NOT promoted** | does what the roadmap asked: the complex bursts no longer cost the map (Lost 43 -> 1, coverage 97 -> 99%, one map, local accuracy unchanged: win60 0.31 vs 0.29 per segment, burst study seg median 0.10 = default), Outdoor-1 Lost 3 -> 0, +1 Outdoor-1 window (33 -> 34), gyro blank-out survived with 0 Lost frames. Against promotion: windows are otherwise unchanged (109/114 TUM + complex + mh01 both), fr1_floor 0.024 -> 0.033 (20 extrapolated end frames, in the 0.01 band), and the single-alignment ATE of a run with bridges is worse (complex 4.75 vs 2.73) because the scale across a gap is only a speed prior |
| map merge, `merge=1` | **accepted as opt-in, NOT promoted** | works (9 of 18 blank-outs merge, ASan clean, TUM/mh01/Outdoor/complex defaults untouched) but cannot help on any evaluation sequence by itself (the long sequences never return to the first map: complex moves 400 m away), window counts do not improve (141 vs 142 /151: one complex window less because the soft reset changes the young-map rules) and a merge across maps with very different drift costs single-alignment accuracy |

Defaults unchanged: `rframe=0`, `merge=0` (plus all earlier defaults). Window totals over the 151 windows (TUM 75 + mh01 11 + complex 28 + Outdoor-1 37): default 142, `rframe=1` 143, `rframe=1 merge=1` 143, `merge=1` 141.

### Open issues
- Scale across a gap: monocular vision cannot see it; the speed prior is right to ~10-30% (final scale error of a bridged part vs the part before: +/-30%), and every bridge is a new chance for a scale break; a visual-inertial scale of both parts would fix it but the parts between the complex bursts are shorter (6 s) than the 10 s the VI initializer needs. The IMU (accelerometer) is not used for the gap itself.
- R-frame position is extrapolated, not observed (up to a few dm); the trigger is a tracking failure, not an earlier low-parallax detection, so frames just before the failure (tracked landmarks 119 -> 17 over 8 frames) are still ordinary frames.
- R-frames cannot rescue a turn into a textureless view or a blank without an IMU (the vision chain dies at once, as in the default).
- Merge: the second map's drift is dumped into the pose graph / loop BA as for any loop; no scale-drift-aware (Sim3 pose graph with per-map scale freedom beyond g2o Sim3) treatment of the very long second map; the deterministic port's mapper pause after an ordinary loop still applies (only a merge resumes it).
- Parameters were tuned on the complex bursts (overlapping windows); the TUM and Outdoor-1 results say nothing about the tuned values since those sequences hardly enter R-mode.
