# Permissive building blocks: survey and two evaluations (2026-10-03)

Scope: components (not whole SLAM systems) that could slot into the C99 stack. Licences were read from the LICENSE file of a fresh
shallow clone unless marked "(not checked)". Learned methods excluded. Nothing here modifies `stella_port/`, `stella_vio/`, `okvis_port/`,
`gnss_fusion/`, `phone_pipeline/`; nothing committed. Sources/builds: `external/blocks/` (0.8 GB, git-ignored), scripts `tools/blocks/`,
results `runs/blocks/`.

## 1. Survey

Effort = to a dependency-free C99 file set with unit checks against the C++ original (agent-days, one agent).

| what | licence (checked) | language / deps | what it gives us | effort |
|---|---|---|---|---|
| **PoseLib** (vlarsson) | BSD-3 | C++17, Eigen | P3P/P4P/up2p/5pt/8pt/upright relpose/homography minimal solvers, LO-RANSAC + Sampson/reprojection LM refinement, gravity overloads | solvers: 0.5-1 day each (small closed forms, Eigen 3x3 only); LM refinement: 1 day; LO-RANSAC wrapper: 1 day |
| OpenGV (Kneip) | BSD-3 (ANU) | C++, Eigen | central/non-central absolute + relative pose, generalized (multi-cam) | overlaps PoseLib; only if multi-camera rigs are needed |
| RansacLib (Sattler) | BSD-3 | header-only C++ templates | LO-RANSAC / PROSAC skeleton | 0.5 day to copy the idea; PoseLib has the same |
| GC-RANSAC / MAGSAC++ (Barath) | BSD-3 (file says dependencies may change it) | C++, OpenCV, Eigen | better inlier scoring (sigma-consensus) | 1-2 days for MAGSAC++ scoring only |
| COLMAP / TheiaSfM | BSD-3 / BSD-3 (UC Regents) | large C++ | relpose/PnP solvers (PoseLib supersedes), SfM | not a block, a system |
| OpenMVG | MPL-2.0 | C++ | solvers, file-level copyleft | no (MPL) |
| **RTKLIB-EX** (rtklibexplorer, 2.4.3 b34 line) / RTKLIB | BSD-2 (+ note: companion binaries keep own licences) | plain C, libm/pthread | SPP, Doppler velocity, ephemeris (GPS/GLO/GAL/BDS/QZS), Klobuchar/Saastamoinen, RTK/PPP, lambda | SPP+velocity+ephemeris: 3-4 days (about 2.5 k LOC of ephemeris.c/pntpos.c/rtkcmn.c subset); it is already C |
| laika (comma.ai) | MIT | Python/numpy | RINEX, ephemeris, SPP, DGNSS; a readable second reference | reference only |
| gnss-sdr | GPL-3 | C++ | software receiver | no (GPL) |
| Kalibr | BSD-3 **with advertising clause** | C++/ROS, heavy | camera-IMU calibration reference (ready tool, not for linking) | use as offline tool; port no |
| Basalt calibration | BSD-3 (not checked) | C++ | camera-IMU calibration alternative | not evaluated |
| fbow (in tree) | MIT | C++ | BoW place recognition (already used by stella) | done |
| DBoW2 / DBoW3 | BSD-like with "author must be notified" clause (DBoW2 LICENSE.txt, DBoW3 LICENSE.txt) | C++ (OpenCV, Boost) | BoW | no, fbow is better licensed |
| OpenCV (ORB/BRISK/KLT/FAST) | Apache-2.0 (not checked) | C++ | already ported ORB/FAST/KLT in stella/pure_c | nothing new |
| Sophus (MIT), GTSAM (BSD), Pangolin | MIT / BSD / MIT | C++ | Lie groups, factor graph | not needed (own 4-DoF smoother) |
| PDR/step libraries, rolling-shutter utilities | none found with a permissive licence and code; searches in the roadmap found papers only | | our own `gf_gait` already exists | skip |

Top picks: PoseLib solvers/refinement (evaluated below), RTKLIB SPP + Doppler-velocity subset (evaluated below), MAGSAC++ scoring (cheap follow-up).

## 2. PoseLib vs stella_vio solvers

Harness `tools/blocks/blocks_eval.cpp` (links PoseLib and the `stella_vio/c` sources read-only; `make_pairs.py`, `blocks_report.py`).
Data: 635 frame pairs (gaps 4/8/16/32 frames) from TUM fr1_xyz, fr1_desk, fr1_floor, fr2_xyz; real cv2-ORB matches (1500 features, ratio 0.8,
undistorted), GT relative pose from mocap, depth for PnP, **real Kinect accelerometer** gravity rotated into the camera frame by one fixed
Kabsch alignment per sequence (residual vs GT gravity: median 2.3 / 4.3 / 2.0 / 0.5 deg for fr1_xyz / desk / floor / fr2_xyz). 600 pairs have >= 50 matches.
"stella init" is the real `sv_init_try_monocular` (H and F RANSAC, 100 iterations, decompose, triangulation checks; seeds=1, or 4 = stella_vio multi-start). PoseLib
variants return a pose for every pair, so a stella-style acceptance test (triangulate inliers, >= 50 valid points, 1 deg parallax at the 50th) was applied to them too (my re-implementation, not
identical code). Success = accepted AND rotation error < 2 deg AND translation-direction error < 20 deg.
Full tables: `runs/blocks/poselib/report.md`, raw rows `res.csv`.

Relative pose / init (600 pairs):

| solver | accepted | success | median rot err (deg) | median time ms |
|---|---|---|---|---|
| stella init, 1 seed | 44% | 9% | 1.76 | 3.8 |
| stella init, 4 seeds | 66% | 20% | 1.55 | 15 |
| stella init, 4 seeds + PoseLib Sampson LM refinement of its pose | 66% | **44%** | 0.66 | 16 |
| PoseLib 5pt LO-RANSAC (+ refine) | 78% | 66% | 0.55 | 8.8 (same at 100 iterations) |
| PoseLib homography + stella decompose | 72% | 30% | 1.79 | 4.9 |
| PoseLib upright 3pt, accelerometer gravity (real) | 76% | 42% | 0.92 | 1.7 |
| PoseLib upright 3pt, ground-truth gravity | 74% | 70% | 0.38 | 1.7 |
| PoseLib upright 3pt, GT gravity + 3 deg error | 87% | 29% | 1.88 | 1.8 |

By GT baseline (success): 5-15 cm: stella 4 seeds 56%, +refine 75%, PoseLib 5pt 91%; < 5 cm: 45 / 47 / 57%; > 30 cm: all < 45% (few matches).

PnP / relocalisation (2D from view 2, 3D from view-1 depth; success = rot < 2 deg and camera centre < 5 cm), injected outliers on top of natural ones:

| solver | +0% | +30% | +50% | +70% | +85% | ms (0%/85%) |
|---|---|---|---|---|---|---|
| stella `sv_pnp_ransac`, 30 iterations (reloc setting) | 90% | 79% | 46% | 8% | 1% | 1.5 / 1.4 |
| stella, 100 iterations | 92% | 88% | 76% | 24% | 2% | 4.8 / 4.8 |
| PoseLib P3P LO-RANSAC + refine | 92% | 93% | 91% | 89% | 71% | 2.2 / 3.8 |
| PoseLib up2p, GT gravity | 97% | 97% | 94% | 93% | 86% | 2.1 / 21 |
| PoseLib up2p, accelerometer gravity | 72% | 72% | 70% | 68% | 64% | 2.7 / 27 |

Accuracy on the clean set (median camera-centre error): P3P 0.57 cm, stella-100 0.86 cm, stella-30 1.0 cm.

Findings:
1. About half of PoseLib's init advantage over stella comes from **non-linear refinement of the final pose**, not from a new minimal solver: refining stella's own output
   with a Sampson LM lifts success 20% -> 44% and halves the rotation error (1.55 -> 0.66 deg) at +1 ms. The remainder (44 -> 66%) is 5pt (calibrated, no F-rank/8pt noise
   amplification) + LO-RANSAC + its scoring. Iteration count is not the issue (5pt at 100 iterations equals 1000).
2. **PnP relocalisation is clearly better with PoseLib at high outlier ratios**: stella's RANSAC collapses at +70% (8-24%), P3P LO-RANSAC holds 89%, and is 2-3x faster per
   successful solve than stella-100. In BoW reloc, 50-80% outliers are normal; at 0-30% they tie.
3. **Gravity-aware solvers help only with accurate gravity.** With ground-truth gravity upright 3pt/up2p beat the generic ones (init 70% vs 66%, 0.38 vs 0.55 deg, 4.5x faster minimal
   solver; PnP +85%: 86% vs 71%). With the raw Kinect accelerometer (0.5-4 deg residual, motion-contaminated) they are *worse* than generic (42% vs 66%; 72% vs 92%), and 3 deg of
   gravity error already destroys the init gain. A phone would need a gyro-fused gravity good to below 1 deg; our `stella_vio` gravity estimate has not been shown to be that accurate.
   Use upright solvers only as a hypothesis generator with a generic fallback.
4. Homography route of PoseLib (decomposed with stella's code) is poor on these mostly non-planar scenes; no reason to prefer it. stella's H/F model selection is not the bottleneck.
Caveats: TUM Kinect, 640x480, one matcher; stella was fed matches rather than its own area-matcher; the acceptance test for PoseLib is my approximation. Single run, deterministic seeds.

## 3. RTKLIB on raw u-blox GNSS

The GVINS bag had been deleted, so the raw `/ublox_driver/*` topics (4400 epochs of `range_meas` at 10 Hz, 217 GPS/GAL/BDS and 75 GLONASS ephemerides, Klobuchar iono) of the first 440 s of
`complex_environment` were re-streamed over HTTP and filtered on the fly (`stream_gnss_from_bag.py`, 5 GB transferred, 21 MB kept; message layout from the `gnss_comm` .msg files only), decoded by
`gnss_raw_decode.py`. This is the only raw-GNSS sequence we hold (Mobile-GVIO/ADVIO are iPhone fixes). Satellite-id mapping was inferred from semi-major axes (GPS 1-32, GLO 33-59, GAL 60-95, BDS 97+; BDS toe is already in GPST). Harness
`rtklib_spp.c` links RTKLIB-EX `pntpos()` (single-frequency L1/E1/B1/G1, Klobuchar + Saastamoinen, Doppler velocity from `estvel`, broadcast ephemeris). GT = the receiver's own RTK solution
(carrier fixed, h_acc <= 0.1 m; positions and NAV-PVT velocity), time-matched after removing the SPP clock bias. 4042 scored epochs.

| configuration | H rms | H median | H p95 | Vh median | Vh p68 | Vh p95 | sats |
|---|---|---|---|---|---|---|---|
| GPS only, 15 deg | 20.4 | 1.8 | 54.5 | 0.37 | 0.55 | 1.55 | 6.3 |
| GPS+GAL+BDS, 15 deg | 10.7 | 2.1 | 25.1 | 0.25 | 0.37 | 0.96 | 17.9 |
| + SNR mask 30 dB-Hz | 3.0 | 1.2 | 3.4 | 0.16 | 0.23 | 0.57 | 16.4 |
| + SNR mask 35 dB-Hz | 3.4 | 1.2 | 3.0 | 0.16 | 0.21 | 0.50 | 14.8 |
| + GLONASS, SNR 35 | 3.2 | 1.4 | 3.3 | 0.15 | 0.20 | 0.42 | 17.0 |
| dual-frequency iono-free (GPS+GAL) | 21 | 3.7 | 40 | 0.58 | - | - | 8.9 (worse: L2C/E5b noise, not pursued) |

(metres, m/s; `runs/blocks/spp_sweep.md`; the SNR threshold was chosen on this sequence, a phone antenna has lower C/N0, so the threshold will not transfer as a constant.)
Doppler velocity vertical rms is about 0.7 m/s with a heavy tail, horizontal median 0.15-0.25 m/s at walking speed 1.5 m/s (10-15% of speed). Doppler sign convention of the u-blox stream equals RTKLIB's.

Sigma calibration (`spp_calib.py`, snr 35): RTKLIB's reported position sigma is conservative, the 3-D error 68th percentile is 0.52x the reported 3-D sigma (95th: 0.74x); its velocity sigma is
optimistic, 68th percentile 1.9x and 95th 3.3x. So use sigma_pos_axis = 0.6 x reported/sqrt(3) with a 1 m floor, sigma_vel_axis = 2 x reported/sqrt(3). Quality drops with satellite count (8-12 sats: H median 2.5 m, p95 9.6 m; > 16 sats: 1.4 / 2.7 m).

Fusion experiment (`fusion_doppler_exp.py`, `gf_run preset=robust` with the optional `vx vy vz sigma` fix columns, nothing edited; odometry = OKVIS2-X mono VIO without GNSS, 7983 poses; ATE SE3 / geo-referenced no-align error, m, batch and causal):

| fixes | batch SE3 / no-align | causal SE3 / no-align |
|---|---|---|
| GNSS alone (1 Hz SPP, snr 35) | 4.42 / 4.92 | - |
| position only | 0.59 / 2.28 | 0.94 / 2.66 |
| position + Doppler velocity (k=2) | 0.57 / 2.28 | 0.91 / 2.64 |
| 120 s position blackout, position only | 0.57 / 2.06 | 1.69 / 3.16 |
| 120 s blackout + Doppler velocity | 0.61 / 2.18 | 1.41 / 3.11 |
| odometry scaled x0.5 (scale unknown), 120 s blackout, position only | 0.73 / 2.23 | 2.01 / 3.38 |
| same + Doppler velocity | 0.71 / 2.15 | **1.20 / 2.70** |
| SPP without SNR mask (heavy tails), position only / + velocity | 7.07 / 7.96 | 14.7 / 19.9 vs 14.9 / 19.9 |
| 5 s fix spacing, position only / + velocity | 0.80 / 2.47 vs 0.77 / 2.47 | 1.65 / 3.38 vs 1.58 / 3.35 |

Estimated fusion gain: with a good (VIO) odometry and fixes every 1-5 s the Doppler velocity is neutral (<= 0.05 m SE3, inside the 0.01 m "neutral" band for no-align). It helps only in the regimes where the roadmap
claimed it: GNSS outages with causal operation (SE3 2.01 -> 1.20, 1.69 -> 1.41) and unknown scale. The static no-align floor (2.3 m) is the SPP bias, not touched by velocity. By far the largest effect is the quality of the SPP itself
(C/N0 gating + multi-constellation: fused no-align 7.96 -> 2.28 m, GNSS alone 18.9 -> 4.9 m). Caveat: one 440 s handheld urban sequence, 10 Hz receiver-grade F9P antenna; no phone raw data exists locally, so the phone
gain is unmeasured (phone Doppler is noisier, but scale is worse, so the benefit may be larger than here).

## 4. Recommendation

1. **Port PoseLib's Sampson / reprojection LM refinement + P3P (+ optional LO step) into `stella_vio`'s relocalisation and init** (about 2-3 agent-days: lambda-twist P3P, 5pt Nister/Stewenius solver already exists in stella_vio as `sv_essential_5pt`
   so reuse it; add a 6x6 / 5-DoF Gauss-Newton with Cauchy weights, 24 iterations). First step costs 1 day and is nearly free: add the refinement of the F/H result in `sv_init.c` (+24 points of init success in our test). Second: replace `sv_pnp_ransac` 30-iteration
   reloc with P3P + LO-RANSAC + refinement (robust at 70% outliers). Validate with the full `--all_gt` discipline in this repo before claiming gains; the numbers above are a sandbox on 4 TUM sequences with cv2-ORB matches, not the stella pipeline end to end.
2. **Do not adopt upright/gravity minimal solvers now.** They pay only with sub-degree gravity. Revisit when `stella_vio` has a gyro-fused gravity with measured error < 1 deg (or phone-reported gravity); then add them as an extra hypothesis generator (about 1 day on top of 1).
3. **RTKLIB: port the SPP + Doppler-velocity subset (`pntpos.c`, `ephemeris.c`, the needed `rtkcmn.c`/time/geodesy parts, about 3-4 agent-days since it is already C99-style)** as the raw-observation front end of `gnss_fusion` (semi-tight, per
   the roadmap), including C/N0 and elevation gating and the sigma factors above (position 0.6x, velocity 2x of RTKLIB's reported value). Expected benefit: much better fixes than the OS fix on Android raw data (our test: median 1.2 m, p95 3 m with gating), and a velocity factor that
   matters in outages and unknown scale, not in the good-GNSS-plus-VIO regime. Blocked on a recorded Android raw-GNSS dataset (roadmap item 3); until then RTKLIB-EX remains the offline reference (`external/blocks/build/rtklib_spp`).
4. Cheap follow-up: MAGSAC++ sigma-consensus scoring (BSD-3) in the RANSAC loops; nothing else in the survey is worth porting now. Keep GPL (gnss-sdr, GVINS) and MPL (OpenMVG) out; Kalibr only as an offline calibration tool.

Reproduce: `external/gnss/venv/bin/python tools/blocks/stream_gnss_from_bag.py external/blocks/gvins_ublox_raw.pkl 440 && ... gnss_raw_decode.py ... external/blocks/gnss_txt`, build `rtklib_spp.c` against `external/blocks/src/RTKLIB/src`
(compile line in the session: rtkcmn ephemeris pntpos preceph sbas ionex rinex trace datum geoid tides sofa lambda, `-DENAGLO -DENAGAL -DENACMP -DENAQZS`), `tools/blocks/spp_sweep.sh`, `fusion_doppler_exp.py main|nomask|sparse`;
PoseLib: cmake in `external/blocks/build/poselib`, `libstella.a` from the `sv_run.c` source list, `make_pairs.py`, `blocks_eval`, `blocks_report.py`.
