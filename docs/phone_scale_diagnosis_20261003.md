# Why camera+IMU systems lose metric scale on the phone sequences (diagnosis, 2026-10-03)

Diagnosis only: no SLAM/library code touched, nothing committed. Scripts `tools/phone_diag/`, outputs `runs/phone_diag/` (txt/json per script; `init_table.md`, `init_variant_scores.md` for the initialiser re-runs).
Run with `external/gnss/venv/bin/python` (numpy only). Data: Mobile-GVIO Indoor-1/2, Outdoor-1/2 (CC BY), ADVIO-15/20 (CC BY-NC), EuRoC MH_01 as control; IMU + GT + the visual trajectories already in `runs/`. No images fetched.

## Answer in one paragraph
Not the accelerometer, not timestamps, not the GT, not rolling shutter. The phone accelerometer carries the GT motion with gain 0.92-1.09 (EuRoC 0.995). The scale is lost because **walking
motion has too little scale-observable signal compared with the error of the monocular visual trajectory**: after the quadratic detrend that any short-window scale estimator implicitly applies (free v0 and g),
the remaining GT displacement over a 3-4 s window is 15-30 mm rms on the phones (EuRoC 44-64 mm), while the visual position error is 17-64 mm (EuRoC 7-8 mm). Regressing accelerometer-integrated
displacement on a noisy visual displacement is an errors-in-variables problem and is biased low by S^2/(S^2+N^2) (S signal, N visual error). That predicted attenuation matches the measured scale ratio of an independent
estimator in 15 of 18 cases within 0.05 (table 5), and the length dependence (ratio rises with window length). Long outdoor collapses (XRSLAM/OKVIS) are not triggered by excitation dips: excitation is stationary and equally low where XRSLAM is metric (Outdoor-1) and where it fails (Outdoor-2).

## 1. H1 accelerometer content: REFUTED as the cause
Method: GT second difference (step m*h) vs the raw accelerometer rotated with GT orientation (R_bI from gyro-vs-GT-omega Kabsch) and filtered with the identical triangular kernel, so both sides have the same band limit
(`h1_accel.py`, `h1_window.py`, `h1_spectral.py`). Validation: EuRoC gives gain 0.98-1.01 and coherence 0.95-1.00 in every band to 4 Hz.

| item | Outdoor-1 | Outdoor-2 | Indoor-1 | Indoor-2 | EuRoC | ADVIO-20 |
|---|---|---|---|---|---|---|
| gravity-free (2 s high-passed) isotropic gain k, H=0.2-0.3 s | 1.01 | 0.92-0.94 | 1.01-1.03 | 1.07-1.09 | 0.995 | n/a (GT frames, below) |
| per-IMU-axis k (same R_bI as gyro; all positive, so no sign/axis error) | 1.01 .99 1.02 | .97 .57 .90 | 1.02 1.02 1.06 | 1.11 1.07 1.03 | .99 .99 1.00 | |
| vertical gain bounds at the 1.5-2.5 Hz step band (coherence) | .74-.99 (.75) | .61-1.02 (.60) | .85-.97 (.88) | .88-.89 (1.00) | .99-1.00 (.99) | .87-1.15 (.96) |
| horizontal gain bounds 0.7-1.5 Hz (coherence) | poor (GT noise) | poor | .56-1.06 (.55) | .91-1.03 (.92) | .99-1.00 (.98) | n/a |
| window k with GT positions, free g, L=2/4/8 s | 1.00/.92/.54 | .90/.82/.61 | .99/.95/.73 | 1.03/.91/.66 | .99/1.00/.99 | |
| accel time offset (scan +-0.4 s; peak) | 0 (+-25 ms) | | | | 0 | |
| gyro-vs-GT delay / gyro gain | +10 ms / 1.07 | +10 / (noisy) | +5 / 1.05 | 0 / 1.05 | 0 / 1.0 | 0 / 1.02 |
| median specific-force norm (static subset) | 10.34 (10.00) | 10.45 | 10.31 (9.82) | 10.34 | 9.80 | 9.81 |

- Gain/bandwidth: no low-pass of the walking band (1-3 Hz) and no attenuation beyond the 0-15 % seen on Indoor-2 vertical. The IMU stream is band-limited above ~12 Hz (gyro PSD 0.7-0.95 Nyquist / 0.2-0.4 Nyquist = 0.00, accelerometer 0.03-0.06; EuRoC 1.4-2.2 = white floor, `h3_timing.txt`), but that is above gait content.
- Units/sign/axes: m/s^2, specific force, signs consistent between accel and gyro (no negative per-axis gain). Mobile static norm is 2-10 % high with dynamic gain ~1.0, i.e. an offset of ~0.5 m/s^2 along the gravity axis, not a gain; consistent with the shipped `_cal` rescale doing nothing to scale (table 6, `noacc`/`cal` rows of init_summary).
- Side finding: Mobile gyro reads ~5-7 % low vs GT omega (1.05-1.07). Rotation residuals after the fitted extrinsic are ~0 (rot_before/after 0.00), not a scale driver.
- Caveat ADVIO: GT position and GT orientation are in inconsistent frames (horizontal accel incoherent, 0.00-0.02; only the vertical axis agrees, gain .94-1.05 at 0.7-1.5 Hz coherence .97), so GT-orientation tests are not used for ADVIO.
- Caveat window k at L=8 s drops to 0.54-0.73 on the phones even with GT positions (EuRoC 0.99): at long windows the GT orientation (LiDAR roll/pitch, 1 deg = 0.17 m/s^2 gravity leak, over 8 s ~5 m) limits the test, not the accelerometer.

## 2. H2 GT quality / lever arm: not the cause (but noisy outdoors)
- GT noise upper bound = |detrended GT displacement - detrended accel double integral|, L=3 s: 6 mm (Indoor-1), 6.6 (Indoor-2), 10 (Outdoor-1), 20 (Outdoor-2); EuRoC 4. Visual error is 17-64 mm, so GT is 2-5x better than the trajectory it judges. Detrended signal amplitudes agree: GT 18.0/21.5/20.5/26.5 mm vs accel-integral 17.2/21.0/17.4/21.2 (Indoor-1/-2, Outdoor-1/-2, L=3), i.e. the GT scale is metric to ~10 %. Outdoor spectral coherence is low (0.6-0.75 even at the step band) because of GT noise, indoors 0.88-1.00.
- Per-window Sim3 truth scale is stable over window length (stella Outdoor-1 67.6-69.3 for L=3-12, ORB-SLAM3 18.0-18.3), so no short-window GT scale bias.
- GT lag: 0-10 ms (gyro), accel-lag scan peaks at 0.
- Lever arm: free lever r in the accelerometer regression fits (0.07,0,0.03)/(0.08,-0.03,-0.13)/(0.11,-0.13,0.05)/(0.10,-0.14,0.08) m (Outdoor-1/-2, Indoor-1/-2) and changes the residual by 0-25 % and gain by <0.05; omega^2 r at 0.3 rad/s and r=0.15 m is 0.014 m/s^2. A joint fit of r from visual-vs-GT positions (`lever_fit.py`) improves the Sim3 residual by only 1-3 % (outdoor) so r is unobservable there and not a driver of the 20-60 mm error.

## 3. H3 timestamps: not the cause
(`h3_timing.txt`, `camera_stamps.txt`.) Mobile IMU: 100.0 Hz, dt std 0.003-0.005 ms, max 10.1 ms, 0 gaps, 0 duplicates; ADVIO std 0.03 ms max 12 ms. Frames: 66.8 ms, std 0.00-0.09 ms (Indoor-1 0.81 ms, max 75 ms), no duplicate stamps; ADVIO 33.3 ms.
Per 30 s window: IMU-vs-GT offset (gyro vs GT omega) Outdoor-1 +10 ms [5,12.5], Outdoor-2 +17.5 [5,25], Indoor-1 +5 [5,7.5], Indoor-2 0 [0,2.5], ADVIO 0: no drift (<=1.4 ms/window).
Camera-vs-IMU offset per 30 s (ORB-SLAM3 mono vs gyro): Mobile -5 to -15 ms (Indoor-1 one window -35), ADVIO -320 ms constant. Sensitivity of the scale ratio (`h4_timing_sens.json`): +-30 ms changes the ratio by <=0.025; +-100 ms lowers it by 0.07-0.16; 300 ms destroys it (0.00-0.15). So ADVIO's 0.32 s offset is a genuine, large, but separate and already-handled issue (raw vs fitted: median scale error 414 % -> 82 % stella, 647 % -> 39 % ORB-SLAM3, advio20, `init_summary.md`); it does not explain the Mobile 0.1-0.5.

## 4. H4 rolling shutter / exposure stamping: negligible for scale
No readout time exists in the data and no images were fetched. A rolling shutter read from the first row (or stamp = exposure start) is equivalent to a constant camera-IMU shift of T_r/2 + t_exp/2 (~10-20 ms); the fitted per-window offsets are already -5..-15 ms and a +-30 ms shift moves the scale ratio by <=0.025 (Outdoor-1 ORB-SLAM3 .456 -> .479, stella .258 -> .270, Indoor-2 .209-.210, Outdoor-2 .586-.600, ADVIO-20 .673-.678). Any residual intra-frame distortion would show as visual position noise, which section 5 already measures in total.

## 5. Cause: scale-observable signal << visual position error (errors-in-variables)
`vis_scale.py`: independent numpy closed-form estimator (s*P_vis = P0 + v0 tau + g tau^2/2 + double-integral of R f; same inputs as `imu_init_eval`) reproduces the C initialiser (ORB-SLAM3 Outdoor-1 L=3: 0.39 vs C 0.39).
`vis_noise.py`: S = rms of the quadratically detrended GT displacement in the window, N = rms of (Sim3-aligned visual - GT) after the same detrend, prediction S^2/(S^2+N^2).

| trajectory (L=4 s) | S mm | N mm | predicted | measured (python estimator) |
|---|---|---|---|---|
| EuRoC stella | 64 | 8 | 0.98 | C init 0.91 |
| Outdoor-1 ORB-SLAM3 / stella | 23 / 22 | 25 / 40 | 0.45 / 0.26 | 0.46 / 0.26 |
| Outdoor-2 ORB-SLAM3 / stella | 32 / 30 | 31 / 35 | 0.50 / 0.42 | 0.59 / 0.44 |
| Indoor-1 stella | 23 | 17 | 0.63 | 0.72 |
| Indoor-2 stella / ORB-SLAM3 | 29 / 27 | 64 / 52 | 0.20 / 0.27 | 0.21 / 0.29 |
| ADVIO-20 ORB-SLAM3 / stella | 18 / 19 | 26 / 33 | 0.35 / 0.28 | 0.68 / 0.49 |

At L=8 S doubles (40-80 mm) and the ratio rises (predicted .40-.87, measured .40-.97): the same trend as the ratio-vs-window curves in `vis_scale_freeg.txt` (e.g. ORB-SLAM3 Outdoor-1 .39/.49/.75/.97 for L=3/4/8/12). ADVIO-20 is the weakest match (measured above predicted).
Band view (`vis_band_snr.txt`, signal power / visual error power): EuRoC 346/175/39/4.8/0.15 for 0.05-0.15/0.15-0.4/0.4-0.8/0.8-1.5/1.5-3 Hz; phones, e.g. Outdoor-1 ORB-SLAM3 22/0.66/0.01/0.01/0.04, Indoor-2 stella 23/3.9/0.06/0.01/0.02, Outdoor-1 stella 0.46/0.21/0.10/0.06/0.04. Above ~0.4 Hz (all of the gait band) the visual error exceeds the walking signal by 10-100x; scale can only come from <0.4 Hz (turns, speed changes), which needs windows of >8-12 s and a visual trajectory without low-frequency drift. Stella Outdoor-1/ADVIO-20 trajectories do not even have SNR>1 at 0.1 Hz (drift in the map scale), which is why they stay at 0.1-0.5 whatever is done.
Why EuRoC works: 3x stronger signal (aggressive handheld motion) and 4-8x smaller position error (near scene, global shutter, 752x480 with good texture).

## 6. H5 long-walk collapse vs excitation: not coincident
(`h5_excitation_full.txt`.) 20 s bins, local scale = GT path length / system path length at 1 s sampling (1 = metric).
- Excitation is stationary: S 20-25 mm (Outdoor-1), 28-38 mm (Outdoor-2), 15-22 mm (ADVIO-20), acceleration rms 0.3-0.8 m/s^2, speed 1.0-1.4 m/s, no slow bins except sequence starts.
- XRSLAM: Outdoor-1 0.85-1.22 in all 18 bins (S 24 mm, acc 0.3); Outdoor-2 0.006 already in the first bin (18 s) with S 22 mm and acc 0.4, falling to 0.0005, even though Outdoor-2 has more excitation than Outdoor-1 (S 32 vs 24 mm). ADVIO-20/15 0.004-0.02 from the start. So XRSLAM's outcome is set by the initialisation (a draw from the biased/noisy distribution above), then nothing in the signal corrects it.
- OKVIS2-X Outdoor-1: 1.17 -> 0.62 -> 0.28 -> 0.16 ... -> 0.001, monotone decay with flat excitation (S 20-32 mm); Outdoor-2 starts at 2-4x too small... then 1.0 at 120-180 s and decays to 0.002; ORB-SLAM3 mono-inertial Outdoor-2 first valid bin 206 then 0.1-0.4 and decay to 0.003. Spearman correlations of log scale with S/acc/speed/yaw have inconsistent sign across systems (-0.7 to +0.8, n<=22 bins each, dominated by the monotone trend) so they carry no evidence.
Interpretation: with SNR<1 per second at all gait frequencies the scale state is a random walk with small restoring information, so it drifts and, when the estimator has multiplicative scale feedback, diverges; no excitation dip is needed.

## 7. Correction and re-run of `imu_init_eval` (derived inputs only, `init_rerun.py`)
Unmodified `imu_init_eval.py` and C binary; only input files / CLI options changed (outputs in `runs/phone_diag/init_rerun/<variant>/`). Table: median over the 12 phone datasets (6 sequences x {ORB-SLAM3, stella} `_fit` inputs) of the per-dataset median ratio estimate/Sim3 truth (`init_variant_scores.md`, full per-dataset grid `init_table.md`).

| variant | L=4 | L=8 | L=12 | phone datasets within 20 % (L=8, of 12) | EuRoC ratio (L=8) |
|---|---|---|---|---|---|
| base | 0.43 | 0.40 | 0.43 | 1 | 0.91 |
| visual noise floor 0.1 m (`--floors`) | 0.46 | 0.50 | 0.50 | 2 | 0.98 |
| accel noise x0.2 | ~0.43 | ~0.40 | | | 0.91 |
| window lengths 24/32 s | unchanged (ORB-SLAM3 Outdoor-2 .54/.58, Indoor-2 stella .22/.23) | | | | 0.91 |
| visual positions Gaussian-smoothed 0.3 s | 1.07 | 0.99 | 1.01 | 7 | 1.30 |
| ... 0.6 s | 1.22 | 1.14 | 1.14 | 6 | 1.56 |
| matched: positions AND accelerometer smoothed 0.6 s | 0.70 | 0.82 | 0.81 | 6 | 1.22 |
| matched 1.0 s | 0.69 | 0.90 | 0.82 | 7 | 1.46 |
| matched 0.3 s | 0.63 | 0.64 | 0.69 | 4 | 1.07 |

Result: scale recovers from 0.4 to 0.8-1.0 (median) on the phones with a band-limited visual trajectory (matched 0.6-1.0 s, L>=8 s), but it is not a clean fix: (i) the unmatched smoothing "works" only because it overshoots to compensate the attenuation (EuRoC 1.3-1.6), (ii) the matched filter keeps EuRoC at 1.07 for 0.3 s but 1.2-1.5 for 0.6-1.0 s (body-frame smoothing of f is not exactly the double-integral smoothing), (iii) stella Outdoor-1 (0.11 -> 0.3-0.5), Indoor-2 stella (.23 -> .66-.78) and Indoor-1 ORB-SLAM3 stay poor because their trajectories carry low-frequency scale drift. Noise tuning, accel rescale, window lengthening alone and time offset do nothing. A principled version needs: use visual-position covariance (the same N estimate) as a weight / SNR gate in an errors-in-variables estimator, use <0.5 Hz content over >=10 s, and feed the estimate to a long-horizon scale state (or GNSS/step-length prior). Not implemented here.

## 8. Verdicts
| H | verdict | key numbers |
|---|---|---|
| H1 accel content | refuted | gain 0.92-1.09 (EuRoC .995), no 1-3 Hz low-pass, axes/signs consistent, delay 0+-25 ms, static +0.5 m/s^2 offset |
| H2 GT / lever | refuted (GT noise 6-20 mm, lever <=15 cm, negligible effect) | GT 2-5x better than the visual error |
| H3 timestamps | refuted | jitter <0.1 ms, offsets stable +-10 ms/30 s; ADVIO 0.32 s real but separate |
| H4 rolling shutter | negligible | +-30 ms -> ratio change <=0.025 |
| H5 excitation | not a trigger | excitation stationary; XRSLAM outcome fixed by init; OKVIS decays monotonically |
| Cause | scale-observable signal (S 15-30 mm / 4 s) below visual position error (N 17-64 mm); EIV bias S^2/(S^2+N^2) predicts measured ratios | EuRoC S 64 / N 8 mm |

Limits: lever arm and GT orientation noise limit the GT-based accelerometer tests at windows >=8 s; ADVIO GT frames inconsistent (GT-orientation tests skipped); N includes small GT noise and the lever contribution (upper bound); one run per variant; stella/ORB-SLAM3 trajectories only (no tightly coupled system trajectory used for the EIV measurement).
