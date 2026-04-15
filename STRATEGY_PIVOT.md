# Strategy Pivot Proposal

*Drafted 2026-04-14 after a session review across the last several agent runs.*

## Diagnosis: why we are stuck

Over the last ~10 LLM sessions the loop has been: pick a small bolt-on (BRIEF
variant, BA variant, LOC trim) → run `--all_gt --force` → reject if ATE
regresses or is neutral → log in `PURE_C_RECOVERY.md`. The rejection log now
has 10+ detailed entries and ATE has not moved on any GT sequence.

Five root causes, in decreasing order of weight:

1. **Local optimum.** `pure_c_brief`'s architecture (Harris corners + KLT
   tracking + random BRIEF as relocalization fallback) has a ceiling. Every
   bolt-on we have tried (IC-angle BRIEF, projected-search BRIEF, prev-frame
   BRIEF, descriptor-only pose fallback, global BA every 10 KFs) has either
   regressed ATE or been neutral. We are out of small bets.
2. **Wrong aspirational target.** `simple_slam_opt.cpp` (our current "beat
   this" reference) gets mean ATE ~0.63 m. ORB-SLAM2 gets ~0.075 m on the
   same xyz sequence — **8× better** than cpp. We have been fighting for
   0.01 m improvements against a reference that is itself 8× worse than the
   real state of the art.
3. **AGENTS.md promotion rule is biased against climbing.** The rule
   "ATE-neutral → do not promote" correctly prevents silent drift, but combined
   with "LOC is first-class" it pushes every session toward defensively-scoped
   edits that cannot change the architecture. Structural rewrites that go
   through a transient regression phase are effectively banned.
4. **Process overhead dominates 5-6h sessions.** A new LLM session spends the
   first ~30 min re-deriving context from `AGENTS.md`, `PURE_C_RECOVERY.md`,
   `BENCHMARKS.md`, `gt_tracking.csv` and recent commits, then another chunk on
   each `--all_gt` cycle (~5 min). The actual code-change budget per session is
   maybe 2-3h, which is not enough to land a structural change in one session
   — so every session plays defense.
5. **No per-frame diagnostic.** We see one ATE number per sequence. We do not
   see *which frames* drift, *which keyframes* are bad, *which map points*
   have the worst reprojection error. Every experiment is a blind bet.

## Proposal: three drastic changes

### 1. Commit to a multi-session ORB-SLAM-style port with a staged plan

Stop trying to outsmart the architecture with single-session bolt-ons. Commit
to a **3-5 session plan** to build a new impl (`simple_slam_c_orb.c`) that
actually has the pieces proven to work on these sequences:

- **Session A:** Scale pyramid (4-5 levels, scale 1.2) + FAST-9 corner
  detector + IC angle computation. Validate by visualizing corners per level.
  No ATE goal yet.
- **Session B:** Oriented BRIEF with ORB's **learned** 256-pair pattern table
  (copy from OpenCV's `orb.cpp`) + brute-force Hamming kNN + Lowe ratio.
  Validate by checking match stability frame-to-frame.
- **Session C:** Wire into the existing E/PnP/triangulation path, replacing
  KLT tracking. Expect regression at first; accept it.
- **Session D:** Tune, fix bugs, land ATE parity with cpp. If parity is not
  reached, this is the first *real* hypothesis rejection in weeks.
- **Session E (optional):** Windowed BA over last K keyframes. Do not
  reintroduce full-map BA (we already rejected it).

Exit criterion for promoting the port: ATE ≤ cpp on at least 3/4 GT
sequences, any regression ≤0.05 m on the remaining one.

Expected ending size: `simple_slam_c_orb.c` at ~550-700 LOC. **LOC will grow.
That is the point.**

### 2. Suspend "LOC is first-class" until ATE catches up

Amend `AGENTS.md` so LOC is a *tiebreaker*, not a first-class metric, **until
mean ATE is within 0.02 m of `cpp`**. Rationale: `pure_c_brief`'s small size
is currently preventing it from matching cpp's accuracy; keeping LOC
first-class is what traps us in the local optimum.

After ATE parity, reinstate LOC as first-class and compress.

Concrete change to `AGENTS.md` "Design Philosophy":
- Move "LOC is High Priority" under a "Phase 2" heading, gated on ATE parity.
- Keep "Benchmark Driven" and "Pure C Focus" as active phase-1 constraints.

### 3. Build a per-frame error diagnostic tool before the next experiment

Add `tools/diagnose_trace.py` (name negotiable) that takes a metrics JSON plus
the GT `.npz` and outputs, per frame: translation error vs GT, rotation error
vs GT, number of tracked map points, whether PnP or E was used, and the
top-K worst map points by reprojection error. Spend one session on this.

This is a force multiplier: every subsequent experiment can be evaluated
against *where* it helps instead of just *whether* mean ATE moved. We would
have caught the "points grow but ATE doesn't" pattern in BRIEF variants in one
session instead of four.

## Stop doing

- Small bolt-on BRIEF/BA/PnP variants on `pure_c_brief`. We have exhausted
  this. `PURE_C_RECOVERY.md` is proof.
- Rejection logging as a substitute for progress. The recovery note is useful
  guardrail but the session ending with "logged a rejection" is a failure mode,
  not a deliverable.
- Comparing to `cpp` as an aspirational ceiling. cpp is a weak reference.
  The real target is ORB-SLAM2 or RTAB-Map.

## Concrete next-session actions

If you accept this proposal, the next session does exactly this, in order:

1. Amend `AGENTS.md` per change #2 (10 min).
2. Write `tools/diagnose_trace.py` per change #3 (1-2 h). Validate on
   `pure_c_brief` xyz output — confirm it surfaces the frames with worst
   drift.
3. Begin Session A of the port: add scale pyramid + FAST-9 to a new file
   `simple_slam_c_orb.c`. Do not wire into benchmark yet.

Do **not** run `--all_gt` in this next session. No production change lands
this session; this is infrastructure work.

## Risk

The port may not reach ATE parity with cpp. That is a real risk — the whole
reason we have been trapped in bolt-on land is that the first try at a piece
of the ORB pipeline (IC-angle BRIEF) already regressed. But the alternative is
more sessions of zero-progress bolt-on cycles, and the current rejection log
makes clear that path is exhausted.

Single-session reject is cheap; multi-session reject is expensive. This
proposal trades guaranteed cheap failures for a harder bet with actual upside.

## Pivot Status (2026-04-15)

- **Sessions A, B, C (DONE)**: Fully implemented the library-free ORB pipeline in `simple_slam_c_orb.c`.
- **Results**: Achieved ATE parity with Python/OpenCV baselines across all 4 GT sequences.
- **Infrastructure**: `tools/diagnose_trace.py` is live and was used to debug the PnP stabilization.
- **Next Up**: **Session D (Pose BA)** to break the ATE plateau and target <0.11 m on XYZ.
