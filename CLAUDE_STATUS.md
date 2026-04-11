# Claude Pure C SLAM Archive

This branch archives the local pure C SLAM work that came from the interrupted Claude session, plus the follow-up review/tuning done before `master` was reset.

## Original Claude task list

- `done`: Bug fixes: RANSAC sampling, NMS break, `srand`
- `done`: Hartley 8-point normalization
- `done`: Bucketed corners + motion-model LK seed
- `in progress`: Map point culling + adaptive KF
- `open`: Code tidiness pass
- `open`: Update docs (`PURE_C.md`, `BENCHMARKS.md`)

## What this archive branch contains

- Claude's local pure C SLAM commits that were ahead of `origin/master`
- The shelved follow-up changes made during review of that work
- Pure C/doc experiments that were kept off `master`

## Notes

- `master` was intentionally reset back before these experiments
- This branch exists so the Claude attempt can be revisited, cherry-picked, or compared later
