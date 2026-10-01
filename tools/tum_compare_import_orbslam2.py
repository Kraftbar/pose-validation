#!/usr/bin/env python3
"""Import the existing ORB-SLAM2 paper-bench results
(runs/orb_port/paper_bench/<seq>/{upstream,port}/) into the shared
runs/tum_compare/<system>/<seq>/ layout as systems
`orbslam2_upstream_st` (single-threaded reference) and `orbslam2_cport`.

Copies (does not symlink, so runs/tum_compare/ is self-contained) the
trajectory/keyframe-trajectory .tum files and derives run.json from
stats.json (runtime_s -> wall_s, frames -> frames_in).
"""
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_ROOT = ROOT / 'runs' / 'orb_port' / 'paper_bench'
OUT_ROOT = ROOT / 'runs' / 'tum_compare'

SEQ_IDS = ('fr1_xyz', 'fr1_desk', 'fr1_floor', 'fr2_xyz', 'fr3_long_office')

SYSTEMS = {
    'orbslam2_upstream_st': ('upstream', 'upstream'),
    'orbslam2_cport': ('port', 'port'),
}


def main():
    for system, (subdir, suffix) in SYSTEMS.items():
        for seq_id in SEQ_IDS:
            src_dir = SRC_ROOT / seq_id / subdir
            if not src_dir.exists():
                print(f'SKIP {system}/{seq_id}: no source dir {src_dir}')
                continue
            stats_path = src_dir / 'stats.json'
            traj_path = src_dir / f'trajectory_{suffix}.tum'
            kf_traj_path = src_dir / f'keyframe_trajectory_{suffix}.tum'
            if not stats_path.exists() or not traj_path.exists():
                print(f'SKIP {system}/{seq_id}: missing stats.json or trajectory')
                continue
            stats = json.loads(stats_path.read_text())

            out_dir = OUT_ROOT / system / seq_id
            out_dir.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(traj_path, out_dir / 'trajectory.tum')

            run_json = {
                'wall_s': stats.get('runtime_s', 0.0),
                'frames_in': stats.get('frames', 0),
                'exit_code': 0,
                'notes': (
                    f"imported from {src_dir.relative_to(ROOT)}; "
                    f"tracked={stats.get('tracked')}, resets={stats.get('resets')}, "
                    f"relocs_success={stats.get('relocs_success')}"
                ),
            }
            if kf_traj_path.exists():
                shutil.copyfile(kf_traj_path, out_dir / 'keyframe_trajectory.tum')
                run_json['keyframe_trajectory'] = 'keyframe_trajectory.tum'
            (out_dir / 'run.json').write_text(json.dumps(run_json, indent=2))
            print(f'imported {system}/{seq_id}: tracked={stats.get("tracked")}/{stats.get("frames")}')


if __name__ == '__main__':
    main()
