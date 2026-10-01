#!/usr/bin/env python3
"""Build lossless per-sequence videos from the original TUM RGB-D image
sequences, for feeding the repo's 7 monocular implementations without
touching their source.

Each implementation decodes video either via OpenCV's VideoCapture
(python/cpp) or via a raw `ffmpeg -f rawvideo -pix_fmt bgr24 -s 640x480`
pipe (c/pure_c/pure_c_brief/pure_c_orb/pure_c_plus, all of which assume a
640x480 frame). The TUM rgb images are already 640x480, so we encode a
lossless FFV1/bgr0 .mkv at the sequence's native frame order (rgb.txt
order == chronological order). bgr0 (BGR + zero pad byte) is the closest
FFV1-supported pixel format to bgr24 and round-trips exactly.

Verified (see runs/tum_compare/NOTES.md) that both the OpenCV
VideoCapture path and the raw ffmpeg-pipe path decode frame-for-frame
byte-identical to cv2.imread() of the source PNGs.

Usage:
    python3 tools/tum_compare_build_videos.py [--seqs fr1_xyz,...] [--force]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_ROOT = ROOT / 'runs' / 'orb_port' / 'paper_bench' / 'data'
OUT_DIR = ROOT / 'runs' / 'tum_compare' / '_videos'

SEQS = {
    'fr1_xyz': 'rgbd_dataset_freiburg1_xyz',
    'fr1_desk': 'rgbd_dataset_freiburg1_desk',
    'fr1_floor': 'rgbd_dataset_freiburg1_floor',
    'fr2_xyz': 'rgbd_dataset_freiburg2_xyz',
    'fr3_long_office': 'rgbd_dataset_freiburg3_long_office_household',
}


def read_rgb_txt(data_dir: Path):
    lines = []
    with open(data_dir / 'rgb.txt') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            ts, rel = line.split(None, 1)
            lines.append((float(ts), rel))
    return lines


def build_one(seq_id: str, force: bool):
    data_dir = DATA_ROOT / SEQS[seq_id]
    if not data_dir.exists():
        print(f'  [{seq_id}] SKIP: data dir missing: {data_dir}')
        return None
    frames = read_rgb_txt(data_dir)
    out_video = OUT_DIR / f'{seq_id}.mkv'
    manifest_path = OUT_DIR / f'{seq_id}_manifest.json'
    if out_video.exists() and manifest_path.exists() and not force:
        print(f'  [{seq_id}] cached ({len(frames)} frames): {out_video}')
        return manifest_path

    tmp_dir = OUT_DIR / f'_{seq_id}_frames'
    if tmp_dir.exists():
        shutil.rmtree(tmp_dir)
    tmp_dir.mkdir(parents=True)
    for i, (ts, rel) in enumerate(frames):
        src = (data_dir / rel).resolve()
        dst = tmp_dir / f'frame_{i:06d}.png'
        os.symlink(src, dst)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        'ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
        '-framerate', '30',
        '-i', str(tmp_dir / 'frame_%06d.png'),
        '-pix_fmt', 'bgr0', '-c:v', 'ffv1', '-level', '3',
        str(out_video),
    ]
    print(f'  [{seq_id}] encoding {len(frames)} frames -> {out_video.name}')
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr, file=sys.stderr)
        raise RuntimeError(f'ffmpeg encode failed for {seq_id}')
    shutil.rmtree(tmp_dir)

    duration = frames[-1][0] - frames[0][0] if len(frames) > 1 else 0.0
    manifest = {
        'seq_id': seq_id,
        'data_dir': str(data_dir),
        'n_frames': len(frames),
        'timestamps': [ts for ts, _ in frames],
        'rel_paths': [rel for _, rel in frames],
        'duration_s': duration,
        'video': str(out_video),
        'video_fps_encoded': 30.0,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2))
    return manifest_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seqs', default=','.join(SEQS.keys()))
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    seq_ids = args.seqs.split(',')
    for seq_id in seq_ids:
        if seq_id not in SEQS:
            print(f'unknown seq id: {seq_id}', file=sys.stderr)
            continue
        build_one(seq_id, args.force)


if __name__ == '__main__':
    main()
