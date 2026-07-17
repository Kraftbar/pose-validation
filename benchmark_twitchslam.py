import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from benchmark import ate_rmse, load_gt_centers


def infer_focal(video_path: Path, default: float | None = None) -> float:
    if default is not None:
        return float(default)
    name = video_path.stem.lower()
    if 'freiburg' in name:
        return 525.0
    if 'kitti' in name:
        return 984.0
    return 525.0


def pose_to_center(pose: np.ndarray) -> np.ndarray:
    rot = pose[:3, :3]
    trans = pose[:3, 3]
    return (-rot.T @ trans).astype(np.float64)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--video_path', required=True)
    parser.add_argument('--gt_npz', default=None)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--focal', type=float, default=None)
    parser.add_argument('--out_json', default=None)
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    video_path = Path(args.video_path)
    if not video_path.is_absolute():
        video_path = root / video_path
    if not video_path.exists():
        raise SystemExit(f'Video not found: {video_path}')

    gt_npz = Path(args.gt_npz) if args.gt_npz else video_path.with_suffix('.npz')
    if not gt_npz.is_absolute():
        gt_npz = root / gt_npz

    twitch_root = root / 'external' / 'twitchslam'
    sys.path.insert(0, str(twitch_root))
    os.environ['HEADLESS'] = '1'

    from slam import SLAM  # type: ignore

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f'Failed to open video: {video_path}')

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    max_frames = min(total_frames, int(args.seconds * fps))
    focal = infer_focal(video_path, args.focal)

    if width > 1024:
        downscale = 1024.0 / width
        focal *= downscale
        height = int(height * downscale)
        width = 1024

    intrinsics = np.array([[focal, 0, width // 2], [0, focal, height // 2], [0, 0, 1]], dtype=np.float64)
    slam = SLAM(width, height, intrinsics)

    timeline: list[dict] = []
    start = time.time()
    frame_id = 0
    while frame_id < max_frames and cap.isOpened():
        if time.time() - start > args.timeout:
            break
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height))
        try:
            slam.process_frame(frame)
        except AssertionError:
            break
        except Exception as exc:
            print(f'twitchslam failed on frame {frame_id}: {exc}', file=sys.stderr)
            break







        
        
        current = slam.mapp.frames[-1]
        center = pose_to_center(current.pose)
        timeline.append({
            'frame_id': frame_id,
            'xyz': center.tolist(),
        })
        frame_id += 1

    cap.release()

    metrics = {
        'video': video_path.stem,
        'video_path': str(video_path.relative_to(root)),
        'frames': len(timeline),
        'points': len(slam.mapp.points),
        'duration_sec': round(time.time() - start, 3),
        'focal': focal,
        'optimizer': 'noop',
        'timeline': timeline,
    }

    if gt_npz.exists() and len(timeline) >= 3:
        gt_all = load_gt_centers(str(gt_npz))
        frame_ids = [item['frame_id'] for item in timeline]
        valid = [i for i, fid in enumerate(frame_ids) if fid < len(gt_all)]
        if len(valid) >= 3:
            est = np.array([timeline[i]['xyz'] for i in valid], dtype=np.float64)
            gt = gt_all[[frame_ids[i] for i in valid]]
            ate = ate_rmse(est, gt)
            metrics.update({
                'ate_rmse': round(ate['ate_rmse'], 4),
                'ate_median': round(ate['ate_median'], 4),
                'ate_max': round(ate['ate_max'], 4),
                'ate_scale': float(ate['scale']),
                'ate_n': ate['n_frames'],
            })

    out_json = Path(args.out_json) if args.out_json else root / 'runs' / 'benchmark_twitchslam' / f'{video_path.stem}.json'
    if not out_json.is_absolute():
        out_json = root / out_json
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(metrics, indent=2))

    print(f"Processed frames={metrics['frames']}, points={metrics['points']}, duration={metrics['duration_sec']}s")
    if 'ate_rmse' in metrics:
        print(
            f"ATE rmse={metrics['ate_rmse']} median={metrics['ate_median']} "
            f"max={metrics['ate_max']} n={metrics['ate_n']}"
        )
    print(f'Wrote metrics to {out_json}')


if __name__ == '__main__':
    main()
