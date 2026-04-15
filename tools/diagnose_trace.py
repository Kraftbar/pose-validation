import argparse
import numpy as np

from gt_trace_common import analyze_metrics_against_gt

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to metrics JSON from SLAM")
    parser.add_argument("gt_npz", help="Path to ground truth NPZ")
    parser.add_argument("--top_k", type=int, default=10, help="Number of worst frames to show")
    args = parser.parse_args()

    result = analyze_metrics_against_gt(args.metrics_json, args.gt_npz)
    summary = result["summary"]
    per_frame = result["per_frame"]
    print(f"R_align:\n{result['alignment']['R']}")

    # Report
    print(f"Alignment: scale={summary['alignment_scale']:.4f}")
    print(f"ATE RMSE: {summary['ate_rmse']:.4f} m")
    print(f"ATE Median: {summary['ate_median']:.4f} m")
    has_rotation = any(frame["rotation_error_deg"] is not None for frame in per_frame)
    if has_rotation:
        print(f"Rotation Error Mean: {summary['rotation_mean']:.4f} deg")
        print(f"Rotation Error Median: {summary['rotation_median']:.4f} deg")

    # Worst frames
    print(f"\nTop {args.top_k} worst frames by translation error:")
    worst_idx = np.argsort([frame["translation_error_m"] for frame in per_frame])[::-1][:args.top_k]
    print(f"{'Frame':>6} {'Error(m)':>10} {'RotErr(deg)':>12} {'Inliers':>8} {'Method':>8} {'KF':>4}")
    for idx in worst_idx:
        frame = per_frame[idx]
        rot_error = 0.0 if frame["rotation_error_deg"] is None else frame["rotation_error_deg"]
        print(
            f"{frame['frame_id']:6d} {frame['translation_error_m']:10.4f} {rot_error:12.4f} "
            f"{frame['inliers']:8d} {frame['method']:8s} {'YES' if frame['is_keyframe'] else 'no':>4}"
        )

if __name__ == "__main__":
    main()
