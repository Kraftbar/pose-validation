import argparse
import numpy as np

from gt_trace_common import analyze_metrics_against_gt

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to metrics JSON from SLAM")
    parser.add_argument("gt_npz", help="Path to ground truth NPZ")
    parser.add_argument("--top_k", type=int, default=10, help="Number of worst frames to show")
    parser.add_argument("--spikes", action="store_true", help="Also show largest frame-to-frame error jumps")
    parser.add_argument("--by_method", action="store_true", help="Also summarize error grouped by pose method")
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
    print(
        f"{'Frame':>6} {'Error(m)':>10} {'RotErr(deg)':>12} {'Inliers':>8} {'Method':>8} "
        f"{'Tracked':>8} {'Linked':>8} {'Relink':>7} {'Added':>7} {'KF':>4}"
    )
    for idx in worst_idx:
        frame = per_frame[idx]
        rot_error = 0.0 if frame["rotation_error_deg"] is None else frame["rotation_error_deg"]
        print(
            f"{frame['frame_id']:6d} {frame['translation_error_m']:10.4f} {rot_error:12.4f} "
            f"{frame['inliers']:8d} {frame['method']:8s} {frame['tracked_count']:8d} "
            f"{frame['linked_points']:8d} {frame['relinked_points']:7d} {frame['points_added']:7d} "
            f"{'YES' if frame['is_keyframe'] else 'no':>4}"
        )

    if args.spikes:
        jumps = []
        prev = None
        for frame in per_frame:
            if prev is not None:
                transition = "" if prev["method"] == frame["method"] else f"{prev['method']}->{frame['method']}"
                jumps.append((frame["translation_error_m"] - prev["translation_error_m"], transition, frame))
            prev = frame
        print(f"\nTop {args.top_k} positive frame-to-frame error jumps:")
        print(
            f"{'Frame':>6} {'Delta(m)':>10} {'Error(m)':>10} {'Inliers':>8} {'Method':>8} "
            f"{'Tracked':>8} {'Linked':>8} {'Relink':>7} {'Added':>7} {'Transition':>12}"
        )
        for delta, transition, frame in sorted(jumps, key=lambda item: item[0], reverse=True)[:args.top_k]:
            print(
                f"{frame['frame_id']:6d} {delta:10.4f} {frame['translation_error_m']:10.4f} "
                f"{frame['inliers']:8d} {frame['method']:8s} {frame['tracked_count']:8d} "
                f"{frame['linked_points']:8d} {frame['relinked_points']:7d} "
                f"{frame['points_added']:7d} {transition:>12}"
            )

    if args.by_method:
        groups = {}
        for frame in per_frame:
            groups.setdefault(frame["method"], []).append(frame)
        print("\nError by method:")
        print(
            f"{'Method':>8} {'Frames':>8} {'Mean(m)':>10} {'Median(m)':>10} {'Max(m)':>10} "
            f"{'TrkMean':>8} {'LnkMean':>8} {'RelMean':>8} {'AddMean':>8}"
        )
        for method in sorted(groups):
            values = np.asarray([frame["translation_error_m"] for frame in groups[method]], dtype=float)
            tracked = np.asarray([frame["tracked_count"] for frame in groups[method]], dtype=float)
            linked = np.asarray([frame["linked_points"] for frame in groups[method]], dtype=float)
            relinked = np.asarray([frame["relinked_points"] for frame in groups[method]], dtype=float)
            added = np.asarray([frame["points_added"] for frame in groups[method]], dtype=float)
            print(
                f"{method:>8} {values.size:8d} {values.mean():10.4f} "
                f"{np.median(values):10.4f} {values.max():10.4f} "
                f"{tracked.mean():8.1f} {linked.mean():8.1f} {relinked.mean():8.1f} {added.mean():8.1f}"
            )

if __name__ == "__main__":
    main()
