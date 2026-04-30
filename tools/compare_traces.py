import argparse

from gt_trace_common import analyze_metrics_against_gt


FIELDS = [
    "translation_error_m",
    "method",
    "inliers",
    "tracked_count",
    "linked_points",
    "pnp_inliers",
    "e_inliers",
    "points_total",
    "points_added",
    "trans_jump",
]


def parse_run(text):
    if "=" not in text:
        raise argparse.ArgumentTypeError("Run must be formatted label=metrics.json")
    label, path = text.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError("Run must be formatted label=metrics.json")
    return label, path


def frame_map(per_frame):
    return {frame["frame_id"]: frame for frame in per_frame}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt", required=True, help="Ground-truth NPZ")
    parser.add_argument("--window", default="120:180", help="Inclusive START:END frame window")
    parser.add_argument("--run", action="append", required=True, type=parse_run,
                        help="Run to compare, formatted label=metrics.json. Provide at least two.")
    parser.add_argument("--sort_by_delta", action="store_true",
                        help="Sort by largest first-vs-second translation-error delta")
    args = parser.parse_args()

    if len(args.run) < 2:
        raise SystemExit("Provide at least two --run entries")
    start_text, end_text = args.window.split(":", 1)
    start, end = int(start_text), int(end_text)

    analyzed = []
    for label, path in args.run:
        result = analyze_metrics_against_gt(path, args.gt, label=label)
        analyzed.append((label, result["summary"], frame_map(result["per_frame"])))

    print("Summaries:")
    for label, summary, _frames in analyzed:
        print(
            f"{label:>14} ATE={summary['ate_rmse']:.4f} "
            f"median={summary['ate_median']:.4f} max={summary['ate_max']:.4f} "
            f"scale={summary['alignment_scale']:.3e}"
        )

    frame_ids = sorted(
        set.intersection(*(set(frames.keys()) for _label, _summary, frames in analyzed))
    )
    frame_ids = [frame_id for frame_id in frame_ids if start <= frame_id <= end]
    if args.sort_by_delta and len(analyzed) >= 2:
        first_frames = analyzed[0][2]
        second_frames = analyzed[1][2]
        frame_ids.sort(
            key=lambda frame_id: first_frames[frame_id]["translation_error_m"]
            - second_frames[frame_id]["translation_error_m"],
            reverse=True,
        )

    labels = [label for label, _summary, _frames in analyzed]
    print(f"\nFrame window {start}:{end}")
    header = ["frame"]
    for label in labels:
        header += [
            f"{label}:err",
            f"{label}:m",
            f"{label}:inl",
            f"{label}:trk",
            f"{label}:lnk",
            f"{label}:pnp",
            f"{label}:E",
            f"{label}:pts",
            f"{label}:add",
            f"{label}:jump",
        ]
    if len(analyzed) >= 2:
        header.append(f"d_err({labels[0]}-{labels[1]})")
    print(" ".join(f"{item:>12}" for item in header))

    for frame_id in frame_ids:
        row = [f"{frame_id:12d}"]
        for _label, _summary, frames in analyzed:
            frame = frames[frame_id]
            row += [
                f"{frame['translation_error_m']:12.4f}",
                f"{frame['method']:>12}",
                f"{frame['inliers']:12d}",
                f"{frame['tracked_count']:12d}",
                f"{frame['linked_points']:12d}",
                f"{frame['pnp_inliers']:12d}",
                f"{frame['e_inliers']:12d}",
                f"{frame['points_total']:12d}",
                f"{frame['points_added']:12d}",
                f"{frame['trans_jump']:12.1f}",
            ]
        if len(analyzed) >= 2:
            a = analyzed[0][2][frame_id]["translation_error_m"]
            b = analyzed[1][2][frame_id]["translation_error_m"]
            row.append(f"{a - b:12.4f}")
        print(" ".join(row))


if __name__ == "__main__":
    main()
