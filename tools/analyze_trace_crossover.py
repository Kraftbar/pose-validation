import argparse
import json
from collections import Counter

import numpy as np

from gt_trace_common import analyze_metrics_against_gt


def parse_run(text):
    if "=" not in text:
        raise argparse.ArgumentTypeError("Run must be formatted label=metrics.json")
    label, path = text.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError("Run must be formatted label=metrics.json")
    return label, path


def load_raw_metrics(path):
    with open(path, "r") as handle:
        return json.load(handle)


def infer_keyframe_reason(frame, metrics):
    if not frame["is_keyframe"]:
        return "none"
    if "kf_period" not in metrics:
        return "kf"
    frame_id = frame["frame_id"]
    if frame_id <= int(metrics.get("kf_warmup_frames", -1)):
        return "warmup"
    kf_period = int(metrics.get("kf_period", 0))
    if kf_period > 0 and frame_id % kf_period == 0:
        return "period"
    if frame["inliers"] < int(metrics.get("kf_min_inliers", 0)):
        return "low_inliers"
    return "rot_or_other"


def analyze_run(label, path, gt_npz):
    result = analyze_metrics_against_gt(path, gt_npz, label=label)
    raw_metrics = load_raw_metrics(path)
    raw_by_frame = {
        int(frame["frame_id"]): frame for frame in raw_metrics.get("timeline", [])
    }
    frames = {}
    for frame in result["per_frame"]:
        frame = dict(frame)
        raw = raw_by_frame.get(frame["frame_id"], {})
        if frame["method"] == "N/A" and "used_pnp" in raw:
            frame["method"] = "PnP" if raw["used_pnp"] else "nonPnP"
            frame["method_id"] = None
        frame["used_pnp"] = bool(raw.get("used_pnp", frame.get("method") == "PnP"))
        frame["keyframe_reason"] = infer_keyframe_reason(frame, raw_metrics)
        frames[frame["frame_id"]] = frame
    return {
        "label": label,
        "path": path,
        "summary": result["summary"],
        "frames": frames,
    }


def common_series(a_frames, b_frames):
    frame_ids = sorted(set(a_frames).intersection(b_frames))
    rows = []
    for frame_id in frame_ids:
        a_err = a_frames[frame_id]["translation_error_m"]
        b_err = b_frames[frame_id]["translation_error_m"]
        rows.append(
            {
                "frame_id": frame_id,
                "a": a_frames[frame_id],
                "b": b_frames[frame_id],
                "delta": a_err - b_err,
            }
        )
    return rows


def first_sustained(rows, persist, margin):
    if persist <= 1:
        for row in rows:
            if row["delta"] > margin:
                return row
        return None
    for start in range(0, len(rows) - persist + 1):
        window = rows[start:start + persist]
        if all(row["delta"] > margin for row in window):
            return rows[start]
    return None


def rolling_windows(rows, size):
    if size <= 0 or len(rows) < size:
        return []
    windows = []
    deltas = np.array([row["delta"] for row in rows], dtype=float)
    errors_a = np.array([row["a"]["translation_error_m"] for row in rows], dtype=float)
    errors_b = np.array([row["b"]["translation_error_m"] for row in rows], dtype=float)
    for start in range(0, len(rows) - size + 1):
        end = start + size
        win_rows = rows[start:end]
        windows.append(
            {
                "start": win_rows[0]["frame_id"],
                "end": win_rows[-1]["frame_id"],
                "mean_delta": float(deltas[start:end].mean()),
                "mean_a": float(errors_a[start:end].mean()),
                "mean_b": float(errors_b[start:end].mean()),
                "max_delta": float(deltas[start:end].max()),
                "min_delta": float(deltas[start:end].min()),
            }
        )
    return windows


def pct(value):
    return "n/a" if value is None else f"{value:.1f}"


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), q))


def summarize_frames(label, frames):
    if not frames:
        return f"{label}: no frames"

    errors = [frame["translation_error_m"] for frame in frames]
    inliers = [frame["inliers"] for frame in frames]
    jumps = [frame["trans_jump"] for frame in frames if frame["trans_jump"] > 0.0]
    linked = [frame["linked_points"] for frame in frames if frame["linked_points"] > 0]
    tracked = [frame["tracked_count"] for frame in frames if frame["tracked_count"] > 0]
    relinked = [frame["relinked_points"] for frame in frames if frame["relinked_points"] > 0]
    pnp_inliers = [frame["pnp_inliers"] for frame in frames if frame["pnp_inliers"] > 0]
    methods = Counter(frame["method"] for frame in frames)
    method_text = ", ".join(f"{name}:{count}" for name, count in sorted(methods.items()))
    keyframe_reasons = Counter(
        frame["keyframe_reason"] for frame in frames if frame["is_keyframe"]
    )
    keyframe_reason_text = ", ".join(
        f"{name}:{count}" for name, count in sorted(keyframe_reasons.items())
    )
    pnp_low = sum(1 for frame in frames
                  if frame["method"] == "PnP" and frame["inliers"] > 0 and frame["inliers"] < 20)
    pnp_frames = sum(1 for frame in frames if frame["method"] == "PnP")
    keyframes = sum(1 for frame in frames if frame["is_keyframe"])
    added = sum(frame["points_added"] for frame in frames)
    points_start = frames[0]["points_total"]
    points_end = frames[-1]["points_total"]

    return (
        f"{label}: frames={len(frames)} mean_err={np.mean(errors):.4f} "
        f"median_err={np.median(errors):.4f} max_err={np.max(errors):.4f} "
        f"methods=[{method_text}] keyframes={keyframes} "
        f"kf_reasons=[{keyframe_reason_text}] "
        f"points={points_start}->{points_end} added={added} "
        f"inliers_med={np.median(inliers):.1f} inliers_min={min(inliers)} "
        f"pnp_low={pnp_low}/{pnp_frames} "
        f"pnp_inl_med={pct(percentile(pnp_inliers, 50))} "
        f"linked_med={pct(percentile(linked, 50))} linked_min={min(linked) if linked else 'n/a'} "
        f"tracked_med={pct(percentile(tracked, 50))} "
        f"relinked_med={pct(percentile(relinked, 50))} "
        f"jump_med={pct(percentile(jumps, 50))} jump_max={pct(percentile(jumps, 100))}"
    )


def print_frame_table(rows, label_a, label_b, top_k, reverse):
    rows = sorted(rows, key=lambda row: row["delta"], reverse=reverse)[:top_k]
    title = "worst" if reverse else "best"
    print(f"\nTop {len(rows)} {title} delta frames ({label_a}-{label_b})")
    header = [
        "frame", f"{label_a}:err", f"{label_b}:err", "delta",
        f"{label_a}:m", f"{label_a}:inl", f"{label_a}:lnk",
        f"{label_a}:pts", f"{label_a}:add", f"{label_a}:jump",
        f"{label_b}:m", f"{label_b}:inl", f"{label_b}:pts", f"{label_b}:add",
    ]
    print(" ".join(f"{item:>12}" for item in header))
    for row in rows:
        a = row["a"]
        b = row["b"]
        values = [
            f"{row['frame_id']:12d}",
            f"{a['translation_error_m']:12.4f}",
            f"{b['translation_error_m']:12.4f}",
            f"{row['delta']:12.4f}",
            f"{a['method']:>12}",
            f"{a['inliers']:12d}",
            f"{a['linked_points']:12d}",
            f"{a['points_total']:12d}",
            f"{a['points_added']:12d}",
            f"{a['trans_jump']:12.1f}",
            f"{b['method']:>12}",
            f"{b['inliers']:12d}",
            f"{b['points_total']:12d}",
            f"{b['points_added']:12d}",
        ]
        print(" ".join(values))


def parse_range(text):
    start_text, end_text = text.split(":", 1)
    return int(start_text), int(end_text)


def rows_in_range(rows, start, end):
    return [row for row in rows if start <= row["frame_id"] <= end]


def main():
    parser = argparse.ArgumentParser(
        description="Compare two GT-aligned traces and report where their frame-error delta crosses over."
    )
    parser.add_argument("--gt", required=True, help="Ground-truth NPZ")
    parser.add_argument("--a", required=True, type=parse_run,
                        help="First run, formatted label=metrics.json")
    parser.add_argument("--b", required=True, type=parse_run,
                        help="Second run, formatted label=metrics.json")
    parser.add_argument("--window", type=int, default=30,
                        help="Rolling window size for mean deltas")
    parser.add_argument("--persist", type=int, default=12,
                        help="Consecutive positive-delta frames required for crossover")
    parser.add_argument("--margin", type=float, default=0.0,
                        help="Positive delta margin for sustained crossover")
    parser.add_argument("--start_frame", type=int,
                        help="Ignore frames before this frame")
    parser.add_argument("--end_frame", type=int,
                        help="Ignore frames after this frame")
    parser.add_argument("--context", type=int, default=15,
                        help="Frames before/after crossover to summarize")
    parser.add_argument("--range", dest="manual_range",
                        help="Optional START:END range to summarize instead of auto crossover context")
    parser.add_argument("--top_k", type=int, default=12,
                        help="Number of best/worst individual frames to print")
    args = parser.parse_args()

    label_a, path_a = args.a
    label_b, path_b = args.b
    run_a = analyze_run(label_a, path_a, args.gt)
    run_b = analyze_run(label_b, path_b, args.gt)
    rows = common_series(run_a["frames"], run_b["frames"])
    if args.start_frame is not None:
        rows = [row for row in rows if row["frame_id"] >= args.start_frame]
    if args.end_frame is not None:
        rows = [row for row in rows if row["frame_id"] <= args.end_frame]
    if not rows:
        raise SystemExit("No common frames")

    print("Summaries:")
    for run in [run_a, run_b]:
        summary = run["summary"]
        print(
            f"{run['label']:>14} ATE={summary['ate_rmse']:.4f} "
            f"median={summary['ate_median']:.4f} max={summary['ate_max']:.4f} "
            f"scale={summary['alignment_scale']:.3e}"
        )
    print(f"\nDelta is {label_a} - {label_b}; positive means {label_a} is worse.")
    print(f"Common frames: {len(rows)} ({rows[0]['frame_id']}:{rows[-1]['frame_id']})")

    sustained = first_sustained(rows, args.persist, args.margin)
    if sustained:
        print(
            f"First sustained positive delta: frame {sustained['frame_id']} "
            f"for {args.persist} frames, margin>{args.margin:.4f}, "
            f"delta={sustained['delta']:.4f}"
        )
    else:
        print(
            f"No sustained positive delta found for {args.persist} frames "
            f"with margin>{args.margin:.4f}"
        )

    windows = rolling_windows(rows, args.window)
    if windows:
        print(f"\nWorst rolling windows, size={args.window}")
        for window in sorted(windows, key=lambda item: item["mean_delta"], reverse=True)[:5]:
            print(
                f"{window['start']:4d}:{window['end']:<4d} "
                f"mean_delta={window['mean_delta']:.4f} "
                f"{label_a}_mean={window['mean_a']:.4f} {label_b}_mean={window['mean_b']:.4f} "
                f"min/max_delta={window['min_delta']:.4f}/{window['max_delta']:.4f}"
            )
        print(f"\nBest rolling windows, size={args.window}")
        for window in sorted(windows, key=lambda item: item["mean_delta"])[:5]:
            print(
                f"{window['start']:4d}:{window['end']:<4d} "
                f"mean_delta={window['mean_delta']:.4f} "
                f"{label_a}_mean={window['mean_a']:.4f} {label_b}_mean={window['mean_b']:.4f} "
                f"min/max_delta={window['min_delta']:.4f}/{window['max_delta']:.4f}"
            )

    if args.manual_range:
        start, end = parse_range(args.manual_range)
    elif sustained:
        start = sustained["frame_id"] - args.context
        end = sustained["frame_id"] + args.context
    else:
        worst = max(windows or [], key=lambda item: item["mean_delta"], default=None)
        start = worst["start"] if worst else rows[0]["frame_id"]
        end = worst["end"] if worst else rows[-1]["frame_id"]
    start = max(start, rows[0]["frame_id"])
    end = min(end, rows[-1]["frame_id"])
    context_rows = rows_in_range(rows, start, end)
    print(f"\nContext summary {start}:{end}")
    print(summarize_frames(label_a, [row["a"] for row in context_rows]))
    print(summarize_frames(label_b, [row["b"] for row in context_rows]))

    print_frame_table(rows, label_a, label_b, args.top_k, reverse=True)
    print_frame_table(rows, label_a, label_b, args.top_k, reverse=False)


if __name__ == "__main__":
    main()
