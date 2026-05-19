import argparse
import json
import statistics
from pathlib import Path

from gt_trace_common import analyze_metrics_against_gt


DEFAULT_CASES = [
    (
        "room",
        "runs/benchmark/test_freiburgroom525_pure_c_plus.json",
        "external/twitchslam/videos/test_freiburgroom525.npz",
        2.5,
    ),
    (
        "desk",
        "runs/benchmark/test_freiburgdesk525_pure_c_plus.json",
        "external/twitchslam/videos/test_freiburgdesk525.npz",
        1.0,
    ),
    (
        "rpy",
        "runs/benchmark/test_freiburgrpy525_pure_c_plus.json",
        "external/twitchslam/videos/test_freiburgrpy525.npz",
        0.15,
    ),
    (
        "xyz",
        "runs/benchmark/test_freiburgxyz525_pure_c_plus.json",
        "external/twitchslam/videos/test_freiburgxyz525.npz",
        0.25,
    ),
]


def parse_case(text):
    if "=" not in text:
        raise argparse.ArgumentTypeError(
            "Case must be label=metrics_json,gt_npz,bad_error_threshold"
        )
    label, rest = text.split("=", 1)
    parts = rest.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "Case must be label=metrics_json,gt_npz,bad_error_threshold"
        )
    metrics_json, gt_npz, threshold_text = parts
    return label, metrics_json, gt_npz, float(threshold_text)


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


def percentile(values, q):
    if not values:
        return 0.0
    values = sorted(values)
    if len(values) == 1:
        return float(values[0])
    pos = (len(values) - 1) * q / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(values) - 1)
    frac = pos - lo
    return float(values[lo] * (1.0 - frac) + values[hi] * frac)


def load_case(label, metrics_json, gt_npz, bad_threshold):
    raw_metrics = load_raw_metrics(metrics_json)
    result = analyze_metrics_against_gt(metrics_json, gt_npz, label=label)
    frames = []
    for frame in result["per_frame"]:
        frame = dict(frame)
        frame["keyframe_reason"] = infer_keyframe_reason(frame, raw_metrics)
        frames.append(frame)
    return {
        "label": label,
        "metrics_json": metrics_json,
        "gt_npz": gt_npz,
        "bad_threshold": bad_threshold,
        "frames": frames,
        "summary": result["summary"],
    }


def summarize_window(case, rows):
    errors = [row["translation_error_m"] for row in rows]
    inliers = [row["inliers"] for row in rows]
    linked = [row["linked_points"] for row in rows if row["linked_points"] > 0]
    tracked = [row["tracked_count"] for row in rows if row["tracked_count"] > 0]
    jumps = [row["trans_jump"] for row in rows if row["trans_jump"] > 0.0]
    keyframes = [row for row in rows if row["is_keyframe"]]
    pnp_rows = [row for row in rows if row["method"] == "PnP"]
    e_rows = [row for row in rows if row["method"] == "E"]
    bad_threshold = case["bad_threshold"]
    return {
        "case": case["label"],
        "start": rows[0]["frame_id"],
        "end": rows[-1]["frame_id"],
        "frames": len(rows),
        "mean_err": statistics.fmean(errors),
        "median_err": statistics.median(errors),
        "max_err": max(errors),
        "bad_frames": sum(1 for value in errors if value > bad_threshold),
        "bad_window": statistics.fmean(errors) > bad_threshold,
        "pnp": len(pnp_rows),
        "e": len(e_rows),
        "pnp_low16": sum(1 for row in pnp_rows if row["inliers"] <= 16),
        "pnp_low20": sum(1 for row in pnp_rows if row["inliers"] <= 20),
        "pnp_jump500k": sum(1 for row in pnp_rows if row["trans_jump"] > 500000.0),
        "jump500k": sum(1 for row in rows if row["trans_jump"] > 500000.0),
        "jump1m": sum(1 for row in rows if row["trans_jump"] > 1000000.0),
        "max_jump": max(jumps, default=0.0),
        "keyframes": len(keyframes),
        "kf_low": sum(1 for row in keyframes if row["keyframe_reason"] == "low_inliers"),
        "kf_period": sum(1 for row in keyframes if row["keyframe_reason"] == "period"),
        "kf_rot": sum(1 for row in keyframes if row["keyframe_reason"] == "rot_or_other"),
        "points_added": sum(row["points_added"] for row in rows),
        "points_start": rows[0]["points_total"],
        "points_end": rows[-1]["points_total"],
        "inliers_med": statistics.median(inliers),
        "inliers_min": min(inliers),
        "linked_med": statistics.median(linked) if linked else 0.0,
        "linked_p10": percentile(linked, 10),
        "linked_min": min(linked, default=0),
        "tracked_med": statistics.median(tracked) if tracked else 0.0,
    }


def build_windows(case, size, step):
    frames = case["frames"]
    out = []
    for start in range(0, len(frames) - size + 1, step):
        out.append(summarize_window(case, frames[start:start + size]))
    return out


def window_rules():
    return [
        (
            "pnp_low>=2 & jump1m>=1",
            lambda w: w["pnp_low16"] >= 2 and w["jump1m"] >= 1,
        ),
        (
            "pnp_jump>=2 & pnp_low>=2",
            lambda w: w["pnp_jump500k"] >= 2 and w["pnp_low16"] >= 2,
        ),
        (
            "kf_low>=8 & added>=600",
            lambda w: w["kf_low"] >= 8 and w["points_added"] >= 600,
        ),
        (
            "kf_low>=8 & jump500k>=6",
            lambda w: w["kf_low"] >= 8 and w["jump500k"] >= 6,
        ),
        (
            "added>=1000 & jump500k>=4",
            lambda w: w["points_added"] >= 1000 and w["jump500k"] >= 4,
        ),
        (
            "linked_med<250 & jump500k>=4",
            lambda w: w["linked_med"] < 250.0 and w["jump500k"] >= 4,
        ),
        (
            "linked_p10<100 & jump500k>=4",
            lambda w: w["linked_p10"] < 100.0 and w["jump500k"] >= 4,
        ),
        (
            "pnp_low>=3 & kf_low>=8",
            lambda w: w["pnp_low16"] >= 3 and w["kf_low"] >= 8,
        ),
        (
            "jump1m>=3 & added>=600",
            lambda w: w["jump1m"] >= 3 and w["points_added"] >= 600,
        ),
    ]


def print_windows(title, windows, top_k):
    print(f"\n{title}")
    print(
        f"{'case':>5} {'range':>9} {'mean':>7} {'max':>7} {'bad':>4} "
        f"{'pnp':>4} {'pL16':>5} {'E':>4} {'KF':>4} {'kfLow':>5} "
        f"{'add':>5} {'j500':>5} {'j1m':>4} {'lnkMed':>7} {'lnkP10':>7}"
    )
    for w in windows[:top_k]:
        print(
            f"{w['case']:>5} {w['start']:4d}:{w['end']:<4d} "
            f"{w['mean_err']:7.3f} {w['max_err']:7.3f} {w['bad_frames']:4d} "
            f"{w['pnp']:4d} {w['pnp_low16']:5d} {w['e']:4d} "
            f"{w['keyframes']:4d} {w['kf_low']:5d} {w['points_added']:5d} "
            f"{w['jump500k']:5d} {w['jump1m']:4d} "
            f"{w['linked_med']:7.1f} {w['linked_p10']:7.1f}"
        )


def print_rule_sweep(windows):
    print("\nWindow rule probes")
    print(
        f"{'rule':32s} {'case':>5} {'flag':>5} {'badW':>5} {'badF':>6} "
        f"{'mean':>7} {'max':>7} ranges"
    )
    labels = sorted({w["case"] for w in windows})
    for name, predicate in window_rules():
        for label in labels:
            case_windows = [w for w in windows if w["case"] == label]
            flagged = [w for w in case_windows if predicate(w)]
            bad_windows = [w for w in flagged if w["bad_window"]]
            bad_frames = sum(w["bad_frames"] for w in flagged)
            mean_err = (
                statistics.fmean(w["mean_err"] for w in flagged) if flagged else 0.0
            )
            max_err = max((w["max_err"] for w in flagged), default=0.0)
            ranges = ",".join(f"{w['start']}:{w['end']}" for w in flagged[:6])
            print(
                f"{name:32s} {label:>5} {len(flagged):5d} "
                f"{len(bad_windows):5d} {bad_frames:6d} "
                f"{mean_err:7.3f} {max_err:7.3f} {ranges}"
            )
        print()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze rolling trace-health windows from saved pure_c_plus metrics."
        )
    )
    parser.add_argument(
        "--case",
        action="append",
        type=parse_case,
        help="Case as label=metrics_json,gt_npz,bad_error_threshold",
    )
    parser.add_argument("--window", type=int, default=30)
    parser.add_argument("--step", type=int, default=1)
    parser.add_argument("--top_k", type=int, default=12)
    args = parser.parse_args()

    if args.window < 1:
        raise SystemExit("--window must be >= 1")
    if args.step < 1:
        raise SystemExit("--step must be >= 1")

    cases = [
        load_case(label, metrics_json, gt_npz, threshold)
        for label, metrics_json, gt_npz, threshold in (args.case or DEFAULT_CASES)
    ]
    all_windows = []
    for case in cases:
        windows = build_windows(case, args.window, args.step)
        all_windows.extend(windows)
        print(
            f"{case['label']}: ATE={case['summary']['ate_rmse']:.4f} "
            f"threshold={case['bad_threshold']:.3f} windows={len(windows)}"
        )
        print_windows(
            f"Top {args.top_k} {case['label']} windows by mean error",
            sorted(windows, key=lambda w: w["mean_err"], reverse=True),
            args.top_k,
        )
    print_rule_sweep(all_windows)


if __name__ == "__main__":
    main()
