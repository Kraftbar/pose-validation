import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

from gt_trace_common import METHOD_NAMES, analyze_metrics_against_gt


def parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def parse_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def percentile(values, pct):
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct))
    return ordered[idx]


def fmt(value, digits=3):
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def load_detail_rows(path, frame_errors):
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            frame_id = parse_int(raw.get("frame_id"), -1)
            method = parse_int(raw.get("method"), -1)
            row = {
                "frame_id": frame_id,
                "source": raw.get("source", ""),
                "decision": raw.get("decision", ""),
                "method_id": method,
                "method": METHOD_NAMES.get(method, str(method)),
                "inliers": parse_int(raw.get("inliers")),
                "match_idx": parse_int(raw.get("match_idx"), -1),
                "query_idx": parse_int(raw.get("query_idx"), -1),
                "train_idx": parse_int(raw.get("train_idx"), -1),
                "cell": parse_int(raw.get("cell"), -1),
                "baseline": parse_float(raw.get("baseline")),
                "reproj": parse_float(raw.get("reproj")),
                "parallax": parse_float(raw.get("parallax")),
                "depth": parse_float(raw.get("depth")),
                "z1": parse_float(raw.get("z1")),
                "z2": parse_float(raw.get("z2")),
                "fb_err": parse_float(raw.get("fb_err")),
                "track_disp": parse_float(raw.get("track_disp")),
                "score": parse_float(raw.get("score")),
                "frame_error": frame_errors.get(frame_id, math.nan),
            }
            rows.append(row)
    return rows


def finite_values(rows, key):
    return [row[key] for row in rows if math.isfinite(row[key])]


def mean(values):
    return sum(values) / len(values) if values else math.nan


def print_group_summary(title, rows):
    if not rows:
        return
    depths = finite_values(rows, "depth")
    reproj = finite_values(rows, "reproj")
    parallax = finite_values(rows, "parallax")
    errors = finite_values(rows, "frame_error")
    frames = {row["frame_id"] for row in rows}
    print(
        f"{title:18s} n={len(rows):6d} frames={len(frames):4d} "
        f"mean_err={fmt(mean(errors), 2):>5s} "
        f"med_depth={fmt(statistics.median(depths) if depths else math.nan, 1):>10s} "
        f"p90_depth={fmt(percentile(depths, 0.90), 1):>10s} "
        f"med_reproj={fmt(statistics.median(reproj) if reproj else math.nan, 1):>8s} "
        f"med_par={fmt(statistics.median(parallax) if parallax else math.nan, 1):>7s}"
    )


def print_decisions(rows):
    decisions = Counter(row["decision"] for row in rows)
    print("Decisions:")
    for decision, count in decisions.most_common():
        frames = {row["frame_id"] for row in rows if row["decision"] == decision}
        print(f"  {decision:16s} rows={count:6d} frames={len(frames):4d}")


def print_accepted_breakdown(rows, bad_error_threshold):
    accepted = [row for row in rows if row["decision"] == "accept"]
    print("\nAccepted rows by method:")
    for method_id in sorted({row["method_id"] for row in accepted}):
        method_rows = [row for row in accepted if row["method_id"] == method_id]
        print_group_summary(METHOD_NAMES.get(method_id, str(method_id)), method_rows)
        print_group_summary("  err<1", [row for row in method_rows if row["frame_error"] < 1.0])
        print_group_summary(
            f"  err>={bad_error_threshold:g}",
            [row for row in method_rows if row["frame_error"] >= bad_error_threshold],
        )

    print("\nAccepted rows by source:")
    for source in sorted({row["source"] for row in accepted}):
        print_group_summary(source or "(none)", [row for row in accepted if row["source"] == source])


def print_depth_probes(rows, thresholds, bad_error_threshold):
    accepted = [
        row
        for row in rows
        if row["decision"] == "accept" and math.isfinite(row["depth"])
    ]
    print("\nDepth probes over accepted rows:")
    print(
        f"{'threshold':>10s} {'method':>7s} {'flagged':>7s} {'frames':>6s} "
        f"{'bad_rows':>8s} {'mean_err':>8s} {'med_depth':>10s}"
    )
    for threshold in thresholds:
        for method_id in sorted({row["method_id"] for row in accepted}):
            flagged = [
                row
                for row in accepted
                if row["method_id"] == method_id and row["depth"] > threshold
            ]
            bad_rows = [
                row for row in flagged if row["frame_error"] >= bad_error_threshold
            ]
            depths = finite_values(flagged, "depth")
            errors = finite_values(flagged, "frame_error")
            method = METHOD_NAMES.get(method_id, str(method_id))
            print(
                f"{threshold:10.0f} {method:>7s} {len(flagged):7d} "
                f"{len({row['frame_id'] for row in flagged}):6d} {len(bad_rows):8d} "
                f"{fmt(mean(errors), 2):>8s} "
                f"{fmt(statistics.median(depths) if depths else math.nan, 1):>10s}"
            )


def print_worst_frames(rows, limit):
    accepted = [row for row in rows if row["decision"] == "accept"]
    by_frame = defaultdict(list)
    for row in accepted:
        by_frame[row["frame_id"]].append(row)
    frame_rows = []
    for frame_id, grouped in by_frame.items():
        depths = finite_values(grouped, "depth")
        reproj = finite_values(grouped, "reproj")
        methods = Counter(row["method"] for row in grouped)
        frame_rows.append(
            {
                "frame_id": frame_id,
                "frame_error": grouped[0]["frame_error"],
                "rows": len(grouped),
                "methods": methods,
                "med_depth": statistics.median(depths) if depths else math.nan,
                "p90_depth": percentile(depths, 0.90),
                "med_reproj": statistics.median(reproj) if reproj else math.nan,
            }
        )
    frame_rows.sort(key=lambda row: row["frame_error"], reverse=True)
    print("\nWorst accepted-admission frames:")
    print(
        f"{'frame':>6s} {'err':>7s} {'rows':>5s} {'methods':>14s} "
        f"{'med_depth':>10s} {'p90_depth':>10s} {'med_reproj':>10s}"
    )
    for row in frame_rows[:limit]:
        method_bits = ",".join(f"{name}:{count}" for name, count in row["methods"].most_common())
        print(
            f"{row['frame_id']:6d} {fmt(row['frame_error'], 3):>7s} "
            f"{row['rows']:5d} {method_bits:>14s} "
            f"{fmt(row['med_depth'], 1):>10s} {fmt(row['p90_depth'], 1):>10s} "
            f"{fmt(row['med_reproj'], 1):>10s}"
        )


def parse_thresholds(raw):
    thresholds = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        thresholds.append(float(part))
    return thresholds


def parse_window(raw):
    start_text, end_text = raw.split(":", 1)
    return int(start_text), int(end_text)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to pure_c_plus metrics JSON")
    parser.add_argument("gt_npz", help="Ground-truth NPZ")
    parser.add_argument("detail_csv", help="Path to --map_admission_detail_dump CSV")
    parser.add_argument("--top_k", type=int, default=12)
    parser.add_argument("--bad_error_threshold", type=float, default=2.0)
    parser.add_argument(
        "--depth_thresholds",
        default="100000,500000,1000000,2000000,5000000",
        help="Comma-separated depth thresholds for accepted-row probes",
    )
    parser.add_argument("--window", help="Optional inclusive START:END frame filter")
    args = parser.parse_args()

    result = analyze_metrics_against_gt(args.metrics_json, args.gt_npz)
    frame_errors = {
        row["frame_id"]: row["translation_error_m"] for row in result["per_frame"]
    }
    rows = load_detail_rows(Path(args.detail_csv), frame_errors)
    if args.window:
        start, end = parse_window(args.window)
        rows = [row for row in rows if start <= row["frame_id"] <= end]
    thresholds = parse_thresholds(args.depth_thresholds)

    summary = result["summary"]
    print(
        f"ATE={summary['ate_rmse']:.4f} median={summary['ate_median']:.4f} "
        f"frames={summary['matched_frames']} detail_rows={len(rows)}"
    )
    print_decisions(rows)
    print_accepted_breakdown(rows, args.bad_error_threshold)
    print_depth_probes(rows, thresholds, args.bad_error_threshold)
    print_worst_frames(rows, args.top_k)


if __name__ == "__main__":
    main()
