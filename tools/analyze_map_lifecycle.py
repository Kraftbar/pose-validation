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


def finite_values(rows, key):
    return [row[key] for row in rows if math.isfinite(row[key])]


def mean(values):
    return sum(values) / len(values) if values else math.nan


def load_lifecycle_rows(path, frame_errors):
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            method = parse_int(raw.get("method"), -1)
            birth_frame = parse_int(raw.get("birth_frame"), -1)
            final_obs = parse_int(raw.get("final_obs"))
            row = {
                "map_idx": parse_int(raw.get("map_idx"), -1),
                "birth_frame": birth_frame,
                "last_seen_frame": parse_int(raw.get("last_seen_frame"), birth_frame),
                "span_frames": parse_int(raw.get("span_frames")),
                "frames_since_seen": parse_int(raw.get("frames_since_seen")),
                "source": raw.get("source", ""),
                "method_id": method,
                "method": METHOD_NAMES.get(method, str(method)),
                "inliers": parse_int(raw.get("inliers")),
                "cell": parse_int(raw.get("cell"), -1),
                "birth_reproj": parse_float(raw.get("birth_reproj")),
                "birth_parallax": parse_float(raw.get("birth_parallax")),
                "birth_depth": parse_float(raw.get("birth_depth")),
                "birth_fb_err": parse_float(raw.get("birth_fb_err")),
                "birth_track_disp": parse_float(raw.get("birth_track_disp")),
                "birth_score": parse_float(raw.get("birth_score")),
                "final_obs": final_obs,
                "good_obs": parse_int(raw.get("good_obs")),
                "bad_obs": parse_int(raw.get("bad_obs")),
                "alive": parse_int(raw.get("alive")) != 0 and final_obs > 0,
                "birth_error": frame_errors.get(birth_frame, math.nan),
            }
            rows.append(row)
    return rows


def print_group_summary(title, rows, bad_error_threshold):
    if not rows:
        return
    alive = [row for row in rows if row["alive"]]
    final_obs = [row["final_obs"] for row in rows]
    spans = [row["span_frames"] for row in rows]
    stale = [row for row in rows if row["frames_since_seen"] >= 100]
    depths = finite_values(rows, "birth_depth")
    reproj = finite_values(rows, "birth_reproj")
    errors = finite_values(rows, "birth_error")
    bad_rows = [row for row in rows if row["birth_error"] >= bad_error_threshold]
    frames = {row["birth_frame"] for row in rows}
    print(
        f"{title:20s} n={len(rows):6d} frames={len(frames):4d} "
        f"alive={100.0 * len(alive) / len(rows):5.1f}% "
        f"med_obs={fmt(statistics.median(final_obs), 1):>6s} "
        f"med_span={fmt(statistics.median(spans), 1):>6s} stale100={len(stale):6d} "
        f"mean_err={fmt(mean(errors), 2):>5s} bad_rows={len(bad_rows):6d} "
        f"med_depth={fmt(statistics.median(depths) if depths else math.nan, 1):>10s} "
        f"p90_depth={fmt(percentile(depths, 0.90), 1):>10s} "
        f"med_reproj={fmt(statistics.median(reproj) if reproj else math.nan, 1):>8s}"
    )


def print_breakdowns(rows, bad_error_threshold):
    print("Births by source:")
    for source in sorted({row["source"] for row in rows}):
        print_group_summary(source or "(none)", [row for row in rows if row["source"] == source],
                            bad_error_threshold)

    print("\nBirths by method:")
    for method_id in sorted({row["method_id"] for row in rows}):
        print_group_summary(METHOD_NAMES.get(method_id, str(method_id)),
                            [row for row in rows if row["method_id"] == method_id],
                            bad_error_threshold)


def print_survival_probes(rows, thresholds, bad_error_threshold):
    print("\nDepth probes over map births:")
    print(
        f"{'threshold':>10s} {'source':>18s} {'rows':>7s} {'alive%':>7s} "
        f"{'bad_rows':>8s} {'mean_err':>8s} {'med_obs':>7s} {'med_span':>8s}"
    )
    for threshold in thresholds:
        for source in sorted({row["source"] for row in rows}):
            flagged = [
                row
                for row in rows
                if row["source"] == source
                and math.isfinite(row["birth_depth"])
                and row["birth_depth"] > threshold
            ]
            if not flagged:
                continue
            alive_pct = 100.0 * sum(1 for row in flagged if row["alive"]) / len(flagged)
            bad_rows = [row for row in flagged if row["birth_error"] >= bad_error_threshold]
            errors = finite_values(flagged, "birth_error")
            obs = [row["final_obs"] for row in flagged]
            spans = [row["span_frames"] for row in flagged]
            print(
                f"{threshold:10.0f} {source:>18s} {len(flagged):7d} "
                f"{alive_pct:6.1f}% {len(bad_rows):8d} {fmt(mean(errors), 2):>8s} "
                f"{fmt(statistics.median(obs), 1):>7s} "
                f"{fmt(statistics.median(spans), 1):>8s}"
            )


def print_worst_birth_frames(rows, limit):
    frames = build_birth_frame_batches(rows)
    frames.sort(key=lambda row: row["birth_error"], reverse=True)
    print("\nWorst birth frames:")
    print(
        f"{'frame':>6s} {'err':>7s} {'rows':>5s} {'alive%':>7s} "
        f"{'methods':>14s} {'med_depth':>10s} {'p90_depth':>10s} "
        f"{'med_reproj':>10s} {'med_obs':>7s} {'med_span':>8s} "
        f"{'stale%':>7s}"
    )
    for row in frames[:limit]:
        methods = ",".join(f"{name}:{count}" for name, count in row["methods"].most_common())
        print(
            f"{row['frame_id']:6d} {fmt(row['birth_error'], 3):>7s} "
            f"{row['rows']:5d} {row['alive_pct']:6.1f}% {methods:>14s} "
            f"{fmt(row['med_depth'], 1):>10s} {fmt(row['p90_depth'], 1):>10s} "
            f"{fmt(row['med_reproj'], 1):>10s} {fmt(row['med_obs'], 1):>7s} "
            f"{fmt(row['med_span'], 1):>8s} {row['stale_pct']:6.1f}%"
        )


def build_birth_frame_batches(rows):
    by_frame = defaultdict(list)
    for row in rows:
        by_frame[row["birth_frame"]].append(row)
    frames = []
    for frame_id, grouped in by_frame.items():
        depths = finite_values(grouped, "birth_depth")
        reproj = finite_values(grouped, "birth_reproj")
        final_obs = [row["final_obs"] for row in grouped]
        spans = [row["span_frames"] for row in grouped]
        since = [row["frames_since_seen"] for row in grouped]
        inliers = [row["inliers"] for row in grouped]
        methods = Counter(row["method"] for row in grouped)
        alive_pct = 100.0 * sum(1 for row in grouped if row["alive"]) / len(grouped)
        stale_pct = 100.0 * sum(1 for row in grouped if row["frames_since_seen"] >= 100) / len(grouped)
        frames.append(
            {
                "frame_id": frame_id,
                "birth_error": grouped[0]["birth_error"],
                "rows": len(grouped),
                "alive_pct": alive_pct,
                "methods": methods,
                "e_frac": methods.get("E", 0) / len(grouped),
                "pnp_frac": methods.get("PnP", 0) / len(grouped),
                "med_depth": statistics.median(depths) if depths else math.nan,
                "p90_depth": percentile(depths, 0.90),
                "med_reproj": statistics.median(reproj) if reproj else math.nan,
                "med_obs": statistics.median(final_obs),
                "med_span": statistics.median(spans),
                "med_since": statistics.median(since),
                "med_inliers": statistics.median(inliers),
                "stale_pct": stale_pct,
            }
        )
    return frames


def print_birth_batch_probes(rows, bad_error_threshold):
    frames = build_birth_frame_batches(rows)
    probes = [
        ("rows>=80", lambda row: row["rows"] >= 80),
        ("med_depth>500k", lambda row: row["med_depth"] > 500000.0),
        ("p90_depth>1m", lambda row: row["p90_depth"] > 1000000.0),
        ("med_reproj>80", lambda row: row["med_reproj"] > 80.0),
        ("med_span<=2", lambda row: row["med_span"] <= 2.0),
        ("stale>=90%", lambda row: row["stale_pct"] >= 90.0),
        ("E>=90%", lambda row: row["e_frac"] >= 0.90),
        (
            "rows>=80 & med_depth>500k",
            lambda row: row["rows"] >= 80 and row["med_depth"] > 500000.0,
        ),
        (
            "rows>=80 & p90_depth>1m",
            lambda row: row["rows"] >= 80 and row["p90_depth"] > 1000000.0,
        ),
        (
            "rows>=80 & med_span<=2",
            lambda row: row["rows"] >= 80 and row["med_span"] <= 2.0,
        ),
        (
            "rows>=80 & stale>=90%",
            lambda row: row["rows"] >= 80 and row["stale_pct"] >= 90.0,
        ),
        (
            "E>=90% & med_depth>500k",
            lambda row: row["e_frac"] >= 0.90 and row["med_depth"] > 500000.0,
        ),
        (
            "E>=90% & rows>=80 & med_depth>500k",
            lambda row: row["e_frac"] >= 0.90
            and row["rows"] >= 80
            and row["med_depth"] > 500000.0,
        ),
        (
            "rows>=80 & depth>500k & span<=6",
            lambda row: row["rows"] >= 80
            and row["med_depth"] > 500000.0
            and row["med_span"] <= 6.0,
        ),
        (
            "rows>=80 & depth>500k & stale>=90%",
            lambda row: row["rows"] >= 80
            and row["med_depth"] > 500000.0
            and row["stale_pct"] >= 90.0,
        ),
    ]

    bad_frames = [row for row in frames if row["birth_error"] >= bad_error_threshold]
    print("\nBirth-frame batch probes:")
    print(
        f"bad_error_threshold={bad_error_threshold:.3f} "
        f"birth_frames={len(frames)} bad_birth_frames={len(bad_frames)}"
    )
    print(
        f"{'probe':34s} {'frames':>6s} {'bad':>5s} {'mean_err':>8s} "
        f"{'med_rows':>8s} {'med_depth':>10s} {'med_span':>8s} {'frames_sample'}"
    )
    for label, predicate in probes:
        flagged = [row for row in frames if predicate(row)]
        if not flagged:
            continue
        flagged_bad = [row for row in flagged if row["birth_error"] >= bad_error_threshold]
        errors = finite_values(flagged, "birth_error")
        row_counts = [row["rows"] for row in flagged]
        depths = finite_values(flagged, "med_depth")
        spans = finite_values(flagged, "med_span")
        frame_ids = ",".join(str(row["frame_id"]) for row in flagged[:20])
        print(
            f"{label:34s} {len(flagged):6d} {len(flagged_bad):5d} "
            f"{fmt(mean(errors), 2):>8s} {fmt(statistics.median(row_counts), 1):>8s} "
            f"{fmt(statistics.median(depths) if depths else math.nan, 1):>10s} "
            f"{fmt(statistics.median(spans) if spans else math.nan, 1):>8s} "
            f"{frame_ids}"
        )


def parse_thresholds(raw):
    thresholds = []
    for part in raw.split(","):
        part = part.strip()
        if part:
            thresholds.append(float(part))
    return thresholds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to pure_c_plus metrics JSON")
    parser.add_argument("gt_npz", help="Ground-truth NPZ")
    parser.add_argument("lifecycle_csv", help="Path to --map_lifecycle_dump CSV")
    parser.add_argument("--top_k", type=int, default=12)
    parser.add_argument("--bad_error_threshold", type=float, default=2.0)
    parser.add_argument(
        "--depth_thresholds",
        default="100000,500000,1000000,2000000,5000000",
        help="Comma-separated birth-depth thresholds for survival probes",
    )
    args = parser.parse_args()

    result = analyze_metrics_against_gt(args.metrics_json, args.gt_npz)
    frame_errors = {
        row["frame_id"]: row["translation_error_m"] for row in result["per_frame"]
    }
    rows = load_lifecycle_rows(Path(args.lifecycle_csv), frame_errors)
    thresholds = parse_thresholds(args.depth_thresholds)
    alive = [row for row in rows if row["alive"]]

    summary = result["summary"]
    print(
        f"ATE={summary['ate_rmse']:.4f} median={summary['ate_median']:.4f} "
        f"frames={summary['matched_frames']} map_births={len(rows)} "
        f"alive={len(alive)}"
    )
    print_breakdowns(rows, args.bad_error_threshold)
    print_survival_probes(rows, thresholds, args.bad_error_threshold)
    print_birth_batch_probes(rows, args.bad_error_threshold)
    print_worst_birth_frames(rows, args.top_k)


if __name__ == "__main__":
    main()
