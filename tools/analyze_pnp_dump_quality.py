import argparse
import statistics
from pathlib import Path

from gt_trace_common import analyze_metrics_against_gt


def parse_pnp_dump(path):
    frames = {}
    current = None
    for raw_line in Path(path).read_text().splitlines():
        parts = raw_line.split()
        if not parts:
            continue
        if parts[0] == "FRAME":
            current = {
                "frame_id": int(parts[1]),
                "corr": int(parts[6]),
                "has_pnp": int(parts[10]),
                "pnp_inliers_dump": int(parts[11]),
                "pnp_rmse": float(parts[15]),
                "lm_rmse": float(parts[19]),
                "obs_values": [],
            }
            frames[current["frame_id"]] = current
        elif parts[0] == "C" and current is not None:
            current["obs_values"].append(int(parts[6]))
        elif parts[0] == "END":
            current = None

    for frame in frames.values():
        obs = frame["obs_values"]
        frame["obs_min"] = min(obs) if obs else 0
        frame["obs_median"] = statistics.median(obs) if obs else 0.0
        frame["obs_mean"] = sum(obs) / len(obs) if obs else 0.0
        frame["obs_ge2"] = sum(1 for value in obs if value >= 2)
        frame["obs_ge3"] = sum(1 for value in obs if value >= 3)
    return frames


def merged_rows(metrics_json, gt_npz, pnp_dump):
    result = analyze_metrics_against_gt(metrics_json, gt_npz)
    dump_frames = parse_pnp_dump(pnp_dump)
    rows = []
    for frame in result["per_frame"]:
        merged = dict(frame)
        merged.update(dump_frames.get(frame["frame_id"], {}))
        rows.append(merged)
    return result["summary"], dump_frames, rows


def print_frame_rows(title, rows, limit):
    print(f"\n{title}:")
    print(
        f"{'frame':>6} {'err':>7} {'m':>4} {'inl':>4} {'pnp':>4} {'E':>4} "
        f"{'lnk':>5} {'corr':>5} {'pnp_rmse':>10} {'lm_rmse':>10} "
        f"{'jump':>10} {'add':>5} {'obs_med':>7}"
    )
    for row in rows[:limit]:
        print(
            f"{row['frame_id']:6d} {row['translation_error_m']:7.3f} "
            f"{row['method']:>4} {row['inliers']:4d} {row['pnp_inliers']:4d} "
            f"{row['e_inliers']:4d} {row['linked_points']:5d} "
            f"{row.get('corr', -1):5d} {row.get('pnp_rmse', 0.0):10.1f} "
            f"{row.get('lm_rmse', 0.0):10.1f} {row['trans_jump']:10.1f} "
            f"{row['points_added']:5d} {row.get('obs_median', 0.0):7.1f}"
        )


def probe_thresholds(rows, bad_error_threshold):
    pnp_rows = [row for row in rows if row["method"] == "PnP"]
    probes = [
        ("jump>500k", lambda row: row["trans_jump"] > 500000.0),
        (
            "inliers<=16 & jump>500k",
            lambda row: row["inliers"] <= 16 and row["trans_jump"] > 500000.0,
        ),
        (
            "pnp_rmse>100 & inliers<=20",
            lambda row: row.get("pnp_rmse", 0.0) > 100.0 and row["inliers"] <= 20,
        ),
        (
            "lm_rmse>100 & inliers<=20",
            lambda row: row.get("lm_rmse", 0.0) > 100.0 and row["inliers"] <= 20,
        ),
        (
            "pnp_rmse>100 & jump>500k",
            lambda row: row.get("pnp_rmse", 0.0) > 100.0 and row["trans_jump"] > 500000.0,
        ),
    ]

    print(f"\nThreshold probes over {len(pnp_rows)} PnP frames:")
    bad_rows = [row for row in pnp_rows if row["translation_error_m"] > bad_error_threshold]
    print(f"bad_error_threshold={bad_error_threshold:.3f}  bad_pnp_frames={len(bad_rows)}")
    for label, predicate in probes:
        flagged = [row for row in pnp_rows if predicate(row)]
        true_positive = [
            row for row in flagged if row["translation_error_m"] > bad_error_threshold
        ]
        mean_err = (
            sum(row["translation_error_m"] for row in flagged) / len(flagged)
            if flagged
            else 0.0
        )
        max_err = max((row["translation_error_m"] for row in flagged), default=0.0)
        frame_ids = ",".join(str(row["frame_id"]) for row in flagged[:30])
        print(
            f"{label:30s} flagged={len(flagged):3d} tp={len(true_positive):3d} "
            f"mean_err={mean_err:7.3f} max_err={max_err:7.3f} frames={frame_ids}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to metrics JSON from pure_c_plus")
    parser.add_argument("gt_npz", help="Ground-truth NPZ")
    parser.add_argument("pnp_dump", help="Path to --pnp_dump output")
    parser.add_argument("--top_k", type=int, default=15)
    parser.add_argument("--bad_error_threshold", type=float, default=2.5)
    args = parser.parse_args()

    summary, dump_frames, rows = merged_rows(args.metrics_json, args.gt_npz, args.pnp_dump)
    pnp_rows = [row for row in rows if row["method"] == "PnP"]
    print(
        f"ATE={summary['ate_rmse']:.4f} median={summary['ate_median']:.4f} "
        f"frames={len(rows)} pnp_dump_frames={len(dump_frames)} pnp_frames={len(pnp_rows)}"
    )

    print_frame_rows(
        "Worst actual frames",
        sorted(rows, key=lambda row: row["translation_error_m"], reverse=True),
        args.top_k,
    )
    print_frame_rows(
        "Top PnP RMSE frames",
        sorted(
            [row for row in pnp_rows if row.get("has_pnp")],
            key=lambda row: row.get("pnp_rmse", 0.0),
            reverse=True,
        ),
        args.top_k,
    )
    print_frame_rows(
        "Top LM RMSE frames",
        sorted(rows, key=lambda row: row.get("lm_rmse", 0.0), reverse=True),
        args.top_k,
    )
    probe_thresholds(rows, args.bad_error_threshold)


if __name__ == "__main__":
    main()
