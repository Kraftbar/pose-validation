import argparse
import itertools
import math
from dataclasses import dataclass
from pathlib import Path

from analyze_map_lifecycle import (
    build_birth_frame_batches,
    load_lifecycle_rows,
)
from gt_trace_common import analyze_metrics_against_gt


@dataclass
class Dataset:
    label: str
    batches: list
    bad_threshold: float


def frame_errors_for(metrics_json, gt_npz):
    result = analyze_metrics_against_gt(metrics_json, gt_npz)
    return {
        row["frame_id"]: row["translation_error_m"]
        for row in result["per_frame"]
    }, result["summary"]


def load_dataset(label, metrics_json, gt_npz, lifecycle_csv, bad_threshold):
    errors, summary = frame_errors_for(metrics_json, gt_npz)
    rows = load_lifecycle_rows(Path(lifecycle_csv), errors)
    batches = build_birth_frame_batches(rows)
    return Dataset(label, batches, bad_threshold), summary


def finite(value):
    return value is not None and math.isfinite(value)


def make_atoms(live_only=False):
    atoms = []
    for threshold in [20, 40, 60, 80, 100, 120, 150]:
        atoms.append((f"rows>={threshold}", lambda row, t=threshold: row["rows"] >= t))
    for threshold in [100000, 250000, 500000, 750000, 1000000, 1500000, 2000000]:
        atoms.append((
            f"med_depth>{threshold:g}",
            lambda row, t=threshold: finite(row["med_depth"]) and row["med_depth"] > t,
        ))
    for threshold in [500000, 1000000, 1500000, 2000000, 3000000, 5000000]:
        atoms.append((
            f"p90_depth>{threshold:g}",
            lambda row, t=threshold: finite(row["p90_depth"]) and row["p90_depth"] > t,
        ))
    for threshold in [20, 50, 80, 120, 200]:
        atoms.append((
            f"med_reproj>{threshold:g}",
            lambda row, t=threshold: finite(row["med_reproj"]) and row["med_reproj"] > t,
        ))
    for threshold in [20, 40, 60, 80]:
        atoms.append((f"med_inliers<={threshold}", lambda row, t=threshold: row["med_inliers"] <= t))
    for threshold in [40, 60, 80, 100]:
        atoms.append((f"med_inliers>={threshold}", lambda row, t=threshold: row["med_inliers"] >= t))
    if not live_only:
        for threshold in [0, 2, 4, 6, 8]:
            atoms.append((
                f"med_span<={threshold:g}",
                lambda row, t=threshold: finite(row["med_span"]) and row["med_span"] <= t,
            ))
        for threshold in [80, 90, 100]:
            atoms.append((f"stale>={threshold}%", lambda row, t=threshold: row["stale_pct"] >= t))
    for threshold in [0.8, 0.9, 1.0]:
        atoms.append((f"E>={threshold:.1f}", lambda row, t=threshold: row["e_frac"] >= t))
        atoms.append((f"PnP>={threshold:.1f}", lambda row, t=threshold: row["pnp_frac"] >= t))
    return atoms


def eval_rule(dataset, funcs):
    flagged = [row for row in dataset.batches if all(func(row) for func in funcs)]
    bad = [row for row in flagged if row["birth_error"] >= dataset.bad_threshold]
    return flagged, bad


def rule_is_redundant(labels):
    labels = set(labels)
    for high, low in [
        ("rows>=150", "rows>=120"),
        ("rows>=150", "rows>=100"),
        ("rows>=150", "rows>=80"),
        ("rows>=120", "rows>=100"),
        ("rows>=120", "rows>=80"),
        ("rows>=100", "rows>=80"),
        ("rows>=80", "rows>=60"),
        ("rows>=60", "rows>=40"),
        ("rows>=40", "rows>=20"),
        ("med_span<=0", "med_span<=2"),
        ("med_span<=0", "med_span<=4"),
        ("med_span<=2", "med_span<=4"),
        ("med_span<=4", "med_span<=6"),
        ("med_span<=6", "med_span<=8"),
        ("stale>=100%", "stale>=90%"),
        ("stale>=90%", "stale>=80%"),
        ("E>=1.0", "E>=0.9"),
        ("E>=0.9", "E>=0.8"),
        ("PnP>=1.0", "PnP>=0.9"),
        ("PnP>=0.9", "PnP>=0.8"),
    ]:
        if high in labels and low in labels:
            return True
    return False


def sample_frames(rows, limit=12):
    ordered = sorted(rows, key=lambda row: row["birth_error"], reverse=True)
    return ",".join(str(row["frame_id"]) for row in ordered[:limit])


def print_results(title, results, limit):
    print(f"\n{title}:")
    print(
        f"{'rule':70s} {'room':>5s} {'room_bad':>8s} {'desk':>5s} "
        f"{'precision':>9s} {'score':>8s} {'room_frames'}"
    )
    for result in results[:limit]:
        precision = result["room_bad"] / result["room_flags"] if result["room_flags"] else 0.0
        print(
            f"{result['label'][:70]:70s} {result['room_flags']:5d} "
            f"{result['room_bad']:8d} {result['desk_flags']:5d} "
            f"{precision:8.2f} {result['score']:8.2f} {result['room_sample']}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("room_metrics")
    parser.add_argument("room_gt")
    parser.add_argument("room_lifecycle")
    parser.add_argument("desk_metrics")
    parser.add_argument("desk_gt")
    parser.add_argument("desk_lifecycle")
    parser.add_argument("--room_bad_threshold", type=float, default=2.0)
    parser.add_argument("--desk_bad_threshold", type=float, default=2.0)
    parser.add_argument("--max_terms", type=int, default=3)
    parser.add_argument("--top_k", type=int, default=20)
    parser.add_argument("--live_only", action="store_true",
                        help="Exclude future-only span/staleness predicates")
    args = parser.parse_args()

    room, room_summary = load_dataset(
        "room", args.room_metrics, args.room_gt, args.room_lifecycle,
        args.room_bad_threshold,
    )
    desk, desk_summary = load_dataset(
        "desk", args.desk_metrics, args.desk_gt, args.desk_lifecycle,
        args.desk_bad_threshold,
    )
    room_bad_total = sum(1 for row in room.batches if row["birth_error"] >= room.bad_threshold)
    desk_bad_total = sum(1 for row in desk.batches if row["birth_error"] >= desk.bad_threshold)
    print(
        f"room ATE={room_summary['ate_rmse']:.4f} batches={len(room.batches)} "
        f"bad_batches={room_bad_total}"
    )
    print(
        f"desk ATE={desk_summary['ate_rmse']:.4f} batches={len(desk.batches)} "
        f"bad_batches={desk_bad_total}"
    )

    atoms = make_atoms(live_only=args.live_only)
    results = []
    seen = set()
    for size in range(1, args.max_terms + 1):
        for combo in itertools.combinations(atoms, size):
            labels = tuple(label for label, _func in combo)
            if rule_is_redundant(labels):
                continue
            funcs = [func for _label, func in combo]
            room_flagged, room_bad = eval_rule(room, funcs)
            if not room_bad:
                continue
            desk_flagged, _desk_bad = eval_rule(desk, funcs)
            signature = (
                tuple(row["frame_id"] for row in room_flagged),
                tuple(row["frame_id"] for row in desk_flagged),
            )
            if signature in seen:
                continue
            seen.add(signature)
            room_good = len(room_flagged) - len(room_bad)
            score = 4.0 * len(room_bad) - 0.75 * room_good - 2.0 * len(desk_flagged)
            results.append({
                "label": " & ".join(labels),
                "room_flags": len(room_flagged),
                "room_bad": len(room_bad),
                "desk_flags": len(desk_flagged),
                "score": score,
                "room_sample": sample_frames(room_flagged),
            })

    results.sort(
        key=lambda row: (
            row["score"],
            row["room_bad"],
            -row["desk_flags"],
            -row["room_flags"],
        ),
        reverse=True,
    )
    print_results("Top scored rules", results, args.top_k)

    no_desk = [row for row in results if row["desk_flags"] == 0]
    no_desk.sort(key=lambda row: (row["room_bad"], row["score"], -row["room_flags"]), reverse=True)
    print_results("Best rules with zero desk flags", no_desk, args.top_k)

    low_desk = [row for row in results if row["desk_flags"] <= 5]
    low_desk.sort(
        key=lambda row: (row["room_bad"], row["score"], -row["desk_flags"], -row["room_flags"]),
        reverse=True,
    )
    print_results("Best rules with <=5 desk flags", low_desk, args.top_k)


if __name__ == "__main__":
    main()
