import argparse

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


def rules():
    return [
        ("pnp", lambda f: f["method"] == "PnP"),
        ("E", lambda f: f["method"] == "E"),
        ("pnp_inl<=16", lambda f: f["method"] == "PnP" and f["inliers"] <= 16),
        ("pnp_inl<=20", lambda f: f["method"] == "PnP" and f["inliers"] <= 20),
        ("jump>500k", lambda f: f["trans_jump"] > 500000.0),
        ("jump>1m", lambda f: f["trans_jump"] > 1000000.0),
        (
            "pnp&jump>500k",
            lambda f: f["method"] == "PnP" and f["trans_jump"] > 500000.0,
        ),
        (
            "pnp&inl<=16&jump>500k",
            lambda f: (
                f["method"] == "PnP"
                and f["inliers"] <= 16
                and f["trans_jump"] > 500000.0
            ),
        ),
        ("linked<150", lambda f: 0 < f["linked_points"] < 150),
        ("linked<250", lambda f: 0 < f["linked_points"] < 250),
        (
            "pnp&linked<250",
            lambda f: f["method"] == "PnP" and 0 < f["linked_points"] < 250,
        ),
        (
            "E&linked<150",
            lambda f: f["method"] == "E" and 0 < f["linked_points"] < 150,
        ),
        (
            "E&linked<150&jump>500k",
            lambda f: (
                f["method"] == "E"
                and 0 < f["linked_points"] < 150
                and f["trans_jump"] > 500000.0
            ),
        ),
        ("KF&add>=100", lambda f: f["is_keyframe"] and f["points_added"] >= 100),
        (
            "KF&add>=100&jump>500k",
            lambda f: (
                f["is_keyframe"]
                and f["points_added"] >= 100
                and f["trans_jump"] > 500000.0
            ),
        ),
        (
            "pnp&predlm=0&inl<=16",
            lambda f: (
                f["method"] == "PnP"
                and f["pred_lm_inliers"] == 0
                and f["inliers"] <= 16
            ),
        ),
    ]


def load_cases(cases):
    loaded = []
    for label, metrics_json, gt_npz, bad_threshold in cases:
        result = analyze_metrics_against_gt(metrics_json, gt_npz, label=label)
        loaded.append(
            {
                "label": label,
                "bad_threshold": bad_threshold,
                "frames": result["per_frame"],
                "summary": result["summary"],
            }
        )
    return loaded


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Sweep simple live trace-health predicates over pure_c_plus traces. "
            "The default cases use the canonical benchmark traces."
        )
    )
    parser.add_argument(
        "--case",
        action="append",
        type=parse_case,
        help="Case as label=metrics_json,gt_npz,bad_error_threshold",
    )
    parser.add_argument("--top_frames", type=int, default=8)
    args = parser.parse_args()

    cases = load_cases(args.case or DEFAULT_CASES)
    print(
        f"{'rule':30s} {'case':>5} {'flag':>5} {'bad':>5} {'tp':>5} "
        f"{'mean':>7} {'max':>7} frames"
    )
    for rule_name, predicate in rules():
        for case in cases:
            frames = case["frames"]
            bad_threshold = case["bad_threshold"]
            flagged = [frame for frame in frames if predicate(frame)]
            bad = [
                frame
                for frame in frames
                if frame["translation_error_m"] > bad_threshold
            ]
            true_positive = [
                frame
                for frame in flagged
                if frame["translation_error_m"] > bad_threshold
            ]
            mean_error = (
                sum(frame["translation_error_m"] for frame in flagged) / len(flagged)
                if flagged
                else 0.0
            )
            max_error = max(
                (frame["translation_error_m"] for frame in flagged), default=0.0
            )
            frame_ids = ",".join(
                str(frame["frame_id"]) for frame in true_positive[:args.top_frames]
            )
            print(
                f"{rule_name:30s} {case['label']:>5} {len(flagged):5d} "
                f"{len(bad):5d} {len(true_positive):5d} "
                f"{mean_error:7.3f} {max_error:7.3f} {frame_ids}"
            )
        print()


if __name__ == "__main__":
    main()
