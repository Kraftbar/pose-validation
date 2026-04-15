import argparse
import csv
import html
from pathlib import Path

import numpy as np

from gt_trace_common import analyze_metrics_against_gt


COLORS = [
    "#2563eb",
    "#dc2626",
    "#16a34a",
    "#9333ea",
    "#ea580c",
    "#0891b2",
    "#4f46e5",
    "#65a30d",
]


def parse_trace_spec(spec):
    if "=" in spec:
        label, path = spec.split("=", 1)
        if label and path:
            return label, path
    return None, spec


def ensure_parent(path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)


def format_float(value):
    if value is None:
        return ""
    return f"{value:.4f}"


def build_ticks(min_value, max_value, count):
    if max_value <= min_value:
        return [min_value]
    return np.linspace(min_value, max_value, count).tolist()


def polyline_points(points, x_min, x_max, y_min, y_max, left, top, width, height):
    if not points:
        return ""

    def scale_x(value):
        if x_max <= x_min:
            return left + width / 2.0
        return left + ((value - x_min) / (x_max - x_min)) * width

    def scale_y(value):
        if y_max <= y_min:
            return top + height / 2.0
        return top + height - ((value - y_min) / (y_max - y_min)) * height

    return " ".join(f"{scale_x(x):.2f},{scale_y(y):.2f}" for x, y in points)


def render_panel(svg_parts, panel, series_list, x_min, x_max, top, left, width, height):
    values = []
    filtered_series = []
    for series in series_list:
        points = [(row["frame_id"], row[panel["key"]]) for row in series["per_frame"] if row[panel["key"]] is not None]
        if not points:
            continue
        filtered_series.append((series, points))
        values.extend(value for _, value in points)

    if not values:
        return False

    y_min = 0.0
    y_max = max(values)
    if y_max <= y_min:
        y_max = 1.0
    y_max *= 1.08

    svg_parts.append(f'<rect x="{left}" y="{top}" width="{width}" height="{height}" fill="white" stroke="#d1d5db"/>')

    for tick_value in build_ticks(y_min, y_max, 6):
        y = top + height - ((tick_value - y_min) / (y_max - y_min)) * height if y_max > y_min else top + height / 2.0
        svg_parts.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + width}" y2="{y:.2f}" stroke="#e5e7eb" stroke-width="1"/>')
        svg_parts.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" '
            f'font-family="sans-serif" font-size="12" fill="#374151">{tick_value:.2f}</text>'
        )

    for tick_value in build_ticks(x_min, x_max, 7):
        x = left + ((tick_value - x_min) / (x_max - x_min)) * width if x_max > x_min else left + width / 2.0
        svg_parts.append(f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + height}" stroke="#f3f4f6" stroke-width="1"/>')
        svg_parts.append(
            f'<text x="{x:.2f}" y="{top + height + 18}" text-anchor="middle" '
            f'font-family="sans-serif" font-size="12" fill="#374151">{int(round(tick_value))}</text>'
        )

    svg_parts.append(
        f'<text x="{left}" y="{top - 12}" font-family="sans-serif" font-size="15" '
        f'font-weight="bold" fill="#111827">{html.escape(panel["title"])}</text>'
    )
    svg_parts.append(
        f'<text x="{left - 54}" y="{top + height / 2:.2f}" transform="rotate(-90 {left - 54},{top + height / 2:.2f})" '
        f'font-family="sans-serif" font-size="12" fill="#374151">{html.escape(panel["y_label"])}</text>'
    )

    for index, (series, points) in enumerate(filtered_series):
        color = COLORS[index % len(COLORS)]
        svg_parts.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2.2" '
            f'points="{polyline_points(points, x_min, x_max, y_min, y_max, left, top, width, height)}"/>'
        )

    return True


def write_svg(output_path, title, series_list):
    all_frame_ids = [row["frame_id"] for series in series_list for row in series["per_frame"]]
    if not all_frame_ids:
        raise ValueError("No matched frames available to plot")

    x_min = min(all_frame_ids)
    x_max = max(all_frame_ids)
    panels = [
        {"key": "translation_error_m", "title": "Per-frame translation error", "y_label": "meters"},
        {"key": "rotation_error_deg", "title": "Per-frame rotation error", "y_label": "degrees"},
    ]
    active_panels = []
    for panel in panels:
        if any(any(row[panel["key"]] is not None for row in series["per_frame"]) for series in series_list):
            active_panels.append(panel)

    width = 1280
    panel_height = 260
    left = 90
    right_margin = 220
    top_margin = 90
    panel_gap = 80
    bottom_margin = 70
    plot_width = width - left - right_margin
    height = top_margin + len(active_panels) * panel_height + max(len(active_panels) - 1, 0) * panel_gap + bottom_margin

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f9fafb"/>',
        f'<text x="{left}" y="34" font-family="sans-serif" font-size="24" font-weight="bold" fill="#111827">{html.escape(title)}</text>',
        f'<text x="{left}" y="58" font-family="sans-serif" font-size="13" fill="#4b5563">Frame-by-frame GT-aligned errors from saved metrics JSON traces</text>',
    ]

    legend_x = width - right_margin + 20
    legend_y = top_margin
    svg_parts.append(
        f'<text x="{legend_x}" y="{legend_y - 18}" font-family="sans-serif" font-size="15" font-weight="bold" fill="#111827">Runs</text>'
    )
    for index, series in enumerate(series_list):
        color = COLORS[index % len(COLORS)]
        y = legend_y + index * 28
        summary = series["summary"]
        legend_text = f"{summary['label']}  RMSE {summary['ate_rmse']:.3f} m"
        svg_parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 22}" y2="{y}" stroke="{color}" stroke-width="3"/>')
        svg_parts.append(
            f'<text x="{legend_x + 30}" y="{y + 4}" font-family="sans-serif" font-size="12" fill="#111827">{html.escape(legend_text)}</text>'
        )

    for panel_index, panel in enumerate(active_panels):
        top = top_margin + panel_index * (panel_height + panel_gap)
        render_panel(svg_parts, panel, series_list, x_min, x_max, top, left, plot_width, panel_height)

    x_axis_y = top_margin + len(active_panels) * panel_height + max(len(active_panels) - 1, 0) * panel_gap + 36
    svg_parts.append(
        f'<text x="{left + plot_width / 2:.2f}" y="{x_axis_y}" text-anchor="middle" '
        f'font-family="sans-serif" font-size="12" fill="#374151">Frame ID</text>'
    )
    svg_parts.append('</svg>')

    ensure_parent(output_path)
    Path(output_path).write_text("\n".join(svg_parts), encoding="utf-8")


def write_csv(output_path, series_list):
    ensure_parent(output_path)
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "label",
                "metrics_json",
                "frame_id",
                "translation_error_m",
                "rotation_error_deg",
                "inliers",
                "method",
                "is_keyframe",
                "points_total",
                "points_added",
            ]
        )
        for series in series_list:
            summary = series["summary"]
            for row in series["per_frame"]:
                writer.writerow(
                    [
                        summary["label"],
                        summary["metrics_json"],
                        row["frame_id"],
                        format_float(row["translation_error_m"]),
                        format_float(row["rotation_error_deg"]),
                        row["inliers"],
                        row["method"],
                        int(row["is_keyframe"]),
                        row["points_total"],
                        row["points_added"],
                    ]
                )


def main():
    parser = argparse.ArgumentParser(
        description="Generate paper-style frame-by-frame GT error plots from saved SLAM metrics JSON traces."
    )
    parser.add_argument("--gt", required=True, help="Ground-truth NPZ containing pose[]")
    parser.add_argument("--output", required=True, help="Output SVG path")
    parser.add_argument("--csv", help="Optional long-form CSV export path")
    parser.add_argument("--title", help="Optional plot title")
    parser.add_argument(
        "traces",
        nargs="+",
        help="Metrics JSON inputs, optionally prefixed with label=path for cleaner legends",
    )
    args = parser.parse_args()

    series_list = []
    for spec in args.traces:
        label, metrics_path = parse_trace_spec(spec)
        series_list.append(analyze_metrics_against_gt(metrics_path, args.gt, label=label))

    title = args.title or f"GT error comparison: {Path(args.gt).stem}"
    write_svg(args.output, title, series_list)
    if args.csv:
        write_csv(args.csv, series_list)

    print(f"Wrote plot: {args.output}")
    if args.csv:
        print(f"Wrote CSV:  {args.csv}")
    print()
    for series in series_list:
        summary = series["summary"]
        print(
            f"{summary['label']:<14} matched={summary['matched_frames']:<4d} "
            f"rmse={summary['ate_rmse']:.4f}m median={summary['ate_median']:.4f}m max={summary['ate_max']:.4f}m"
        )


if __name__ == "__main__":
    main()
