import json
from pathlib import Path

import numpy as np


METHOD_NAMES = {
    0: "Init",
    1: "E",
    2: "PnP",
    3: "Flow",
}


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """Align src -> dst via similarity transform. Returns (R, t, scale)."""
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("Expected matching Nx3 arrays for Umeyama alignment")
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    var_s = (src_c ** 2).sum() / n
    cov = (dst_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = float((D * S.diagonal()).sum() / var_s) if with_scale and var_s > 1e-12 else 1.0
    t = mu_d - scale * R @ mu_s
    return R, t, scale


def apply_alignment(src: np.ndarray, R, t, scale) -> np.ndarray:
    return (scale * R @ src.T).T + t


def rotation_error(R1, R2):
    """Compute rotation error in degrees between two 3x3 matrices."""
    R_err = R1.T @ R2
    cos_th = (np.trace(R_err) - 1.0) / 2.0
    cos_th = np.clip(cos_th, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_th)))


def load_metrics_json(metrics_json):
    with open(metrics_json, "r") as handle:
        metrics = json.load(handle)
    timeline = metrics.get("timeline")
    if not timeline:
        raise ValueError(f"No timeline found in {metrics_json}")
    return metrics


def load_gt_poses(gt_npz):
    gt_data = np.load(gt_npz)
    if "pose" not in gt_data:
        raise ValueError(f"No 'pose' array found in {gt_npz}")
    gt_poses = gt_data["pose"]
    gt_R = gt_poses[:, :3, :3]
    gt_t = gt_poses[:, :3, 3]
    gt_centers = (-gt_R.transpose(0, 2, 1) @ gt_t[:, :, None]).squeeze(-1)
    return gt_poses, gt_centers


def infer_label(metrics_json):
    stem = Path(metrics_json).stem
    known_impls = ["pure_c_brief", "pure_c_orb", "pure_c", "cpp", "c"]
    for impl in known_impls:
        if stem.endswith(f"_{impl}"):
            return impl
    if stem.startswith("test_"):
        return "python"
    return stem


def analyze_metrics_against_gt(metrics_json, gt_npz, label=None):
    metrics = load_metrics_json(metrics_json)
    timeline = metrics["timeline"]
    gt_poses, gt_centers = load_gt_poses(gt_npz)

    frame_ids = np.array([frame["frame_id"] for frame in timeline], dtype=int)
    est_xyz = np.array([frame["xyz"] for frame in timeline], dtype=float)

    valid_indices = [index for index, frame_id in enumerate(frame_ids) if 0 <= frame_id < len(gt_centers)]
    if len(valid_indices) < 3:
        raise ValueError(f"Not enough matched frames for alignment in {metrics_json}")

    matched_frame_ids = frame_ids[valid_indices]
    est_matched = est_xyz[valid_indices]
    gt_matched = gt_centers[matched_frame_ids]

    R_align, t_align, scale_align = umeyama_alignment(est_matched, gt_matched, with_scale=True)
    est_aligned = apply_alignment(est_matched, R_align, t_align, scale_align)
    trans_errors = np.linalg.norm(est_aligned - gt_matched, axis=1)

    rot_errors = np.full(len(valid_indices), np.nan, dtype=float)
    for output_index, timeline_index in enumerate(valid_indices):
        rotation_values = timeline[timeline_index].get("rotation")
        if rotation_values is None:
            continue
        rotation_array = np.asarray(rotation_values, dtype=float)
        if rotation_array.size != 9:
            continue
        R_est = rotation_array.reshape(3, 3)
        R_est_aligned = R_est @ R_align.T
        R_gt = gt_poses[matched_frame_ids[output_index], :3, :3]
        rot_errors[output_index] = rotation_error(R_est_aligned, R_gt)

    per_frame = []
    for output_index, timeline_index in enumerate(valid_indices):
        frame = timeline[timeline_index]
        rotation_value = None if np.isnan(rot_errors[output_index]) else float(rot_errors[output_index])
        method_id = frame.get("method")
        per_frame.append(
            {
                "frame_id": int(frame["frame_id"]),
                "timeline_index": int(timeline_index),
                "translation_error_m": float(trans_errors[output_index]),
                "rotation_error_deg": rotation_value,
                "inliers": int(frame.get("inliers", 0)),
                "method_id": method_id,
                "method": METHOD_NAMES.get(method_id, "N/A"),
                "is_keyframe": bool(frame.get("is_keyframe", False)),
                "points_total": int(frame.get("points_total", 0)),
                "points_added": int(frame.get("points_added", 0)),
                "tracked_count": int(frame.get("tracked_count", 0)),
                "linked_points": int(frame.get("linked_points", 0)),
                "linked_before_relink": int(frame.get("linked_before_relink", frame.get("linked_points", 0))),
                "relinked_points": int(frame.get("relinked_points", 0)),
                "pnp_inliers": int(frame.get("pnp_inliers", 0)),
                "pred_lm_inliers": int(frame.get("pred_lm_inliers", 0)),
                "e_inliers": int(frame.get("e_inliers", 0)),
                "trans_jump": float(frame.get("trans_jump", 0.0)),
            }
        )

    rotation_values = rot_errors[~np.isnan(rot_errors)]
    summary = {
        "label": label or infer_label(metrics_json),
        "metrics_json": str(metrics_json),
        "gt_npz": str(gt_npz),
        "matched_frames": int(len(valid_indices)),
        "ate_rmse": float(np.sqrt((trans_errors ** 2).mean())),
        "ate_median": float(np.median(trans_errors)),
        "ate_max": float(np.max(trans_errors)),
        "alignment_scale": float(scale_align),
        "rotation_mean": None if rotation_values.size == 0 else float(np.mean(rotation_values)),
        "rotation_median": None if rotation_values.size == 0 else float(np.median(rotation_values)),
    }

    return {
        "summary": summary,
        "per_frame": per_frame,
        "alignment": {
            "R": R_align,
            "t": t_align,
            "scale": scale_align,
        },
    }
