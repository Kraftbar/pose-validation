import json
import numpy as np
import argparse
from pathlib import Path

def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """Align src -> dst via similarity transform.  Returns (R, t, scale)."""
    assert src.shape == dst.shape and src.ndim == 2 and src.shape[1] == 3
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
    return np.degrees(np.arccos(cos_th))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_json", help="Path to metrics JSON from SLAM")
    parser.add_argument("gt_npz", help="Path to ground truth NPZ")
    parser.add_argument("--top_k", type=int, default=10, help="Number of worst frames to show")
    args = parser.parse_args()

    with open(args.metrics_json, "r") as f:
        metrics = json.load(f)

    gt_data = np.load(args.gt_npz)
    gt_poses = gt_data["pose"] # (N, 4, 4) world -> cam

    timeline = metrics["timeline"]
    frame_ids = [f["frame_id"] for f in timeline]
    est_xyz = np.array([f["xyz"] for f in timeline])
    
    # GT camera centers
    gt_R = gt_poses[:, :3, :3]
    gt_t = gt_poses[:, :3, 3]
    gt_centers = (-gt_R.transpose(0, 2, 1) @ gt_t[:, :, None]).squeeze(-1)

    # Match frame IDs
    valid_indices = [i for i, fid in enumerate(frame_ids) if fid < len(gt_centers)]
    est_matched = est_xyz[valid_indices]
    gt_matched = gt_centers[[frame_ids[i] for i in valid_indices]]

    if len(est_matched) < 3:
        print("Not enough matched frames for alignment.")
        return

    # Align
    R_align, t_align, scale_align = umeyama_alignment(est_matched, gt_matched, with_scale=True)
    print(f"R_align:\n{R_align}")
    est_aligned = apply_alignment(est_matched, R_align, t_align, scale_align)

    # Per-frame translation error
    trans_errors = np.linalg.norm(est_aligned - gt_matched, axis=1)

    # Per-frame rotation error
    rot_errors = []
    has_rotation = "rotation" in timeline[0]
    if has_rotation:
        for i in valid_indices:
            fid = frame_ids[i]
            # est_rotation is R_cam_to_world? 
            # In simple_slam_c_brief, pose.m is world-to-camera?
            # Let's check. camera_center_from_pose does -R^T @ t, so pose is world-to-cam.
            # But the rotation I output is pose_get_rotation, which is world-to-cam.
            # GT pose is also world-to-cam.
            R_est = np.array(timeline[i]["rotation"]).reshape(3, 3)
            # We need to align the estimated rotation too?
            # est_aligned = R_align @ R_est? No, R_align is for camera centers.
            # Actually, rotation error should be invariant to global translation/scale.
            # But it IS sensitive to global rotation.
            # So we should apply R_align to the estimated rotation.
            # If R_est is world_to_cam, then R_est_aligned = R_est @ R_align.T ?
            # Let's think: x_cam = R_est @ x_world + t_est
            # x_world_aligned = R_align @ x_world + t_align
            # x_cam = R_est_aligned @ x_world_aligned + t_est_aligned
            # x_cam = R_est_aligned @ (R_align @ x_world + t_align) + t_est_aligned
            # x_cam = R_est_aligned @ R_align @ x_world + ...
            # So R_est = R_est_aligned @ R_align  => R_est_aligned = R_est @ R_align.T
            R_est_aligned = R_est @ R_align.T
            R_gt = gt_poses[fid, :3, :3]
            rot_errors.append(rotation_error(R_est_aligned, R_gt))
    else:
        rot_errors = [0.0] * len(valid_indices)

    # Report
    print(f"Alignment: scale={scale_align:.4f}")
    print(f"ATE RMSE: {np.sqrt((trans_errors**2).mean()):.4f} m")
    print(f"ATE Median: {np.median(trans_errors):.4f} m")
    if has_rotation:
        print(f"Rotation Error Mean: {np.mean(rot_errors):.4f} deg")
        print(f"Rotation Error Median: {np.median(rot_errors):.4f} deg")

    # Worst frames
    print(f"\nTop {args.top_k} worst frames by translation error:")
    worst_idx = np.argsort(trans_errors)[::-1][:args.top_k]
    print(f"{'Frame':>6} {'Error(m)':>10} {'RotErr(deg)':>12} {'Inliers':>8} {'Method':>8} {'KF':>4}")
    for idx in worst_idx:
        i = valid_indices[idx]
        f = timeline[i]
        method_str = {0:"Init", 1:"E", 2:"PnP", 3:"Flow"}.get(f.get("method", -1), "N/A")
        print(f"{f['frame_id']:6d} {trans_errors[idx]:10.4f} {rot_errors[idx]:12.4f} {f['inliers']:8d} {method_str:8s} {'YES' if f['is_keyframe'] else 'no':>4}")

if __name__ == "__main__":
    main()
