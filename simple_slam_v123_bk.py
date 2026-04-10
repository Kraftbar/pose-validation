# Version: 1.12 - Freiburg evaluation support
# - Add --video_path and --gt_npz CLI options
# - Compute ATE RMSE by aligning estimated camera centers to GT using Umeyama
# - Cap processed frames to GT length when provided
#
# Version: 1.11 - Improvements
# - Compose poses cumulatively (use absolute world->camera poses)
# - Correct triangulation mapping by returning/using a validity mask; enforce cheirality in both cameras
# - Use KNN + ratio test for ORB matching (more robust than cross-check)
# - Guard feature extraction when no corners are found
# - Anchor first pose during BA application (skip updating frame 0)
# - Remove unsafe point truncation that invalidated indices

import numpy as np
import cv2
from scipy.optimize import least_squares
import pyvista as pv
import argparse
from time import time

# Global map data
frames = []  # List of {'id': int, 'pose': np.array(4x4), 'kps': np.array(Nx2), 'des': np.array(Nx32), 'point_indices': list}
points = []  # List of {'pt3d': np.array(3), 'observations': list of (frame_id, uv)}

def extract_features(image):
    orb = cv2.ORB_create()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv_kps = cv2.goodFeaturesToTrack(gray, maxCorners=5000, qualityLevel=0.01, minDistance=7)
    if cv_kps is None or len(cv_kps) == 0:
        return np.empty((0, 2), dtype=np.float32), None
    kps = [cv2.KeyPoint(x=float(pt[0][0]), y=float(pt[0][1]), size=20.0) for pt in cv_kps]
    kps, des = orb.compute(image, kps)
    if kps is None or des is None or len(kps) == 0:
        return np.empty((0, 2), dtype=np.float32), None
    return np.array([(kp.pt[0], kp.pt[1]) for kp in kps], dtype=np.float32), des

def match_features(des1, des2):
    if des1 is None or des2 is None or len(des1) == 0 or len(des2) == 0:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn_matches = bf.knnMatch(des1, des2, k=2)
    good = []
    for m_n in knn_matches:
        if len(m_n) < 2:
            continue
        m, n = m_n
        if m.distance < 0.75 * n.distance:
            good.append(m)
    good = sorted(good, key=lambda x: x.distance)
    return good

def estimate_pose(kps1, kps2, matches, K):
    if len(matches) < 8:
        return np.eye(4), np.array([], dtype=bool)
    pts1 = np.float32([kps1[m.queryIdx] for m in matches])
    pts2 = np.float32([kps2[m.trainIdx] for m in matches])
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        return np.eye(4), np.array([], dtype=bool)
    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
    pose_rel = np.hstack((R, t))
    pose_rel = np.vstack((pose_rel, [0, 0, 0, 1]))
    return pose_rel, mask.ravel() == 1

def triangulate_points(pose1, pose2, pts1, pts2, K):
    # pose1/pose2 are world->camera extrinsics
    proj1 = K @ pose1[:3]
    proj2 = K @ pose2[:3]
    pts4d = cv2.triangulatePoints(proj1, proj2, pts1.T, pts2.T)
    pts3d_h = (pts4d[:3] / pts4d[3]).T  # world coordinates
    # Cheirality: positive depth in both cameras
    R1, t1 = pose1[:3, :3], pose1[:3, 3]
    R2, t2 = pose2[:3, :3], pose2[:3, 3]
    z1 = (R1 @ pts3d_h.T + t1.reshape(3, 1))[2]
    z2 = (R2 @ pts3d_h.T + t2.reshape(3, 1))[2]
    mask = (z1 > 0) & (z2 > 0)
    return pts3d_h[mask], mask.ravel()

def reprojection_error(params, num_frames, num_points, observations, K):
    poses = params[:num_frames * 6].reshape((num_frames, 6))
    pts3d = params[num_frames * 6:].reshape((num_points, 3))
    residuals = []
    for frame_id, pt_id, uv in observations:
        rvec = poses[frame_id][:3]
        tvec = poses[frame_id][3:]
        R = cv2.Rodrigues(rvec)[0]
        pt = pts3d[pt_id]
        proj = K @ (R @ pt + tvec)
        proj = proj[:2] / proj[2]
        residuals.append(proj - uv)
    return np.array(residuals).flatten()

def camera_center_from_world_to_camera(pose_w2c: np.ndarray) -> np.ndarray:
    R = pose_w2c[:3, :3]
    t = pose_w2c[:3, 3]
    return (-R.T @ t).reshape(3)

def umeyama_alignment(src_points: np.ndarray, dst_points: np.ndarray, with_scale: bool = True):
    # src -> dst: find s, R, t that minimizes || s R src + t - dst ||
    assert src_points.shape == dst_points.shape and src_points.shape[1] == 3
    n = src_points.shape[0]
    mu_src = src_points.mean(axis=0)
    mu_dst = dst_points.mean(axis=0)
    X = src_points - mu_src
    Y = dst_points - mu_dst
    cov = (Y.T @ X) / n
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt
    if with_scale:
        var_src = (X ** 2).sum() / n
        s = (S.sum()) / (var_src + 1e-12)
    else:
        s = 1.0
    t = mu_dst - s * (R @ mu_src)
    return s, R, t

def process_frame(image, K):
    global frames, points
    frame_id = len(frames)
    kps, des = extract_features(image)
    pose = np.eye(4)  # absolute world->camera pose
    point_indices = []
    good_matches_mask = None
    observations = []
    if frame_id > 0:
        prev_kps = frames[-1]['kps']
        prev_des = frames[-1]['des']
        matches = match_features(prev_des, des)
        pose_rel, good_matches_mask = estimate_pose(prev_kps, kps, matches, K)
        if good_matches_mask is not None and good_matches_mask.size > 0:
            # Compose absolute pose: pose_curr = pose_rel ∘ pose_prev (world->cam)
            R_prev = frames[-1]['pose'][:3, :3]
            t_prev = frames[-1]['pose'][:3, 3]
            R_rel = pose_rel[:3, :3]
            t_rel = pose_rel[:3, 3]
            R_curr = R_rel @ R_prev
            t_curr = R_rel @ t_prev + t_rel
            pose = np.eye(4)
            pose[:3, :3] = R_curr
            pose[:3, 3] = t_curr

            good_matches1 = np.float32([prev_kps[m.queryIdx] for m in matches])[good_matches_mask]
            good_matches2 = np.float32([kps[m.trainIdx] for m in matches])[good_matches_mask]
            if len(good_matches1) > 0 and len(good_matches2) > 0:
                pts3d, tri_mask = triangulate_points(frames[-1]['pose'], pose, good_matches1, good_matches2, K)
                # tri_mask corresponds to good_matches arrays; add only valid ones
                valid_idx = np.where(tri_mask)[0]
                for j, idx_valid in enumerate(valid_idx):
                    pt3 = pts3d[j]
                    points.append({'pt3d': pt3, 'observations': [(frame_id-1, good_matches1[idx_valid]), (frame_id, good_matches2[idx_valid])]} )
                    new_point_index = len(points) - 1
                    point_indices.append(new_point_index)
                    frames[-1]['point_indices'].append(new_point_index)
        else:
            print("No good matches for triangulation.")
    frames.append({'id': frame_id, 'pose': pose, 'kps': kps, 'des': des, 'point_indices': point_indices})
    # Optimize every 3 frames if not too large (heavier for beefy hardware)
    if frame_id % 3 == 0 and frame_id > 0 and len(points) < 50:
        all_observations = []
        for pt_id, pt in enumerate(points):
            for fid, uv in pt['observations']:
                all_observations.append((fid, pt_id, uv))
        params = []
        for f in frames:
            rvec = cv2.Rodrigues(f['pose'][:3, :3])[0].flatten()
            tvec = f['pose'][:3, 3].flatten()
            params.extend(np.concatenate((rvec, tvec)))
        for p in points:
            params.extend(p['pt3d'])
        params = np.array(params)
        result = least_squares(reprojection_error, params, args=(len(frames), len(points), all_observations, K), ftol=1e-8)  # Tighter ftol for convergence
        optimized_params = result.x
        idx = 0
        for f_i, f in enumerate(frames):
            rvec = optimized_params[idx:idx+3]
            tvec = optimized_params[idx+3:idx+6]
            if f_i != 0:  # anchor first pose
                f['pose'][:3, :3] = cv2.Rodrigues(rvec)[0]
                f['pose'][:3, 3] = tvec
            idx += 6
        for p in points:
            p['pt3d'] = optimized_params[idx:idx+3]
            idx += 3

def visualize():
    plotter = pv.Plotter()
    # Plot points (downsample to 1000 for speed)
    if points:
        pts = np.array([p['pt3d'] for p in points])
        if pts.ndim == 2 and pts.shape[1] == 3:
            if len(pts) > 1000:
                pts = pts[np.random.choice(len(pts), 1000, replace=False)]
            point_cloud = pv.PolyData(pts)
            plotter.add_points(point_cloud, color='black', point_size=1)
        else:
            print("No valid 3D points to plot.")
    else:
        print("No points in map.")
    # Plot camera poses with full rotation axes
    arrow_length = 5
    colors = ['red', 'green', 'blue']  # x, y, z
    for f in frames:
        pos = f['pose'][:3, 3]
        for i, color in enumerate(colors):
            dir_vec = f['pose'][:3, i] * arrow_length
            arrow = pv.Arrow(start=pos, direction=dir_vec, tip_length=0.3, shaft_radius=0.05)
            plotter.add_mesh(arrow, color=color)
    plotter.show()

# Run SLAM with flag
parser = argparse.ArgumentParser()
parser.add_argument('--input', default='webcam', choices=['webcam', 'video'], help='Input: webcam or video')
parser.add_argument('--video_path', default='test_kitti984.mp4', help='Path to video when --input=video')
parser.add_argument('--gt_npz', default=None, help='Optional NPZ file with ground-truth poses under key "pose"')
parser.add_argument('--seconds', type=float, default=5.0, help='Duration to process for videos (seconds)')
args = parser.parse_args()

cap = None
if args.input == 'webcam':
    for i in range(10):  # Try indices 0-9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Using webcam index {i}")
            break
    if not cap.isOpened():
        print("No webcam found. Use --input=video")
        exit()
else:
    cap = cv2.VideoCapture(args.video_path)

# Get W, H from first frame
ret, frame = cap.read()
if not ret:
    print("Failed to read first frame.")
    exit()
H, W = frame.shape[:2]
# Update K for dynamic W,H (assume fx=fy=W, cx=W/2, cy=H/2 for simple)
K = np.array([[W, 0, W/2], [0, W, H/2], [0, 0, 1]])
Kinv = np.linalg.inv(K)
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset to start

fps = cap.get(cv2.CAP_PROP_FPS) if cap.isOpened() else 30  # Default 30 if unknown
if fps is None or fps <= 1e-3:
    fps = 30
max_frames = int(fps * float(args.seconds))  # default seconds
gt_poses = None
if args.gt_npz is not None:
    try:
        data = np.load(args.gt_npz)
        if 'pose' in data:
            gt_poses = data['pose']
            if gt_poses.ndim == 3 and gt_poses.shape[1:] == (4, 4):
                max_frames = gt_poses.shape[0]
        else:
            print('Warning: gt_npz does not contain key "pose"')
    except Exception as e:
        print(f'Failed to load gt_npz: {e}')
frame_count = 0

start_time = time()
while cap.isOpened() and frame_count < max_frames:
    ret, frame = cap.read()
    if not ret:
        break
    process_frame(frame, K)
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    frame_count += 1

cap.release()
cv2.destroyAllWindows()

# Evaluate against GT if provided
if gt_poses is not None and len(frames) > 1:
    est_centers = np.array([camera_center_from_world_to_camera(f['pose']) for f in frames])
    N = min(est_centers.shape[0], gt_poses.shape[0])
    est_centers = est_centers[:N]
    # Two GT interpretations: T_cw (world->cam) and T_wc (cam->world)
    gt_cw = gt_poses[:N]
    gt_centers_from_cw = np.array([camera_center_from_world_to_camera(T) for T in gt_cw])
    gt_wc = gt_poses[:N]
    gt_centers_from_wc = np.array([T[:3, 3] for T in gt_wc])

    def aligned_rmse(src, dst):
        s, R, t = umeyama_alignment(src, dst, with_scale=True)
        src_hat = (s * (R @ src.T)).T + t
        err = np.linalg.norm(src_hat - dst, axis=1)
        return float(np.sqrt((err ** 2).mean())), s

    rmse_cw, s_cw = aligned_rmse(est_centers, gt_centers_from_cw)
    rmse_wc, s_wc = aligned_rmse(est_centers, gt_centers_from_wc)
    if rmse_cw <= rmse_wc:
        print(f'GT assumed as world->camera (T_cw). ATE RMSE={rmse_cw:.3f}, scale={s_cw:.3f}, frames={N}')
    else:
        print(f'GT assumed as camera->world (T_wc). ATE RMSE={rmse_wc:.3f}, scale={s_wc:.3f}, frames={N}')
else:
    print(f'Processed frames={len(frames)}, points={len(points)}')

visualize()