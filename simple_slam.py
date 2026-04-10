# Version: 1.24  (Local Windowed BA + Sub-pixel refinement)
# Context:
# - Input can be webcam (auto-detect idx 0-9) or VIDEO_PATH.
# - Frames store pose (world->cam), ORB features, and loose 2D↔3D links.
# - Points store pt3d and a list of observations: (frame_id, uv).
# - When enough 3D-2D correspondences exist, we use PnP+RANSAC for pose instead of essential matrix.
# - Keyframe policy: add a keyframe when inliers low OR rotation exceeds threshold.
# - Processing resolution capped at MAX_PROC_W pixels wide to keep large videos fast.
# - Culling: drop points with <2 observations; cap total points to avoid memory blowups.
# - BA: Local Windowed BA optimizes last N keyframes and seen points. Vectorized for speed.
#
# Benchmark: see BENCHMARKS.md  (latest: freiburg ATE RMSE 0.1788 m @ v1.24)

VIDEO_PATH = 'test_kitti984.mp4'
SECONDS = 5.0
TIMEOUT = 30.0
GT_NPZ = None

# --- Tunables ---
USE_PNP = True
PNP_MIN_CORR = 8
PNP_FLAG = None
INLIER_MIN_FOR_TRI = 8
KEYFRAME_MIN_INLIERS = 20
KEYFRAME_MAX_ROT_DEG = 5.0

MAX_PROC_W = 640
MAX_POINTS = 15000
CULL_MIN_OBS = 2
LOCAL_BA_WINDOW = 10
BA_MIN_GAP_SEC = 0.5
BA_MAX_NFEV = 30

import numpy as np
import cv2
import time
import threading
import queue
import argparse
import os
import platform
import json

# SciPy is optional; if not present, BA is disabled gracefully
try:
    from scipy.optimize import least_squares  # type: ignore
    from scipy.sparse import lil_matrix # type: ignore
    _SCIPY_AVAILABLE = True
except Exception:
    _SCIPY_AVAILABLE = False

# Initialize PnP flag early (before any solvePnP usage)
if PNP_FLAG is None:
    PNP_FLAG = cv2.SOLVEPNP_AP3P

# Global map data
frames = []  # {'id', 'pose'(4x4), 'kps'(Nx2), 'des'(Nx32), 'uv_to_point': dict[(ix,iy)]=pt_idx}
points = []  # {'pt3d'(3,), 'observations': [(frame_id, uv), ...]}
last_keyframe_id = -1
_last_ba_time = 0.0
ba_runs = 0

# Lightweight runtime stats for realtime iteration and analysis
frame_stats = []  # one entry per processed frame
agg_stats = {
    'pnp_frames': 0,
    'tri_points_total': 0,
}

def extract_features(image):
    # Stronger feature density, compute ORB on gray
    orb = cv2.ORB_create(nfeatures=3000)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv_kps = cv2.goodFeaturesToTrack(gray, maxCorners=4000, qualityLevel=0.005, minDistance=5)
    if cv_kps is None or len(cv_kps) == 0:
        return np.empty((0, 2), dtype=np.float32), None
    # Sub-pixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, cv_kps, (5, 5), (-1, -1), criteria)
    
    kps = [cv2.KeyPoint(x=float(pt[0][0]), y=float(pt[0][1]), size=20.0) for pt in cv_kps]
    kps, des = orb.compute(gray, kps)
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
        if m.distance < 0.80 * n.distance:
            good.append(m)
    good.sort(key=lambda x: x.distance)
    return good

def estimate_pose_E(kps1, kps2, matches, K):
    if len(matches) < 8:
        return np.eye(4), np.array([], dtype=bool)
    pts1 = np.float32([kps1[m.queryIdx] for m in matches])
    pts2 = np.float32([kps2[m.trainIdx] for m in matches])










    



    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.5)
    if E is None:
        return np.eye(4), np.array([], dtype=bool)
    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=t.ravel()
    return T, mask.ravel() == 1

def solve_pnp(world_pts, image_pts, K):
    # Returns T_rel (world->cam) from 3D-2D with RANSAC
    ok, rvec, tvec, inl = cv2.solvePnPRansac(world_pts, image_pts, K, None,
                                             flags=PNP_FLAG, reprojectionError=3.0, iterationsCount=100)
    if not ok or inl is None or len(inl) < 6:
        return None, 0
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec.ravel()
    return T, len(inl)

def triangulate_points(pose1, pose2, pts1, pts2, K):
    P1 = K @ pose1[:3]
    P2 = K @ pose2[:3]
    pts4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
    # Robust divide
    w = pts4d[3]
    mask = np.abs(w) > 1e-8
    X = np.zeros((pts4d.shape[1], 3))
    X[mask] = (pts4d[:3, mask] / w[mask]).T
    # Cheirality
    z1 = (pose1[:3,:3] @ X.T + pose1[:3,3:4]) [2]
    z2 = (pose2[:3,:3] @ X.T + pose2[:3,3:4]) [2]
    mask &= (z1>0) & (z2>0)
    mask &= np.isfinite(X).all(axis=1)
    return X[mask], mask.ravel()

def reprojection_error(params, n_local_f, n_local_p, fixed_poses, obs_by_frame, K):
    local_poses = params[:n_local_f * 6].reshape((n_local_f, 6))
    pts3d = params[n_local_f * 6:].reshape((n_local_p, 3))
    residuals = []
    for (is_fixed, f_idx), (p_indices, uvs) in obs_by_frame.items():
        pose = fixed_poses[f_idx] if is_fixed else local_poses[f_idx]
        rvec = pose[:3]; tvec = pose[3:]
        proj, _ = cv2.projectPoints(pts3d[p_indices], rvec, tvec, K, None)
        err = (proj.reshape(-1, 2) - uvs).ravel()
        # Robustness: replace NaNs/Infs with large values
        err[~np.isfinite(err)] = 1e6
        residuals.append(err)
    if not residuals: return np.array([], dtype=np.float64)
    return np.concatenate(residuals)

def camera_center_from_world_to_camera(T):
    R = T[:3,:3]; t=T[:3,3]
    return (-R.T @ t).reshape(3)

def rot_trans_between(Ta, Tb):
    R = Tb[:3,:3] @ Ta[:3,:3].T
    dR = cv2.Rodrigues(R)[0].ravel()
    ang = np.linalg.norm(dR) * 180.0/np.pi
    dt = np.linalg.norm(Tb[:3,3] - Ta[:3,3])
    return ang, dt

def quant_uv(uv):
    # Use floor to match index-based lookup more robustly
    return (int(uv[0]), int(uv[1]))

def cull_points():
    global points, frames
    if not points: return
    keep = np.ones(len(points), dtype=bool)
    for i, p in enumerate(points):
        if len(p['observations']) < CULL_MIN_OBS:
            keep[i]=False
    if keep.all(): return
    # Reindex points; update frame uv_to_point mappings
    old_to_new = {}
    new_points = []
    for i, k in enumerate(np.where(keep)[0]):
        old_to_new[k]=i
        new_points.append(points[k])
    for f in frames:
        newmap={}
        for uv_i, old_idx in f['uv_to_point'].items():
            if old_idx in old_to_new:
                newmap[uv_i]=old_to_new[old_idx]
        f['uv_to_point']=newmap
    points = new_points

def run_ba_if_due(K):
    global _last_ba_time, ba_runs
    if not _SCIPY_AVAILABLE: return
    if time.time() - _last_ba_time < BA_MIN_GAP_SEC: return
    if len(points)==0 or len(frames)<2: return

    # Local Windowed BA (optimize local poses and all points they see)
    local_indices = list(range(max(0, len(frames)-LOCAL_BA_WINDOW), len(frames)))
    local_points = set()
    for f_idx in local_indices:
        for p_idx in frames[f_idx]['uv_to_point'].values():
            local_points.add(p_idx)
    local_points = sorted(list(local_points))
    if not local_points: return

    fixed_indices = set()
    for p_idx in local_points:
        for f_idx, uv in points[p_idx]['observations']:
            if f_idx not in local_indices:
                fixed_indices.add(f_idx)
    fixed_indices = sorted(list(fixed_indices))

    f_map = {idx: i for i, idx in enumerate(local_indices)}
    p_map = {idx: i for i, idx in enumerate(local_points)}
    fix_f_map = {idx: i for i, idx in enumerate(fixed_indices)}

    params = []
    for f_idx in local_indices:
        f = frames[f_idx]
        r = cv2.Rodrigues(f['pose'][:3,:3])[0].ravel()
        t = f['pose'][:3,3].ravel()
        params.extend(np.concatenate([r, t]))
    for p_idx in local_points:
        params.extend(points[p_idx]['pt3d'])
    params = np.array(params, dtype=np.float64)

    fixed_poses = []
    for f_idx in fixed_indices:
        f = frames[f_idx]
        r = cv2.Rodrigues(f['pose'][:3,:3])[0].ravel()
        t = f['pose'][:3,3].ravel()
        fixed_poses.append(np.concatenate([r, t]))
    fixed_poses = np.array(fixed_poses)

    obs_by_frame = {}
    for p_idx in local_points:
        p_param_idx = p_map[p_idx]
        for f_idx, uv in points[p_idx]['observations']:
            key = None
            if f_idx in f_map: key = (False, f_map[f_idx])
            elif f_idx in fix_f_map: key = (True, fix_f_map[f_idx])
            if key:
                if key not in obs_by_frame: obs_by_frame[key] = ([], [])
                obs_by_frame[key][0].append(p_param_idx)
                obs_by_frame[key][1].append(uv)
    for key in obs_by_frame:
        obs_by_frame[key] = (np.array(obs_by_frame[key][0], dtype=np.int32), 
                             np.array(obs_by_frame[key][1], dtype=np.float32))

    if not obs_by_frame: return

    # Build Sparsity Mask
    n_vars = len(params)
    total_res = sum(len(o[0]) * 2 for o in obs_by_frame.values())
    sparsity = lil_matrix((total_res, n_vars), dtype=int)
    res_idx = 0
    for (is_fixed, f_local_idx), (p_indices, _) in obs_by_frame.items():
        for p_param_idx in p_indices:
            # Each observation adds 2 residuals (x, y)
            if not is_fixed:
                # Pose variables: 6 * f_local_idx to 6 * f_local_idx + 5
                sparsity[res_idx:res_idx+2, 6*f_local_idx:6*f_local_idx+6] = 1
            # Point variables: 6 * n_local_f + 3 * p_param_idx to 6 * n_local_f + 3 * p_param_idx + 2
            p_start = 6 * len(local_indices) + 3 * p_param_idx
            sparsity[res_idx:res_idx+2, p_start:p_start+3] = 1
            res_idx += 2

    res = least_squares(reprojection_error, params, jac_sparsity=sparsity,
                        args=(len(local_indices), len(local_points), fixed_poses, obs_by_frame, K),
                        loss='huber', ftol=1e-3, max_nfev=BA_MAX_NFEV)
    opt = res.x
    idx = 0
    # The first frame in the local window is always anchored (not updated)
    # to prevent the local map from floating away from the global map.
    for i, f_idx in enumerate(local_indices):
        if i == 0: 
            idx += 6
            continue
        r=opt[idx:idx+3]; t=opt[idx+3:idx+6]; idx+=6
        frames[f_idx]['pose'][:3,:3] = cv2.Rodrigues(r)[0]
        frames[f_idx]['pose'][:3,3] = t
    for p_idx in local_points:
        points[p_idx]['pt3d'] = opt[idx:idx+3]; idx+=3
    _last_ba_time = time.time()
    ba_runs += 1

def process_frame(image, K):
    global frames, points, last_keyframe_id, agg_stats
    fid = len(frames)
    kps, des = extract_features(image)
    pose = np.eye(4)
    uv_to_point = {}

    # Pose estimation
    inliers = 0
    used_pnp = False
    if fid==0:
        pass
    else:
        prev = frames[-1]
        matches = match_features(prev['des'], des)

        # Build 3D-2D correspondences for PnP from prev frame's mapped points
        p3d=[]; p2d=[]
        if USE_PNP and prev['uv_to_point']:
            for m in matches:
                uv_prev = prev['kps'][m.queryIdx]
                key = quant_uv(uv_prev)
                if key in prev['uv_to_point']:
                    pid = prev['uv_to_point'][key]
                    p3d.append(points[pid]['pt3d'])
                    p2d.append(kps[m.trainIdx])
        if USE_PNP and len(p3d) >= PNP_MIN_CORR:
            T_rel, inl = solve_pnp(np.array(p3d, np.float32), np.array(p2d, np.float32), K)
            if T_rel is not None:
                used_pnp = True
                inliers = inl
                R_prev = prev['pose'][:3,:3]; t_prev = prev['pose'][:3,3]
                R_rel = T_rel[:3,:3]; t_rel = T_rel[:3,3]
                pose[:3,:3] = R_rel @ prev['pose'][:3,:3]
                pose[:3,3]  = R_rel @ t_prev + t_rel
            else:
                T_rel, mask = estimate_pose_E(prev['kps'], kps, matches, K)
                inliers = int(mask.sum()) if mask is not None else 0
                R_rel = T_rel[:3,:3]; t_rel = T_rel[:3,3]
                pose[:3,:3] = R_rel @ prev['pose'][:3,:3]
                pose[:3,3]  = R_rel @ prev['pose'][:3,3] + t_rel
        else:
            T_rel, mask = estimate_pose_E(prev['kps'], kps, matches, K)
            inliers = int(mask.sum()) if mask is not None else 0
            R_rel = T_rel[:3,:3]; t_rel = T_rel[:3,3]
            pose[:3,:3] = R_rel @ prev['pose'][:3,:3]
            pose[:3,3]  = R_rel @ prev['pose'][:3,3] + t_rel

    # Append frame (needed before we can write current uv_to_point during triangulation)
    frames.append({'id': fid, 'pose': pose, 'kps': kps, 'des': des, 'uv_to_point': uv_to_point})

    # Decide keyframe
    make_kf = False
    points_added_this_frame = 0
    if fid==0 or last_keyframe_id<0:
        make_kf = True
        last_keyframe_id = fid
    else:
        ang, _ = rot_trans_between(frames[last_keyframe_id]['pose'], pose)
        if inliers < KEYFRAME_MIN_INLIERS or ang > KEYFRAME_MAX_ROT_DEG:
            make_kf = True

    # Triangulate vs previous frame if keyframe and enough inliers
    if make_kf and fid>0 and inliers >= INLIER_MIN_FOR_TRI:
        # Always triangulate against the immediately previous frame (fid-1)
        prev = frames[fid-1]
        # Re-match prev<->curr for triangulation
        matches = match_features(prev['des'], des)
        # Filter with essential for geometry-consistent pairs
        T_rel, mask = estimate_pose_E(prev['kps'], kps, matches, K)
        if mask is not None and mask.any():
            good_idx = np.where(mask)[0]
            pts1 = np.float32([prev['kps'][matches[i].queryIdx] for i in good_idx])
            pts2 = np.float32([kps[matches[i].trainIdx]      for i in good_idx])
            X, tri_mask = triangulate_points(prev['pose'], pose, pts1, pts2, K)
            if len(X)>0:
                # Add points, link observations
                slots = max(0, MAX_POINTS - len(points))
                add_n = min(slots, len(X))
                for j in range(add_n):
                    p3 = X[j]
                    uv1 = pts1[tri_mask][j]
                    uv2 = pts2[tri_mask][j]
                    pid = len(points)
                    points.append({'pt3d': p3, 'observations': [(prev['id'], uv1), (fid, uv2)]})
                    # Map rounded uvs to this point for future PnP
                    frames[prev['id']]['uv_to_point'][quant_uv(uv1)] = pid
                    frames[fid]['uv_to_point'][quant_uv(uv2)] = pid
                points_added_this_frame = add_n
                agg_stats['tri_points_total'] += add_n
                last_keyframe_id = fid

    # Cull occasionally
    if fid % 10 == 0:
        cull_points()

    # BA time-boxed
    run_ba_if_due(K)

    # Update frame-level stats
    if used_pnp:
        agg_stats['pnp_frames'] += 1
    frame_stats.append({
        'frame_id': fid,
        'inliers': int(inliers),
        'used_pnp': bool(used_pnp),
        'is_keyframe': bool(make_kf),
        'points_added': int(points_added_this_frame),
        'points_total': int(len(points)),
        'xyz': camera_center_from_world_to_camera(pose).tolist(),
    })

def main():
    # CLI args for headless and tunables
    global USE_PNP, PNP_MIN_CORR, INLIER_MIN_FOR_TRI, KEYFRAME_MIN_INLIERS, KEYFRAME_MAX_ROT_DEG, MAX_PROC_W, MAX_POINTS, CULL_MIN_OBS, BA_MIN_GAP_SEC, BA_MAX_NFEV, _last_ba_time
    parser = argparse.ArgumentParser()
    parser.add_argument('--video_path', type=str, default=VIDEO_PATH)
    parser.add_argument('positional_video', nargs='?', default=None, help='Optional positional video path (no flags)')
    parser.add_argument('--seconds', type=float, default=SECONDS)
    parser.add_argument('--timeout', type=float, default=TIMEOUT)
    parser.add_argument('--use_webcam_first', action='store_true', help='Try webcams before video path')
    parser.add_argument('--no_imshow', action='store_true', help='Disable cv2.imshow for headless runs')
    parser.add_argument('--no_plot', action='store_true', help='Disable 3D plotting for headless runs')
    parser.add_argument('--metrics_out', type=str, default='', help='Optional path to write JSON metrics')
    # Tunables
    parser.add_argument('--use_pnp', type=int, choices=[0,1], default=int(USE_PNP))
    parser.add_argument('--pnp_min_corr', type=int, default=PNP_MIN_CORR)
    parser.add_argument('--inlier_min_for_tri', type=int, default=INLIER_MIN_FOR_TRI)
    parser.add_argument('--kf_min_inliers', type=int, default=KEYFRAME_MIN_INLIERS)
    parser.add_argument('--kf_max_rot_deg', type=float, default=KEYFRAME_MAX_ROT_DEG)
    parser.add_argument('--max_proc_w', type=int, default=MAX_PROC_W, help='Max processing width in pixels')
    parser.add_argument('--max_points', type=int, default=MAX_POINTS)
    parser.add_argument('--cull_min_obs', type=int, default=CULL_MIN_OBS)
    parser.add_argument('--ba_min_gap_sec', type=float, default=BA_MIN_GAP_SEC)
    parser.add_argument('--ba_max_nfev', type=int, default=BA_MAX_NFEV)
    args = parser.parse_args()

    # Auto headless in non-GUI/WSL environments unless user explicitly asked to show
    if not args.no_imshow and not args.no_plot:
        is_wsl = 'microsoft' in platform.uname().release.lower() or os.getenv('WSL_INTEROP')
        has_display = bool(os.getenv('DISPLAY'))
        if is_wsl or not has_display:
            args.no_imshow = True
            args.no_plot = True
            print("[auto] Headless mode enabled (no_imshow/no_plot)")

    # Positional override for minimal usage
    if args.positional_video:
        args.video_path = args.positional_video

    # Apply tunables
    USE_PNP = bool(args.use_pnp)
    PNP_MIN_CORR = args.pnp_min_corr
    INLIER_MIN_FOR_TRI = args.inlier_min_for_tri
    KEYFRAME_MIN_INLIERS = args.kf_min_inliers
    KEYFRAME_MAX_ROT_DEG = args.kf_max_rot_deg
    MAX_PROC_W = args.max_proc_w
    MAX_POINTS = args.max_points
    CULL_MIN_OBS = args.cull_min_obs
    BA_MIN_GAP_SEC = args.ba_min_gap_sec
    BA_MAX_NFEV = args.ba_max_nfev

    # --- Input selection ---
    cap=None; is_webcam=False
    if args.use_webcam_first:
        for i in range(10):
            cap=cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Using webcam index {i}"); is_webcam=True; break
    if not cap or not cap.isOpened():
        cap=cv2.VideoCapture(args.video_path)
        if not cap.isOpened():
            print("Failed to open video or webcam."); return

    ret, frame = cap.read()
    if not ret: print("Failed to read first frame."); return
    H,W = frame.shape[:2]
    # Halve first, then cap width so large videos don't bottleneck feature extraction
    proc_W, proc_H = W//2, H//2
    if proc_W > MAX_PROC_W:
        scale = MAX_PROC_W / proc_W
        proc_W = MAX_PROC_W
        proc_H = int(proc_H * scale)
    K = np.array([[proc_W, 0, proc_W/2],
                  [0, proc_W, proc_H/2],
                  [0, 0, 1]], dtype=np.float64)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    fps = cap.get(cv2.CAP_PROP_FPS) if cap.isOpened() else 30
    if fps<=1e-3: fps=30
    max_frames = float('inf') if is_webcam else int(fps*args.seconds)

    # Choose processing mode: inline in headless (more robust), threaded otherwise
    use_threaded = not (args.no_imshow and args.no_plot)
    if use_threaded:
        proc_queue = queue.Queue(maxsize=2)
        def worker():
            while True:
                item = proc_queue.get()
                if item is None:
                    break
                try:
                    process_frame(item, K)
                except Exception as e:
                    print(f"Worker error: {e}")
        t = threading.Thread(target=worker)
        t.start()

    # Live viz (optional)
    if not args.no_plot:
        import matplotlib.pyplot as plt  # lazy import
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        plt.ion(); fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
    _last_ba_time = time.time()  # reset so BA gap is measured from processing start, not epoch
    start = time.time(); cnt=0
    while cap.isOpened() and cnt<max_frames:
        if time.time()-start > args.timeout:
            print("Processing timeout exceeded, breaking loop."); break
        ret, orig = cap.read()
        if not ret: break
        img = cv2.resize(orig, (proc_W, proc_H))
        if use_threaded:
            try:
                proc_queue.put_nowait(img)
            except queue.Full:
                try:
                    _ = proc_queue.get_nowait()  # drop oldest
                    proc_queue.put_nowait(img)
                except Exception:
                    pass
        else:
            # Inline processing in headless mode
            process_frame(img, K)
        if not args.no_imshow:
            cv2.imshow('Frame', orig)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
        cnt+=1

        # Periodic textual progress for realtime feel
        if cnt % 10 == 0:
            print(f"Frames={len(frames)} Points={len(points)}")

        # Update plot
        if not args.no_plot:
            ax.clear()
            npts = len(points)
            if npts>0:
                if npts>1500:
                    idx = np.random.choice(npts, 1500, replace=False)
                    pts = np.array([points[i]['pt3d'] for i in idx])
                else:
                    pts = np.array([p['pt3d'] for p in points])
                ax.scatter(pts[:,0], pts[:,1], pts[:,2], s=1)
            if frames:
                centers = np.array([camera_center_from_world_to_camera(f['pose']) for f in frames])
                ax.plot(centers[:,0], centers[:,1], centers[:,2], 'r-')
                ax.scatter(centers[-1,0], centers[-1,1], centers[-1,2], s=50)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            import matplotlib.pyplot as plt
            plt.draw(); plt.pause(0.001)

    # Cleanup
    if use_threaded:
        proc_queue.put(None); t.join()
    cap.release();
    if not args.no_imshow:
        cv2.destroyAllWindows()
    if not args.no_plot:
        import matplotlib.pyplot as plt
        plt.ioff(); plt.show()

    # Metrics
    # Aggregate metrics
    avg_inliers = float(np.mean([s['inliers'] for s in frame_stats[1:]])) if len(frame_stats) > 1 else 0.0






    

    kf_count = int(sum(1 for s in frame_stats if s['is_keyframe']))
    metrics = {
        'frames': len(frames),
        'points': len(points),
        'duration_sec': round(time.time()-start, 3),
        'used_pnp': USE_PNP,
        'video_path': args.video_path,
        'keyframes': kf_count,
        'ba_runs': ba_runs,
        'pnp_frames': agg_stats['pnp_frames'],
        'tri_points_total': agg_stats['tri_points_total'],
        'avg_inliers_after_first': round(avg_inliers, 3),
        'timeline': frame_stats,  # per-frame metrics (optional, can be large)
    }
    print(f"Processed frames={metrics['frames']}, points={metrics['points']}, duration={metrics['duration_sec']}s")
    # Auto-save metrics if no path provided (so analyze_runs can find something)
    if not args.metrics_out:
        try:
            os.makedirs('runs/default', exist_ok=True)
            seconds_tag = int(args.seconds) if isinstance(args.seconds, (int, float)) else 0
            auto_metrics_path = f"runs/default/sec-{seconds_tag}_auto.json"
            with open(auto_metrics_path, 'w') as f:
                json.dump(metrics, f, indent=2)
            print(f"Wrote metrics to {auto_metrics_path}")
        except Exception as e:
            print(f"Failed to auto-write metrics: {e}")
    if args.metrics_out:
        try:
            with open(args.metrics_out, 'w') as f:
                json.dump(metrics, f, indent=2)
            print(f"Wrote metrics to {args.metrics_out}")
        except Exception as e:
            print(f"Failed to write metrics: {e}")

if __name__ == "__main__":
    main()
