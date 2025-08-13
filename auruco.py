"""
Version: 1.3 - Enhancements
- CLI options: input source (images/video/webcam), glob pattern or video path, marker id/size, calibration npz, frame limit
- Uses ArUcoDetector (OpenCV >=4.7) with DetectorParameters; subpixel corner refinement for better pose
- Supports consistent world anchoring to a chosen marker id (default: 1)
- Optional CSV export of camera trajectory; saves overlay image
- Keeps Plotly 3D visualization with camera positions and floor overlay
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import plotly.graph_objects as go
import glob
from PIL import Image  # For resizing
import argparse
import os

def load_calibration(npz_path: str | None, fallback_w: int, fallback_h: int):
    if npz_path and os.path.exists(npz_path):
        try:
            data = np.load(npz_path)
            K = data['K'] if 'K' in data else data['camera_matrix']
            D = data['D'] if 'D' in data else data.get('dist', np.zeros((4, 1)))
            return K, D
        except Exception as e:
            print(f"Failed to load calibration npz: {e}")
    # Fallback: assume fx=fy=W, cx=W/2, cy=H/2
    K = np.array([[fallback_w, 0, fallback_w/2], [0, fallback_w, fallback_h/2], [0, 0, 1]], dtype=float)
    D = np.zeros((4, 1), dtype=float)
    return K, D

def detect_and_pose(gray, detector, K, D, marker_size_cm: float, refine_subpix: bool = True):
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return [], None, None
    # Refine detected corners
    if refine_subpix:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        for i in range(len(corners)):
            cv2.cornerSubPix(gray, corners[i], (3, 3), (-1, -1), term)
    obj_rvecs, obj_tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size_cm, K, D)
    detections = []
    for i in range(len(ids)):
        rvec = obj_rvecs[i, 0, :]
        tvec = obj_tvecs[i, 0, :]
        detections.append((int(ids[i][0]), rvec, tvec))
    return detections, corners, ids

def pose_camera_in_marker(rvec, tvec):
    R_ct = cv2.Rodrigues(rvec)[0]
    R_tc = R_ct.T
    C_in_marker = -R_tc @ tvec.reshape(3, 1)
    return C_in_marker.flatten(), R_tc

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', choices=['images', 'video', 'webcam'], default='images')
    parser.add_argument('--glob', default='snapshot_640_480_*.png', help='Glob for images when --input=images')
    parser.add_argument('--video_path', default='', help='Path to video when --input=video')
    parser.add_argument('--camera_npz', default='', help='Optional calibration npz (K, D or camera_matrix, dist)')
    parser.add_argument('--marker_id', type=int, default=1, help='Marker id to anchor world to')
    parser.add_argument('--marker_size_cm', type=float, default=10.0, help='Marker size in cm (side length)')
    parser.add_argument('--aruco_dict', type=str, default='DICT_ARUCO_ORIGINAL', help='OpenCV aruco dict name')
    parser.add_argument('--max_frames', type=int, default=0, help='Optional cap on processed frames (0 = no cap)')
    parser.add_argument('--alpha', type=float, default=0.5, help='Overlay blending alpha')
    parser.add_argument('--csv_out', type=str, default='aruco_poses.csv', help='CSV to export camera positions')
    parser.add_argument('--vis', choices=['plotly', 'pyvista'], default='plotly', help='Visualization backend')
    parser.add_argument('--show_images', action='store_true', help='Show annotated frames while processing')
    parser.add_argument('--save_images_dir', type=str, default='', help='Optional directory to save annotated frames')
    args = parser.parse_args()

    # Prepare detector
    dict_id = getattr(aruco, args.aruco_dict, aruco.DICT_ARUCO_ORIGINAL)
    aruco_dict = aruco.getPredefinedDictionary(dict_id)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    frames_iter = []
    source_desc = ''
    if args.input == 'images':
        image_files = sorted(glob.glob(args.glob))
        if not image_files:
            print(f"No images found for glob: {args.glob}")
            return
        frames_iter = image_files
        source_desc = f"images:{len(image_files)}"
        # Prime calibration from first image size
        sample = cv2.imread(image_files[0])
        if sample is None:
            print("Failed to read first image")
            return
        H, W = sample.shape[:2]
        K, D = load_calibration(args.camera_npz, W, H)
    elif args.input == 'video':
        if not args.video_path:
            print('Provide --video_path for video input')
            return
        cap = cv2.VideoCapture(args.video_path)
        if not cap.isOpened():
            print(f'Failed to open video: {args.video_path}')
            return
        ret, frame0 = cap.read()
        if not ret:
            print('Failed to read first frame from video')
            return
        H, W = frame0.shape[:2]
        K, D = load_calibration(args.camera_npz, W, H)
        frames_iter = cap
        source_desc = f"video:{args.video_path}"
    else:  # webcam
        cap = None
        for i in range(10):
            c = cv2.VideoCapture(i)
            if c.isOpened():
                cap = c
                print(f"Using webcam index {i}")
                break
        if cap is None or not cap.isOpened():
            print('No webcam found')
            return
        ret, frame0 = cap.read()
        if not ret:
            print('Failed to read from webcam')
            return
        H, W = frame0.shape[:2]
        K, D = load_calibration(args.camera_npz, W, H)
        frames_iter = cap
        source_desc = 'webcam'

    cam_poses = []  # list of (pos_cam_in_world, R_world)
    overlay = None
    processed = 0
    world_marker = args.marker_id
    # For world-aligned mosaic (Z=0 plane)
    mosaic_items: list[tuple[np.ndarray, np.ndarray]] = []  # (frame_bgr, H_inv)
    # Prepare output dir for annotated frames
    if args.save_images_dir:
        os.makedirs(args.save_images_dir, exist_ok=True)

    def process_frame(frame):
        nonlocal overlay, processed
        if frame is None:
            return
        overlay = frame.astype(np.float32) if overlay is None else cv2.addWeighted(overlay, 1 - args.alpha, frame.astype(np.float32), args.alpha, 0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections, corners, ids = detect_and_pose(gray, detector, K, D, args.marker_size_cm)
        # Annotate frame with detections
        annotated = frame.copy()
        if ids is not None and corners is not None and len(corners) > 0:
            aruco.drawDetectedMarkers(annotated, corners, ids)
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, args.marker_size_cm, K, D)
                for rv, tv in zip(rvecs, tvecs):
                    aruco.drawAxis(annotated, K, D, rv[0], tv[0], args.marker_size_cm * 0.5)
            except Exception:
                pass
        if not detections:
            if args.show_images:
                cv2.imshow('ArUco', annotated)
                cv2.waitKey(1)
            if args.save_images_dir:
                cv2.imwrite(os.path.join(args.save_images_dir, f"frame_{processed:06d}.png"), annotated)
            return
        # Select anchor marker if present, else skip
        ids_present = [d[0] for d in detections]
        if world_marker not in ids_present:
            if args.show_images:
                cv2.imshow('ArUco', annotated)
                cv2.waitKey(1)
            if args.save_images_dir:
                cv2.imwrite(os.path.join(args.save_images_dir, f"frame_{processed:06d}.png"), annotated)
            return
        idx = ids_present.index(world_marker)
        _, rvec, tvec = detections[idx]
        pos_cam, R_world = pose_camera_in_marker(rvec, tvec)
        cam_poses.append((pos_cam, R_world))
        # Store homography inverse for world mosaic
        R_ct, _ = cv2.Rodrigues(rvec)
        H = K @ np.column_stack((R_ct[:, 0], R_ct[:, 1], tvec.reshape(3)))
        try:
            H_inv = np.linalg.inv(H)
            mosaic_items.append((frame.copy(), H_inv))
        except np.linalg.LinAlgError:
            pass
        # Show/save annotated
        if args.show_images:
            cv2.putText(annotated, f"Anchor ID {world_marker}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0), 2)
            cv2.imshow('ArUco', annotated)
            cv2.waitKey(1)
        if args.save_images_dir:
            cv2.imwrite(os.path.join(args.save_images_dir, f"frame_{processed:06d}.png"), annotated)

    if isinstance(frames_iter, list):
        for path in frames_iter:
            frame = cv2.imread(path)
            if frame is None:
                continue
            process_frame(frame)
            processed += 1
            if args.max_frames > 0 and processed >= args.max_frames:
                break
    else:
        # Video/Webcam iterator
        while True:
            ret, frame = frames_iter.read()
            if not ret:
                break
            process_frame(frame)
            processed += 1
            if args.max_frames > 0 and processed >= args.max_frames:
                break

    # Save simple (image-space) overlay for reference
    overlay_img = None
    if overlay is not None:
        overlay_img = cv2.cvtColor(overlay.astype(np.uint8), cv2.COLOR_BGR2RGB)
        cv2.imwrite('overlay.jpg', overlay_img)

    # Build world-aligned mosaic on Z=0 plane if we have homographies
    overlay_world_rgb = None
    x_min = y_min = x_max = y_max = None
    if mosaic_items:
        # Compute plane extents by mapping image corners
        all_pts = []
        for img, H_inv in mosaic_items:
            h, w = img.shape[:2]
            corners_img = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
            ones = np.ones((4, 1), dtype=np.float32)
            pts_h = np.hstack((corners_img, ones)) @ H_inv.T
            pts_plane = (pts_h[:, :2] / pts_h[:, 2:3])  # (X, Y) in world units (cm)
            all_pts.append(pts_plane)
        all_pts = np.vstack(all_pts)
        x_min, y_min = np.min(all_pts, axis=0)
        x_max, y_max = np.max(all_pts, axis=0)
        # Scale: pixels per cm
        px_per_cm = 2.0
        Wm = int(max(100, np.ceil((x_max - x_min) * px_per_cm)))
        Hm = int(max(100, np.ceil((y_max - y_min) * px_per_cm)))
        # Translation from world to mosaic pixels
        T = np.array([[px_per_cm, 0, -x_min * px_per_cm],
                      [0, px_per_cm, -y_min * px_per_cm],
                      [0, 0, 1]], dtype=np.float32)
        accum = np.zeros((Hm, Wm, 3), dtype=np.float32)
        weight = np.zeros((Hm, Wm), dtype=np.float32)
        for img, H_inv in mosaic_items:
            M = T @ H_inv.astype(np.float32)
            warped = cv2.warpPerspective(img, M, (Wm, Hm), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            mask = cv2.warpPerspective(np.ones(img.shape[:2], dtype=np.float32), M, (Wm, Hm), flags=cv2.INTER_NEAREST)
            accum += warped.astype(np.float32)
            weight += mask
        weight3 = np.clip(weight, 1e-6, None)[:, :, None]
        overlay_world_rgb = (accum / weight3).astype(np.uint8)
        cv2.imwrite('overlay_world.jpg', cv2.cvtColor(overlay_world_rgb, cv2.COLOR_BGR2RGB))

    # Export CSV
    if cam_poses:
        traj = np.array([p[0] for p in cam_poses])
        try:
            np.savetxt(args.csv_out, traj, delimiter=',', header='x,y,z', comments='')
            print(f'Saved trajectory CSV: {args.csv_out} ({traj.shape[0]} poses)')
        except Exception as e:
            print(f'Failed to write CSV: {e}')

    # Visualization
    if cam_poses:
        if args.vis == 'plotly':
            fig = go.Figure()
            positions = np.array([pose[0] for pose in cam_poses])
            fig.add_trace(go.Scatter3d(
                x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
                mode='markers', marker=dict(size=5, color='red'),
                name=f'Camera Positions ({source_desc})'
            ))
            # Origin
            fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', marker=dict(size=5, color='black'), name='Origin'))
            # Orientation (forward as blue cones)
            arrow_length = 10
            for pos, rot in cam_poses:
                forward = rot @ np.array([0, 0, 1])
                fig.add_trace(go.Cone(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    u=[forward[0] * arrow_length], v=[forward[1] * arrow_length], w=[forward[2] * arrow_length],
                    sizemode='absolute', sizeref=arrow_length / 2, showscale=False,
                    colorscale=[[0, 'blue'], [1, 'blue']]
                ))
            # Floor overlay using world-aligned mosaic if available
            if overlay_world_rgb is not None and x_min is not None:
                pil_img = Image.fromarray(cv2.cvtColor(overlay_world_rgb, cv2.COLOR_BGR2RGB))
                # downscale for plotting speed
                pil_img = pil_img.resize((min(200, pil_img.width), int(min(200, pil_img.width) * pil_img.height / pil_img.width)), Image.Resampling.LANCZOS)
                img_array = np.array(pil_img)
                intensity = img_array.mean(axis=2)
                x = np.linspace(x_min, x_max, img_array.shape[1])
                y = np.linspace(y_min, y_max, img_array.shape[0])
                z = np.zeros((img_array.shape[0], img_array.shape[1]))
                fig.add_trace(go.Surface(x=x, y=y, z=z, surfacecolor=intensity, colorscale='gray', showscale=False, opacity=1.0))
            fig.update_layout(title='ArUco Camera Poses and Floor Overlay (Z=0)', scene=dict(aspectmode='cube'))
            fig.show()
        else:
            import pyvista as pv
            plotter = pv.Plotter()
            # Points
            positions = np.array([pose[0] for pose in cam_poses])
            if len(positions) > 0:
                cloud = pv.PolyData(positions)
                plotter.add_points(cloud, color='red', point_size=8)
            # Camera axes as arrows
            arrow_length = 10
            colors = ['red', 'green', 'blue']
            for pos, rot in cam_poses:
                for i, color in enumerate(colors):
                    dir_vec = rot[:, i] * arrow_length
                    arrow = pv.Arrow(start=pos, direction=dir_vec, tip_length=0.3, shaft_radius=0.05)
                    plotter.add_mesh(arrow, color=color)
            # Add world-aligned mosaic as textured plane if available
            if overlay_world_rgb is not None and x_min is not None:
                texture = pv.Texture(cv2.cvtColor(overlay_world_rgb, cv2.COLOR_BGR2RGB))
                # Plane centered at mosaic center, sized by extents
                center = ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 0.0)
                size_i = float(x_max - x_min)
                size_j = float(y_max - y_min)
                plane = pv.Plane(center=center, i_size=size_i, j_size=size_j, direction=(0, 0, 1))
                plotter.add_mesh(plane, texture=texture)
            # Show
            plotter.show()
    else:
        print('No positions detected.')

if __name__ == '__main__':
    main()