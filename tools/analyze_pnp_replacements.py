import argparse
import json
import math
from dataclasses import dataclass

import cv2
import numpy as np

from gt_trace_common import load_gt_poses, umeyama_alignment, apply_alignment


@dataclass
class PnPFrame:
    frame_id: int
    fx: float
    fy: float
    cx: float
    cy: float
    predicted_t: np.ndarray
    pure_ok: bool
    pure_inliers: int
    pure_t: np.ndarray
    pure_rmse: float
    lm_t: np.ndarray
    lm_rmse: float
    image: np.ndarray
    world: np.ndarray
    obs: np.ndarray


def parse_dump(path):
    frames = []
    with open(path, "r") as handle:
        tokens = iter(handle.read().split())
    for token in tokens:
        if token != "FRAME":
            raise ValueError(f"Expected FRAME, got {token}")
        frame_id = int(next(tokens))
        fx = float(next(tokens))
        fy = float(next(tokens))
        cx = float(next(tokens))
        cy = float(next(tokens))
        _count = int(next(tokens))
        predicted_t = np.array([float(next(tokens)), float(next(tokens)), float(next(tokens))])
        pure_ok = bool(int(next(tokens)))
        pure_inliers = int(next(tokens))
        pure_t = np.array([float(next(tokens)), float(next(tokens)), float(next(tokens))])
        pure_rmse = float(next(tokens))
        lm_t = np.array([float(next(tokens)), float(next(tokens)), float(next(tokens))])
        lm_rmse = float(next(tokens))

        image = []
        world = []
        obs = []
        token = next(tokens)
        while token != "END":
            if token != "C":
                raise ValueError(f"Expected C or END, got {token}")
            image.append([float(next(tokens)), float(next(tokens))])
            world.append([float(next(tokens)), float(next(tokens)), float(next(tokens))])
            obs.append(int(next(tokens)))
            token = next(tokens)
        frames.append(
            PnPFrame(
                frame_id=frame_id,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                predicted_t=predicted_t,
                pure_ok=pure_ok,
                pure_inliers=pure_inliers,
                pure_t=pure_t,
                pure_rmse=pure_rmse,
                lm_t=lm_t,
                lm_rmse=lm_rmse,
                image=np.asarray(image, dtype=np.float32),
                world=np.asarray(world, dtype=np.float32),
                obs=np.asarray(obs, dtype=np.int32),
            )
        )
    return frames


def camera_center(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    t = np.asarray(tvec, dtype=float).reshape(3)
    return -(R.T @ t)


def score_pose(frame, rvec, tvec):
    K = np.array([[frame.fx, 0.0, frame.cx], [0.0, frame.fy, frame.cy], [0.0, 0.0, 1.0]])
    projected, _ = cv2.projectPoints(frame.world, rvec, tvec, K, None)
    projected = projected.reshape(-1, 2)
    err = np.linalg.norm(projected - frame.image, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    z = (R[2, :] @ frame.world.T) + float(np.asarray(tvec).reshape(3)[2])
    center = camera_center(rvec, tvec)
    jump = float(np.linalg.norm(np.asarray(tvec).reshape(3) - frame.predicted_t))
    return {
        "cv_inl2": int(np.sum(err < 2.0)),
        "cv_inl3": int(np.sum(err < 3.0)),
        "cv_inl5": int(np.sum(err < 5.0)),
        "cv_mederr": float(np.median(err)) if err.size else math.inf,
        "cv_posz": float(np.mean(z > 0.1)) if z.size else 0.0,
        "cv_jump": jump,
        "cv_center": center,
    }


def ate_errors(est, gt):
    R, t, scale = umeyama_alignment(est, gt, with_scale=True)
    aligned = apply_alignment(est, R, t, scale)
    errors = np.linalg.norm(aligned - gt, axis=1)
    return errors, float(np.sqrt(np.mean(errors ** 2)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", required=True)
    parser.add_argument("--metrics", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--window", default=None, help="Optional START:END frame filter")
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--bottom", type=int, default=10)
    parser.add_argument("--sweep_rules", action="store_true", help="Rank multi-frame replacement rules")
    args = parser.parse_args()

    start = end = None
    if args.window:
        start_text, end_text = args.window.split(":", 1)
        start, end = int(start_text), int(end_text)

    with open(args.metrics, "r") as handle:
        metrics = json.load(handle)
    timeline = metrics["timeline"]
    frame_ids = np.array([frame["frame_id"] for frame in timeline], dtype=int)
    est = np.array([frame["xyz"] for frame in timeline], dtype=float)
    _gt_poses, gt_centers = load_gt_poses(args.gt)
    valid = np.array([i for i, frame_id in enumerate(frame_ids) if 0 <= frame_id < len(gt_centers)], dtype=int)
    est_valid = est[valid].copy()
    gt_valid = gt_centers[frame_ids[valid]]
    frame_to_valid = {int(frame_ids[i]): out_i for out_i, i in enumerate(valid)}

    base_errors, base_rmse = ate_errors(est_valid, gt_valid)
    frames = parse_dump(args.dump)
    rows = []
    for frame in frames:
        if start is not None and not (start <= frame.frame_id <= end):
            continue
        if frame.frame_id not in frame_to_valid or len(frame.world) < 6:
            continue
        K = np.array([[frame.fx, 0.0, frame.cx], [0.0, frame.fy, frame.cy], [0.0, 0.0, 1.0]])
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            frame.world,
            frame.image,
            K,
            None,
            iterationsCount=100,
            reprojectionError=3.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_AP3P,
        )
        if not ok:
            continue
        score = score_pose(frame, rvec, tvec)
        idx = frame_to_valid[frame.frame_id]
        replaced = est_valid.copy()
        replaced[idx] = score["cv_center"]
        repl_errors, repl_rmse = ate_errors(replaced, gt_valid)
        rows.append(
            {
                "frame": frame.frame_id,
                "base_err": float(base_errors[idx]),
                "repl_err": float(repl_errors[idx]),
                "frame_delta": float(base_errors[idx] - repl_errors[idx]),
                "rmse_delta": float(base_rmse - repl_rmse),
                "base_rmse": base_rmse,
                "repl_rmse": repl_rmse,
                "pure_ok": frame.pure_ok,
                "pure_inl": frame.pure_inliers,
                "idx": idx,
                **{key: value for key, value in score.items() if key != "cv_center"},
                "cv_center": score["cv_center"],
            }
        )

    if args.sweep_rules:
        rules = []
        for pure_mode in ("any", "pure_failed", "pure_ok"):
            for min_inl2 in (0, 3, 6, 9, 12, 16, 24, 32):
                for min_gain in (-20, -10, -5, 0, 5, 10, 20):
                    for max_mederr in (5, 10, 15, 30, 60, 120, 300, math.inf):
                        for min_posz in (0.0, 0.5, 0.8, 0.95):
                            for max_jump in (5000, 20000, 50000, 150000, 500000, math.inf):
                                selected = []
                                for row in rows:
                                    if pure_mode == "pure_failed" and row["pure_ok"]:
                                        continue
                                    if pure_mode == "pure_ok" and not row["pure_ok"]:
                                        continue
                                    if row["cv_inl2"] < min_inl2:
                                        continue
                                    if row["cv_inl2"] - row["pure_inl"] < min_gain:
                                        continue
                                    if row["cv_mederr"] > max_mederr:
                                        continue
                                    if row["cv_posz"] < min_posz:
                                        continue
                                    if row["cv_jump"] > max_jump:
                                        continue
                                    selected.append(row)
                                if not selected:
                                    continue
                                replaced = est_valid.copy()
                                for row in selected:
                                    replaced[row["idx"]] = row["cv_center"]
                                _errors, rmse = ate_errors(replaced, gt_valid)
                                rules.append(
                                    {
                                        "delta": base_rmse - rmse,
                                        "rmse": rmse,
                                        "count": len(selected),
                                        "pure_mode": pure_mode,
                                        "min_inl2": min_inl2,
                                        "min_gain": min_gain,
                                        "max_mederr": max_mederr,
                                        "min_posz": min_posz,
                                        "max_jump": max_jump,
                                        "frames": ",".join(str(row["frame"]) for row in selected[:20]),
                                    }
                                )
        rules.sort(key=lambda rule: rule["delta"], reverse=True)
        print(f"base_rmse {base_rmse:.6f}")
        print("rank delta rmse count pure_mode min_inl2 min_gain max_mederr min_posz max_jump frames")
        for rank, rule in enumerate(rules[: args.top], start=1):
            max_mederr = "inf" if math.isinf(rule["max_mederr"]) else f"{rule['max_mederr']:.0f}"
            max_jump = "inf" if math.isinf(rule["max_jump"]) else f"{rule['max_jump']:.0f}"
            print(
                f"{rank:4d} {rule['delta']:.6f} {rule['rmse']:.6f} {rule['count']:5d} "
                f"{rule['pure_mode']} {rule['min_inl2']:8d} {rule['min_gain']:8d} "
                f"{max_mederr:>10} {rule['min_posz']:.2f} {max_jump:>8} {rule['frames']}"
            )
        return

    rows.sort(key=lambda row: row["rmse_delta"], reverse=True)
    winners = [row for row in rows if row["rmse_delta"] > 0.0]
    print(f"base_rmse {base_rmse:.6f}")
    print(f"candidates {len(rows)}")
    print(f"rmse_winners {len(winners)}")
    print("top_rmse_winners")
    print("frame base_err repl_err frame_delta rmse_delta pure_ok pure_inl cv_inl2 cv_inl3 cv_mederr cv_posz cv_jump")
    for row in rows[: args.top]:
        print(
            f"{row['frame']:5d} {row['base_err']:.4f} {row['repl_err']:.4f} "
            f"{row['frame_delta']:.4f} {row['rmse_delta']:.6f} {int(row['pure_ok'])} "
            f"{row['pure_inl']:4d} {row['cv_inl2']:4d} {row['cv_inl3']:4d} "
            f"{row['cv_mederr']:.4f} {row['cv_posz']:.4f} {row['cv_jump']:.3f}"
        )
    if args.bottom:
        print("bottom_rmse_losers")
        print("frame base_err repl_err frame_delta rmse_delta pure_ok pure_inl cv_inl2 cv_inl3 cv_mederr cv_posz cv_jump")
        for row in sorted(rows, key=lambda item: item["rmse_delta"])[: args.bottom]:
            print(
                f"{row['frame']:5d} {row['base_err']:.4f} {row['repl_err']:.4f} "
                f"{row['frame_delta']:.4f} {row['rmse_delta']:.6f} {int(row['pure_ok'])} "
                f"{row['pure_inl']:4d} {row['cv_inl2']:4d} {row['cv_inl3']:4d} "
                f"{row['cv_mederr']:.4f} {row['cv_posz']:.4f} {row['cv_jump']:.3f}"
            )


if __name__ == "__main__":
    main()
