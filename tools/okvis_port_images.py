#!/usr/bin/env python3
"""okvis_port_images.py <seq> [--cams cam0,cam1] [--root DIR] [--keep-png]

Prepares the inputs of the native-BRISK replay (check_ok_frontend) and of the end-to-end C system (check_ok_system,
okvis_c_euroc): tools/check_okvis_port.py --data ROOT. Fetches the EuRoC camera folders and imu0 with
tools/vio_harness/fetch_seq_stream.py into ROOT/<seq>/mav0 (own directory, default external/vio/data/okvis_brisk_tmp),
decodes every PNG with the reference's OpenCV 4.6 (cv::imread IMREAD_GRAYSCALE, okvis_port/reference_tools/okvis_png2gray.cc)
into ROOT/<seq>/gray/cam<i>.gray and deletes the PNGs again (about 1.3 GB per camera for MH_01_easy stays; mav0/imu0 is kept).
EuRoC licence: non-commercial, never commit images.
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OCV = ROOT / "external/vio/deps/opencv"
BUILD = ROOT / "runs/okvis_port/c_build"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("--cams", default="cam0,cam1")
    ap.add_argument("--root", default=str(ROOT / "external/vio/data/okvis_brisk_tmp"))
    ap.add_argument("--keep-png", action="store_true")
    a = ap.parse_args()
    root = Path(a.root)
    cams = a.cams.split(",")
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / "okvis_png2gray"
    subprocess.run(["g++", "-O2", "-std=c++17", str(ROOT / "okvis_port/reference_tools/okvis_png2gray.cc"),
                    f"-I{OCV}/include/opencv4", f"-L{OCV}/lib", "-lopencv_imgcodecs", "-lopencv_core",
                    f"-Wl,-rpath,{OCV}/lib", "-o", str(exe)], check=True)
    mav = root / a.seq / "mav0"
    gray = root / a.seq / "gray"
    if all((gray / f"{c}.gray").exists() for c in cams) and (mav / "imu0/data.csv").exists():
        print(f"images ready: {gray}")
        return 0
    if not all((mav / c / "data").is_dir() for c in cams) or not (mav / "imu0/data.csv").exists():
        env = dict(os.environ, VIO_DATA_ROOT=str(root))
        subprocess.run([sys.executable, str(ROOT / "tools/vio_harness/fetch_seq_stream.py"), a.seq, ",".join(cams + ["imu0"])],
                       check=True, env=env)
    gray.mkdir(parents=True, exist_ok=True)
    for c in cams:
        subprocess.run([str(exe), str(mav / c / "data"), str(gray / f"{c}.gray")], check=True)
    if not a.keep_png:
        for c in cams:
            shutil.rmtree(mav / c)
    print(f"images ready: {gray}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
