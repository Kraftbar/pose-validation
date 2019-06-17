"""
Saves a series of snapshots with the current camera as snapshot_<width>_<height>_<nnn>.jpg

Arguments:
    --f <output folder>     default: current folder
    --n <file name>         default: snapshot
    --w <width px>          default: none
    --h <height px>         default: none

Buttons:
    q           - quit
    space bar   - save the snapshot
    
  
"""

import cv2
import time
import sys
import argparse
import os


from  camerapose import ArucoSingleTracker

__author__ = "Tiziano Fiorenzani"
__date__ = "01/06/2018"



def main():
    # ---- DEFAULT VALUES ---
    SAVE_FOLDER = "."
    FILE_NAME = "snapshot"
    FRAME_WIDTH = 0
    FRAME_HEIGHT = 0

    # ----------- PARSE THE INPUTS -----------------
    parser = argparse.ArgumentParser(
        description="Saves snapshot from the camera. \n q to quit \n spacebar to save the snapshot")
    parser.add_argument("--folder", default=SAVE_FOLDER, help="Path to the save folder (default: current)")
    parser.add_argument("--name", default=FILE_NAME, help="Picture file name (default: snapshot)")
    parser.add_argument("--dwidth", default=FRAME_WIDTH, type=int, help="<width> px (default the camera output)")
    parser.add_argument("--dheight", default=FRAME_HEIGHT, type=int, help="<height> px (default the camera output)")
    parser.add_argument("--raspi", default=False, type=bool, help="<bool> True if using a raspberry Pi")
    args = parser.parse_args()

    SAVE_FOLDER = args.folder
    FILE_NAME = args.name
    FRAME_WIDTH = args.dwidth
    FRAME_HEIGHT = args.dheight


    # ---------- Calibration matrix ---------------
    calibrationFile = "../camera.xml"
    calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
    camera_matrix = calibrationParams.getNode("camera_matrix").mat()
    camera_distortion = calibrationParams.getNode("distortion_coefficients").mat()





    #--- Get the camera calibration path
    aruco_tracker = ArucoSingleTracker(id_to_find=1, marker_size=35, show_video=True,savefolder="logsAndCode", camera_matrix=camera_matrix, camera_distortion=camera_distortion)
    
    aruco_tracker.track(verbose=False)
    aruco_tracker.stop()

    print("Files saved")

if __name__ == "__main__":
    main()



