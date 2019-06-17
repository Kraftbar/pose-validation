    
import cv2
import time
import sys
import argparse
import os

def save_snaps(cap,nSnap,  folder =".",width=0, height=0,  name="snapshot"):
    print(folder)
    try:
        if not os.path.exists(folder):
            os.makedirs(folder)
            # ----------- CREATE THE FOLDER -----------------
            folder = os.path.dirname(folder)

            try:
                os.stat(folder)
            except:
                os.mkdir(folder)
                w       = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                h       = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                ret, frame = cap.read()
                fileName    = "%s/%s_%d_%d_" %(folder, name, w, h)
                print("Saving image ", nSnap)
                cv2.imwrite("%s%d.png"%(fileName, nSnap), frame)
    except:
        pass


    w       = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h       = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    ret, frame = cap.read()

    fileName    = "%s/%s_%d_%d_" %(folder, name, w, h)
    print("Saving image ", nSnap)
    cv2.imwrite("%s%d.png"%(fileName, nSnap), frame)





