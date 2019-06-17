import os
import cv2
from cv2 import aruco
import numpy as np


###
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
plt.ion()
plt.show()

##
calibrationFile = "camera.xml"
calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
camera_matrix = calibrationParams.getNode("camera_matrix").mat()
dist_coeffs = calibrationParams.getNode("distortion_coefficients").mat()
print(camera_matrix)
print(dist_coeffs )
#r = calibrationParams.getNode("R").mat()
#new_camera_matrix = calibrationParams.getNode("newCameraMatrix").mat()
#cv.InitUndistortMap(cameraMatrix, distCoeffs, map1, map2)
aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_250 )
markerLength = 22 # Here, our measurement unit is centimetre.
arucoParams = aruco.DetectorParameters_create()

## main
cap = cv2.VideoCapture(0)

i=0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    imgRemapped_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)





    corners, ids, rejectedImgPoints = aruco.detectMarkers(imgRemapped_gray, aruco_dict, parameters=arucoParams) # Detect aruco
##
    grayshow = aruco.drawDetectedMarkers(imgRemapped_gray, corners)
    print(ids)
##
    if ids != None: # if aruco marker detected
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker

    #####
        R =cv2.Rodrigues(rvec)  
        camR = np.transpose(R[0])
        R =cv2.Rodrigues(camR)
        camTvec= -camR * tvec; # calculate your camera translation vector
        print(tvec[0][0][0],"asdasd")
        print(tvec,"asdasd")
    #####        
        i=i+1


        
        ax.scatter(tvec[0][0][0],tvec[0][0][1], tvec[0][0][2  ],c='r', marker= 'o')
        plt.draw()
        plt.pause(0.001)

    ###
        imgWithAruco = aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0))
        imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
    else:   # if aruco marker is NOT detected
        imgWithAruco = frame  # assign imRemapped_color to imgWithAruco directly

    cv2.imshow("aruco", imgWithAruco)   # display
    
    
    cv2.imshow('frame',grayshow)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


