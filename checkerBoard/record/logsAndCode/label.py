import cv2
import numpy as np
import os

import cv2
import csv

import re 



# global constants
img = None
tl_list = []
br_list = []
object_list = []

# constants
image_folder = 'logRun6'
savedir = 'annotations'
obj = 'fidget_spinner'

w=640
h=480
calibrationFile = "../../camera.xml"
calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
mtx = calibrationParams.getNode("camera_matrix").mat()
dist = calibrationParams.getNode("distortion_coefficients").mat()
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
print(mtx)
print(newcameramtx)
print(roi)
def helperConv(input): 
  
     # get a list of all numbers separated by  
     # lower case characters  
     # \d+ is a regular expression which means 
     # one or more digit 
     # output will be like ['100','564','365'] 
     numbers = re.findall('\d+',input) 
  
     # now we need to convert each number into integer 
     # int(string) converts string into integer 
     # we will map int() function onto all elements  
     # of numbers list 
     numbers = map(int,numbers) 
  
     return (max(numbers)-3193)/24

def drawCircBouy(image,n):
    radius=10
    with open('LABELDATAbouy.txt') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for index,row in enumerate(csv_reader):
            if(index==(n)):
                row = row[:-1]
                row = list(map(int, row))
                if(len(row)==2):
                    cv2.circle(image,(row[0],row[1]), radius, (0,255,0), 1)
                if(len(row)==4):
                    cv2.circle(image,(row[0],row[1]), radius, (0,255,0), 1)
                    cv2.circle(image,(row[2],row[3]), radius, (0,255,0), 1)
                if(len(row)==6):
                    cv2.circle(image,(row[0],row[1]), radius, (0,255,0), 1)
                    cv2.circle(image,(row[2],row[3]), radius, (0,255,0), 1)
                    cv2.circle(image,(row[4],row[5]), radius, (0,255,0), 1)
    return image


def drawCircCage(image,n):
    radius=30
    with open('LABELDATAcage.txt') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for index,row in enumerate(csv_reader):
            if(index==(n)):
                row = row[:-1]
                row = list(map(int, row))
                if(len(row)==2):
                    cv2.circle(image,(row[0],row[1]), radius, (255,0,0), 1)
                if(len(row)==4):
                    cv2.circle(image,(row[0],row[1]), radius, (255,0,0), 1)
                    cv2.circle(image,(row[2],row[3]), radius, (255,0,0), 1)
    return image

def mouseRGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = image[y,x,0]
        colorsG = image[y,x,1]
        colorsR = image[y,x,2]
        colors = image[y,x]
        print(x,",",y, end =",") 


cv2.namedWindow('mouseRGB')

cv2.setMouseCallback('mouseRGB',mouseRGB)
#Do until esc pressed

if __name__ == '__main__': 

    for n, image_file in enumerate(  sorted(os.scandir(image_folder), key=lambda x: (x.is_dir(), x.name))   ):
        
        # filenum=helperConv(image_file.path)

        image = cv2.imread(image_file.path)
        image   = cv2.undistort(image, mtx, dist, None, newcameramtx)


        while 1:
            cv2.imshow('mouseRGB',image)

            if cv2.waitKey(20) & 0xFF == 27:
                print("")
                break

cv2.destroyAllWindows()
