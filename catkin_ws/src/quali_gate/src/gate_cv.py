import cv2
import numpy as np
import math

image = cv2.imread('gate.png')
redMasked = image.copy()
redMasked[:,:,0:2] = 0
gray = cv2.cvtColor(redMasked,cv2.COLOR_BGR2GRAY)
gray = gray*(gray>50)
canny = cv2.Canny(gray,50,150)

lines = cv2.HoughLinesP(canny,rho=6,theta=np.pi/180,threshold=60,minLineLength=200,maxLineGap=70)
hough = image.copy()
frameCenterX = np.size(canny,1)/2
frameCenterY = np.size(canny,0)/2
cv2.circle(hough,(int(frameCenterX),int(frameCenterY)),5,(0,0,255),-1)

if lines is not None:
    linesnp = np.array(lines[:,0,:])
    delx = np.abs(linesnp[:,0]-linesnp[:,2])
    dely = np.abs(linesnp[:,1]-linesnp[:,3])
    horizontal = linesnp[dely<50]
    vertical = linesnp[delx<50]
    leftRef = np.min(vertical[:,0],axis=0)
    left = vertical[vertical[:,0] < leftRef+50]
    rightRef = np.max(vertical[:,0],axis=0)
    right = vertical[vertical[:,0] > rightRef-50]
    topAvg = np.average(horizontal,axis=0)
    leftAvg = np.average(left, axis=0)
    rightAvg = np.average(right, axis=0)
    lines = np.array([topAvg,leftAvg,rightAvg]).tolist()
    bottomY = (leftAvg[1]+rightAvg[1])/2
    topY = (topAvg[1]+topAvg[3])/2
    avgY = (bottomY+topY)/2
    print(leftAvg[2])
    avgX = (leftAvg[0]+leftAvg[2]+rightAvg[0]+rightAvg[2])/4
    cv2.circle(hough,(int(avgX),int(avgY)),5,(0,255,0),-1) 
    print('Center of gate is at: ', avgX, ',', avgY)
    print('Center of frame is at ', frameCenterX, ',', frameCenterY)

if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i]
            cv2.line(hough, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (0,150,0), 3, cv2.LINE_AA)

cv2.imshow('Main',image)
cv2.imshow('Hough',hough)
cv2.waitKey(0)
cv2.destroyAllWindows()