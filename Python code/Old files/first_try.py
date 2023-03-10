#149, 5
#149,423
#624, 432
#640, 0 
import time
import numpy as np 
import cv2 as cv
from adafruit_servokit import ServoKit
kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important

cap = cv.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

radii = []
start = time.time()
for f in np.arange(0,200,1):
    ret, frame = cap.read()
    frame = frame[5:435, 165:635, :]
    blurredframe = cv.medianBlur(frame, 3)
    hsv = cv.cvtColor(blurredframe, cv.COLOR_BGR2HSV)
    lower_value1 = np.array([00,100,20])
    higher_value1 = np.array([30,255,255])
    mask = cv.inRange(hsv, lower_value1, higher_value1)
    #lower_value2 = np.array([160,100,20])
    #higher_value2 = np.array([179,255,255])
    #mask2 = cv.inRange(hsv, lower_value2, higher_value2)
    #mask = cv.bitwise_or(mask1, mask2)
    res = cv.bitwise_and(frame, frame, mask)
    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, 2, 500, param1=300,param2=10,minRadius=25,maxRadius=60)
    if type(circles) == np.ndarray and circles.size != 0:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            print(i[1])
            kit.servo[0].angle = ((i[1] - 30) / 410) * 180
    cv.imshow("Feed", res)
    k = cv.waitKey(2)
    if k == 27:
        break
stop = time.time()
print(200 / (stop - start))
cap.release()
cv.destroyAllWindows()