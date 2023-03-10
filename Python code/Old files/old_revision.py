from cmath import asin
from re import X
import numpy as np 
import cv2 as cv
import time
import tkinter as tk
from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important
kit.servo[2].set_pulse_width_range(500, 2500) #Very important

center_x = 240
center_y = 240
prevX = 0
prevY = 0
X_integral = 0
Y_integral = 0
X_derivative = 0
Y_derivative = 0


def motor_coeff(theta):
    motor_pos = np.array([90,330,210])
    erreur = np.array([0,0,0])
    erreur = (theta - motor_pos + 180 + 360) % 360 - 180

    dir = np.ones(3)
    dir[abs(erreur)>90] = - 1 #If dir is -1, the motors should lower | Else, they should go up 

    coeffs = np.ones(3)
    if theta > 180:
        theta_inverse = theta - 180
    else:
        theta_inverse = theta + 180 

    coeffs[dir<0] = 1 - abs((theta - motor_pos[dir < 0] + 180 + 360) % 360 - 180) / 90 
    coeffs[dir>0] = 1 - abs((theta_inverse - motor_pos[dir > 0] + 180 + 360) % 360 - 180) / 90 

    coeffs = coeffs * dir

    return(coeffs)

def moveMotors(theta, radius):
    intensity = (radius / 200) * 30
    initial_angle = np.array([120,120,110])
    coeffs = motor_coeff(theta)
    motor_angles = (coeffs * intensity * -1) + initial_angle 
    
    if max(motor_angles)>170:
        motor_angles[np.argmax(motor_angles)] = 170
        print("angle too high")
        return
    if min(motor_angles)<80: 
        motor_angles[np.argmin(motor_angles)] = 80
        print("angle too low")
        return
    for i in np.arange(0,3):
        kit.servo[i].angle = motor_angles[i]

def initialPos():
    kit.servo[0].angle = 120
    kit.servo[1].angle = 120
    kit.servo[2].angle = 110

cap = cv.VideoCapture(0)

def CartesianToPolar(x,y, xcenter, ycenter):
    
    r = np.sqrt((xcenter - x)**2 + (ycenter - y)**2)
    theta_1 = np.arccos((x-xcenter) / r)
    theta_2 = np.arcsin((y-ycenter)/ r)

    if x > xcenter:
        if y > ycenter:
            theta = theta_1
        else:
            theta = 2*np.pi - abs(theta_2)
    else:
        if y > ycenter:
            theta = theta_1
        else:
            theta = np.pi + abs(theta_2)

    theta = (180 / np.pi) * theta 
    return(r,theta)



def ballFinder():
    _, frame = cap.read()
    frame = frame[:, 93:550, :]
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_value = np.array([00,100,100])
    higher_value = np.array([50,255,255])
    mask = cv.inRange(hsv, lower_value, higher_value)
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    result = (0,0)
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 1500:
            (x,y), radius = cv.minEnclosingCircle(cnt)
            x = int(x)
            y = 480 - int(y)
            radius = int(radius)
            if radius > 20:
                result = (x,y)
    return(result)

def CenterCalibration():
    for i in np.arange(0,150):
        _, frame = cap.read()
        frame = frame[:,93:550,:]
        cv.circle(frame,(center_x,center_y),3,[255,130,130],3)
        cv.imshow("Calibration", frame)
        cv.waitKey(1)



def pid(x,y):
    global center_x
    global center_y
    global Kp, Ki, Kd
    global X_integral, Y_integral, X_derivative, Y_derivative
    Kp = 0.4
    Ki = 0.0025
    Kd = 5
    if abs(Ki*X_integral) > 400:
        Ki = 0 
    if abs(Ki*Y_integral) > 400:
        Ki = 0 
    pid_X = Kp * (center_x - x) + Ki * X_integral + Kd * X_derivative
    pid_Y = Kp * (center_y - y) + Ki * Y_integral + Kd * Y_derivative
    
        
    return (pid_X, pid_Y)

initialPos()
CenterCalibration()
cv.destroyAllWindows()
while True:
    x, y = ballFinder()
    #x,y = 250, 400
    #start = time.time()
    if x != y != 0:
        futureX, futureY = pid(x,y)
        radius, theta = CartesianToPolar(futureX, futureY, 0,0)
        moveMotors(theta, radius)
        X_derivative = prevX - x
        Y_derivative = prevY - y
        X_integral += (center_x - x)
        Y_integral += (center_y - y)
        prevX = x
        prevY = y
        
        #print(theta, radius)
    else :
        initialPos()
        X_integral = 0
        Y_integral = 0 
    #stop = time.time()
    #print(stop - start)

cap.release()

