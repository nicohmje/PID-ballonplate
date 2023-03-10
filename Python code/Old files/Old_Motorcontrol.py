import numpy as np
from adafruit_servokit import ServoKit
import time


kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important
kit.servo[2].set_pulse_width_range(500, 2500) #Very important

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
    intensity = (radius / 150) * 25
    print(intensity)
    initial_angle = np.array([120,120,110])
    coeffs = motor_coeff(theta)
    motor_angles = (coeffs * intensity * -1) + initial_angle 
    print(motor_angles)
    if max(motor_angles)>160:
        print("angle too high")
        return
    if min(motor_angles)<80: 
        print("angle too low")
        return
    for i in np.arange(0,3):
        kit.servo[i].angle = motor_angles[i]

def initialPos():
    kit.servo[0].angle = 120
    kit.servo[1].angle = 120
    kit.servo[2].angle = 110

moveMotors(266, 160)

time.sleep(10)
initialPos()





