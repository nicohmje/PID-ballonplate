import numpy as np 
center_x = 250
center_y = 240 

x = 300
y = 300

def CartesianToPolar(x,y):
    center_x = 250
    center_y = 240
    r = np.sqrt((center_x - x)**2 + (center_y - y)**2)
    theta_1 = np.arccos((x-center_x) / r)
    theta_2 = np.arcsin((y-center_y)/ r)

    if x > center_x:
        if y > center_y:
            theta = theta_1
        else:
            theta = 2*np.pi - abs(theta_2)
    else:
        if y > center_y:
            theta = theta_1
        else:
            theta = np.pi + abs(theta_2)

    theta = (180 / np.pi) * theta 
    return(r,theta)

print(CartesianToPolar(x,y))