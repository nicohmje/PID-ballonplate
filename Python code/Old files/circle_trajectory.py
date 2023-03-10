import json
import numpy as np

def CircleTrajCoord():
    global circle_coord_x, circle_coord_y
    circle_coord_x = []
    circle_coord_y = []
    radius = 150
    x = np.arange(0,481)
    y = np.arange(0,481)
    center = 240
    for i in x:
        for j in y:
            if ((i-center)**2 + (j-center)**2 == 150**2):
                print(i,j)

CircleTrajCoord()

240,198 ,150 ,120 ,96 ,90  ,96 ,120 ,150 ,198 ,240 ,282 ,330 ,360 ,384 ,390 ,384 ,360 ,330 ,282 

90,96,120,150,198,240,282,330,360,384,360,330,282,240,198,150,120,96
