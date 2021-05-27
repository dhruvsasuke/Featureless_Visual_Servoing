import cv2
import numpy as np

x_centre = 319.5
y_centre = 239.5
scalingFactor = 5000.0
focalLength = 938.0

def generate_pointcloud(bin_img, depth_map):
    global x_centre, y_centre, scalingFactor, focalLength
    pixels = (bin_img[:,:,0]>0).nonzero()
    depth = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY)
    points = []
    for i in range(len(pixels[0])):
        Z = depth[pixels[0][i], pixels[1][i]] / scalingFactor
        X = (pixels[0][i] - x_centre) * Z / focalLength
        Y = (pixels[1][i] - y_centre) * Z / focalLength
        # print(X,Y,Z)
        points.append((X,Y,Z))
    return points
