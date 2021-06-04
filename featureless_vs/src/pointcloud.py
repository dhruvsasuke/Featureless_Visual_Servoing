import cv2
import numpy as np

x_centre = 319.5
y_centre = 239.5
# scalingFactor = 273.46
focalLength = 531.0 

def generate_pointcloud(bin_img, depth):
    global x_centre, y_centre, scalingFactor, focalLength
    pixels = (bin_img[:,:,0]>0).nonzero()
    points = []
    for i in range(len(pixels[0])):
        Z = (depth[pixels[0][i], pixels[1][i]])
        X = (pixels[0][i] - x_centre) * Z / focalLength
        Y = (pixels[1][i] - y_centre) * Z / focalLength
        # print(X,Y,Z)
        points.append((X,Y,Z))
    return points
