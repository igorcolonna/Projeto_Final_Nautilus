import cv2
import numpy as np

def cleaning(image, hsv_min, hsv_max):
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,hsv_min, hsv_max)

    reversemask = 255-mask

    # params = cv2.SimpleBlobDetector_Params()

    # # Change thresholds
    # params.minThreshold = 0;
    # params.maxThreshold = 100;
        
    # # Filter by Area.
    # params.filterByArea = True
    # params.minArea = 30
    # params.maxArea = 20000
        
    # # Filter by Circularity
    # params.filterByCircularity = True
    # params.minCircularity = 0.1
        
    # # Filter by Convexity
    # params.filterByConvexity = True
    # params.minConvexity = 0.5
        
    # # Filter by Inertia
    # params.filterByInertia =True
    # params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(None)

    keypoints = detector.detect(reversemask)
    for i, KeyPoints in enumerate(keypoints):
        



    return keypoints, reversemask

def get_nemo_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])

    center_x    = 0.5*cols
    center_y    = 0.5*rows

    x = (keyPoint.pt[0] - center_x)/(center_x)
    y = (keyPoint.pt[1] - center_y)/(center_y)
    return(x,y)


