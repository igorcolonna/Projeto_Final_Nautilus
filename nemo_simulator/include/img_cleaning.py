import cv2
import numpy as np

def cleaning(image, hsv_min, hsv_max):
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,hsv_min, hsv_max)

    reversemask = 255-mask

    detector = cv2.SimpleBlobDetector(reversemask)

    keypoints = detector.detect(reversemask)

    return keypoints, reversemask

def get_nemo_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])

    center_x    = 0.5*cols
    center_y    = 0.5*rows

    x = (keyPoint.pt[0] - center_x)/(center_x)
    y = (keyPoint.pt[1] - center_y)/(center_y)
    return(x,y)


