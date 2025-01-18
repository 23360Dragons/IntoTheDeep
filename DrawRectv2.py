import cv2
import numpy as np

def runPipeline(image, llrobot):
    lower_hsv = {6, 142, 0 , 0}
    upper_hsv = {28, 239, 255, 0}
    
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels 
    # that do not fall within the following HSV Min/Max values
    img_threshold = cv2.inRange(img_hsv, lower_hsv, upper_hsv)
    
    return _, image, llrobot