import cv2
import numpy as np

def runPipeline(image, llrobot):
    contoursRects = []
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels 
    # that do not fall within the following HSV Min/Max values
    img_threshold = cv2.inRange(img_hsv, (6, 142, 0, 0), (28, 239, 255, 0))
    
    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for points in contours:
        contours2f = np.array(points, dtype=np.float32)
        rect = cv2.minAreaRect(contours2f)
        contoursRects.append(rect)

    biggestRect = None
    for rect in contoursRects:
        if rect is not None:
            if biggestRect is None or (rect[1][0] * rect[1][1] > biggestRect[1][0] * biggestRect[1][1] and rect[1][0] * rect[1][1] > 1000):
                biggestRect = rect
        
    rectPoints = []
    if biggestRect is not None:
        rectPoints = cv2.boxPoints(biggestRect) 
        # Get the 4 points of the rectangle
        matOfPoint = np.int0(rectPoints) 
        # Convert to integer type 
        # Draw the rectangle on the input image 
        cv2.polylines(input, [matOfPoint], isClosed=True, color=[0, 255, 0, 0], thickness=3)
    
    llpython = rectPoints
    # initialize an empty array of values to send back to the robot
    # 
    # # if contours have been detected, draw them 
    # if len(contours) > 0:
    #     cv2.drawContours(image, contours, -1, 255, 2)
    #             
    #     # record the largest contour
    #     largestContour = max(contours, key=cv2.contourArea)
    # 
    #     # get the unrotated bounding box that surrounds the contour
    #     x,y,w,h = cv2.boundingRect(largestContour)
    # 
    #     # draw the unrotated bounding box
    #     cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
    # 
    #     # record some custom data to send back to the robot
    #     llpython = [x,y,w,h]  

    #return the biggest rect, the modified image, and the rect points
    return biggestRect, image, llpython