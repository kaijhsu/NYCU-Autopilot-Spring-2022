import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    # h, w, c = frame.shape
    if ret == False:
        print("Cannot read video")
        break
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_range = np.array([110, 50, 50])
    upper_range = np.array([130, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_range, upper_range)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    # for i in range(h):
    #     for j in range(w):
    #         print()
    cv2.imshow('Result', result)
    cv2.waitKey(0)
