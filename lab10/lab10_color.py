import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    # h, w, c = frame.shape
    if ret == False:
        print("Cannot read video")
        break
    

    # for i in range(h):
    #     for j in range(w):
    #         print()
    cv2.imshow('Result', edges_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
