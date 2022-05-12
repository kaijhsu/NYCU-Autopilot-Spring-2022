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

    frame = cv2.cvtColor(result,cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(15, 15), 0)
    edges_frame = cv2.Canny(blur_gray, 30, 70)

    # for i in range(h):
    #     for j in range(w):
    #         print()
    cv2.imshow('Result', edges_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
