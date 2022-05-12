import numpy as np
import cv2
from numba import njit

@njit
def isBlue(hsv_pixel):
    H = hsv_pixel[0]
    S = hsv_pixel[1]
    V = hsv_pixel[2]
    return True if H >= 180 and H <= 280 else False

    # return True if H >= 180 and H <= 280 and V >= 20 and S >= 30 else False

@njit
def scanColor(hsv_frame):
    height, width, channels = hsv_frame.shape
    count = 0
    for y in range(height):
        for x in range(width):
            if isBlue(hsv_frame[y, x]):
                count += 1
    return count

def gridBlueDetector(hsv_frame):
    height, width, channels = hsv_frame.shape

    hsv_grids = []
    stepHeight = int(height/3)
    stepWidth = int(width/3)
    for y in range(3):
        for x in range(3):
            y1 = y*stepHeight
            y2 = y1+stepHeight
            x1 = x*stepWidth
            x2 = x1 + stepWidth
            hsv_grids.append( hsv_frame[y1:y2, x1:x2] )
    detect_list = []
    for hsv_grid in hsv_grids:
        cnt = scanColor(hsv_grid)
        print(cnt)
    
    print(detect_list)

def main():

    cap = cv2.VideoCapture(0)
    while(True):
        ret, frame = cap.read()
        if ret == False:
            continue
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gridBlueDetector(hsv_frame)
        cv2.imshow("frame", frame)
        cv2.waitKey(10)


if __name__ == '__main__':
    main()
    