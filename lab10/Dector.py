import numpy as np
import cv2
# from numba import njit


# @njit
def scanColor(frame):
    height, width = frame.shape
    count = 0
    for y in range(height):
        for x in range(width):
            count += frame[y, x]
    return count

def gridDetector(edges_frame, threshold):


    # cv2.imshow("frame", edges_frame)
    # cv2.waitKey(10)

    height, width = edges_frame.shape

    grids = []
    stepHeight = int(height/3)
    stepWidth = int(width/3)
    for y in range(3):
        for x in range(3):
            y1 = y*stepHeight
            y2 = y1+stepHeight
            x1 = x*stepWidth
            x2 = x1 + stepWidth
            grids.append( edges_frame[y1:y2, x1:x2] )
    detect_list = []

    for grid in grids:
        detect_list.append(scanColor(grid))

    for i in range(len(detect_list)):
        detect_list[i] = 1 if detect_list[i] >= threshold else 0

    # detect_list = np.reshape(detect_list, (3,3))
    return detect_list


def main():

    cap = cv2.VideoCapture(0)
    while(True):
        ret, frame = cap.read()
        if ret == False:
            continue
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gridDetector(frame, 3000)


if __name__ == '__main__':
    main()
