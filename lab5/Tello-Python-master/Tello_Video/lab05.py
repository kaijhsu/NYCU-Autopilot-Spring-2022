from cv2 import VideoCapture
import tello
import cv2
import time
import numpy as np


def main():
    drone = tello.Tello('', 8889)
    time.sleep(10)
    while(True):
        frame = drone.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Lab5-1
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=parameters)
        frame = cv2.aruco.drawDetectedMarkers(
            frame, markerCorners, markerIds)

        # Lab5-1 + 4-1
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            markerCorners, 15, mtx, dist)

        cv2.imshow("drone", frame)
        cv2.waitKey(1)
        key = cv2.waitKey(1)
        if key != -1:
            drone.keyboard(key)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
