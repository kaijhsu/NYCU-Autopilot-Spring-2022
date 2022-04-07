import cv2
import numpy as np
import cv2.aruco
from djitellopy import Tello
import time



# cap = cv2.VideoCapture(0)  # device
drone = Tello()
drone.connect()
time.sleep(10)


patternSize = (9, 6)
objpoints = []
imgpoints = []
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()


f = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_READ)
mtx = f.getNode("intrinsic").mat()
dist = f.getNode("distortion").mat()

f.release()
while(True):
    drone.streamon()
    frame = drone.get_frame_read()
    frame = frame.frame
    #ret, frame = cap.read()
    #if ret == False:
    #    print("Cannot read video")
    #    break
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    frame = cv2.aruco.drawDetectedMarkers(frame,markerCorners, markerIds)
    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners,15, mtx, dist)
    if rvec is None:
        cv2.imshow('frame', frame)
        cv2.waitKey(33)
        continue
    frame = cv2.aruco.drawAxis(frame, mtx,dist, rvec, tvec, 0.5)
    string = f"x: {np.round(tvec[0][0][0], 3)}  y: {np.round(tvec[0][0][1], 3)} z: {np.round(tvec[0][0][2], 3)}"

    centerX = int((markerCorners[0][0][0][0] + markerCorners[0][0][1][0] + markerCorners[0][0][2][0] + markerCorners[0][0][3][0])/4)
    centerY = int((markerCorners[0][0][0][1] + markerCorners[0][0][1][1] + markerCorners[0][0][2][1] + markerCorners[0][0][3][1])/4)
    coordinate = centerX+15, centerY-15
    cv2.putText(frame, string, coordinate, cv2.FONT_HERSHEY_DUPLEX,
                0.5, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    cv2.waitKey(33)
