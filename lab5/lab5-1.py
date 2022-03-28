import cv2
import numpy as np
import cv2.aruco

cap = cv2.VideoCapture(1)

while(True):
    ret, frame = cap.read()
    if ret == False:
        print("Cannot read video")
        break
    # Load the predefined dictionary
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    # Initialize the detector parameters using default values
    parameters = cv2.aruco.DetectorParameters_create()
    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
        frame, dictionary, parameters=parameters)

    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    #Pose estimation for single markers. 
    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion) 

    cv2.imshow('frame', frame)
    cv2.waitKey(33)
