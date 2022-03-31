import math
from djitellopy import Tello
from keyboard_djitellopy import keyboard
from pyimagesearch.pid import PID
from pyimagesearch.objcenter import ObjCenter
import time
import cv2
import numpy as np
from numpy.linalg import norm

drone = Tello()
drone.connect()

global is_flying

fs = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_READ)
intrinsic = fs.getNode("intrinsic").mat()
distortion = fs.getNode("distortion").mat()
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

z_pid = PID(kP = 0.7, kI = 0.0001, kD = 0.1)
y_pid = PID(kP = 0.7, kI = 0.0001, kD = 0.1)
yaw_pid = PID(kP = 0.7, kI = 0.0001, kD = 0.1)

yaw_pid.initialize()
z_pid.initialize()
y_pid.initialize()

time.sleep(5)



while True:
    
    
    drone.streamon()
    frame = drone.get_frame_read()
    frame = frame.frame

    key = cv2.waitKey(5)
    if key != -1:
        keyboard(drone, key);
    
    # # detect marker
    # markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    # frame = cv2.aruco.drawDetectedMarkers(frame,markerCorners, markerIds)
    # rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners,15, intrinsic, distortion)
    # if rvec is None:
    #     cv2.imshow('frame', frame)
    #     cv2.waitKey(5)
    #     continue

    # #get update_z
    # R = rvec[0][0][2]
    # Z = np.multiply(R, [0,0,1])
    # update_z = tvec[0][0][2] - 100
    # update_z = z_pid.update(update_z, sleep=0)
    # update_z = min(40, update_z)
    # update_z = max(-40, update_z)

    # update_y = tvec[0][0][1]
    # update_y = y_pid.update(update_y, sleep=0)
    
    # dst = cv2.Rodrigues(rvec)
    # dst = np.degrees(dst[0][0][2])
    # angle = dst


    # frame = cv2.aruco.drawAxis(frame, intrinsic,distortion, rvec, tvec, 0.5)
    # string = f"x: {np.round(tvec[0][0][0], 3)}  y: {np.round(tvec[0][0][1], 3)} z: {np.round(tvec[0][0][2], 3)}"
    # string2 = f"{dst}"

    # centerX = int((markerCorners[0][0][0][0] + markerCorners[0][0][1][0] + markerCorners[0][0][2][0] + markerCorners[0][0][3][0])/4)
    # centerY = int((markerCorners[0][0][0][1] + markerCorners[0][0][1][1] + markerCorners[0][0][2][1] + markerCorners[0][0][3][1])/4)
    # coordinate = centerX+15, centerY-15
    # coordinate2 = centerX+15, centerY+30
    # cv2.putText(frame, string, coordinate, cv2.FONT_HERSHEY_DUPLEX,
    #             0.5, (0, 255, 255), 1, cv2.LINE_AA)
    # cv2.putText(frame, string2, coordinate2, cv2.FONT_HERSHEY_DUPLEX,
    #             0.5, (0, 255, 255), 1, cv2.LINE_AA)


    
    
    # angle = yaw_pid.update(angle, sleep=0)
    # angle = int(angle)
    # if angle > 5:
    #     drone.rotate_counter_clockwise(angle)
    # elif angle < -5:
    #     drone.rotate_clockwise(-1*angle)
    # else:
    # drone.send_rc_control(0, int(update_z//2), int(-1*update_y//2), angle)

    # print(f"raw zdistance: {tvec[0][0][2]},    update_z: {update_z}")
    cv2.imshow('frame', frame)
    