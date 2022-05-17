import cv2
import numpy as np
import time
import math
from djitellopy import Tello
from pyimagesearch.pid import PID
from Dector import lineDetector
from KeyBoard import keyboard




def main():
    drone = Tello()
    drone.connect()
    time.sleep(5)
    global isFliying
    fs = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_READ)
    intrinsic = fs.getNode("intrinsic").mat()
    distortion = fs.getNode("distortion").mat()

    z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    y_pid = PID(kP=0.7, kI=0.001, kD=0.1)
    yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.2)
    x_pid = PID(kP=0.7, kI=0.001, kD=0.1)


    z_pid.initialize()
    y_pid.initialize()
    x_pid.initialize()
    yaw_pid.initialize()

    maxSpeed = 40
    stage = -1
    Right = True
    Up = True
    lower_range = np.array([101, 50, 38])
    upper_range = np.array([110, 255, 255])

    while (True):
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_range, upper_range)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        frameb = cv2.cvtColor(result,cv2.COLOR_HSV2RGB)
        gray = cv2.cvtColor(frameb, cv2.COLOR_RGB2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(9, 9), 0)
        edges_frame = cv2.Canny(blur_gray, 30, 70)
        fixframe = frame
        cntarray,fixframe = lineDetector(edges_frame,fixframe)

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=parameters)
        fixframe = cv2.aruco.drawDetectedMarkers(fixframe, markerCorners, markerIds)
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            markerCorners, 15, intrinsic, distortion)
        x_update = 0
        z_update = 0
        y_update = 0
        yaw_update_deg = 0
        max_x_fix = 15

        if markerIds is not None and stage == -1 :
            if 0 in markerIds :
                idx_0 = markerIds.tolist().index([0])
                if tvec[idx_0,0,2] < 80 and tvec[idx_0,0,1] < 20:
                    stage = 0
                    time.sleep(5)
                else:
                    fixframe = cv2.aruco.drawAxis(
                        frame, intrinsic, distortion, rvec[idx_0], tvec[idx_0], 5)

                    z_update = tvec[idx_0, 0, 2] - 70
                    z_update = z_pid.update(z_update, sleep=0)
                    if z_update > maxSpeed:
                        z_update = maxSpeed
                    elif z_update < -maxSpeed:
                        z_update = -maxSpeed

                    height = drone.get_height()
                    delta_height = int(height - 140)
                    if height > 160:
                        if abs(delta_height) > 20 :
                            if delta_height < 0:
                                drone.move_up(-1*delta_height)
                            else:
                                drone.move_down(delta_height)
                    else:
                        y_update = -tvec[idx_0, 0, 1] - 10
                        y_update = y_pid.update(y_update, sleep=0)
                        if y_update < 0:
                            y_update *= 2
                        if y_update > maxSpeed:
                            y_update = maxSpeed
                        elif y_update < -maxSpeed:
                            y_update = -maxSpeed

                    # cv2.putText(frame, str(x_update), (20, 460),
                    #             cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5, cv2.LINE_AA)
        if stage != -1:
            if cntarray[0] > 0 and cntarray[1] == 0:
                if stage == 3:
                    stage = 4
                if stage == 7:
                    stage = 8
                    Up = False
                if Right == True:
                    drone.move_right(20)
                else:
                    drone.move_left(20)
                    # corner
            elif cntarray[0] > 0 and cntarray[1] > 0:
                if stage == 0:
                    stage = 1
                if stage == 2:
                    stage = 3
                if stage == 4:
                    stage = 5
                if stage == 6:
                    stage = 7
                if stage == 8:
                    stage = 9
                if Up == True:
                    drone.move_up(20)
                else:
                    drone.move_down(20)
                    # vertical
            elif cntarray[0] == 0 and cntarray[1] > 0:
                if stage == 1:
                    stage = 2
                if stage == 5:
                    stage = 6
                    Right = False
                if stage == 9:
                    stage = 10
                if Up == True:
                    drone.move_up(20)
                else:
                    drone.move_down(20)
            else:
                drone.move_back(20)
        cv2.putText(frame, str(cntarray)+str(stage), (20, 460),
                    cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5, cv2.LINE_AA)
        # cv2.imshow("edge frame",edges_frame)
        cv2.imshow('frame', fixframe)
        key = cv2.waitKey(1)
        if key == -1:
            y_update = min(y_update, 30)
            drone.send_rc_control(int(x_update), int(z_update), int(
                y_update), int(yaw_update_deg))
            print("no input")
        else:
            keyboard(drone, key)

        # cv2.imshow("frame", frame)
        cv2.waitKey(33)

if __name__ == "__main__":
    main()
