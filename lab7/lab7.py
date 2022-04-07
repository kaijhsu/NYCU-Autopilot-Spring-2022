

import cv2
import numpy as np
import time
import math
from djitellopy import Tello
from pyimagesearch.pid import PID


def keyboard(self, key):
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        # is_flying = True
    if key == ord('2'):
        self.land()
        # is_flying = False
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) * ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) * degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print(battery)


def main():
    drone = Tello()
    drone.connect()
    time.sleep(10)
    global isFliying
    fs = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_READ)
    intrinsic = fs.getNode("intrinsic").mat()
    distortion = fs.getNode("distortion").mat()

    z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    y_pid = PID(kP=0.7, kI=0.01, kD=0.1)
    yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.2)

    z_pid.initialize()
    y_pid.initialize()
    yaw_pid.initialize()

    maxSpeed = 40

    while (True):
        #drone.send_rc_control(0, 0, 0, 0)
        #drone.send_rc_control(0, 0, 0, 0)
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        cv2.imshow("frame", frame)
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=parameters)
        frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            markerCorners, 15, intrinsic, distortion)
        z_update = 0
        y_update = 0
        #yaw_update = 0
        yaw_update_deg = 0
        if 0 in markerIds:
            idx_0 =markerIds.tolist().index([0])
            try:
                frame = cv2.aruco.drawAxis(
                    frame, intrinsic, distortion, rvec[idx_0], tvec[idx_0], 5)
                tvec_str = "x=%4.0f y=%4.0f z=%4.0f" % (
                    tvec[idx_0][0][0], tvec[idx_0][0][1], tvec[idx_0][0][2])
                rvec_str = "x=%4.0f y=%4.0f z=%4.0f" % (
                    rvec[idx_0][0][0], rvec[idx_0][0][1], rvec[idx_0][0][2])
                # cv2.putText(frame, tvec_str, (20, 460),
                #             cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
                #cv2.putText(frame, rvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
                print("s")
                z_update = tvec[idx_0, 0, 2] - 100
                z_update = z_pid.update(z_update, sleep=0)
                if z_update > maxSpeed:
                    z_update = maxSpeed
                elif z_update < -maxSpeed:
                    z_update = -maxSpeed

                y_update = -tvec[idx_0, 0, 1] - 6
                y_update = y_pid.update(y_update, sleep=0)
                if y_update > maxSpeed:
                    y_update = maxSpeed
                if y_update < 0:
                    y_update *= 2
                if y_update > 0:
                    y_update *= 2
                elif y_update < -maxSpeed:
                    y_update = -maxSpeed

                cv2.putText(frame, str(y_update), (20, 460),
                            cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5, cv2.LINE_AA)

                yaw_update_deg = math.degrees(rvec[idx_0][0][2])
                yaw_update_deg = yaw_pid.update(yaw_update_deg, sleep=0)
                if yaw_update_deg > maxSpeed:
                    yaw_update_deg = maxSpeed
                elif yaw_update_deg < -maxSpeed:
                    yaw_update_deg = -maxSpeed

            except:
                print("tvec_0:")
                print(tvec[idx_0])

        key = cv2.waitKey(1)
        if key == -1:
            drone.send_rc_control(0, int(z_update), int(
                y_update), int(yaw_update_deg))
            print("no input")
        else:
            keyboard(drone, key)

        cv2.imshow("frame", frame)
        cv2.waitKey(30)


if __name__ == "__main__":
    main()
