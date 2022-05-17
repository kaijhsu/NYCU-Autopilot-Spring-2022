import cv2
import numpy as np
from djitellopy import Tello

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
        print("battery:  "+str(battery))
        print("battery:  "+str(battery))
        print("battery:  "+str(battery))


def main():
        
    cap = cv2.VideoCapture(0)
    stage = 0
    Right = True
    Up = True
    while(True):
        ret, frame = cap.read()
        # h, w, c = frame.shape
        if ret == False:
            print("Cannot read video")
            break
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_range = np.array([101, 50, 38])
        upper_range = np.array([110, 255, 255])
        mask = cv2.inRange(hsv_frame, lower_range, upper_range)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (13, 13), 0)
        edges_frame = cv2.Canny(blur_gray, 30, 70)
        # dilation = cv2.dilate(edges_frame, (7, 7), iterations=3)
        # erosion = cv2.erode(dilation, (7, 7), iterations=3)
        lines = cv2.HoughLines(edges_frame, 1, 3.14159 / 180, 60)
        # print(lines)
        cnt = [0, 0]
        if lines is not None:
            print(len(lines))
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                if x2 == x1:
                    cnt[1] += 1
                else:
                    ratio = (y2-y1)/(x2-x1)
                    if -1 < ratio <= 1:
                        cnt[0] += 1
                    else:
                        cnt[1] += 1
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        # horizontal
        if cnt[0] > 0 and cnt[1] == 0:
            if stage == 3:
                stage = 4
            if stage == 7:
                stage = 8
                Up = False
            if Right == True:
                ...
                # drone.move_right(20)
            else:
                ...
                # drone.move_left(20)
                # corner
        elif cnt[0] > 0 and cnt[1] > 0:
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
                ...
                # drone.move_up(20)
            else:
                ...
                # drone.move_down(20)
                # vertical
        elif cnt[0] == 0 and cnt[1] > 0:
            if stage == 1:
                stage = 2
            if stage == 5:
                stage = 6
                Right = False
            if stage == 9:
                stage = 10
            if Up == True:
                ...
                # drone.move_up(20)
            else:
                ...
                # drone.move_down(20)
        else:
            ...
            # drone.move_backward(20)
        cv2.imshow('Frame', frame)
        cv2.imshow('Result', edges_frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()