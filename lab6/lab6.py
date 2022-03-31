from djitellopy import Tello
from keyboard_djitellopy import keyboard
import time
import cv2

drone = Tello()
drone.connect()

time.sleep(5)

while True:
    

    drone.streamon()
    frame = drone.get_frame_read()
    frame = frame.frame
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key != -1:
        keyboard(drone, key);