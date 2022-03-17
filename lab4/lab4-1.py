import cv2

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    # ret, corner = 

    cv2.imshow('frame', frame)
    cv2.waitKey(33)