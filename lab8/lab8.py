import cv2
import dlib 

cap = cv2.VideoCapture(-1)
detector = dlib.get_frontal_face_detector()
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def main():
    while True:
        ret, frame = cap.read()
        if ret != True :
            print("can't read!")
            exit(1)

        #detect face
        face_rects = detector(frame, 0)
        for i, d in enumerate(face_rects):
            x1 = d.left()
            y1 = d.top()
            x2 = d.right()
            y2 = d.bottom()
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
        
        #detect body
        rects, weights = hog.detectMultiScale(frame, winStride=(4, 4), scale=1.1, useMeanshiftGrouping = False)
        for rect in rects:
            x1 = rect[0]
            y1 = rect[1]
            x2 = x1 + rect[2]
            y2 = x2 + rect[3]
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)


        cv2.imshow("cap", frame)
        key = cv2.waitKey(1)
        if key != -1:
            cv2.destroyAllWindows()
            break

main()