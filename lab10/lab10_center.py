import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)

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
    lines = cv2.HoughLinesP(edges_frame, 1, np.pi / 180, 60)
    xsum = 0
    ysum = 0
    weight_sum = 0
    weighted_pos_sum = np.array([0, 0], dtype=np.int32)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            point1 = np.array([x1, y1], dtype=np.int32)
            point2 = np.array([x2, y2], dtype=np.int32)
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
            # 用線的長度當作權重
            weight = np.linalg.norm(point1 - point2)
            # 所有權重的總和
            weight_sum += weight
            # 用線中點代表一條線
            pos = np.add(point1, point2)/2
            # (線中點*權重)的總和
            weighted_pos_sum = np.add(pos * weight, weighted_pos_sum)
        # 找出中心點
        tmp = (weighted_pos_sum / weight_sum).astype(int)
        center = (tmp[0], tmp[1])
        print("center", center)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

    cv2.imshow('Frame', frame)
    cv2.imshow('Result', blur_gray)
    cv2.waitKey(1)


# import cv2
# import numpy as np


# cap = cv2.VideoCapture(0)

# while(True):
#     ret, frame = cap.read()
#     # h, w, c = frame.shape
#     if ret == False:
#         print("Cannot read video")
#         break
#     hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     lower_range = np.array([101, 50, 38])
#     upper_range = np.array([110, 255, 255])
#     mask = cv2.inRange(hsv_frame, lower_range, upper_range)
#     result = cv2.bitwise_and(frame, frame, mask=mask)

#     img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
#     gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
#     blur_gray = cv2.GaussianBlur(gray, (13, 13), 0)
#     edges_frame = cv2.Canny(blur_gray, 30, 70)
#     lines = cv2.HoughLines(edges_frame, 1, 3.14159 / 180, 60)
#     cnt = [0, 0]
#     if lines is not None:
#         print(len(lines))
#         for line in lines:
#             rho, theta = line[0]
#             a = np.cos(theta)
#             b = np.sin(theta)
#             x0 = a*rho
#             y0 = b*rho
#             x1 = int(x0 + 1000*(-b))
#             y1 = int(y0 + 1000*(a))
#             x2 = int(x0 - 1000*(-b))
#             y2 = int(y0 - 1000*(a))
#             if x2 == x1:
#                 cnt[1] += 1
#             else:
#                 ratio = (y2-y1)/(x2-x1)
#                 if -1 < ratio <= 1:
#                     cnt[0] += 1
#                 else:
#                     cnt[1] += 1
#             cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
#     cv2.imshow('Result', blur_gray)
#     cv2.waitKey(1)
