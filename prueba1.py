import cv2
import numpy as np

import warnings

warnings.filterwarnings("ignore")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_yellow = np.array([20, 50, 50])
        # upper_yellow = np.array([40, 255, 255])
        # mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_green = np.array([40, 30, 30])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

        img_green = cv2.medianBlur(res_green, 3)
        ccimg_green = cv2.cvtColor(img_green, cv2.COLOR_HSV2BGR)
        cimg_green = cv2.cvtColor(ccimg_green, cv2.COLOR_BGR2GRAY)

        circles_green = cv2.HoughCircles(cimg_green, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
        if circles_green is not None:
            print("green circle is found")
            circles_green = np.uint16(np.around(circles_green))
            for i in circles_green[0, :]:
                cv2.circle(cimg_green, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cimg_green, (i[0], i[1]), 2, (0, 0, 255), 3)
        cv2.imshow('detected circles green', cimg_green)
        cv2.imshow('res_green', res_green)
        

        lower_red = np.array([0,100,100])
        upper_red = np.array([5,255,255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)                
        res_red = cv2.bitwise_and(frame, frame, mask=mask_red)

        img_red = cv2.medianBlur(res_red, 3)
        ccimg_red = cv2.cvtColor(img_red, cv2.COLOR_HSV2BGR)
        cimg_red = cv2.cvtColor(ccimg_red, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(cimg_red, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
        if circles is not None:
            print("red circle is found")
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(cimg_red, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cimg_red, (i[0], i[1]), 2, (0, 0, 255), 3)
        cv2.imshow('detected circles red', cimg_red)
        cv2.imshow('res_red', res_red)
    else:
        print("Read Failed")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()