import cv2 as cv
import numpy as np

def nothing(x):
    pass

video = cv.VideoCapture(0)

cv.namedWindow("Trackbars") 
cv.createTrackbar("L-H", "Trackbars", 110, 255, nothing)
cv.createTrackbar("L-S", "Trackbars", 50, 255, nothing)
cv.createTrackbar("L-V", "Trackbars", 50, 255, nothing)
cv.createTrackbar("U-H", "Trackbars", 130, 255, nothing)
cv.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

font = cv.FONT_HERSHEY_COMPLEX

while True:
    _, frame = video.read()
    
    hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    l_h = cv.getTrackbarPos("L-H","Trackbars")
    l_s = cv.getTrackbarPos("L-S","Trackbars")
    l_v = cv.getTrackbarPos("L-V","Trackbars")
    
    u_h = cv.getTrackbarPos("U-H","Trackbars")
    u_s = cv.getTrackbarPos("U-S","Trackbars")
    u_v = cv.getTrackbarPos("U-V","Trackbars")
    
    #blue_color_lower = np.array([110,50,50])    
    #blue_color_upper= np.array([130,255,255])

    #blue_color_lower = np.array([85,128,70])    
    #blue_color_upper= np.array([155,255,255])

    blue_color_lower = np.array([l_h,l_s,l_v])
    blue_color_upper= np.array([u_h,u_s,u_v])
    
    mask = cv.inRange(hsv,blue_color_lower,blue_color_upper)
    kernel = np.ones((5,5), np.uint8)
    mask = cv.erode(mask, kernel)
    #detectamos contornos
    contours, y = cv.findContours(mask , cv.RETR_TREE , cv.CHAIN_APPROX_SIMPLE ) 
    
    for cnt in contours:
        area = cv.contourArea(cnt)
        approx = cv.approxPolyDP(cnt, 0.01*cv.arcLength(cnt,True), True)
        x = approx.ravel()[0]
        ye = approx.ravel()[1]
        if area > 4000:
            cv.drawContours(frame,[approx],0,(0,0,0),5)
            if len(approx) == 4:
                cv.putText(frame,"Rectangle",(x,ye),font,1,(0,0,0))
    
    cv.imshow("frame",frame)
    cv.imshow("hsv",hsv)
    cv.imshow("mask",mask)
    
    key = cv.waitKey(1)
    if key ==27:
        break
    
video.release()
cv.destroyAllWindows()
