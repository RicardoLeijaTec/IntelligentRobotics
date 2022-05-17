import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()

l_h = 0
l_s  = 0
l_v  = 0
u_h = 0
u_s = 0
u_v = 0

def hsv_handler(data):
	global l_h, l_s, l_v, u_h, u_s, u_v
	min_max_split = data.data.split("|")
	#print(data.data)
	#print(min_max_split)
	min_hsv = min_max_split[0]
	#print(min_hsv_array)
	max_hsv = min_max_split[1]
	#print(max_hsv_array)
	min_hsv_array = min_hsv.split(",")
	max_hsv_array = max_hsv.split(",")
	#print(min_hsv_array)
	#print(max_hsv_array)
	l_h = int(min_hsv_array[0])
	l_s = int(min_hsv_array[1])
	l_v = int(min_hsv_array[2])
	u_h = int(max_hsv_array[0])
	u_s = int(max_hsv_array[1])
	u_v = int(max_hsv_array[2])
	#print(l_h, l_s, l_v, u_h, u_s, u_v)
	

def nothing(x):
    	pass


def handler(data):
	global l_h, l_s, l_v, u_h, u_s, u_v
	frame = bridge.imgmsg_to_cv2(data, "bgr8")
	# Grayscale encoding applied to the initial image
	#gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

	# Blur the image for better edge detection
	#img_blur = cv.GaussianBlur(hsv, (5,5),cv.BORDER_DEFAULT)
	# resize = cv.resize(frame, (288,162)) # Image resizing

	#lower_green = np.array([40, 30, 30])
        #upper_green = np.array([80, 255, 255])
	lower_green = np.array([50, 40, 40])
        upper_green = np.array([70, 255, 255])
        mask_green = cv.inRange(hsv, lower_green, upper_green)
        res_green = cv.bitwise_and(frame, frame, mask=mask_green)
	resize = cv.resize(res_green, (288,162)) # Image resizing

        img_green = cv.medianBlur(res_green, 3)
        ccimg_green = cv.cvtColor(img_green, cv.COLOR_HSV2BGR)
        cimg_green = cv.cvtColor(ccimg_green, cv.COLOR_BGR2GRAY)
	resize_cimg = cv.resize(cimg_green, (288,162)) # Image resizing

        circles_green = cv.HoughCircles(cimg_green, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
        if circles_green is not None:
            print("green circle is found")
	'''
            circles_green = np.uint16(np.around(circles_green))
            for i in circles_green[0, :]:
                cv.circle(cimg_green, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv.circle(cimg_green, (i[0], i[1]), 2, (0, 0, 255), 3)
        #cv.imshow('detected circles green', resize_cimg)'''
        cv.imshow('res_green', resize)
        
	
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        mask_red = cv.inRange(hsv, lower_red, upper_red)                
        res_red = cv.bitwise_and(frame, frame, mask=mask_red)
	resize_red = cv.resize(res_red, (288,162)) # Image resizing

        img_red = cv.medianBlur(res_red, 3)
        ccimg_red = cv.cvtColor(img_red, cv.COLOR_HSV2BGR)
        cimg_red = cv.cvtColor(ccimg_red, cv.COLOR_BGR2GRAY)
	resize_cimg_red = cv.resize(cimg_red, (288,162)) # Image resizing

        circles = cv.HoughCircles(cimg_red, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
        if circles is not None:
		print("red circle is found")
	'''
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			cv.circle(cimg_red, (i[0], i[1]), i[2], (0, 255, 0), 2) # draw the outer circle
			cv.circle(cimg_red, (i[0], i[1]), 2, (0, 0, 255), 3) # draw the center of the circle
	#cv.imshow('detected circles red', resize_cimg_red)'''
	cv.imshow('res_red', resize_red)
	#else:
		#print("Read Failed")
	cv.waitKey(3)
	
	cv_image_to_imgmsg = bridge.cv2_to_imgmsg(resize, encoding="passthrough")
	image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
	image_publisher.publish(cv_image_to_imgmsg)


def opencv_listener():
	rospy.init_node('image_listener', anonymous = True)
	rospy.Subscriber("/video_source/raw", Image, handler)
	rospy.Subscriber("/color_publisher", String, hsv_handler)
	rate = rospy.Rate(30)
	#rospy.spin()
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == "__main__":
	try:
		opencv_listener()
	except rospy.ROSInterruptException:
		pass
