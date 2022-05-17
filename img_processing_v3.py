import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge

bridge = CvBridge()
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True) # Publisher al nodo de velocidades
found_circles = [0,0] # [0] = green, [1] = red

def nothing(x):
    	pass


def handler(data):
    global cmd_vel_pub
    global found_circles
    v_to_cmd_vel = Twist()

	frame = bridge.imgmsg_to_cv2(data, "bgr8")
	hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
	#img_blur = cv.GaussianBlur(hsv, (5,5),cv.BORDER_DEFAULT) # Blur the image for better edge detection
	lower_green = np.array([50, 70, 70])
	upper_green = np.array([70, 255, 255])
	mask_green = cv.inRange(hsv, lower_green, upper_green)
	res_green = cv.bitwise_and(frame, frame, mask=mask_green)
	resize = cv.resize(res_green, (288,162)) # Image resizing

	img_green = cv.medianBlur(res_green, 3)
	ccimg_green = cv.cvtColor(img_green, cv.COLOR_HSV2BGR)
	cimg_green = cv.cvtColor(ccimg_green, cv.COLOR_BGR2GRAY)
	circles_green = cv.HoughCircles(cimg_green, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
	if circles_green is not None:
        found_circles[0] = found_circles[0] + 1
		print("green circle is found", found_circles[0])
		v_to_cmd_vel.linear.x = 0.8
		cmd_vel_pub.publish(v_to_cmd_vel)
	cv.imshow('res_green', resize)
        
	lower_red_1 = np.array([0,100,20])
	upper_red_1 = np.array([8,255,255])
	lower_red_2 = np.array([175,100,20])
	upper_red_2 = np.array([179,255,255])
	lower_mask_red = cv.inRange(hsv, lower_red_1, upper_red_1)
	upper_mask_red = cv.inRange(hsv, lower_red_2, upper_red_2)
	full_red_mask = cv.add(lower_mask_red, upper_mask_red)

	res_red = cv.bitwise_and(frame, frame, mask=full_red_mask)
	resize_red = cv.resize(res_red, (288,162)) # Image resizing

	img_red = cv.medianBlur(res_red, 3)
	ccimg_red = cv.cvtColor(img_red, cv.COLOR_HSV2BGR)
	cimg_red = cv.cvtColor(ccimg_red, cv.COLOR_BGR2GRAY)
	circles = cv.HoughCircles(cimg_red, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
	if circles is not None:
        found_circles[1] = found_circles[1] + 1
		print("red circle is found", found_circles[1])
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
	cv.imshow('res_red', resize_red)
	else:
		print("No circle detected")
	cv.waitKey(3)
	
	cv_image_to_imgmsg = bridge.cv2_to_imgmsg(resize, encoding="passthrough")
	image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
	image_publisher.publish(cv_image_to_imgmsg)


def opencv_listener():
	rospy.init_node('image_listener', anonymous = True)
	rospy.Subscriber("/video_source/raw", Image, handler)
	rate = rospy.Rate(30)
	#rospy.spin()
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == "__main__":
	try:
		opencv_listener()
	except rospy.ROSInterruptException:
		pass
