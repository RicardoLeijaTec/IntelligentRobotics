import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()
'''
def opencv_talker(image):
	image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
	rospy.init_node('talker_image', anonymous = True)
	rate = rospy.Rate(10) # 10 Hz

	while not rospy.is_shutdown():
		#image = #obtener imagen
		#rospy.loginfo(resize)
		image_publisher.publish(image)
		rate.sleep()'''
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
	#print('AAA')
    	pass


def handler(data):
	global l_h, l_s, l_v, u_h, u_s, u_v
	frame = bridge.imgmsg_to_cv2(data, "bgr8")
	# Grayscale encoding applied to the initial image
	#gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	font = cv.FONT_HERSHEY_COMPLEX
	hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

	# Blur the image for better edge detection
	#img_blur = cv.GaussianBlur(hsv, (5,5),cv.BORDER_DEFAULT)
	
	resize = cv.resize(frame, (288,162)) # Image resizing
	# Image show
	#cv.imshow("Post processed image", resize)

	blue_color_lower = np.array([l_h,l_s,l_v])
	blue_color_upper= np.array([u_h,u_s,u_v])
	#blue_color_lower = np.array([90,64,96])
	#blue_color_upper= np.array([150,255,255])

	mask = cv.inRange(resize,blue_color_lower,blue_color_upper)
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
			cv.drawContours(resize,[approx],0,(0,0,0),5)
			if len(approx) == 4:
				cv.putText(resize,"Rectangle",(x,ye),font,1,(0,0,0))
		if area > 4000:
			cv.drawContours(resize,[approx],0,(0,0,0),5)
			if len(approx) == 1:
				cv.putText(resize,"Circle",(x,ye),font,1,(0,0,0))

	cv.imshow("Resized frame", resize)
   	cv.imshow("Mask",mask)
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