import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge

bridge = CvBridge()

image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
light_state_publisher = rospy.Publisher("red_light_green_light", String, queue_size = 10)
circle_center_position = rospy.Publisher("circe_position", Pose, queue_size = 10)
found_circles = [0,0,0] # [0] = green, [1] = red, [2] = blue

lower_red_1 = np.array([0,100,20])
upper_red_1 = np.array([8,255,255])
lower_red_2 = np.array([175,100,20])
upper_red_2 = np.array([179,255,255])

LOWER_GREEN = np.array([50, 75, 75])
UPPER_GREEN = np.array([70, 255, 255])

LOWER_BLUE = np.array([105, 50, 25])
UPPER_BLUE = np.array([135, 255, 255])

center_position = Pose() # global center_position

minArea=500
sort = True
drawCon = True
filter = 0
c=(255, 0, 0)
conFound = []


def color_detection(frame, lower_bound, upper_bound, kernel_size = 5):
	frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	color_mask = cv.inRange(frame, lower_bound, upper_bound)
	frame = cv.bitwise_and(frame, frame, mask=color_mask)
	frame = cv.medianBlur(frame, kernel_size)
	#HSV2BGR then BGR2GRAY or HSV2GRAY
	frame = cv.cvtColor(frame, cv.COLOR_HSV2BGR)
	frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	return frame


def video_handler(data):
	global found_circles
	global image_publisher
	
	global lower_red_1, lower_red_2, upper_red_1, upper_red_2
	global LOWER_GREEN, UPPER_GREEN
	global LOWER_BLUE, UPPER_BLUE

	global minArea, sort, drawCon, filter, c, conFound
	
	frame = bridge.imgmsg_to_cv2(data, "bgr8")
	cimg_blue = color_detection(frame, LOWER_BLUE, UPPER_BLUE, 5)
	
	contours, _ = cv.findContours(cimg_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv.contourArea(cnt)
		if area > minArea:
			peri = cv.arcLength(cnt, True)
			approx = cv.approxPolyDP(cnt, 0.02 * peri, True)
			# print(len(approx))
			if len(approx) == filter or filter == 0:
				if drawCon: cv.drawContours(frame, cnt, -1, c, 3)
				x, y, w, h = cv.boundingRect(approx)
				cx, cy = x + (w // 2), y + (h // 2)
				cv.rectangle(frame, (x, y), (x + w, y + h), c, 2)
				cv.circle(frame, (cx, cy), 5, c, cv.FILLED)
				conFound.append({"cnt": cnt, "area": area, "bbox": [x, y, w, h], "center": [cx, cy]})
				center_position.position.x = cx
				center_position.position.z = area
				circle_center_position.publish(center_position)
				# print(conFound)
	# if sort:
	# 	conFound = sorted(conFound, key=lambda x: x["area"], reverse=True)
	# cv.imshow('imagen', frame)
	# cv.waitKey(3)
	
	#img_blur = cv.GaussianBlur(hsv, (5,5),cv.BORDER_DEFAULT) # Blur the image for better edge detection
	# RED LIGHT
	# lower_mask_red = cv.inRange(hsv, lower_red_1, upper_red_1)
	# upper_mask_red = cv.inRange(hsv, lower_red_2, upper_red_2)
	# full_red_mask = cv.add(lower_mask_red, upper_mask_red)
	# res_red = cv.bitwise_and(frame, frame, mask=full_red_mask)
	# img_red = cv.medianBlur(res_red, 5)
	# ccimg_red = cv.cvtColor(img_red, cv.COLOR_HSV2BGR)
	# cimg_red = cv.cvtColor(ccimg_red, cv.COLOR_BGR2GRAY)
	# circles_red = cv.HoughCircles(cimg_red, cv.HOUGH_GRADIENT, 1, 700, param1=30, param2=20, minRadius=5, maxRadius=700)
	# if circles_red is not None:
	# 	light_state_publisher.publish("red_light")
	# 	found_circles[1] = found_circles[1] + 1
	# 	print("red circle is found", found_circles[1])
	# cv.imshow('res_red', resize_final)

	# GREEN LIGHT
	# mask_green = cv.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
	# res_green = cv.bitwise_and(frame, frame, mask=mask_green)
	# img_green = cv.medianBlur(res_green, 3)
	# ccimg_green = cv.cvtColor(img_green, cv.COLOR_HSV2BGR)
	# cimg_green = cv.cvtColor(ccimg_green, cv.COLOR_BGR2GRAY)
	# circles_green = cv.HoughCircles(cimg_green, cv.HOUGH_GRADIENT, 1, 700, param1=30, param2=20, minRadius=5, maxRadius=700)
	# if circles_green is not None:
	# 	light_state_publisher.publish("green_light")
	# 	found_circles[0] = found_circles[0] + 1
	# 	print("green circle is found", found_circles[0])
	# # cv.imshow('res_green', cimg_green)
	
	# BLUE LIGHT
	# mask_blue = cv.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
	# res_blue = cv.bitwise_and(frame, frame, mask=mask_blue)
	# img_blue = cv.medianBlur(res_blue, 5)
	# ccimg_blue = cv.cvtColor(img_blue, cv.COLOR_HSV2BGR)
	# cimg_blue = cv.cvtColor(ccimg_blue, cv.COLOR_BGR2GRAY)
	# resize_final = cv.resize(cimg_blue, (1280/5,720/5)) # Image resizing
	# circles_blue = cv.HoughCircles(cimg_blue, cv.HOUGH_GRADIENT, 1, 1000, param1=30, param2=20, minRadius=5, maxRadius=700)
	# if circles_blue is not None:
	# 	found_circles[2] = found_circles[2] + 1
	# 	print("blue circle is found", found_circles[2])
	# 	circles_blue = np.uint16(np.around(circles_blue))
	# 	for i in circles_blue[0,:]:
	# 		center_position.position.x = i[0]
	# 		center_position.position.z = i[2]
	# 		circle_center_position.publish(center_position)
	# 		# draw the outer circle
	# 		cv.circle(resize_final,(i[0]/5,i[1]/5),i[2]/5,(0,255,0),2)
	# 		# draw the center of the circle
	# 		cv.circle(resize_final,(i[0]/5,i[1]/5),2,(0,0,255),3)
	# 		# print(i[0],i[1])
	# cv.imshow('res_blue', resize_final)
	# cv.waitKey(3)
	
	cv_image_to_imgmsg = bridge.cv2_to_imgmsg(frame, encoding = "passthrough")
	image_publisher.publish(cv_image_to_imgmsg)
	


def opencv_listener():
	rospy.init_node('image_listener', anonymous = True)
	rospy.Subscriber("/video_source/raw", Image, video_handler)
	rate = rospy.Rate(30)
	#rospy.spin()
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == "__main__":
	try:
		opencv_listener()
	except rospy.ROSInterruptException:
		pass
