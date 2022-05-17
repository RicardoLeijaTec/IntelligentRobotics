import rospy
import sys
import cv2 as cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
# Topic where the processed image will be published
canny_publisher = rospy.Publisher("canny_publisher", Image, queue_size = 10)


def opencv_handler(data):
	global canny_publisher
	# Image to cv2 format
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	# Grayscale image processing
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	# Image blur for better edge detection
	img_blur = cv2.GaussianBlur(gray, (3,3),0)
	
	# Canny edge detection
	edges = cv2.Canny(image = img_blur, threshold1 = 100, threshold2 = 100)
	
	# Image resizing made for visualizing a smoother video stream
	resize = cv2.resize(edges, (320,180))	
	
	# Display Canny Edge Detection
	cv2.imshow('Canny edge detection', resize)

	# cv2 to Image format
	cv_image_to_imgmsg = bridge.cv2_to_imgmsg(resize, encoding="passthrough")

	# Processed image publisher
	canny_publisher.publish(cv_image_to_imgmsg)
	cv2.waitKey(3)


def main():
	# Node initialization
	rospy.init_node('canny_robot_implementation')
	# Image source subscriber
	image_sub = rospy.Subscriber("/video_source/raw", Image, opencv_handler)
	rospy.spin()

if __name__ == "__main__":
	main() 