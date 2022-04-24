import rospy
import sys
import cv2 as cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
#comando_pub = rospy.Publisher("publisherrrrr",String)

def Opencv_callback(data):
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	resize = cv2.resize(cv_image, (320,180)) # Resize image
	linea = cv2.line(resize, (142, 160), (178, 160), (0,255,0), 5)

	print(resize[142][160],resize[178][160])
	cv2.imshow("Imagen_linea", linea)

	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	# Blur the image for better edge detection
	img_blur = cv2.GaussianBlur(gray, (3,3),0)
	edges = cv2.Canny(image = img_blur, threshold1 = 100, threshold2 = 100) # Canny edge detection
	resize = cv2.resize(edges, (320,180)) # Resize image
	cv2.imshow('Canny edge detection', resize) # Display Canny Edge Detection

	cv2.waitKey(3)


def main():
	rospy.init_node('prueba_vision_1p0')
	image_sub = rospy.Subscriber("/video_source/raw", Image, Opencv_callback)
	rospy.spin()

if __name__ == "__main__":
	main()