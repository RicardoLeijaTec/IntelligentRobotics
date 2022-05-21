#!/usr/bin/env python
import sys
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
current_frame = None

def click_event(event, x, y, flags, param):
  global current_frame
  filename = './test.jpg'
  if(len(sys.argv) > 1):
    filename = sys.argv[1]
  cv.imwrite(filename, current_frame)


def show_img(frame):
  cv.imshow('Window', frame)
  cv.waitKey(3)


def video_handler(data):
  global current_frame
  current_frame = bridge.imgmsg_to_cv2(data, "bgr8")
  show_img(current_frame)


def main():
  cv.namedWindow('Window')
  cv.setMouseCallback('Window', click_event)
  rospy.init_node('image_listener', anonymous = True)
  rospy.Subscriber("/video_source/raw", Image, video_handler)
  rospy.spin()


if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
