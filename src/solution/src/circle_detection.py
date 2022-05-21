#!/usr/bin/env python
import sys, getopt
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge
from solution.msg import CirclePos
import color_detection

bridge = CvBridge()

# Publishers
image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
position_publisher = rospy.Publisher("circe_position", CirclePos, queue_size = 10)

COLORS = {
  'green': {
    'name': 'green',
    'blur': 5,
    'rgb': (0, 255, 0),
    'hue': 85,
  },
  'blue': {
    'name': 'blue',
    'blur': 5,
    'rgb': (0, 0, 255),
    'hue': 214,
  },
  'red': {
    'name': 'red',
    'blur': 5,
    'rgb': (255, 0, 0),
    'hue': 356,
  },
}

MIN_AREA = 6000
filter = 0
conFound = []

detecting_color = 'blue'

def show_img(frame):
  cv.imshow('Detection', frame)
  cv.waitKey(3)


def detect_color(frame, color):
  kernel_size = color['blur']

  frame = np.copy(frame)
  frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
  
  color_mask = color_detection.in_range(frame, color['hue']) 

  frame = cv.bitwise_and(frame, frame, mask=color_mask)
  frame = cv.medianBlur(frame, kernel_size)

  #HSV2BGR then BGR2GRAY or HSV2GRAY
  frame = cv.cvtColor(frame, cv.COLOR_HSV2BGR)
  frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  return frame


def detect_circle(frame, color):
  global position_publisher
  global MIN_AREA, filter, conFound

  circle_position = CirclePos()
  color_filtered = detect_color(frame, color)
  #show_img(color_filtered)
  contours, _ = cv.findContours(color_filtered, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  for cnt in contours:
    area = cv.contourArea(cnt)
    if area > MIN_AREA:
      peri = cv.arcLength(cnt, True)
      approx = cv.approxPolyDP(cnt, 0.02 * peri, True)
      # centers, radius = cv.minEnclosingCircle(approx)
      if len(approx) == filter or filter == 0:
        cv.drawContours(frame, cnt, -1, color['rgb'], 3)
        x, y, w, h = cv.boundingRect(approx)
        cx, cy = x + (w // 2), y + (h // 2)
        cv.rectangle(frame, (x, y), ((x + w), (y + h)), color['rgb'], 2)
        cv.circle(frame, (cx, cy), 5, color['rgb'], cv.FILLED)
        conFound.append({
          "cnt": cnt, "area": area,
          "bbox": [x, y, w, h], "center": [cx, cy]
        })
        circle_position.x = cx
        circle_position.area = area
        circle_position.color = color['name']
        position_publisher.publish(circle_position)
  return frame


def video_handler(data):
  global image_publisher, detecting_color, conFound

  frame = bridge.imgmsg_to_cv2(data, "bgr8")

  height = frame.shape[0]
  width = frame.shape[1]

  frame = cv.resize(frame, (width//2, height//2))
  frame = detect_circle(frame, COLORS[detecting_color])

  frame = cv.resize(frame, (width//4, height//4))
  show_img(frame)

  cv_image_to_imgmsg = bridge.cv2_to_imgmsg(frame, encoding = "passthrough")
  image_publisher.publish(cv_image_to_imgmsg)



def main():
  rospy.init_node('image_listener', anonymous = True)
  rospy.Subscriber("/video_source/raw", Image, video_handler)
  rospy.spin()


if __name__ == "__main__":
  try:
    opts, args = getopt.getopt(sys.argv[1:], "hc:", ["color="])
    if(len(opts) == 0):
      raise getopt.GetoptError('')
  except getopt.GetoptError:
    print('Usage:')
    print('circle_detection.py -h -c <color>')
    sys.exit(2)
     
  for opt, arg in opts:
    if opt == '-h':
      print('Usage:')
      print('circle_detection.py -c <color>')
      sys.exit()
    elif opt in ("-c", "--color"):
      if(arg in ['blue', 'green', 'red']):
        detecting_color = arg

  try:
    main()
  except rospy.ROSInterruptException:
    pass
