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
import detection

bridge = CvBridge()

# Publishers
image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
position_publisher = rospy.Publisher("circe_position", CirclePos, queue_size = 10)

COLORS = {
  'green': {
    'name': 'green',
    'color_boundaries': [
      (np.array([45, 57, 80]), np.array([75, 255, 255]))
    ],
    'blur': 5,
    'rgb': (0, 255, 0),
    'mask_generator': lambda frame: detection.in_range(frame, 85),
    'hue': 85,
  },
  'blue': {
    'name': 'blue',
    'color_boundaries': [
      (np.array([105, 50, 25]), np.array([135, 255, 255]))
    ],
    'blur': 5,
    'rgb': (0, 0, 255),
    'mask_generator': lambda frame: detection.in_range(frame, 214),
    'hue': 214,
  },
  'red': {
    'name': 'red',
    'color_boundaries': [
      (np.array([0,100,20]), np.array([8,255,255])),
      (np.array([175,100,20]), np.array([179,255,255]))
    ],
    'blur': 5,
    'rgb': (255, 0, 0),
    'mask_generator': lambda frame: detection.in_range(frame, 356),
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


def color_detection(frame, color):
  color_boundaries = color['color_boundaries']
  kernel_size = color['blur']

  frame = np.copy(frame)
  frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
  
  if('mask_generator' in color):
    color_mask = color['mask_generator'](frame)
  else:
    assert len(color_boundaries) > 0

    lower, upper = color_boundaries[0]
    color_mask = cv.inRange(frame, lower, upper)
    
    for (lower, upper) in color_boundaries[1:]:
      color_mask = cv.add(color_mask, cv.inRange(frame, lower, upper))
  
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
  color_filtered = color_detection(frame, color)
  # show_img(color_filtered)
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

  frame = detect_circle(frame, COLORS[detecting_color])

  frame = cv.resize(frame, (1280//4, 720//4))
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