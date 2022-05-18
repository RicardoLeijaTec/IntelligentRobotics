import sys, getopt
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge
from robotSiTocar.msg import CirclePos

bridge = CvBridge()

# Publishers
image_publisher = rospy.Publisher("image_publisher", Image, queue_size = 10)
position_publisher = rospy.Publisher("circe_position", CirclePos, queue_size = 10)

COLORS = {
  'green': {
    'name': 'green',
    'color_boundaries': [
      (np.array([50, 75, 75]), np.array([70, 255, 255]))
    ],
    'blur': 3,
  },
  'blue': {
    'name': 'blue',
    'color_boundaries': [
      (np.array([105, 50, 25]), np.array([135, 255, 255]))
    ],
    'blur': 5,
  },
  'red': {
    'name': 'red',
    'color_boundaries': [
      (np.array([0,100,20]), np.array([8,255,255])),
      (np.array([175,100,20]), np.array([179,255,255]))
    ],
    'blur': 5,
  },
}

MIN_AREA = 500
drawCon = True
filter = 0
c = (255, 0, 0)
conFound = []

detecting_color = 'blue'


def color_detection(frame, color_boundaries, kernel_size = 5):
  frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
  
  if(len(color_boundaries) > 0):
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

  global MIN_AREA, drawCon, filter, c, conFound

  circle_position = CirclePos()
  color_filtered = color_detection(frame, color['color_boundaries'], color['blur'])
  contours, _ = cv.findContours(color_filtered, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  for cnt in contours:
    area = cv.contourArea(cnt)
    if area > MIN_AREA:
      peri = cv.arcLength(cnt, True)
      approx = cv.approxPolyDP(cnt, 0.02 * peri, True)
      if len(approx) == filter or filter == 0:
        if drawCon:
          cv.drawContours(frame, cnt, -1, c, 3)
        x, y, w, h = cv.boundingRect(approx)
        cx, cy = x + (w // 2), y + (h // 2)
        cv.rectangle(frame, (x, y), (x + w, y + h), c, 2)
        cv.circle(frame, (cx, cy), 5, c, cv.FILLED)
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
  global image_publisher, detecting_color

  frame = bridge.imgmsg_to_cv2(data, "bgr8")

  frame = detect_circle(frame, COLORS[detecting_color])

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
