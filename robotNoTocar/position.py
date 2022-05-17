import rospy
import sys
import math
from math import pi
import numpy as np
from geometry_msgs.msg import Pose


def declare_position():
	rospy.init_node('declare_position', anonymous = True)
	pub = rospy.Publisher('target_position', Pose, queue_size = 10)
	pos = Pose()
	
	desired_position = [0.0,3.0]
	
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		pos.position.x = desired_position[0]
		pos.position.y = desired_position[1]
		pub.publish(pos)
		rate.sleep()
	
	
if __name__ == "__main__":
	try:
		declare_position()
	except rospy.ROSInterruptException:
		pass
