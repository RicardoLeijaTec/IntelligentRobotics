import rospy
import sys
import math
from math import pi
import numpy as np
#from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose



def declare_Pos():
	#global desired_position
	rospy.init_node('declare_Pos', anonymous = True)
	pub = rospy.Publisher('final_pos', Pose, queue_size = 10)
	#my_msg = Float32MultiArray()
	
	desired_position = [0.0,5.0]
	pos = Pose()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		pos.position.x = desired_position[0]
		pos.position.y = desired_position[1]
		pub.publish(pos)
		rate.sleep()
		
if __name__ == "__main__":
	try:
		declare_Pos()
	except rospy.ROSInterruptException:
		pass