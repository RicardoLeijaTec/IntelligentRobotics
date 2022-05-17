# Code developed by Drip Team Remastered

# Import needed libraries
import sys
import rospy
from geometry_msgs.msg import Pose

# Final position sender
def send_pos():
	rospy.init_node('send_pos', anonymous = True) # Node initialization
	pub = rospy.Publisher('target_position', Pose, queue_size = 10) # Topic where the position is published
	
	pos = Pose() # Object of pose type
	desired_position = [0.0,3.0] # Array with the target position coordinates (x,y) 
	rate = rospy.Rate(10) # Data sending rate
	while not rospy.is_shutdown(): 
		pos.position.x = desired_position[0] # X coordinate assigned to the Pose type object
		pos.position.y = desired_position[1] # Y coordinate assigned to the Pose type object
		pub.publish(pos) # Publish of the Pose object to the topic
		rate.sleep()
	
# Main function
if __name__ == "__main__":
	try:
		send_pos()
	except rospy.ROSInterruptException:
		pass
