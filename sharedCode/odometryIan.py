from math import cos, sin, pi

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Pose
import numpy as np

class Odom:
	p = Pose()
	wl = 0
	wr = 0
	dt = 0.01
	posPub = rospy.Publisher('position', Pose, queue_size = 10)
	r = 0.05 # Wheel radius, 0.05m
	l = 0.19 # Distance between robot wheels 0.19m

	def lSpeed(self, data):
		self.wl = data.data
	
	def rSpeed(self, data):
		self.wr = data.data
	
	def calculate(self):
		
		euler = euler_from_quaternion((self.p.orientation.x,self.p.orientation.y,self.p.orientation.z,self.p.orientation.w))
		yaw = euler[2]
		self.p.position.x += (self.r*(self.wr+self.wl)/2*self.dt*cos(yaw))
		self.p.position.y += (self.r*(self.wr+self.wl)/2*self.dt*sin(yaw))
		yaw+=self.r*(self.wr-self.wl)/self.l*self.dt
		quat = quaternion_from_euler(0, 0, yaw)
		print(yaw)
		if(yaw>pi):
			yaw*=-1
		elif(yaw<-pi):
			yaw*=-1
		self.p.orientation.x = quat[0]
		self.p.orientation.y = quat[1]
		self.p.orientation.z = quat[2]
		self.p.orientation.w = quat[3]
		self.posPub.publish(self.p)

		
	def restart(self,_):
		self.p = Pose()

	def getPose(self):
		return self.p


def main():
	rospy.init_node('Odometry')
	r = rospy.Rate(100)
	odometria = Odom()
	rospy.Subscriber("/wl", Float32, odometria.lSpeed)
	rospy.Subscriber("/wr", Float32, odometria.rSpeed)
	rospy.Subscriber("position/restart",Empty, odometria.restart)

	while not rospy.is_shutdown():
		odometria.calculate()
		r.sleep()


if __name__ == "__main__":
    main()
