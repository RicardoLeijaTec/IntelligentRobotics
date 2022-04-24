#!/usr/bin/env python
from cmath import pi
from math import atan2, cos, sin, sqrt
import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32, Float32MultiArray


class Puzzlebot:
    def __init__(self, r, l, thethai, xi, yi):
        self.r = r
        self.l = l
        self.thetha = thethai
        self.x = xi
        self.y = yi
        self.wl = 0
        self.wr = 0
        self.dt = .1
        
    def whr(self, msg):
        pubr.publish(msg.data)
        self.wr = msg.data

    def whl(self, msg):
        publ.publish(msg.data)
        self.wl = msg.data

    def position(self):
        self.x = self.x + self.r*((self.wr+self.wl)/2)*self.dt*cos(self.thetha)
        self.y = self.y + self.r*((self.wr+self.wl)/2)*self.dt*sin(self.thetha)
        self.thetha = self.thetha + self.r*((self.wr-self.wl)/self.l)*self.dt
        if(self.thetha > 2*pi):
            self.thetha = 0
        if(self.thetha < 0):
            self.thetha = 2*pi
        pubp.publish(self.x,self.y,self.thetha)
        print("Posicion actualizada")

    def error(self,xt,yt):
        et = atan2(xt,yt)-self.thetha
        ed = sqrt(pow((xt-self.x),2)+pow((yt-self.y),2))
        pubed.publish(et)
        pubet.publish(ed)
        
if __name__ == '__main__':
    try:
        puzzlebot = Puzzlebot(0.05, 0.19, 0, 0, 0)
        pubr = rospy.Publisher("vel_wr", Float32, queue_size=10) #Opcional
        publ = rospy.Publisher("vel_wl", Float32, queue_size=10) #Opcional
        pubp = rospy.Publisher("pos", Vector3, queue_size=10)
        pubed = rospy.Publisher("errort",Float32, queue_size=10)
        pubet = rospy.Publisher("errord", Float32, queue_size=10)
        rospy.init_node("position")
        rate = rospy.Rate(10)
        rospy.Subscriber("wr", Float32, puzzlebot.whr)
        rospy.Subscriber("wl", Float32, puzzlebot.whl)
        while not rospy.is_shutdown():
            puzzlebot.position()
            puzzlebot.error(10,0)
            rate.sleep
    except rospy.ROSInterruptException:
        pass
