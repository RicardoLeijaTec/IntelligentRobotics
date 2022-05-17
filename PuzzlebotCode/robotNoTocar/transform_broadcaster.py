import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import math 
from math import pi, atan2
import time 
import numpy 

final_pos_x = 0.0  
final_pos_y = 0.0

def desired_pos_frame(data):
    global final_pos_x, final_pos_y
    final_pos_x = data.position.x
    final_pos_y = data.position.y

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id =  "world"
    t.child_frame_id = "punto_final"
    t.transform.translation.x = final_pos_x
    t.transform.translation.y = final_pos_y
    t.transform.translation.z = 0.0
    br.sendTransform(t)
    

if __name__ ==  '__main__':
    rospy.init_node('transform_broadcaster')
    desired_position = rospy.Subscriber("/target_position", Pose, desired_pos_frame)