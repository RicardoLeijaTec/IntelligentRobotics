import rospy
import sys
import math
import time
from math import pi, atan2
import numpy as np
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose, Twist
from solution.msg import CirclePos
import matplotlib

# Robot location
xk = 0.0  # x position at k timestep in meters
yk = 0.0  # y position at k timestep in meters
theta_k = 0.0  # angle at k timestep in radians #### theta r es igual a theta k
'''	Values of theta must be contained within a single circle: EITHER: -pi <= theta < pi OR: 0 <= theta < 2pi'''

# Robot constants
r = 0.05  # Wheel radius, 0.05m
l = 0.19  # Distance between robot wheels 0.19m

# Measured variables
w_l, w_r = 0.0, 0.0  # Wheels velocity (rad/s)
dt = 0.01  # Time between samples (s)

final_pos_x = 0.0  # Posicion en x del centro del circulo en la imagen
final_pos_y = 0.0  # Posicion en y del centro del circulo en la imagen
final_pos_z = 0.0  # Radio del circulo

# Estado del robot, 0 es para corregir angulo, 1 es para ir a la ubicacion deseada
robot_state = 0
# 2 es para recibir otra posicion, se pueden agregar mas estados
lost = 0  # Verificacion de si el robot se paso el objetivo o no
halt = False  # Wait for target position
light_state = ""  # Traffic light state

# Publisher al nodo de velocidades
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
actual_position_pub = rospy.Publisher('actual_position', Pose, queue_size=10, latch=True)
actual_pos = Pose()

distance_until_center = -11
iteraciones_centrado = 0

def get_target(data):
    global halt, final_pos_x, final_pos_y, final_pos_z
    final_pos_x = data.position.x
    final_pos_y = data.position.y
    final_pos_z = data.position.z
    # halt = False


def odom_l(data):
    global w_l
    w_l = data.data


def odom_r(data):
    global w_r
    w_r = data.data


def light_state_handler(data):
    global light_state
    light_state = data.data


def go_to_blue_circle(x_distance_to_center, radius):
	global cmd_vel_pub, light_state
	twist_robot = Twist()
	vel_w = 0.5*(x_distance_to_center/640)
	vel_v = 0.5/(radius+1)
	
	if(light_state == "red_light"):
		print("rojo")
		twist_robot.linear.x = 0.0
		twist_robot.angular.z = 0.0
		cmd_vel_pub.publish(twist_robot)
		# return False
	else:
		pass

	if(radius > 200):
		twist_robot.linear.x = 0.0
		twist_robot.angular.z = 0.0
		cmd_vel_pub.publish(twist_robot)
		# return True
	# else:
	# 	cmd_vel_pub.publish(twist_robot)
	cmd_vel_pub.publish(twist_robot)
	print("vel_w",vel_w,"vel_v", vel_v)


def correct_angle(x_distance):
	global cmd_vel_pub, light_state
	global iteraciones_centrado
	w_to_cmd_vel = Twist()
	vel_w = 0.05*(abs(x_distance)/640)
	if(light_state == "red_light"):
		w_to_cmd_vel.angular.z = 0.0
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False
	else:
		pass

	if(x_distance < 0):
		vel_w = -vel_w
	print("distancia",x_distance,"vel",vel_w)

	if(-5 <= x_distance <= 5):
		iteraciones_centrado = iteraciones_centrado + 1
		return iteraciones_centrado
		# return True
	else:
		w_to_cmd_vel.angular.z = vel_w
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False
		

def reach_goal(distance, radius):
	global cmd_vel_pub, robot_state, iteraciones_centrado
	v_to_cmd_vel = Twist()
	iteraciones_centrado = 0

	if(light_state == "red_light"):
		v_to_cmd_vel.linear.z = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		return False
	else:
		pass
	
	# if(abs(distance_until_center) > 128):
	# 	robot_state = 0
	# 	return
	if(radius < 200):
		v_to_cmd_vel.linear.x = 0.06
		cmd_vel_pub.publish(v_to_cmd_vel)
	else:
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		robot_state = 3
	

def actuate():
	global theta_k, xk, yk, lost
	global robot_state
	global actual_position_pub, actual_pos
	global distance_until_center

	theta_k = theta_k + r*((w_r - w_l)/l)*dt
	degree = theta_k*180/pi

	xk = xk + r*((w_r + w_l)/2)*dt*math.cos(theta_k)
	yk = yk + r*((w_r + w_l)/2)*dt*math.sin(theta_k)
	distance_until_center = 640 - final_pos_x
    # lost = atan2(final_pos_y - yk, final_pos_x - xk)*180/pi
	if(theta_k > 2*math.pi):
		theta_k = 0
	if(theta_k < 0):
		theta_k = 2*math.pi
    # sys.stdout.write("x: %+06.2f\ty: %+06.2f\tLost: %+06.2f\tDegree: %+06.3f\t\tDistanceTillCenter: %+06.3f\tState: %d\n" % ((xk, yk, lost, degree, distance_until_center, robot_state)))
	if(robot_state == 0 and correct_angle(distance_until_center) == 100):
		robot_state = 1
	# elif(robot_state == 1 and reach_goal(distance_until_center, final_pos_z)):
	# 	robot_state = 2
	# if(robot_state == 0 and go_to_blue_circle(distance_until_center, final_pos_z)):
	# 	robot_state = 1
	# go_to_blue_circle(distance_until_center, final_pos_z)
	# print("robot_state", robot_state)
	print("distance_until_center",final_pos_x,"radio", final_pos_z)

	actual_pos.position.x = xk
	actual_pos.position.y = yk
	actual_pos.orientation.w = theta_k
	actual_position_pub.publish(actual_pos)


def main():
    global cmd_vel_pub

    rospy.init_node('control_odom')
    r = rospy.Rate(100)
    rl_gl_subscriber = rospy.Subscriber("/red_light_green_light", String, light_state_handler)
    wl_subscriber = rospy.Subscriber("/wl", Float32, odom_l)
    wr_subscriber = rospy.Subscriber("/wr", Float32, odom_r)
    circle_center_position = rospy.Subscriber("/circe_position", Pose, get_target)
    while not rospy.is_shutdown():
        if(not halt):
            actuate()
        r.sleep()


if __name__ == "__main__":
    main()