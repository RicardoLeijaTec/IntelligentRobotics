import rospy
import sys
import math
import time
from math import pi, atan2
import numpy as np
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose, Twist
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

final_pos_x = 0.0  # Constante que nos da la posicion objetivo del robot en "x"
final_pos_y = 0.0  # Constante que nos da la posicion objetivo del robot en "y"

e_theta = 0.07  # Error en el angulo
e_d = 0.0  # Distancia faltante

# Estado del robot, 0 es para corregir angulo, 1 es para ir a la ubicacion deseada
robot_state = 0
# 2 es para recibir otra posicion, se pueden agregar mas estados
lost = 0  # Verificacion de si el robot se paso el objetivo o no
halt = True  # Wait for target position
light_state = ""  # Traffic light state

# Publisher al nodo de velocidades
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)

actual_position_pub = rospy.Publisher('actual_position', Pose, queue_size=10, latch=True)

actual_pos = Pose()


def get_target(data):
    global halt, final_pos_x, final_pos_y
    final_pos_x = data.position.x
    final_pos_y = data.position.y
    halt = False


def odom_l(data):
    global w_l
    w_l = data.data


def odom_r(data):
    global w_r
    w_r = data.data


def light_state_handler(data):
    global light_state
    light_state = data.data


def correct_angle(error):
	global cmd_vel_pub, light_state
	w_to_cmd_vel = Twist()
	vel_w = 0.2 * error

	if(light_state == "red_light"):
		w_to_cmd_vel.angular.z = 0.0
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False
	else:
		pass

	if(-0.005 < error < 0.005):
		w_to_cmd_vel.angular.z = 0.0
		cmd_vel_pub.publish(w_to_cmd_vel)
		return True
	if(error >= 0.1):
		w_to_cmd_vel.angular.z = vel_w
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False


def reach_goal(distancia_total, distancia_faltante, lost):
	global cmd_vel_pub
	v_to_cmd_vel = Twist()

	if(light_state == "red_light"):
		v_to_cmd_vel.linear.z = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		return False
	else:
		pass

	# minimum of 0
	if(distancia_total != 0):
		vel_v = max(0, 0.45*(distancia_faltante/distancia_total))
	else:
		vel_v = 0

	if(lost < 0):
		vel_v = -vel_v

	distancia_recorrida = distancia_total - distancia_faltante
	if(distancia_recorrida >= distancia_total or distancia_faltante < 0.0095):
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		return True
	else:
		v_to_cmd_vel.linear.x = vel_v
		cmd_vel_pub.publish(v_to_cmd_vel)
		return False


def actuate():
    global theta_k, xk, yk, d_t, e_d, lost
    global robot_state
    global actual_position_pub, actual_pos

    e_theta = atan2(final_pos_y, final_pos_x) - theta_k
    theta_k = theta_k + r*((w_r - w_l)/l)*dt
    degree = theta_k*180/pi

    xk = xk + r*((w_r + w_l)/2)*dt*math.cos(theta_k)
    yk = yk + r*((w_r + w_l)/2)*dt*math.sin(theta_k)

    d_t = math.sqrt((final_pos_x)**2 + (final_pos_y) **2)  # Distancia total por recorrer
    e_d = math.sqrt((final_pos_x - xk)**2 + (final_pos_y - yk)**2)

    lost = atan2(final_pos_y - yk, final_pos_x - xk)*180/pi

    if(theta_k > 2*math.pi):
        theta_k = 0
    if(theta_k < 0):
        theta_k = 2*math.pi

    sys.stdout.write("x: %+06.2f\ty: %+06.2f\tLost: %+06.2f\tDegree: %+06.3f\t\tError: %+06.3f" % ((xk, yk, lost, degree, e_theta)))
    sys.stdout.write("\t\td_r: %+06.2f\td_t: %+06.2f\td_f: %+06.2f | %d\n" % (d_t-e_d, d_t, e_d, robot_state))

    if(robot_state == 0 and correct_angle(e_theta)):
        robot_state = 1

    elif(robot_state == 1 and reach_goal(d_t, e_d, lost)):
        robot_state = 2

    actual_pos.position.x = xk
    actual_pos.position.y = yk
    actual_pos.orientation.w = theta_k  # Ver como subir angulo a esta cosa, o usar el quaternion en vez de point	
    actual_position_pub.publish(actual_pos)


def main():
    global cmd_vel_pub

    rospy.init_node('control_odom')
    r = rospy.Rate(100)
    rospy.Subscriber("/red_light_green_light", String, light_state_handler)
    rospy.Subscriber("/wl", Float32, odom_l)
    rospy.Subscriber("/wr", Float32, odom_r)
    rospy.Subscriber("/target_position", Pose, get_target)
    while not rospy.is_shutdown():
        if(not halt):
            actuate()
        r.sleep()


if __name__ == "__main__":
    main()

# TO DO:
# Comprobar si al llamarlo 2 veces causa problemas o no afecta en el procesamiento  PROBADO
# RECIBIR LOS VALORES DE LOS SUBSCRIBERS EN LA FUNCION DE LOCALIZACION              FUNCIONA
# CALCULO CONSTANTE DE LA POSICION (no se si se reinicia cada que llama al main)    FUNCIONA
# DAR SALIDA DE LOS 3 VALORES (prints de los valores)                               FUNCIONA
# HACER QUE EL VALOR DE THETA OSCILE ENTRE LOS RANGOS DE VALORES DE THETA           FUNCIONA
# PROBAR DIFERENTES dt                                                              FUNCIONA
# DEFINICION DE LAS VARIABLES FUERA DE LA FUNCION                                   FUNCIONA
# PUBLICAR LOS VALORES (SOLO LOS TENEMOS CON PRINTS)                                READY
